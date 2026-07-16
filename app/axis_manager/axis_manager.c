/*
 * app/axis_manager — protocol v3 implementation.
 *
 * The cyclic command is a tiny 8-byte streaming frame: controlword,
 * joystick_value, command_counter. Everything else (mode, target_position,
 * target_time, profile params, joystick scaling) lives on the motor MCU
 * and is set via SDO (cia402_od_write_begin) before triggering a move.
 *
 * Pieces:
 *  - Data model — values written by protocol modules via cmc_od.
 *  - Setup-sequencer — when the desired setup differs from what the motor
 *    MCU has, queue an SDO write. One in flight at a time; the rest queue.
 *  - compose_cyclic_cmd — builds the streaming command each tick. Adds
 *    NEW_SETPOINT bit when start_move is pulsed and the setup is fully
 *    applied. JOYSTICK mode streams joystick_value directly; the motor MCU
 *    applies it live without needing the trigger.
 *
 * === Table of contents (line numbers approximate — see grep for exact) ===
 *
 *   ~100  Module state (s_axis + friends)
 *   ~170  Operation-level arbitration state (JOYSTICK / SHOT_RECALL / HOMING)
 *   ~330  Scaling helpers (unit conversion)
 *   ~445  Persistence (blob save/load, PERSIST_REGION_CONFIG co-tenants)
 *   ~540  Lifecycle (axis_manager_init)
 *   ~610  Desired-setup snapshot (clamp/scale before SDO)
 *   ~650  SDO setup-sequencer (one-in-flight write pipeline)
 *   ~820  Cyclic command composition
 *   ~920  axis_manager_tick — main entry point
 *   ~950  On-board UP/DOWN button poll + JOYSTICK-mode jog
 *  ~1240  State getters (backing 0x30xx RO OD entries)
 *  ~1260  Command latches (enable/quick_stop/clear_fault/start_move)
 *  ~1305  Mode + targets (backing 0x3020..0x302A RW OD entries)
 *  ~1440  Limits + calibration (backing 0x303x)
 *  ~1500  Load-factor SDO helper
 *  ~1565  Motor-proxy SDO writes (vel_accel_* + motor-save shared handle)
 *  ~1700  Motor-proxy bootsync (SDO-read each accel param on first start)
 *  ~1848  Motor-save sequencer (disable → SDO save → re-enable)
 *  ~1975  Home-to-endstop sequencer + is_homed cache + encoder-type detect
 *
 * TODO — future refactor: extract home_sequencer, motor_save, and
 * bootsync into their own .c files under app/axis_manager/. Each is a
 * self-contained state machine with its own statics; moving them out
 * would shrink this file to ~1500 lines. Deferred: they touch each
 * other + reset_motor_od_submodules touches all their statics on the
 * motor-in-bootloader edge, so extraction needs a small internal header
 * and Makefile updates. Not urgent — the file is well-sectioned above.
 */

#include "axis_manager.h"

#include "app/cia402/cia402.h"
#include "app/led_indicator/led_indicator.h"  /* persist-blob co-tenant */
#include "app/log/log.h"
#include "app/persist/persist.h"
#include "bsp/buttons/buttons.h"   /* on-board UP/DOWN button reads (debounced) */
#include "bsp/time/time.h"

#include <string.h>
#include <math.h>

/* Motor MCU OD entries we SDO-write during setup (per mc_if_od.h v3). */
#define OD_MODES_OF_OPERATION     0x6060u
#define OD_TARGET_TORQUE          0x6071u   /* int32 raw = amps / MC_IF_CUR_SCALE (1e-3 A/LSB) */
#define OD_TARGET_POSITION        0x607Au
#define OD_TARGET_POSITION_TIME   0x607Bu
#define OD_PROFILE_VELOCITY       0x6081u
#define OD_PROFILE_ACCELERATION   0x6083u
#define OD_PROFILE_DECELERATION   0x6084u

/* Translate AXIS_OP_MODE_* into the motor MCU's CiA-402 mode_of_operation.
 *
 * Note (v3): the motor MCU no longer has a joystick mode. AXIS_OP_MODE_JOYSTICK
 * is a CMC-only application concept; on the motor side it's just velocity
 * control — axis_manager streams `velocity_setpoint = joystick_value *
 * joystick_max_velocity` every cycle, the motor follows it live. */
static int8_t axis_mode_to_cia402(axis_op_mode_t m)
{
    switch (m) {
    case AXIS_OP_MODE_JOYSTICK:         return (int8_t)MC_IF_MODE_PROFILE_VELOCITY;
    case AXIS_OP_MODE_PROFILE_VELOCITY: return (int8_t)MC_IF_MODE_PROFILE_VELOCITY;
    case AXIS_OP_MODE_PROFILE_POSITION: return (int8_t)MC_IF_MODE_PROFILE_POSITION;
    case AXIS_OP_MODE_HOLD:             return (int8_t)MC_IF_MODE_PROFILE_VELOCITY;
    case AXIS_OP_MODE_TORQUE:           return (int8_t)MC_IF_MODE_TORQUE;
    case AXIS_OP_MODE_OFF:
    default:                             return (int8_t)MC_IF_MODE_DISABLED;
    }
}

/* String helpers — used purely for log readability. Kept here so all the
 * axis-mode/state names are co-located with the to_cia402 map above. */
static const char *axis_mode_name(axis_op_mode_t m)
{
    switch (m) {
    case AXIS_OP_MODE_OFF:              return "OFF";
    case AXIS_OP_MODE_JOYSTICK:         return "JOYSTICK";
    case AXIS_OP_MODE_PROFILE_VELOCITY: return "PROFILE_VELOCITY";
    case AXIS_OP_MODE_PROFILE_POSITION: return "PROFILE_POSITION";
    case AXIS_OP_MODE_HOLD:             return "HOLD";
    case AXIS_OP_MODE_TORQUE:           return "TORQUE";
    default:                            return "INVALID";
    }
}

static const char *axis_state_name(axis_state_t s)
{
    switch (s) {
    case AXIS_STATE_DISABLED: return "DISABLED";
    case AXIS_STATE_READY:    return "READY";
    case AXIS_STATE_RUNNING:  return "RUNNING";
    case AXIS_STATE_FAULT:    return "FAULT";
    default:                  return "?";
    }
}

static const char *seq_step_name(int step)
{
    switch (step) {
    case 0: return "IDLE";
    case 1: return "MODE";
    case 2: return "TARGET_POSITION";
    case 3: return "TARGET_POSITION_TIME";
    case 4: return "TARGET_TORQUE";
    case 5: return "PROFILE_VELOCITY";
    case 6: return "PROFILE_ACCELERATION";
    case 7: return "PROFILE_DECELERATION";
    default: return "?";
    }
}

/*----------------------------------------------------------------------------
 * Module state
 *---------------------------------------------------------------------------*/

typedef struct {
    /* State (RO from outside, written by axis_manager_tick) */
    axis_state_t     state;
    axis_op_mode_t   op_mode_actual;
    float            position_actual_rad;
    float            velocity_actual_rad_s;
    uint16_t         error_code;
    uint8_t          error_register;

    /* Command latches (set by setters, consumed by tick) */
    bool             enable_latch;
    bool             quick_stop_pulse;
    bool             clear_fault_pulse;
    bool             start_move_pulse;       /* triggers NEW_SETPOINT once setup is applied */

    /* Mode + per-mode targets (commanded by protocol modules) */
    axis_op_mode_t   op_mode_commanded;
    float            joystick_value;         /* -1.0 .. +1.0 (normalised) */
    /* joystick_max_velocity is DERIVED, not operator-settable — it tracks
     * velocity_limit_rad_s (motion limit, 0x3030) scaled by the currently-
     * selected joystick speed profile (NORMAL / MEDIUM / FINE).
     * Recomputed by recompute_joystick_max_velocity() whenever either
     * input changes. 0x3022 is exposed RO in the OD for observation. */
    float            joystick_max_velocity;
    uint8_t          joy_profile;            /* AXIS_JOY_PROFILE_* — CAMERAD JOY_PROFILE_* selects */
    uint8_t          axis_role;              /* 0x3070 — CAMERAD_AXIS_* bitmap value; PAN default */
    float            target_velocity_rad_s;
    float            target_position_rad;
    float            target_time_s;
    float            target_current_a;       /* REQ-0012 — used in AXIS_OP_MODE_TORQUE */
    float            button_current_a;       /* 0x302C — magnitude UP/DOWN buttons apply (TORQUE mode only) */

    /* Joystick raw input + linear-with-deadband calibration (0x3026-0x302A) */
    int32_t          joystick_raw;             /* latest raw value from protocol module */
    int32_t          joystick_raw_center;      /* rest position */
    int32_t          joystick_raw_full_pos;    /* raw at full positive deflection */
    int32_t          joystick_raw_full_neg;    /* raw at full negative deflection */
    uint32_t         joystick_raw_deadband;    /* +/- around center -> 0 output */

    /* Limits */
    float            velocity_limit_rad_s;
    float            position_limit_lo_rad;
    float            position_limit_hi_rad;
    float            accel_limit_rad_s2;

    /* Load factor cache — mirrors the motor MCU's 0x2300:5 vel_load_factor
     * (REQ-0014). The CMC web slider POSTs a new value; we cache it for
     * GET responses + fire an ad-hoc SDO write so the motor adopts it
     * for its velocity loop. Persisted by the motor MCU, not by us. */
    float            load_factor;
    /* Motor-proxied velocity-demand accel ramp (motor's 0x2300:6/7/8).
     * Cached locally so the web GET doesn't need an SDO read round-trip;
     * the motor's flash is authoritative (persisted via motor-save sequencer). */
    float            vel_accel_up_rad_s2;
    float            vel_accel_dn_rad_s2;
    float            vel_accel_jerk_rad_s3;

    /* Cached from the v4 cyclic status header (REQ-0013/ADR-033). The
     * motor MCU now puts position + a movement-status bitfield in the
     * fixed header so the CMC always has them, independent of the
     * host-configurable telemetry-map content. */
    uint16_t         movement_status;

    /* Operator-settable, persisted. When 1, axis_manager fires a home
     * command once per boot as soon as the motor's encoder type is known
     * and reports incremental. 0 = never auto-home. Backs 0x3043. */
    uint8_t          home_on_boot;
} axis_t;

static axis_t s_axis;

/*----------------------------------------------------------------------------
 * Operation arbitration state
 *
 * s_active_op is the currently-in-flight operation family (or NONE for
 * idle). Only cross-family transitions arbitrate; same-family requests
 * pass through as retarget. Completion detection lives in _tick_active_op
 * and clears s_active_op back to NONE.
 *
 * JOYSTICK_QUIESCENT_HOLD_MS gates the joystick->NONE transition: after
 * both joystick_value hits 0 and motor movement_status.MOVING clears, we
 * wait this long before releasing the JOYSTICK operation. Prevents rapid
 * flapping when the operator briefly parks the stick at centre and then
 * pushes again — but short enough that a genuine "release, then hit shot
 * recall" gesture flows through without operator perceiving a lockout.
 *---------------------------------------------------------------------------*/
static axis_operation_t s_active_op                 = AXIS_OPERATION_NONE;
static uint32_t         s_active_op_started_ms      = 0u;   /* time_ms() of NONE->op transition, for logging */
static uint32_t         s_joystick_quiescent_since_ms = 0u; /* 0 = not quiescent; else first tick both stopped */
#define JOYSTICK_QUIESCENT_HOLD_MS  200u

/* Forward declarations — bodies live near the home sequencer where the
 * related state is declared. */
static void tick_active_op(void);
static void abort_motor_homing_best_effort(void);

static const char *op_name(axis_operation_t op)
{
    switch (op) {
        case AXIS_OPERATION_NONE:        return "NONE";
        case AXIS_OPERATION_HOMING:      return "HOMING";
        case AXIS_OPERATION_SHOT_RECALL: return "SHOT_RECALL";
        case AXIS_OPERATION_JOYSTICK:    return "JOYSTICK";
        default:                         return "?";
    }
}

static void set_active_op(axis_operation_t new_op)
{
    if (s_active_op == new_op) return;
    LOG_INFO("axis_manager: active_op %s -> %s", op_name(s_active_op), op_name(new_op));
    s_active_op = new_op;
    if (new_op != AXIS_OPERATION_NONE) {
        s_active_op_started_ms = time_ms();
    }
    s_joystick_quiescent_since_ms = 0u;
}

axis_operation_t axis_manager_get_active_op(void)
{
    return s_active_op;
}

axis_begin_result_t axis_manager_try_begin_op(axis_operation_t desired)
{
    if (desired == AXIS_OPERATION_NONE) return AXIS_BEGIN_REJECTED;  /* use stop_op */
    if (s_active_op == desired) {
        /* Same-family retarget — always allowed. Refresh quiescent tracker so
         * a JOYSTICK that keeps arriving doesn't accidentally auto-clear. */
        s_joystick_quiescent_since_ms = 0u;
        return AXIS_BEGIN_CONTINUED;
    }
    if (s_active_op == AXIS_OPERATION_NONE) {
        set_active_op(desired);
        return AXIS_BEGIN_STARTED;
    }
    /* Cross-family conflict — reject. Operator must issue STOP first. Log at
     * INFO so a busy panel operator can see why their command was dropped. */
    LOG_INFO("axis_manager: %s request rejected (active_op = %s)",
             op_name(desired), op_name(s_active_op));
    return AXIS_BEGIN_REJECTED;
}

void axis_manager_stop_op(void)
{
    /* Always succeeds. Issues the appropriate motor-side stop primitive for
     * the currently-active op then clears active_op to NONE.
     *
     * Deliberately best-effort on the motor-side SDO writes: if the single
     * SDO slot is busy the abort write won't land immediately, but the
     * higher-level state is already cleared so a competing joystick trim /
     * shot recall waiting behind the block can proceed on the next tick.
     * The motor will settle to safe state on its own via the cyclic stream
     * (velocity_setpoint = 0 as soon as we exit the op) even if the SDO
     * fell through. */
    switch (s_active_op) {
        case AXIS_OPERATION_HOMING:
            abort_motor_homing_best_effort();
            break;
        case AXIS_OPERATION_JOYSTICK:
            /* Zero the demand. Motor will decelerate via vel_accel_dn. */
            s_axis.joystick_raw   = 0;
            s_axis.joystick_value = 0.0f;
            break;
        case AXIS_OPERATION_SHOT_RECALL:
            /* cmc_state owns s_next_shot; caller (cmc_state_stop_movement)
             * clears it. Nothing to do here at the SDO level — the motor
             * MCU stops on its own once we stop refreshing NEW_SETPOINT. */
            break;
        default:
            break;
    }
    set_active_op(AXIS_OPERATION_NONE);
}

/* Setup snapshot (what the motor MCU has confirmed via OD_WRITE_RESP OK).
 *
 * v3: velocity_setpoint is streamed in the cyclic, not SDO-written, so it's
 * not in this snapshot. joystick_scale also not present — the joystick
 * concept lives entirely on the LCMC. */
typedef struct {
    int8_t   mode;
    int32_t  target_position_scaled;
    uint32_t target_position_time_ms;
    int32_t  target_torque_scaled;
    uint32_t profile_velocity_scaled;
    uint32_t profile_acceleration_scaled;
    uint32_t profile_deceleration_scaled;
} setup_snapshot_t;

static setup_snapshot_t s_applied;       /* what the motor MCU has */
static bool             s_first_sync;    /* before first SDO success, force-sync everything */

/* Setup-sequencer in-flight tracking */
typedef enum {
    SEQ_IDLE = 0,
    SEQ_MODE,
    SEQ_TARGET_POSITION,
    SEQ_TARGET_POSITION_TIME,
    SEQ_TARGET_TORQUE,
    SEQ_PROFILE_VELOCITY,
    SEQ_PROFILE_ACCELERATION,
    SEQ_PROFILE_DECELERATION,
} seq_step_t;

static seq_step_t          s_seq_step;
static cia402_od_handle_t  s_seq_handle = CIA402_OD_HANDLE_INVALID;

/* NEW_SETPOINT pulse window — held for a few cycles to ensure the motor
 * MCU sees the rising edge even if cyclic frames are dropped. */
#define NEW_SETPOINT_PULSE_CYCLES   3u
static uint8_t s_new_setpoint_remaining;

/* HALT pulse — same shape as NEW_SETPOINT. Operator-driven controlled stop
 * from CAMERAD panel STOP key (via cmc_state). The motor MCU latches its
 * own stopped/holding state from the rising edge, so we don't need to
 * sustain HALT here. Few-cycle pulse for robust edge detection. */
#define HALT_PULSE_CYCLES           3u
static uint8_t s_halt_remaining;

/* FAULT_RESET pulse — same shape and same reason as NEW_SETPOINT / HALT.
 * Holding the bit for 3 cycles guarantees that at least one of those
 * cycles is actually transmitted to the motor as a cyclic_cmd frame
 * (rather than getting interleaved with cia402 OD traffic and lost),
 * which is what made "Clear Fault" reliably require two presses with
 * the previous single-cycle pulse. */
#define FAULT_RESET_PULSE_CYCLES    3u
static uint8_t s_fault_reset_remaining;

/*----------------------------------------------------------------------------
 * Scaling helpers
 *---------------------------------------------------------------------------*/

static float clamp_abs(float x, float limit)
{
    if (limit < 0.0f) limit = -limit;
    if (x >  limit) return  limit;
    if (x < -limit) return -limit;
    return x;
}

static float clamp_range(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t to_scaled_i32(float si, float scale)
{
    float raw_f = si / scale;
    if (raw_f >  2147483520.0f) return  2147483520;
    if (raw_f < -2147483520.0f) return -2147483520;
    return (int32_t)(raw_f + (raw_f >= 0.0f ? 0.5f : -0.5f));
}

static uint32_t to_scaled_u32(float si, float scale)
{
    if (si <= 0.0f) return 0u;
    float raw_f = si / scale;
    if (raw_f > 4294967040.0f) return 4294967040u;
    return (uint32_t)(raw_f + 0.5f);
}

/* Joystick speed profiles — CAMERAD JOY_PROFILE_* keypresses cycle
 * through these to give the operator a hardware coarse/fine control
 * over stick sensitivity. Scale factor multiplies the motion limit
 * (velocity_limit, 0x3030) to produce the effective joystick_max_velocity.
 * NORMAL = 1.0 means full-stick reaches the configured motion ceiling. */
#define AXIS_JOY_PROFILE_NORMAL   (0u)   /* 1.0x  — full stick reaches velocity_limit */
#define AXIS_JOY_PROFILE_MEDIUM   (1u)   /* 0.5x  — half speed for medium-sensitivity work */
#define AXIS_JOY_PROFILE_FINE     (2u)   /* 0.15x — precise framing */

static float joy_profile_scale(uint8_t p)
{
    switch (p) {
    case AXIS_JOY_PROFILE_MEDIUM: return 0.5f;
    case AXIS_JOY_PROFILE_FINE:   return 0.15f;
    case AXIS_JOY_PROFILE_NORMAL:
    default:                      return 1.0f;
    }
}

/* Derive joystick_max_velocity from the current motion limit and profile.
 * Called whenever either input changes (velocity_limit setter, profile
 * setter, or persist-load). */
static void recompute_joystick_max_velocity(void)
{
    s_axis.joystick_max_velocity =
        s_axis.velocity_limit_rad_s * joy_profile_scale(s_axis.joy_profile);
}

/* Apply the joystick calibration to the current raw value, write the
 * result into joystick_value. Linear with deadband, symmetric output
 * cap (joystick_max_velocity for both directions). Called when the raw
 * value or any cal entry changes.
 *
 * Output is the normalised SI value (-1..+1), NOT scaled to rad/s yet;
 * current_velocity_demand_rad_s() multiplies by joystick_max_velocity. */
static void recompute_joystick_value_from_raw(void)
{
    int32_t raw      = s_axis.joystick_raw;
    int32_t center   = s_axis.joystick_raw_center;
    int32_t full_pos = s_axis.joystick_raw_full_pos;
    int32_t full_neg = s_axis.joystick_raw_full_neg;
    int32_t deadband = (int32_t)s_axis.joystick_raw_deadband;
    float   v;

    int32_t delta = raw - center;
    if (delta >  deadband) {
        int32_t span = (full_pos - center) - deadband;
        v = (span > 0) ? ((float)(delta - deadband) / (float)span) : 0.0f;
    } else if (delta < -deadband) {
        int32_t span = (center - full_neg) - deadband;
        v = (span > 0) ? -((float)(-delta - deadband) / (float)span) : 0.0f;
    } else {
        v = 0.0f;
    }

    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    s_axis.joystick_value = v;
}

/* Live velocity demand to stream in the cyclic command, based on the
 * current op-mode. JOYSTICK and PROFILE_VELOCITY both produce a velocity;
 * the cyclic field is the same in both. PROFILE_POSITION and HOLD/OFF
 * don't drive a streaming velocity (motor uses its trajectory engine for
 * PROFILE_POSITION, HALT for HOLD, disabled for OFF). */
static float current_velocity_demand_rad_s(void)
{
    switch (s_axis.op_mode_commanded) {
    case AXIS_OP_MODE_JOYSTICK:
        return clamp_abs(s_axis.joystick_value * s_axis.joystick_max_velocity,
                         s_axis.velocity_limit_rad_s);
    case AXIS_OP_MODE_PROFILE_VELOCITY:
        return clamp_abs(s_axis.target_velocity_rad_s, s_axis.velocity_limit_rad_s);
    case AXIS_OP_MODE_PROFILE_POSITION:
    case AXIS_OP_MODE_HOLD:
    case AXIS_OP_MODE_OFF:
    default:
        return 0.0f;
    }
}

/*----------------------------------------------------------------------------
 * Persistence
 *
 * Subset of axis_t that survives reboot. Anything PERSIST-flagged in the
 * CMC OD lives here. The blob is what app/persist writes/reads to/from
 * the CMC's internal flash region 0.
 *
 * Bump AXIS_PERSIST_VERSION on any layout change so old flash images are
 * cleanly rejected by persist_load (caller falls back to coded defaults).
 *---------------------------------------------------------------------------*/

/* v3 (2026-06-25): removed payload_weight_kg — the operator-tunable load
 * concept moved to motor-owned 0x2300:5 vel_load_factor (REQ-0014). Boards
 * with a v2 blob in flash get rejected by persist_load (version mismatch)
 * and fall through to coded defaults — operator needs to re-Save to migrate.
 *
 * v4 (2026-06-26): appended 3 bytes of LED indicator colour (R/G/B) — see
 * app/led_indicator. The blob remains operator-tunables only; we co-locate
 * here rather than spinning up a 4th persist region for 3 bytes (the
 * existing CONFIG region is the only one with spare layout headroom).
 *
 * v6 (2026-07-09): appended 1 byte home_on_boot (0x3043). Boards with a v5
 * blob get rejected → fall through to defaults (home_on_boot = 0, no
 * behaviour change). Operator re-Saves to migrate. */
#define AXIS_PERSIST_VERSION   6u

typedef struct __attribute__((packed)) {
    float    joystick_max_velocity;
    int32_t  joystick_raw_center;
    int32_t  joystick_raw_full_pos;
    int32_t  joystick_raw_full_neg;
    uint32_t joystick_raw_deadband;
    float    velocity_limit_rad_s;
    float    position_limit_lo_rad;
    float    position_limit_hi_rad;
    float    accel_limit_rad_s2;
    uint8_t  led_rgb[3];                 /* v4: led_indicator colour */
    uint8_t  axis_role;                  /* v5: 0x3070 CAMERAD_AXIS_* bitmap */
    uint8_t  home_on_boot;               /* v6: 0x3043 axis_home_on_boot */
} axis_persist_blob_t;

static void apply_persist_blob(const axis_persist_blob_t *b)
{
    /* joystick_max_velocity in the persist blob is legacy — the v4 layout
     * carried it as an operator-settable field. As of 4.8.0 it's derived
     * from velocity_limit × profile_scale (see recompute_joystick_max_velocity),
     * so we IGNORE b->joystick_max_velocity on load and recompute after the
     * limit + profile have been applied. The persist blob layout is kept
     * unchanged so the calibration + motion limits + LED colour survive
     * the upgrade — see the 4.8.0 CHANGELOG entry for the migration note. */
    s_axis.joystick_raw_center   = b->joystick_raw_center;
    s_axis.joystick_raw_full_pos = b->joystick_raw_full_pos;
    s_axis.joystick_raw_full_neg = b->joystick_raw_full_neg;
    s_axis.joystick_raw_deadband = b->joystick_raw_deadband;
    s_axis.velocity_limit_rad_s  = b->velocity_limit_rad_s;
    s_axis.position_limit_lo_rad = b->position_limit_lo_rad;
    s_axis.position_limit_hi_rad = b->position_limit_hi_rad;
    s_axis.accel_limit_rad_s2    = b->accel_limit_rad_s2;
    /* v5: axis_role. Sanitize on load — an invalid saved value (e.g. 0,
     * or something outside the 8 CAMERAD_AXIS_* bits) falls back to PAN
     * so the CMC still processes MOVEMENT frames after a corrupted save. */
    s_axis.axis_role = b->axis_role;
    if (s_axis.axis_role == 0u) s_axis.axis_role = 0x01u;  /* PAN default */
    /* v6: home_on_boot. Sanitize to 0 or 1 in case a partially-corrupted
     * blob loaded through the CRC (unlikely but cheap). */
    s_axis.home_on_boot = (b->home_on_boot != 0u) ? 1u : 0u;
    led_indicator_apply_persist(b->led_rgb);
    recompute_joystick_max_velocity();
}

static void capture_persist_blob(axis_persist_blob_t *b)
{
    /* Write the current derived value into the legacy slot so a v4 reader
     * still gets something meaningful (backwards compat during a rollback).
     * Load path ignores it anyway (see apply_persist_blob above). */
    b->joystick_max_velocity = s_axis.joystick_max_velocity;
    b->joystick_raw_center   = s_axis.joystick_raw_center;
    b->joystick_raw_full_pos = s_axis.joystick_raw_full_pos;
    b->joystick_raw_full_neg = s_axis.joystick_raw_full_neg;
    b->joystick_raw_deadband = s_axis.joystick_raw_deadband;
    b->velocity_limit_rad_s  = s_axis.velocity_limit_rad_s;
    b->position_limit_lo_rad = s_axis.position_limit_lo_rad;
    b->position_limit_hi_rad = s_axis.position_limit_hi_rad;
    b->accel_limit_rad_s2    = s_axis.accel_limit_rad_s2;
    led_indicator_capture_persist(b->led_rgb);
    b->axis_role = s_axis.axis_role;
    b->home_on_boot = s_axis.home_on_boot;
}

bool axis_manager_save_to_flash(void)
{
    axis_persist_blob_t blob;
    capture_persist_blob(&blob);
    bool ok = persist_save(PERSIST_REGION_CONFIG, &blob, sizeof(blob),
                           AXIS_PERSIST_VERSION);
    if (ok) {
        LOG_INFO("axis_manager: config saved to flash (%u B, v%u)",
                 (unsigned)sizeof(blob), (unsigned)AXIS_PERSIST_VERSION);
    } else {
        LOG_ERROR("axis_manager: config save FAILED");
    }
    return ok;
}

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

void axis_manager_init(void)
{
    memset(&s_axis,    0, sizeof(s_axis));
    memset(&s_applied, 0, sizeof(s_applied));
    s_axis.state             = AXIS_STATE_DISABLED;
    s_axis.op_mode_actual    = AXIS_OP_MODE_OFF;
    s_axis.op_mode_commanded = AXIS_OP_MODE_OFF;
    /* Pre-authorise enable at boot. The compose_cyclic_cmd gate at
     * line ~752 only lets the ENABLE bit through when mode != OFF, so
     * the drive still sits disabled here (safe: motor spins freely).
     * The moment mode leaves OFF — CAMERAD SELECT, button press, PC-tool
     * mode write, etc. — the drive activates immediately without needing
     * a separate axis_manager_request_enable(true) call. */
    s_axis.enable_latch      = true;

    s_axis.joy_profile            = AXIS_JOY_PROFILE_NORMAL;  /* boot to full-speed; not persisted */
    s_axis.axis_role              = 0x01u;                    /* CAMERAD_AXIS_PAN — persisted, override on load */
    s_axis.velocity_limit_rad_s   = 10.0f;
    recompute_joystick_max_velocity();   /* derived — needs velocity_limit + profile set first */
    s_axis.position_limit_lo_rad  = -INFINITY;
    s_axis.position_limit_hi_rad  = +INFINITY;
    s_axis.accel_limit_rad_s2     = 100.0f;
    s_axis.load_factor            = 1.0f;     /* no scaling until operator slides */

    /* Joystick cal defaults: matched to CAMERAD MOVEMENT's signed-int8
     * wire range (-127..+127, 0 = centered) since that's the primary
     * input today. Other sources (VISCA U7, PC tool F32 etc.) need
     * different cal — override the four 0x3027-0x302A entries from the
     * PC tool's CMC Setup tab and "Save to flash". The cal is per-axis,
     * not per-input-source: assume one stick source at a time. */
    s_axis.joystick_raw           = 0;
    s_axis.joystick_raw_center    = 0;
    s_axis.joystick_raw_full_pos  = 127;
    s_axis.joystick_raw_full_neg  = -127;
    s_axis.joystick_raw_deadband  = 0;

    /* Applied snapshot starts with sentinel values that force the
     * sequencer to write everything once on the first opportunity. */
    s_applied.mode                        = (int8_t)0x7F;
    s_applied.target_position_scaled      = INT32_MIN;
    s_applied.target_position_time_ms     = UINT32_MAX;
    s_applied.target_torque_scaled        = INT32_MIN;
    s_applied.profile_velocity_scaled     = UINT32_MAX;
    s_applied.profile_acceleration_scaled = UINT32_MAX;
    s_applied.profile_deceleration_scaled = UINT32_MAX;

    s_first_sync           = true;
    s_seq_step             = SEQ_IDLE;
    s_seq_handle           = CIA402_OD_HANDLE_INVALID;
    s_new_setpoint_remaining = 0;
    s_halt_remaining         = 0;
    s_fault_reset_remaining  = 0;

    /* Attempt to load persisted config from flash. On success this
     * overrides the coded defaults above; on failure (uninitialised
     * region, CRC mismatch, version bump) the defaults stand and we
     * carry on as a fresh-from-factory boot. */
    axis_persist_blob_t blob;
    size_t loaded = 0;
    if (persist_load(PERSIST_REGION_CONFIG, &blob, sizeof(blob),
                     AXIS_PERSIST_VERSION, &loaded)
        && loaded == sizeof(blob)) {
        apply_persist_blob(&blob);
        LOG_INFO("axis_manager: ready (v3, persisted config loaded). state=DISABLED");
    } else {
        LOG_INFO("axis_manager: ready (v3, factory defaults). state=DISABLED");
    }
}

/*----------------------------------------------------------------------------
 * Desired setup snapshot — current value, clamped + scaled
 *---------------------------------------------------------------------------*/

static void compute_desired(setup_snapshot_t *out)
{
    out->mode                        = axis_mode_to_cia402(s_axis.op_mode_commanded);

    float p = clamp_range(s_axis.target_position_rad,
                          s_axis.position_limit_lo_rad,
                          s_axis.position_limit_hi_rad);
    out->target_position_scaled      = to_scaled_i32(p, MC_IF_POS_SCALE);

    /* target_time SI -> ms; saturate to UINT32_MAX. */
    if (s_axis.target_time_s <= 0.0f) {
        out->target_position_time_ms = 0u;
    } else {
        float ms = s_axis.target_time_s * 1000.0f;
        if (ms > 4.0e9f) out->target_position_time_ms = UINT32_MAX;
        else             out->target_position_time_ms = (uint32_t)(ms + 0.5f);
    }

    /* REQ-0012: torque mode commanded in amps via 0x302B axis_target_current.
     * Convert to the raw int32 motor expects at 0x6071 (MC_IF_CUR_SCALE
     * = 1e-3 A/LSB). Effective only when op_mode_actual on the motor is
     * TORQUE; in other modes the motor MCU ignores 0x6071. We still send
     * the value so a mode switch into TORQUE picks up the most recent
     * setpoint without an extra round-trip. */
    out->target_torque_scaled        = to_scaled_i32(s_axis.target_current_a, MC_IF_CUR_SCALE);

    out->profile_velocity_scaled     = to_scaled_u32(s_axis.velocity_limit_rad_s, MC_IF_VEL_SCALE);
    out->profile_acceleration_scaled = to_scaled_u32(s_axis.accel_limit_rad_s2,   MC_IF_ACC_SCALE);
    out->profile_deceleration_scaled = to_scaled_u32(s_axis.accel_limit_rad_s2,   MC_IF_ACC_SCALE);
}

/*----------------------------------------------------------------------------
 * SDO setup-sequencer
 *
 * If any of the desired setup values differ from what the motor MCU has,
 * kick off ONE SDO write per tick. Walks the parameters in a fixed order;
 * once all match, sequencer is idle.
 *
 * Returns true if a write is in flight (or just completed); compose_cyclic_cmd
 * uses this to decide whether to fire NEW_SETPOINT (only when the motor MCU's
 * stored setup matches what we want).
 *---------------------------------------------------------------------------*/

/* True if a setup SDO is currently in flight (s_seq_step != IDLE), OR
 * if there are unwritten desired-vs-applied diffs the sequencer hasn't
 * yet managed to queue (e.g. cia402's OD pipeline was busy at the time
 * setup_sequencer_tick tried — start_sdo_write returned IDLE and the
 * step never advanced). Without this, NEW_SETPOINT can fire before
 * TARGET_POSITION reaches the motor, causing the move to use the last-
 * applied target (often 0). Bug discovered 2026-06-25.
 *
 * CRITICAL: only check fields the sequencer actually writes. Listing a
 * field here that's NOT in setup_sequencer_tick's diff chain wedges
 * NEW_SETPOINT forever (the diff never clears). target_torque_scaled
 * is computed by compute_desired but the sequencer has no branch to
 * write it — keep it OUT of this check. */
static bool setup_sequencer_busy(void)
{
    if (s_seq_step != SEQ_IDLE) return true;

    setup_snapshot_t d;
    compute_desired(&d);
    if (d.mode                        != s_applied.mode)                        return true;
    if (d.target_position_scaled      != s_applied.target_position_scaled)      return true;
    if (d.target_position_time_ms     != s_applied.target_position_time_ms)     return true;
    if (d.target_torque_scaled        != s_applied.target_torque_scaled)        return true;
    if (d.profile_velocity_scaled     != s_applied.profile_velocity_scaled)     return true;
    if (d.profile_acceleration_scaled != s_applied.profile_acceleration_scaled) return true;
    if (d.profile_deceleration_scaled != s_applied.profile_deceleration_scaled) return true;
    return false;
}

static void apply_completed_step(MC_IfOdResult_t res, const setup_snapshot_t *desired)
{
    if (res != MC_IF_OD_OK) {
        /* Failure surfaces here AND in the per-failure log line — the
         * caller leaves `applied` unchanged so we retry on the next
         * cycle. The first 8 are logged at WARN; beyond that we
         * suppress to avoid log flood if something stays broken. */
        static uint32_t s_failures = 0;
        if (s_failures < 8u) {
            LOG_WARN("axis_manager: SDO sdo done step=%s FAIL result=0x%02X (will retry)",
                     seq_step_name((int)s_seq_step), (unsigned)res);
            s_failures++;
        }
        return;
    }

    /* Success — log per-step transition so the operator can see the
     * cmd-acked chain. MODE is the headline one for set_op_mode debugging. */
    switch (s_seq_step) {
    case SEQ_MODE:
        LOG_INFO("axis_manager: SDO sdo done MODE 0x6060 = %d (motor accepted)",
                 (int)desired->mode);
        s_applied.mode = desired->mode;
        break;
    case SEQ_TARGET_POSITION:
        LOG_INFO("axis_manager: SDO sdo done TARGET_POSITION 0x607A = %ld (scaled)",
                 (long)desired->target_position_scaled);
        s_applied.target_position_scaled = desired->target_position_scaled;
        break;
    case SEQ_TARGET_POSITION_TIME:
        LOG_INFO("axis_manager: SDO sdo done TARGET_POSITION_TIME 0x607B = %lu ms",
                 (unsigned long)desired->target_position_time_ms);
        s_applied.target_position_time_ms = desired->target_position_time_ms;
        break;
    case SEQ_TARGET_TORQUE:
        LOG_INFO("axis_manager: SDO sdo done TARGET_TORQUE 0x6071 = %ld (scaled)",
                 (long)desired->target_torque_scaled);
        s_applied.target_torque_scaled = desired->target_torque_scaled;
        break;
    case SEQ_PROFILE_VELOCITY:
        LOG_INFO("axis_manager: SDO sdo done PROFILE_VELOCITY 0x6081 = %lu (scaled)",
                 (unsigned long)desired->profile_velocity_scaled);
        s_applied.profile_velocity_scaled = desired->profile_velocity_scaled;
        break;
    case SEQ_PROFILE_ACCELERATION:
        LOG_INFO("axis_manager: SDO sdo done PROFILE_ACCELERATION 0x6083 = %lu (scaled)",
                 (unsigned long)desired->profile_acceleration_scaled);
        s_applied.profile_acceleration_scaled = desired->profile_acceleration_scaled;
        break;
    case SEQ_PROFILE_DECELERATION:
        LOG_INFO("axis_manager: SDO sdo done PROFILE_DECELERATION 0x6084 = %lu (scaled)",
                 (unsigned long)desired->profile_deceleration_scaled);
        s_applied.profile_deceleration_scaled = desired->profile_deceleration_scaled;
        break;
    default: break;
    }
}

/* Start an SDO write. Returns the new sequencer step on success, SEQ_IDLE
 * on cia402 queue-full (caller retries next tick). */
static seq_step_t start_sdo_write(seq_step_t step, uint16_t idx, uint8_t sub,
                                  MC_IfOdType_t type, const void *data, uint8_t len)
{
    cia402_od_handle_t h = cia402_od_write_begin(idx, sub, type, data, len);
    if (h == CIA402_OD_HANDLE_INVALID) {
        /* SDO queue full — happens when cia402's OD pipeline already has
         * an OD_READ/WRITE in flight (probably from the PC tool). We'll
         * retry on the next tick. Log throttled so a stuck pipeline
         * doesn't flood the log. */
        static uint32_t s_busy_log = 0;
        if (s_busy_log < 8u) {
            LOG_INFO("axis_manager: SDO sdo start step=%s 0x%04X DEFERRED (cia402 queue full)",
                     seq_step_name((int)step), (unsigned)idx);
            s_busy_log++;
        }
        return SEQ_IDLE;
    }
    LOG_INFO("axis_manager: SDO sdo start step=%s 0x%04X (handle issued)",
             seq_step_name((int)step), (unsigned)idx);
    s_seq_handle = h;
    return step;
}

static void setup_sequencer_tick(void)
{
    /* Stage 1: if a write is in flight, poll for completion. */
    if (s_seq_step != SEQ_IDLE) {
        MC_IfOdResult_t res    = MC_IF_OD_OK;
        uint8_t         buf[8] = {0};
        uint8_t         vlen   = sizeof(buf);
        if (!cia402_od_poll(s_seq_handle, &res, buf, &vlen)) return;
        setup_snapshot_t desired;
        compute_desired(&desired);
        apply_completed_step(res, &desired);
        s_seq_step   = SEQ_IDLE;
        s_seq_handle = CIA402_OD_HANDLE_INVALID;
        if (res == MC_IF_OD_OK) s_first_sync = false;
    }

    if (s_seq_step != SEQ_IDLE) return;   /* still busy (shouldn't reach here) */

    /* Stage 2: find first parameter where desired != applied; start its SDO. */
    setup_snapshot_t desired;
    compute_desired(&desired);

    if (desired.mode != s_applied.mode) {
        s_seq_step = start_sdo_write(SEQ_MODE, OD_MODES_OF_OPERATION, 0,
                                     MC_IF_T_I8, &desired.mode, 1);
    } else if (desired.target_position_scaled != s_applied.target_position_scaled) {
        s_seq_step = start_sdo_write(SEQ_TARGET_POSITION, OD_TARGET_POSITION, 0,
                                     MC_IF_T_I32, &desired.target_position_scaled, 4);
    } else if (desired.target_position_time_ms != s_applied.target_position_time_ms) {
        s_seq_step = start_sdo_write(SEQ_TARGET_POSITION_TIME, OD_TARGET_POSITION_TIME, 0,
                                     MC_IF_T_U32, &desired.target_position_time_ms, 4);
    } else if (desired.target_torque_scaled != s_applied.target_torque_scaled) {
        /* REQ-0012 — write motor 0x6071 target_torque (in raw mA equivalents
         * after MC_IF_CUR_SCALE). Motor consumes it only when 0x6060 = 4. */
        s_seq_step = start_sdo_write(SEQ_TARGET_TORQUE, OD_TARGET_TORQUE, 0,
                                     MC_IF_T_I32, &desired.target_torque_scaled, 4);
    } else if (desired.profile_velocity_scaled != s_applied.profile_velocity_scaled) {
        s_seq_step = start_sdo_write(SEQ_PROFILE_VELOCITY, OD_PROFILE_VELOCITY, 0,
                                     MC_IF_T_U32, &desired.profile_velocity_scaled, 4);
    } else if (desired.profile_acceleration_scaled != s_applied.profile_acceleration_scaled) {
        s_seq_step = start_sdo_write(SEQ_PROFILE_ACCELERATION, OD_PROFILE_ACCELERATION, 0,
                                     MC_IF_T_U32, &desired.profile_acceleration_scaled, 4);
    } else if (desired.profile_deceleration_scaled != s_applied.profile_deceleration_scaled) {
        s_seq_step = start_sdo_write(SEQ_PROFILE_DECELERATION, OD_PROFILE_DECELERATION, 0,
                                     MC_IF_T_U32, &desired.profile_deceleration_scaled, 4);
    }
    /* velocity_setpoint streams in the cyclic frame, not via SDO — no
     * sequencer step. (SEQ_TARGET_TORQUE was wired in by REQ-0012.) */
}

/*----------------------------------------------------------------------------
 * Cyclic command composition
 *---------------------------------------------------------------------------*/

static axis_state_t derive_state_from_status(const MC_IfCyclicStatusHeader_t *hdr)
{
    if (hdr->statusword & MC_IF_SW_FAULT)   return AXIS_STATE_FAULT;
    if (hdr->statusword & MC_IF_SW_ENABLED) return AXIS_STATE_RUNNING;
    if (hdr->statusword & MC_IF_SW_READY)   return AXIS_STATE_READY;
    return AXIS_STATE_DISABLED;
}

static void compose_cyclic_cmd(MC_IfCyclicCommand_t *out)
{
    memset(out, 0, sizeof(*out));

    uint16_t cw = 0;

    /* Clear-fault: rising edge for one cycle. */
    /* Latch the request into the multi-cycle pulse counter; hold the bit
     * for FAULT_RESET_PULSE_CYCLES so it's reliably seen by the motor
     * even if cia402 is interleaving OD traffic for some of those cycles. */
    if (s_axis.clear_fault_pulse) {
        s_fault_reset_remaining  = FAULT_RESET_PULSE_CYCLES;
        s_axis.clear_fault_pulse = false;
    }
    if (s_fault_reset_remaining > 0) {
        cw |= MC_IF_CW_FAULT_RESET;
        s_fault_reset_remaining--;
    }

    /* Quick-stop: clear QUICK_STOP bit and force-disable. */
    if (s_axis.quick_stop_pulse) {
        s_axis.enable_latch      = false;
        s_axis.quick_stop_pulse  = false;
        s_new_setpoint_remaining = 0;   /* abandon any pending trigger */
        s_halt_remaining         = 0;   /* abandon any pending HALT pulse */
        s_fault_reset_remaining  = 0;   /* abandon any pending fault-reset pulse */
        out->controlword         = cw;     /* QUICK_STOP bit cleared = quick-stop active */
        return;
    }

    /* Default: QUICK_STOP set (1 = normal). */
    cw |= MC_IF_CW_QUICK_STOP;

    if (s_axis.state == AXIS_STATE_FAULT) {
        out->controlword = cw;
        return;
    }

    /* Enable / HALT / HOLD treatment. */
    if (s_axis.enable_latch && s_axis.op_mode_commanded != AXIS_OP_MODE_OFF) {
        cw |= MC_IF_CW_ENABLE;
        if (s_axis.op_mode_commanded == AXIS_OP_MODE_HOLD) {
            cw |= MC_IF_CW_HALT;
        }
    }

    /* Operator-driven HALT (from CAMERAD panel STOP key via cmc_state).
     * Multi-cycle pulse — motor MCU latches its own stop/hold state on
     * the rising edge. Auto-expires; no clear-on-new-move bookkeeping
     * needed since the pulse is finite. */
    if (s_halt_remaining > 0) {
        cw |= MC_IF_CW_HALT;
        s_halt_remaining--;
    }

    /* Stream the live velocity demand. The motor MCU applies it as the
     * active velocity target in any velocity-class mode (and ignores it
     * in PROFILE_POSITION / HOLD / OFF — they don't drive a streaming
     * velocity). Only emit when enabled, so a disabled drive doesn't see
     * a non-zero setpoint racing the ENABLE bit. */
    if (s_axis.enable_latch && s_axis.state != AXIS_STATE_FAULT) {
        float v = current_velocity_demand_rad_s();
        out->velocity_setpoint = to_scaled_i32(v, MC_IF_VEL_SCALE);
    }

    /* NEW_SETPOINT: rising edge fires only when setup is fully applied
     * (sequencer idle and no remaining diffs). Held high for a few
     * cycles to ensure the motor MCU sees the rising edge robustly.
     * Logged once at the rising edge so the trace shows when the
     * trigger is actually sent (vs. stuck waiting for SDO setup). */
    if (s_axis.start_move_pulse) {
        if (!setup_sequencer_busy()) {
            s_new_setpoint_remaining = NEW_SETPOINT_PULSE_CYCLES;
            s_axis.start_move_pulse  = false;
            LOG_INFO("axis_manager: NEW_SETPOINT triggered (setup applied, sending pulse for %u cycles)",
                     (unsigned)NEW_SETPOINT_PULSE_CYCLES);
        }
        /* else: keep the pulse pending; sequencer is still applying setup. */
    }
    if (s_new_setpoint_remaining > 0) {
        cw |= MC_IF_CW_NEW_SETPOINT;
        s_new_setpoint_remaining--;
    }

    out->controlword = cw;
}

/*----------------------------------------------------------------------------
 * Tick
 *---------------------------------------------------------------------------*/

/* Defined further down in the load-factor accessor block. Forward-declared
 * here so axis_manager_tick can drain the pending SDO write each cycle. */
static void poll_load_factor_sdo(void);
static void reset_motor_od_submodules(void);   /* body near end of file — after all statics are declared */
/* Forward decls for the proxy + motor-save sequencer (defined right after
 * load_factor). Both run every tick to keep their state machines alive.
 * proxy_motor_f32_begin is also forward-declared so the limit-setters
 * higher up in the file can call it. */
static void poll_motor_proxy_sdo(void);
static void tick_motor_save(void);
static void tick_proxy_bootsync(void);
static void tick_home_sequencer(void);
static bool proxy_motor_f32_begin(uint32_t slot_idx, float v);

/* Proxy slot indices — keep in sync with the s_proxy_slots[] table further
 * down. Defined here (above the limit-setters that reference them) because
 * the table itself lives near the proxy machinery, far below. */
#define PROXY_SLOT_VEL_ACCEL_UP    (0u)
#define PROXY_SLOT_VEL_ACCEL_DN    (1u)
#define PROXY_SLOT_VEL_ACCEL_JERK  (2u)
#define PROXY_SLOT_VEL_LIMIT       (3u)
#define PROXY_SLOT_ACCEL_LIMIT     (4u)
#define PROXY_SLOT_POS_LIMIT_LO    (5u)
#define PROXY_SLOT_POS_LIMIT_HI    (6u)
#define PROXY_SLOT_COUNT           (7u)

/*----------------------------------------------------------------------------
 * On-board UP/DOWN buttons — temporary JOYSTICK-mode jog
 *
 * Reads debounced button state from bsp/buttons (GPIO + debounce live there,
 * NOT here — keeps app/ off the HAL). Pressing a button momentarily forces
 * the axis into JOYSTICK mode and slams the joystick value to its end stop;
 * releasing both buttons restores the previous mode AND the physical
 * stick's reading. This lets the operator jog the axis from the on-board
 * buttons regardless of whatever mode is currently active (web UI / PC
 * tool may be in PROFILE_VELOCITY, PROFILE_POSITION, HOLD, etc.).
 *
 * Mapping: UP held -> joystick_value = +1.0 (max), DOWN held -> -1.0 (min),
 * BOTH held -> 0.0 (operator's hand presumed on both — refuse to pick a
 * direction). Downstream the JOYSTICK-mode mapping scales by
 * joystick_max_velocity / velocity_limit just like a real stick deflection.
 *
 * Ownership model: while neither button is held, op_mode + joystick_value
 * are owned by whoever (PC tool, web, physical stick) was driving them.
 * On the first button press we snapshot op_mode_commanded, switch it to
 * JOYSTICK, and start writing joystick_value each tick. On release of BOTH
 * buttons we restore the snapshotted op_mode_commanded and recompute
 * joystick_value from the current ADC reading once — handing control back
 * to the physical stick with no zero-glitch.
 *
 * Edge cases:
 * - If the snapshotted mode WAS JOYSTICK, the restore is a no-op
 *   (op_mode_commanded comparison short-circuits in axis_manager_set_op_mode).
 * - If someone else (PC tool, web) writes op_mode_commanded WHILE a button
 *   is held, the next release will restore to the snapshot — clobbering
 *   that change. This is intentional: the buttons own the mode while held.
 *
 * Previously (CHANGELOG <= 4.6.x) these buttons drove target_current_a as
 * a TORQUE-mode jog. The 0x302C axis_button_current OD entry is now
 * effectively dead — the field still exists for backwards compatibility
 * but is no longer read by this handler. Don't repurpose the entry; the
 * PC tool's CMC Setup page row for it should be removed in a follow-up.
 *---------------------------------------------------------------------------*/
/* Post-release hold exit is gated on the motor's own MC_IF_MOVE_MOVING
 * bit in movement_status (cyclic header, REQ-0013). The motor MCU clears
 * MOVING when drive is enabled AND |vel demand| AND |vel meas| are both
 * under ~0.01 rad/s — its own observer is the authoritative judge of
 * "stopped", and it's far more accurate than anything CMC could derive
 * from position deltas on this side. No time cap — we wait as long as
 * the motor takes to decelerate via the JOYSTICK-mode vel_accel_dn ramp.
 *
 * (Earlier attempt used s_axis.velocity_actual_rad_s — that field is a
 * documented TODO at the cyclic-status update site and stays at 0,
 * which is why the hold previously exited on the first tick.) */

static bool           s_buttons_own_mode;        /* true while CMC is forcing JOYSTICK mode from buttons */
static axis_op_mode_t s_buttons_prev_mode;       /* op_mode_commanded snapshot from before button engaged */
static bool           s_buttons_in_release_hold; /* true between BOTH-buttons-released and motor-stopped */

/* Auto-fault-clear: if the motor sits in AXIS_STATE_FAULT for this long the
 * CMC fires a clear_fault pulse itself (defence against transient faults the
 * operator hasn't noticed — overcurrent spike, momentary lost-encoder, etc.).
 * Counter is exposed read-only via OD 0x3014 axis_auto_fault_clears so the
 * operator can audit how often this is happening (non-zero after a quiet
 * session = something's wrong even though the motor looks fine). U16 wraps
 * after 65535 — at 5 s minimum spacing that's ~91 hours of continuous
 * faulting before the wrap, so not worth widening. Since-boot only, not
 * persisted (more useful as "what happened this session" than lifetime). */
#define AUTO_FAULT_CLEAR_DELAY_MS  5000u
static uint32_t s_fault_active_since_ms;     /* 0 = not in fault, else time_ms() at entry */
static uint16_t s_auto_fault_clear_count;

static void poll_joystick_buttons(void)
{
    bool up_held   = bsp_buttons_up_pressed();
    bool down_held = bsp_buttons_down_pressed();
    bool any_held  = up_held || down_held;

    if (any_held) {
        float new_value;
        if (up_held && down_held)  new_value = 0.0f;     /* safety zero */
        else if (up_held)          new_value = +1.0f;    /* max joystick */
        else                       new_value = -1.0f;    /* min joystick */

        /* A re-press during the release-hold (operator changed their mind
         * before velocity settled) cancels the hold and resumes normal
         * button ownership of joystick_value. */
        s_buttons_in_release_hold = false;

        if (!s_buttons_own_mode) {
            /* First press — arbitrate through try_begin_op so a mid-home
             * or mid-shot-recall button press doesn't silently abort. If
             * REJECTED, we swallow the button and don't touch mode/value. */
            axis_begin_result_t r = axis_manager_try_begin_op(AXIS_OPERATION_JOYSTICK);
            if (r == AXIS_BEGIN_REJECTED) {
                /* Log once per rejection edge — poll_joystick_buttons runs
                 * ~1 kHz so a stuck spurious "pressed" pin would spam. Reuse
                 * s_buttons_own_mode as the edge tracker: still false here,
                 * so we log; the log-throttle keeps it manageable via the
                 * "buttons engaged" absence downstream. */
                LOG_INFO("axis_manager: buttons rejected (active_op = %s)",
                         op_name(axis_manager_get_active_op()));
                return;
            }
            /* Snapshot the current commanded mode and force JOYSTICK so
             * downstream uses joystick_value as the demand. Route through
             * axis_manager_set_op_mode (NOT a direct field write) so the
             * validation, edge-detect logging, and "SDO pending" notice
             * match every other mode change in the system.
             *
             * Also auto-enable the axis. The motor is left disabled at
             * boot (and after panel-timeout deselect) — buttons need to
             * be usable as a standalone control surface even when no
             * panel has yet sent SELECT, so press = enable. Matches the
             * cmc_state SELECT auto-enable pattern. "Fail-on" semantics
             * apply (we never disable on release; only an explicit
             * 0x3010 write or panel timeout drops enable). */
            s_buttons_prev_mode = s_axis.op_mode_commanded;
            s_buttons_own_mode  = true;
            LOG_INFO("axis_manager: buttons engaged %s (joystick_value = %+.2f, will restore mode %u on release)",
                     (up_held && down_held) ? "BOTH" : (up_held ? "UP" : "DOWN"),
                     (double)new_value, (unsigned)s_buttons_prev_mode);
            (void)axis_manager_request_enable(true);
            (void)axis_manager_set_op_mode((uint8_t)AXIS_OP_MODE_JOYSTICK);
        }
        s_axis.joystick_value = new_value;
    } else if (s_buttons_own_mode) {
        /* Both released — enter the release-hold instead of restoring the
         * mode immediately. Switching out of JOYSTICK while the motor is
         * still spinning would skip the joystick-mode accel ramp (each
         * other mode has its own velocity-handling semantics: HOLD pulls
         * hard to current position, OFF cuts torque, PROFILE_* uses its
         * own trajectory engine) — that's the audible "sudden stop" the
         * operator was seeing. By forcing joystick_value = 0 and holding
         * JOYSTICK mode, the motor decelerates through the configured
         * vel_accel_dn ramp. We only restore the snapshotted mode once
         * |velocity_actual| has settled near zero. */
        if (!s_buttons_in_release_hold) {
            s_buttons_in_release_hold = true;
            LOG_INFO("axis_manager: buttons released -> holding JOYSTICK + joystick_value=0, "
                     "will restore mode %u once motor reports MOVING cleared",
                     (unsigned)s_buttons_prev_mode);
        }
        /* Force zero every tick so a non-centred physical stick can't
         * keep driving the motor through the hold (operator's hand is
         * off the stick — buttons are the active control surface). */
        s_axis.joystick_value = 0.0f;

        if ((s_axis.movement_status & MC_IF_MOVE_MOVING) == 0u) {
            LOG_INFO("axis_manager: motor stopped (movement_status=0x%04X) -> restoring mode %u, "
                     "physical stick takes over",
                     (unsigned)s_axis.movement_status,
                     (unsigned)s_buttons_prev_mode);
            (void)axis_manager_set_op_mode((uint8_t)s_buttons_prev_mode);
            recompute_joystick_value_from_raw();
            s_buttons_own_mode        = false;
            s_buttons_in_release_hold = false;
        }
    }
    /* else: no button activity and no prior ownership — everything passes
     * through normally (physical stick drives joystick_value, PC tool /
     * web drive op_mode_commanded). */
}

void axis_manager_tick(void)
{
    /* 1. Pull latest status from cia402 (peek — non-consuming). */
    MC_IfCyclicStatusHeader_t hdr;
    uint8_t blob[MC_IF_TLM_BLOB_MAX];
    uint8_t blob_len = 0;
    if (cia402_peek_cyclic_status(&hdr, blob, &blob_len)) {
        /* Snapshot everything BEFORE update so all the edge-detect logs
         * have access to old values. */
        uint16_t       prev_error    = s_axis.error_code;
        axis_state_t   prev_state    = s_axis.state;
        axis_op_mode_t prev_op_mode  = s_axis.op_mode_actual;

        s_axis.error_code     = hdr.error_code;
        s_axis.error_register = (uint8_t)(hdr.error_code >> 8);  /* placeholder */

        if (prev_error != s_axis.error_code) {
            LOG_INFO("axis_manager: error_code 0x%04X -> 0x%04X (from motor)",
                     (unsigned)prev_error, (unsigned)s_axis.error_code);
        }

        s_axis.state          = derive_state_from_status(&hdr);
        if (s_axis.state == AXIS_STATE_RUNNING || s_axis.state == AXIS_STATE_READY) {
            s_axis.op_mode_actual = s_axis.op_mode_commanded;
        } else {
            s_axis.op_mode_actual = AXIS_OP_MODE_OFF;
        }

        if (prev_state != s_axis.state) {
            LOG_INFO("axis_manager: state %s -> %s (statusword=0x%04X err=0x%04X)",
                     axis_state_name(prev_state), axis_state_name(s_axis.state),
                     (unsigned)hdr.statusword, (unsigned)hdr.error_code);
        }

        /* Auto-fault-clear: track how long the fault has been active and
         * fire a clear pulse once the dwell exceeds AUTO_FAULT_CLEAR_DELAY_MS.
         * Re-arms the timer after each attempt so it retries every 5 s while
         * the fault persists (a stuck fault → counter increments steadily,
         * giving the operator a visible "this isn't clearing" signal).
         * Manual clears via 0x3012 work normally — once state != FAULT the
         * timer resets, so the operator's clear doesn't double-count here. */
        if (s_axis.state == AXIS_STATE_FAULT) {
            if (s_fault_active_since_ms == 0u) {
                /* First tick we observe the fault — start the dwell timer. */
                s_fault_active_since_ms = time_ms();
            } else if (time_elapsed_ms(s_fault_active_since_ms) >= AUTO_FAULT_CLEAR_DELAY_MS) {
                /* Dwell exceeded — pulse FAULT_RESET, bump the count, and
                 * re-arm for another 5 s in case the fault is sticky. */
                s_axis.clear_fault_pulse = true;
                s_auto_fault_clear_count++;
                s_fault_active_since_ms  = time_ms();
                LOG_WARN("axis_manager: auto-clearing fault (err=0x%04X, count now %u)",
                         (unsigned)s_axis.error_code, (unsigned)s_auto_fault_clear_count);
            }
        } else if (s_fault_active_since_ms != 0u) {
            /* Fault cleared (by us, the operator, or naturally) — disarm. */
            s_fault_active_since_ms = 0u;
        }
        if (prev_op_mode != s_axis.op_mode_actual) {
            LOG_INFO("axis_manager: op_mode_actual %s -> %s (mode_display=%d)",
                     axis_mode_name(prev_op_mode), axis_mode_name(s_axis.op_mode_actual),
                     (int)hdr.mode_display);
        }

        /* v4 (REQ-0013/ADR-033): position + movement status now live in
         * the fixed header so we don't have to parse the host-configurable
         * telemetry blob to get the most fundamental motor-feedback signal.
         * Scale from MC_IF_POS_SCALE (1e-5 rad/LSB) into SI rad. */
        s_axis.position_actual_rad = (float)hdr.position_actual_scaled * MC_IF_POS_SCALE;

        /* Log on edge so the operator can see when motor reports moving /
         * arriving at target. Don't log on every cycle. */
        uint16_t prev_move = s_axis.movement_status;
        s_axis.movement_status = hdr.movement_status;
        if (prev_move != s_axis.movement_status) {
            LOG_INFO("axis_manager: movement_status 0x%04X -> 0x%04X (moving=%d on_target=%d)",
                     (unsigned)prev_move, (unsigned)s_axis.movement_status,
                     (int)((s_axis.movement_status & MC_IF_MOVE_MOVING)    != 0),
                     (int)((s_axis.movement_status & MC_IF_MOVE_ON_TARGET) != 0));
        }
        /* velocity_actual_rad_s still TODO — would need its own header field
         * or a parsed telemetry-blob entry. Not blocking any current feature. */
    }

    /* 2. Drain any pending ad-hoc SDO writes so they release cia402's OD
     * slot — has to happen BEFORE the setup-sequencer tries to issue its
     * own writes, otherwise the slot stays held by us.
     * load_factor has its own dedicated handle/pending tracking (legacy);
     * the new vel_accel_* + motor-save sequencer share a single proxy
     * handle (only one of the three can be in flight at a time, which is
     * fine because they're operator-driven and low-rate). */
    /* Pause all motor-OD sub-modules while the motor MCU is in its
     * bootloader (dual_bootloader_design.md §6). The bootloader doesn't
     * implement fault_flags / vel_load_factor / cpr / motor_save / etc.,
     * so firing OD requests to those entries from here would leave the
     * sub-modules holding stale handles (motor bootloader replies with
     * NO_OBJECT or similar; the sub-modules' state machines don't expect
     * that mid-operation) and would deadlock the cia402 OD slot — which
     * blocks the PC tool's bootloader OD writes too. On the rising edge
     * of "motor entered bootloader" we abort any in-flight OD to release
     * whatever sub-module handle was held. */
    static bool s_prev_motor_in_bl = false;
    bool motor_in_bl = cia402_motor_in_bootloader();
    if (motor_in_bl && !s_prev_motor_in_bl) {
        LOG_INFO("axis_manager: motor entered bootloader — pausing motor-OD sub-modules + aborting in-flight OD");
        cia402_od_abort();
        reset_motor_od_submodules();
    } else if (!motor_in_bl && s_prev_motor_in_bl) {
        LOG_INFO("axis_manager: motor left bootloader — resuming motor-OD sub-modules");
    }
    s_prev_motor_in_bl = motor_in_bl;

    if (!motor_in_bl) {
        poll_load_factor_sdo();
        tick_proxy_bootsync();   /* reads first (no-op once complete) */
        poll_motor_proxy_sdo();  /* writes second (round-robin across dirty slots) */
        tick_motor_save();
        tick_home_sequencer();   /* home-to-endstop procedure + is_homed cache */
    }
    tick_active_op();        /* operation-level completion detection (HOMING/SHOT_RECALL/JOYSTICK -> NONE) */

    /* 3. Sample the on-board UP/DOWN buttons. In TORQUE mode this drives
     * target_current_a; the next step (sequencer) will pick up the new
     * value as a desired-vs-applied diff and queue the 0x6071 SDO write. */
    poll_joystick_buttons();

    /* 4. Drive the SDO setup-sequencer (one outstanding write at a time). */
    setup_sequencer_tick();

    /* 5. Compose and push the cyclic command for the next outbound frame. */
    MC_IfCyclicCommand_t cmd;
    compose_cyclic_cmd(&cmd);
    cia402_set_cyclic_cmd(&cmd);
}

/*----------------------------------------------------------------------------
 * State getters
 *---------------------------------------------------------------------------*/

axis_state_t   axis_manager_get_state           (void) { return s_axis.state;                 }
axis_op_mode_t axis_manager_get_op_mode_actual  (void) { return s_axis.op_mode_actual;        }
float          axis_manager_get_position_actual (void) { return s_axis.position_actual_rad;   }
float          axis_manager_get_velocity_actual (void) { return s_axis.velocity_actual_rad_s; }

uint16_t       axis_manager_get_movement_status (void) { return s_axis.movement_status; }
bool           axis_manager_is_moving           (void) { return (s_axis.movement_status & MC_IF_MOVE_MOVING)    != 0u; }
bool           axis_manager_is_on_target        (void) { return (s_axis.movement_status & MC_IF_MOVE_ON_TARGET) != 0u; }
uint16_t       axis_manager_get_error_code      (void) { return s_axis.error_code;            }
uint8_t        axis_manager_get_error_register  (void) { return s_axis.error_register;        }
uint16_t       axis_manager_get_auto_fault_clears(void){ return s_auto_fault_clear_count;     }

/*----------------------------------------------------------------------------
 * Command latches
 *---------------------------------------------------------------------------*/

bool axis_manager_request_enable(bool enable)
{
    s_axis.enable_latch = enable;
    LOG_INFO("axis_manager: enable=%d", (int)enable);
    return true;
}

bool axis_manager_request_quick_stop(void)
{
    s_axis.quick_stop_pulse = true;
    LOG_INFO("axis_manager: QUICK_STOP requested (hard decel + disable)");
    return true;
}

/* Controlled-stop + hold (one-shot). Used by CAMERAD panel STOP keys via
 * cmc_state_stop_movement. Pulses the HALT bit in the cyclic controlword
 * for HALT_PULSE_CYCLES cycles — the motor MCU latches its own stop/hold
 * state from the rising edge, so no sustained assertion needed on our
 * side. Next CUT/FADE just works (motor handles the un-halt on its
 * own NEW_SETPOINT processing). For a hard quick-stop + disable, use
 * axis_manager_request_quick_stop instead. */
bool axis_manager_request_halt(void)
{
    s_halt_remaining = HALT_PULSE_CYCLES;
    LOG_INFO("axis_manager: HALT pulse (one-shot, %u cycles — motor MCU latches stop)",
             (unsigned)HALT_PULSE_CYCLES);
    return true;
}

bool axis_manager_request_clear_fault(void)
{
    s_axis.clear_fault_pulse = true;
    return true;
}

bool axis_manager_request_start_move(void)
{
    s_axis.start_move_pulse = true;
    /* No need to "clear HALT" here — HALT is a finite pulse now and the
     * motor MCU is responsible for unlatching its own stopped state on
     * the next NEW_SETPOINT trigger. */
    return true;
}

/*----------------------------------------------------------------------------
 * Mode + targets
 *---------------------------------------------------------------------------*/

axis_op_mode_t axis_manager_get_op_mode(void) { return s_axis.op_mode_commanded; }

bool axis_manager_set_op_mode(uint8_t v)
{
    switch (v) {
    case AXIS_OP_MODE_OFF:
    case AXIS_OP_MODE_JOYSTICK:
    case AXIS_OP_MODE_PROFILE_VELOCITY:
    case AXIS_OP_MODE_PROFILE_POSITION:
    case AXIS_OP_MODE_HOLD:
    case AXIS_OP_MODE_TORQUE: {
        axis_op_mode_t old = s_axis.op_mode_commanded;
        s_axis.op_mode_commanded = (axis_op_mode_t)v;
        if (old != s_axis.op_mode_commanded) {
            /* The commanded mode is now `v`; an SDO write to motor MCU's
             * 0x6060 will follow in setup_sequencer_tick. Watch for
             * 'sdo start' and 'sdo done' lines + 'op_mode_actual ->' line
             * to see the full chain — if any of those is missing, that
             * tells you where the change is getting stuck. */
            LOG_INFO("axis_manager: set_op_mode %s -> %s (commanded; SDO pending)",
                     axis_mode_name(old), axis_mode_name((axis_op_mode_t)v));
        }
        return true;
    }
    default:
        LOG_WARN("axis_manager: set_op_mode REJECTED unknown value %u", (unsigned)v);
        return false;
    }
}

float axis_manager_get_joystick_value(void) { return s_axis.joystick_value; }
bool  axis_manager_set_joystick_value(float v)
{
    if (v < -1.0f || v > 1.0f) return false;
    s_axis.joystick_value = v;
    return true;
}

float axis_manager_get_joystick_max_velocity(void) { return s_axis.joystick_max_velocity; }
/* Removed in 4.8.0 — joystick_max_velocity is now derived from
 * (velocity_limit × joy_profile_scale). 0x3022 is RO in the OD; writes
 * return ACCESS-denied. See axis_manager_set_joy_profile below. */

uint8_t axis_manager_get_joy_profile(void) { return s_axis.joy_profile; }
bool    axis_manager_set_joy_profile(uint8_t p)
{
    if (p != AXIS_JOY_PROFILE_NORMAL &&
        p != AXIS_JOY_PROFILE_MEDIUM &&
        p != AXIS_JOY_PROFILE_FINE) {
        return false;
    }
    if (s_axis.joy_profile != p) {
        LOG_INFO("axis_manager: joy_profile %u -> %u (scale %.2f) — joystick_max_velocity = velocity_limit * scale",
                 (unsigned)s_axis.joy_profile, (unsigned)p, (double)joy_profile_scale(p));
    }
    s_axis.joy_profile = p;
    recompute_joystick_max_velocity();
    return true;
}

uint8_t axis_manager_get_home_on_boot(void) { return s_axis.home_on_boot; }
bool    axis_manager_set_home_on_boot(uint8_t v)
{
    uint8_t nv = (v != 0u) ? 1u : 0u;
    if (s_axis.home_on_boot != nv) {
        LOG_INFO("axis: home_on_boot %u -> %u", (unsigned)s_axis.home_on_boot, (unsigned)nv);
    }
    s_axis.home_on_boot = nv;
    return true;
}

uint8_t axis_manager_get_axis_role(void) { return s_axis.axis_role; }
bool    axis_manager_set_axis_role(uint8_t r)
{
    /* Accept only single-bit values matching CAMERAD_AXIS_* (PAN..FADER).
     * Rejects 0 and multi-bit (a CMC represents one axis, not several). */
    if (r != 0x01u && r != 0x02u && r != 0x04u && r != 0x08u
     && r != 0x10u && r != 0x20u && r != 0x40u && r != 0x80u) {
        return false;
    }
    if (s_axis.axis_role != r) {
        LOG_INFO("axis: role 0x%02X -> 0x%02X", (unsigned)s_axis.axis_role, (unsigned)r);
    }
    s_axis.axis_role = r;
    return true;
}

float axis_manager_get_target_velocity(void) { return s_axis.target_velocity_rad_s; }
bool  axis_manager_set_target_velocity(float v)
{
    if (isnan(v)) return false;
    s_axis.target_velocity_rad_s = v;
    return true;
}

float axis_manager_get_target_position(void) { return s_axis.target_position_rad; }
bool  axis_manager_set_target_position(float v)
{
    if (isnan(v)) return false;
    s_axis.target_position_rad = v;
    return true;
}

float axis_manager_get_target_time(void) { return s_axis.target_time_s; }
bool  axis_manager_set_target_time(float v)
{
    if (v < 0.0f || isnan(v)) return false;
    s_axis.target_time_s = v;
    return true;
}

/* REQ-0012 — Commanded current [A] for AXIS_OP_MODE_TORQUE. Just stores;
 * the actual SDO write to motor 0x6071 happens via setup_sequencer_tick
 * once compute_desired picks up the new value. NaN rejected; otherwise
 * accept (negative current = opposite-direction torque is valid). */
float axis_manager_get_target_current(void) { return s_axis.target_current_a; }
bool  axis_manager_set_target_current(float amps)
{
    if (isnan(amps)) return false;
    if (s_axis.target_current_a != amps) {
        LOG_INFO("axis_manager: target_current = %ld mA (sdo write to motor 0x6071 queued)",
                 (long)lroundf(amps * 1000.0f));
    }
    s_axis.target_current_a = amps;
    return true;
}

/* On-board UP/DOWN button current magnitude [A]. Reject NaN and negatives
 * (sign comes from which button — magnitude only here). 0 disables the
 * buttons effectively (UP/DOWN still take ownership but command 0 A). */
float axis_manager_get_button_current(void) { return s_axis.button_current_a; }
bool  axis_manager_set_button_current(float amps)
{
    if (isnan(amps) || amps < 0.0f) return false;
    if (s_axis.button_current_a != amps) {
        LOG_INFO("axis_manager: button_current = %ld mA (UP -> +I, DOWN -> -I in TORQUE mode)",
                 (long)lroundf(amps * 1000.0f));
    }
    s_axis.button_current_a = amps;
    return true;
}

/*----------------------------------------------------------------------------
 * Limits
 *---------------------------------------------------------------------------*/

/* Translate a CMC-side position limit (which may be +/-INFINITY meaning
 * "unset / no limit") to the value the motor expects at 0x2600:6/7. The
 * motor treats `lo >= hi` as disabled; we map any non-finite value to
 * 0.0, so if either side is unset they'll both end up at 0 and the
 * motor disables enforcement. */
static float finite_or_zero(float v) { return isfinite(v) ? v : 0.0f; }

float axis_manager_get_velocity_limit(void) { return s_axis.velocity_limit_rad_s; }
bool  axis_manager_set_velocity_limit(float v)
{
    if (v < 0.0f || isnan(v)) return false;
    s_axis.velocity_limit_rad_s = v;
    /* joystick_max_velocity tracks (velocity_limit × joy_profile_scale) —
     * update it now so the CAMERAD JOYSTICK-mode demand cap moves with
     * whatever the operator just set. */
    recompute_joystick_max_velocity();
    /* Motor enforces its own envelope at 0x2600:4. 0 = disabled per motor
     * CHANGELOG [4.5.0]; we forward the operator's value as-is (0 from
     * the operator just disables motor-side enforcement, CMC-side
     * clamping still happens via compute_desired). */
    return proxy_motor_f32_begin(PROXY_SLOT_VEL_LIMIT, v);
}

float axis_manager_get_position_limit_lo(void) { return s_axis.position_limit_lo_rad; }
bool  axis_manager_set_position_limit_lo(float v)
{
    if (isnan(v)) return false;
    s_axis.position_limit_lo_rad = v;
    return proxy_motor_f32_begin(PROXY_SLOT_POS_LIMIT_LO, finite_or_zero(v));
}

float axis_manager_get_position_limit_hi(void) { return s_axis.position_limit_hi_rad; }
bool  axis_manager_set_position_limit_hi(float v)
{
    if (isnan(v)) return false;
    s_axis.position_limit_hi_rad = v;
    return proxy_motor_f32_begin(PROXY_SLOT_POS_LIMIT_HI, finite_or_zero(v));
}

float axis_manager_get_accel_limit(void) { return s_axis.accel_limit_rad_s2; }
bool  axis_manager_set_accel_limit(float v)
{
    if (v < 0.0f || isnan(v)) return false;
    s_axis.accel_limit_rad_s2 = v;
    /* Motor enforces its own envelope at 0x2600:5; same semantics as
     * the velocity-limit case. */
    return proxy_motor_f32_begin(PROXY_SLOT_ACCEL_LIMIT, v);
}

/* Load factor — see header. Set fires an SDO write to motor 0x2300:5
 * (vel_load_factor, REQ-0014). We don't AWAIT the result synchronously,
 * but the handle is REMEMBERED and polled each tick — this is critical
 * because cia402's OD pipeline holds state OD_COMPLETE until polled,
 * and a stuck OD_COMPLETE wedges every subsequent SDO write (mode
 * changes, target_position writes, PC tool reads — everything). */
#define MOTOR_OD_VEL_LOAD_FACTOR_IDX  (0x2300u)
#define MOTOR_OD_VEL_LOAD_FACTOR_SUB  (5u)

static cia402_od_handle_t s_load_factor_handle = CIA402_OD_HANDLE_INVALID;
static bool               s_load_factor_pending = false;

float axis_manager_get_load_factor(void) { return s_axis.load_factor; }
bool  axis_manager_set_load_factor(float v)
{
    if (isnan(v) || v < 0.3f || v > 2.0f) {
        LOG_WARN("axis_manager: set_load_factor REJECTED v=%d/1000 (need 0.3..2.0)",
                 (int)(v * 1000.0f));
        return false;
    }
    s_axis.load_factor = v;

    /* Don't try to issue a new SDO if we still have one in flight — we'd
     * just overwrite our own handle and leak the previous OD_COMPLETE slot.
     * The tick path drains the existing one first; the operator can re-slide
     * afterwards if they were too fast. */
    if (s_load_factor_pending) {
        LOG_INFO("axis_manager: load_factor cached %d/1000 (previous SDO still in flight; will not chain)",
                 (int)(v * 1000.0f));
        return true;
    }

    cia402_od_handle_t h = cia402_od_write_begin(
        MOTOR_OD_VEL_LOAD_FACTOR_IDX, MOTOR_OD_VEL_LOAD_FACTOR_SUB,
        MC_IF_T_F32, &s_axis.load_factor, sizeof(float));
    if (h == CIA402_OD_HANDLE_INVALID) {
        LOG_WARN("axis_manager: load_factor=%d/1000 cached, cia402 OD pipeline "
                 "busy — write not issued. Re-slide once it frees up.",
                 (int)(v * 1000.0f));
        return true;     /* cache still good; just no motor-side effect this time */
    }
    s_load_factor_handle  = h;
    s_load_factor_pending = true;
    LOG_INFO("axis_manager: load_factor -> %d/1000 (SDO write queued to motor 0x2300:5, handle=%d)",
             (int)(v * 1000.0f), (int)h);
    return true;
}

/* Called from axis_manager_tick to drain the load-factor SDO write so
 * the cia402 OD pipeline doesn't stay in OD_COMPLETE. We don't act on
 * the result — just log success/failure and free the handle. */
static void poll_load_factor_sdo(void)
{
    if (!s_load_factor_pending) return;
    MC_IfOdResult_t res = MC_IF_OD_OK;
    uint8_t buf[8] = {0};
    uint8_t vlen   = sizeof(buf);
    if (!cia402_od_poll(s_load_factor_handle, &res, buf, &vlen)) {
        return;  /* still pending — try again next tick */
    }
    if (res == MC_IF_OD_OK) {
        LOG_INFO("axis_manager: load_factor SDO -> motor 0x2300:5 OK (=%d/1000)",
                 (int)(s_axis.load_factor * 1000.0f));
    } else {
        LOG_WARN("axis_manager: load_factor SDO -> motor 0x2300:5 FAIL result=0x%02X "
                 "(motor MCU may not yet implement REQ-0014)", (unsigned)res);
    }
    s_load_factor_handle  = CIA402_OD_HANDLE_INVALID;
    s_load_factor_pending = false;
}

/*----------------------------------------------------------------------------
 * Generic motor-proxy SDO write (used for vel_accel_up/dn/jerk and the
 * motor-save magic write). Mirrors the load_factor pattern but shared
 * across all proxied entries with a SMALL DIRTY QUEUE so back-to-back
 * setters (e.g. apply_config_json doing set_up + set_dn + set_jerk in
 * one POST) don't drop writes when the cia402 OD pipeline is busy with
 * the first one. Each (idx, sub) slot has its own dirty flag + cached
 * value; the tick polls completion of the current in-flight, then
 * advances to the next dirty slot if any.
 *---------------------------------------------------------------------------*/
typedef struct {
    uint16_t    idx;
    uint8_t     sub;
    const char *name;
    float       value;     /* most-recent value the setter recorded */
    bool        dirty;     /* true if motor's flash differs from this cache (needs SDO) */
} motor_proxy_slot_t;

/* PROXY_SLOT_* macros defined near the top of the file (forward-decl
 * area) because the limit-setters far above reference them. The table
 * here is the canonical mapping of slot index -> (idx, sub, name). */
static motor_proxy_slot_t s_proxy_slots[PROXY_SLOT_COUNT] = {
    [PROXY_SLOT_VEL_ACCEL_UP]   = { 0x2300u, 6u, "vel_accel_up",    0.0f, false },
    [PROXY_SLOT_VEL_ACCEL_DN]   = { 0x2300u, 7u, "vel_accel_dn",    0.0f, false },
    [PROXY_SLOT_VEL_ACCEL_JERK] = { 0x2300u, 8u, "vel_accel_jerk",  0.0f, false },
    [PROXY_SLOT_VEL_LIMIT]      = { 0x2600u, 4u, "max_velocity",    0.0f, false },
    [PROXY_SLOT_ACCEL_LIMIT]    = { 0x2600u, 5u, "max_accel",       0.0f, false },
    [PROXY_SLOT_POS_LIMIT_LO]   = { 0x2600u, 6u, "pos_limit_lo",    0.0f, false },
    [PROXY_SLOT_POS_LIMIT_HI]   = { 0x2600u, 7u, "pos_limit_hi",    0.0f, false },
};

/* In-flight tracking. At most one SDO active at a time (cia402 limit). */
static cia402_od_handle_t s_proxy_handle      = CIA402_OD_HANDLE_INVALID;
static bool               s_proxy_pending     = false;
static uint32_t           s_proxy_inflight_slot = 0;     /* index into s_proxy_slots */

/* Mark a slot dirty + cache its value. Does NOT issue the SDO directly —
 * poll_motor_proxy_sdo() will pick it up on the next tick that finds the
 * pipeline free. This is what lets three back-to-back setters survive
 * the cia402 OD bottleneck: all three slots end up dirty, the tick drains
 * them one at a time. */
static bool proxy_motor_f32_begin(uint32_t slot_idx, float v)
{
    if (slot_idx >= PROXY_SLOT_COUNT) return false;
    motor_proxy_slot_t *s = &s_proxy_slots[slot_idx];
    s->value = v;
    s->dirty = true;
    LOG_INFO("axis_manager: %s queued = %d/1000 (motor 0x%04X:%u)",
             s->name, (int)(v * 1000.0f),
             (unsigned)s->idx, (unsigned)s->sub);
    return true;
}

/* Drain a completed proxy SDO (if any) and, if the pipeline is free,
 * fire the next dirty slot. Called from axis_manager_tick. */
static void poll_motor_proxy_sdo(void)
{
    /* Step 1: drain any in-flight SDO completion. */
    if (s_proxy_pending) {
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_proxy_handle, &res, buf, &vlen)) return;
        motor_proxy_slot_t *inflight = &s_proxy_slots[s_proxy_inflight_slot];
        if (res == MC_IF_OD_OK) {
            LOG_INFO("axis_manager: %s SDO -> motor 0x%04X:%u OK",
                     inflight->name, (unsigned)inflight->idx, (unsigned)inflight->sub);
        } else {
            LOG_WARN("axis_manager: %s SDO -> motor 0x%04X:%u FAIL result=0x%02X",
                     inflight->name, (unsigned)inflight->idx, (unsigned)inflight->sub,
                     (unsigned)res);
        }
        s_proxy_handle  = CIA402_OD_HANDLE_INVALID;
        s_proxy_pending = false;
        /* Fall through — pipeline is now free, can start next dirty slot. */
    }

    /* Step 2: find next dirty slot and start its SDO. Round-robin from
     * the last in-flight slot so heavy traffic on one entry doesn't
     * starve the others. */
    for (uint32_t i = 0; i < PROXY_SLOT_COUNT; ++i) {
        uint32_t slot_idx = (s_proxy_inflight_slot + 1u + i) % PROXY_SLOT_COUNT;
        motor_proxy_slot_t *s = &s_proxy_slots[slot_idx];
        if (!s->dirty) continue;
        cia402_od_handle_t h = cia402_od_write_begin(
            s->idx, s->sub, MC_IF_T_F32, &s->value, sizeof(float));
        if (h == CIA402_OD_HANDLE_INVALID) {
            /* cia402 still busy (e.g. load_factor or setup_sequencer write
             * in flight). Try again next tick. */
            return;
        }
        s->dirty               = false;
        s_proxy_handle         = h;
        s_proxy_pending        = true;
        s_proxy_inflight_slot  = slot_idx;
        LOG_INFO("axis_manager: %s -> motor 0x%04X:%u = %d/1000 (SDO h=%d)",
                 s->name, (unsigned)s->idx, (unsigned)s->sub,
                 (int)(s->value * 1000.0f), (int)h);
        return;
    }
}

/*----------------------------------------------------------------------------
 * vel_accel_up / vel_accel_dn / vel_accel_jerk (motor 0x2300:6/7/8)
 *---------------------------------------------------------------------------*/

#define MOTOR_OD_VEL_GAINS_IDX        (0x2300u)
#define MOTOR_OD_VEL_ACCEL_UP_SUB     (6u)
#define MOTOR_OD_VEL_ACCEL_DN_SUB     (7u)
#define MOTOR_OD_VEL_ACCEL_JERK_SUB   (8u)

float axis_manager_get_vel_accel_up  (void) { return s_axis.vel_accel_up_rad_s2;   }
float axis_manager_get_vel_accel_dn  (void) { return s_axis.vel_accel_dn_rad_s2;   }
float axis_manager_get_vel_accel_jerk(void) { return s_axis.vel_accel_jerk_rad_s3; }

bool axis_manager_set_vel_accel_up(float v)
{
    if (isnan(v) || v < 0.0f) return false;
    s_axis.vel_accel_up_rad_s2 = v;
    return proxy_motor_f32_begin(PROXY_SLOT_VEL_ACCEL_UP, v);
}
bool axis_manager_set_vel_accel_dn(float v)
{
    if (isnan(v) || v < 0.0f) return false;
    s_axis.vel_accel_dn_rad_s2 = v;
    return proxy_motor_f32_begin(PROXY_SLOT_VEL_ACCEL_DN, v);
}
bool axis_manager_set_vel_accel_jerk(float v)
{
    if (isnan(v) || v < 0.0f) return false;
    s_axis.vel_accel_jerk_rad_s3 = v;
    return proxy_motor_f32_begin(PROXY_SLOT_VEL_ACCEL_JERK, v);
}

/*----------------------------------------------------------------------------
 * Motor-proxy bootsync — SDO-read each accel param on first start so the
 * CMC's cache (and therefore the web UI) reflects the motor's actual
 * flash values rather than the firmware default of 0. Runs once after
 * cia402 is initialised and link is up; each slot reads one at a time
 * because the cia402 OD pipeline serialises. Skips a slot whose dirty
 * flag is set (the operator already typed a value the proxy is trying
 * to write — don't clobber it with the stale motor value).
 *---------------------------------------------------------------------------*/
typedef enum {
    BOOTSYNC_IDLE = 0,    /* not started yet */
    BOOTSYNC_BUSY,        /* an SDO read is in flight */
    BOOTSYNC_DONE,        /* all slots synced (or skipped) — sequencer retires */
} bootsync_state_t;

static bootsync_state_t   s_bootsync_state         = BOOTSYNC_IDLE;
static cia402_od_handle_t s_bootsync_handle        = CIA402_OD_HANDLE_INVALID;
static uint32_t           s_bootsync_inflight_slot = 0;
static uint32_t           s_bootsync_next_slot     = 0;
static uint32_t           s_bootsync_start_ms      = 0;

#define BOOTSYNC_KICKOFF_DELAY_MS  500u   /* let cia402 + motor settle before pinging */
/* Periodic auto-resync — re-read all proxied motor entries every N ms so the
 * CMC cache picks up changes a PC tool (or any other writer) made directly
 * to the motor's OD without going through axis_manager's setters. Without
 * this, the web UI shows stale values after a PC-tool-initiated change.
 * 5 s is a balance between freshness and SDO traffic (7 reads × ~5 ms each
 * = ~35 ms of pipeline time per resync, so 0.7% utilization at this rate). */
#define BOOTSYNC_RESYNC_PERIOD_MS  5000u
static uint32_t s_bootsync_last_done_ms;

/* Restart the resync state machine: forget completion, start reading from
 * slot 0 again. Public so other modules (e.g. web on demand) can trigger
 * an immediate refresh. No-op if a resync is already in flight. */
void axis_manager_request_motor_resync(void)
{
    if (s_bootsync_state != BOOTSYNC_DONE) return;  /* already running */
    s_bootsync_state     = BOOTSYNC_IDLE;
    s_bootsync_next_slot = 0;
    /* Skip the kickoff delay on restart — only the first-ever boot needs
     * it (motor was still settling). After that, motor is up. */
    s_bootsync_start_ms  = time_ms() - BOOTSYNC_KICKOFF_DELAY_MS;
}

static void tick_proxy_bootsync(void)
{
    /* Auto-restart every BOOTSYNC_RESYNC_PERIOD_MS once the previous pass
     * completed. Lets the CMC cache track motor-side changes without any
     * external trigger. */
    if (s_bootsync_state == BOOTSYNC_DONE) {
        if (time_elapsed_ms(s_bootsync_last_done_ms) >= BOOTSYNC_RESYNC_PERIOD_MS) {
            axis_manager_request_motor_resync();
        } else {
            return;
        }
    }

    /* Wait a beat after init before kicking off — the motor MCU's SPI
     * link is still settling and the first SDO often gets dropped if
     * fired too early. Once we've started, the state machine drives. */
    if (s_bootsync_state == BOOTSYNC_IDLE) {
        if (s_bootsync_start_ms == 0u) s_bootsync_start_ms = time_ms();
        if (time_elapsed_ms(s_bootsync_start_ms) < BOOTSYNC_KICKOFF_DELAY_MS) return;
    }

    /* Drain completed read, if any. */
    if (s_bootsync_state == BOOTSYNC_BUSY) {
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_bootsync_handle, &res, buf, &vlen)) return;

        motor_proxy_slot_t *s = &s_proxy_slots[s_bootsync_inflight_slot];
        if (res == MC_IF_OD_OK && vlen == sizeof(float)) {
            float v;
            memcpy(&v, buf, sizeof(float));
            /* Don't clobber the cache if the operator already typed a new
             * value (slot was marked dirty before we got here). Otherwise
             * adopt the motor's flash value and mirror into s_axis. */
            if (!s->dirty) {
                /* Only log when the value actually changes — periodic
                 * resync would otherwise spam unchanged values every
                 * BOOTSYNC_RESYNC_PERIOD_MS. */
                bool changed = (s->value != v);
                s->value = v;
                switch (s_bootsync_inflight_slot) {
                case PROXY_SLOT_VEL_ACCEL_UP:   s_axis.vel_accel_up_rad_s2   = v; break;
                case PROXY_SLOT_VEL_ACCEL_DN:   s_axis.vel_accel_dn_rad_s2   = v; break;
                case PROXY_SLOT_VEL_ACCEL_JERK: s_axis.vel_accel_jerk_rad_s3 = v; break;
                case PROXY_SLOT_VEL_LIMIT:      s_axis.velocity_limit_rad_s  = v;
                                                recompute_joystick_max_velocity(); break;
                case PROXY_SLOT_ACCEL_LIMIT:    s_axis.accel_limit_rad_s2    = v; break;
                /* Motor's 0 means "disabled" — translate back to CMC's +/-INF
                 * convention so the CMC-side clamping in compute_desired
                 * remains permissive (no clamp) until the operator sets a
                 * real value. */
                case PROXY_SLOT_POS_LIMIT_LO:
                    s_axis.position_limit_lo_rad = (v == 0.0f) ? -INFINITY : v;
                    break;
                case PROXY_SLOT_POS_LIMIT_HI:
                    s_axis.position_limit_hi_rad = (v == 0.0f) ? +INFINITY : v;
                    break;
                default: break;
                }
                if (changed) {
                    LOG_INFO("axis_manager: motor-resync %s = %d/1000 (from motor 0x%04X:%u)",
                             s->name, (int)(v * 1000.0f),
                             (unsigned)s->idx, (unsigned)s->sub);
                }
            } else {
                LOG_INFO("axis_manager: motor-resync %s skipped (already dirty — operator set value first)",
                         s->name);
            }
        } else {
            LOG_WARN("axis_manager: bootsync %s read FAIL result=0x%02X vlen=%u",
                     s->name, (unsigned)res, (unsigned)vlen);
        }
        s_bootsync_handle = CIA402_OD_HANDLE_INVALID;
        s_bootsync_state  = BOOTSYNC_IDLE;   /* fall through to start next */
    }

    /* Issue next read if there's one outstanding. */
    if (s_bootsync_next_slot >= PROXY_SLOT_COUNT) {
        if (s_bootsync_state != BOOTSYNC_DONE) {
            s_bootsync_state = BOOTSYNC_DONE;
            s_bootsync_last_done_ms = time_ms();
            /* Only log the very first completion — periodic resyncs would
             * spam this every BOOTSYNC_RESYNC_PERIOD_MS. The individual
             * per-slot value lines above are still rate-limited by being
             * inside the success branch. */
            static bool s_logged_once;
            if (!s_logged_once) {
                s_logged_once = true;
                LOG_INFO("axis_manager: bootsync complete — motor values mirrored into CMC cache (will auto-resync every %u ms)",
                         (unsigned)BOOTSYNC_RESYNC_PERIOD_MS);
            }
        }
        return;
    }
    motor_proxy_slot_t *s = &s_proxy_slots[s_bootsync_next_slot];
    cia402_od_handle_t h = cia402_od_read_begin(s->idx, s->sub, MC_IF_T_F32);
    if (h == CIA402_OD_HANDLE_INVALID) return;  /* cia402 busy — retry next tick */
    s_bootsync_handle        = h;
    s_bootsync_inflight_slot = s_bootsync_next_slot;
    s_bootsync_state         = BOOTSYNC_BUSY;
    s_bootsync_next_slot++;
}

/*----------------------------------------------------------------------------
 * Motor-save sequencer
 *
 * Motor MCU can only commit a save while the power stage is OFF (motor
 * ADR-010 / CHANGELOG [4.2.0] store_status PENDING bit). To save without
 * an operator-visible "save pending" state, axis_manager wraps the SDO
 * write to 0x2800:1 = MC_IF_SAVE_MAGIC with controlword disable +
 * re-enable. Re-enable is gated on the pre-save enable state — operators
 * who had the motor intentionally disabled don't suddenly find it
 * re-enabled by a config save.
 *
 * State machine, advanced each tick by tick_motor_save():
 *   IDLE         nothing in progress
 *   DISABLING    enable_latch cleared; waiting MOTOR_SAVE_DISABLE_MS for the
 *                statusword to actually drop the ENABLED bit on the wire
 *   WRITING_SDO  SDO write of SAVE_MAGIC to 0x2800:1 in flight
 *   REENABLE     write completed; re-enable enable_latch if it was set
 *                pre-save; back to IDLE
 *
 * Total wall-clock ~300-500 ms typical (disable settle + flash erase
 * + flash program + re-enable). Non-blocking — runs across many ticks.
 *---------------------------------------------------------------------------*/
#define MOTOR_SAVE_DISABLE_MS         250u   /* time for power stage to actually be OFF */
#define MOTOR_SAVE_OVERALL_TIMEOUT_MS 5000u  /* sequencer aborts if it can't finish in this long */

typedef enum {
    MOTOR_SAVE_IDLE = 0,
    MOTOR_SAVE_DISABLING,
    MOTOR_SAVE_WRITING_SDO,
    MOTOR_SAVE_REENABLE,
} motor_save_state_t;

static motor_save_state_t s_motor_save_state    = MOTOR_SAVE_IDLE;
static uint32_t           s_motor_save_phase_ms = 0;
static uint32_t           s_motor_save_start_ms = 0;
static bool               s_motor_save_was_enabled = false;
static cia402_od_handle_t s_motor_save_handle      = CIA402_OD_HANDLE_INVALID;

bool axis_manager_request_motor_save(void)
{
    if (s_motor_save_state != MOTOR_SAVE_IDLE) {
        LOG_WARN("axis_manager: motor save requested but sequencer already busy (state=%d)",
                 (int)s_motor_save_state);
        return false;
    }
    s_motor_save_was_enabled = s_axis.enable_latch;
    s_motor_save_start_ms    = time_ms();
    s_motor_save_phase_ms    = s_motor_save_start_ms;

    LOG_INFO("axis_manager: motor save START (was_enabled=%d) — disabling for %u ms",
             (int)s_motor_save_was_enabled, (unsigned)MOTOR_SAVE_DISABLE_MS);
    s_axis.enable_latch = false;   /* take effect next compose_cyclic_cmd */
    s_motor_save_state  = MOTOR_SAVE_DISABLING;
    return true;
}

static void tick_motor_save(void)
{
    if (s_motor_save_state == MOTOR_SAVE_IDLE) return;

    if (time_elapsed_ms(s_motor_save_start_ms) > MOTOR_SAVE_OVERALL_TIMEOUT_MS) {
        LOG_ERROR("axis_manager: motor save TIMEOUT after %u ms (state=%d) — aborting, restoring enable_latch=%d",
                  (unsigned)MOTOR_SAVE_OVERALL_TIMEOUT_MS,
                  (int)s_motor_save_state, (int)s_motor_save_was_enabled);
        s_axis.enable_latch = s_motor_save_was_enabled;
        s_motor_save_state  = MOTOR_SAVE_IDLE;
        s_motor_save_handle = CIA402_OD_HANDLE_INVALID;
        return;
    }

    switch (s_motor_save_state) {
    case MOTOR_SAVE_DISABLING: {
        if (time_elapsed_ms(s_motor_save_phase_ms) < MOTOR_SAVE_DISABLE_MS) return;
        /* Disable settle window elapsed — start the SDO write. If the
         * cia402 OD pipeline is busy with something else (e.g. a
         * load_factor or vel_accel proxy), try again next tick. */
        uint16_t magic = MC_IF_SAVE_MAGIC;
        cia402_od_handle_t h = cia402_od_write_begin(
            0x2800u, 1u, MC_IF_T_U16, &magic, sizeof(magic));
        if (h == CIA402_OD_HANDLE_INVALID) return;
        s_motor_save_handle   = h;
        s_motor_save_state    = MOTOR_SAVE_WRITING_SDO;
        s_motor_save_phase_ms = time_ms();
        LOG_INFO("axis_manager: motor save WRITE 0x2800:1 = 0x%04X (SDO h=%d)",
                 (unsigned)MC_IF_SAVE_MAGIC, (int)h);
        break;
    }
    case MOTOR_SAVE_WRITING_SDO: {
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_motor_save_handle, &res, buf, &vlen)) return;
        if (res == MC_IF_OD_OK) {
            LOG_INFO("axis_manager: motor save SDO accepted (motor wrote to flash)");
        } else {
            LOG_WARN("axis_manager: motor save SDO FAIL result=0x%02X "
                     "(motor refused — drive may still be enabled?)", (unsigned)res);
        }
        s_motor_save_handle = CIA402_OD_HANDLE_INVALID;
        s_motor_save_state  = MOTOR_SAVE_REENABLE;
        s_motor_save_phase_ms = time_ms();
        break;
    }
    case MOTOR_SAVE_REENABLE: {
        /* Brief grace period before re-enabling, so the motor's
         * statusword has time to stabilise after the save. */
        if (time_elapsed_ms(s_motor_save_phase_ms) < 50u) return;
        if (s_motor_save_was_enabled) {
            s_axis.enable_latch = true;
            LOG_INFO("axis_manager: motor save DONE — re-enabling axis (was enabled pre-save)");
        } else {
            LOG_INFO("axis_manager: motor save DONE — leaving disabled (was disabled pre-save)");
        }
        s_motor_save_state = MOTOR_SAVE_IDLE;
        break;
    }
    default:
        s_motor_save_state = MOTOR_SAVE_IDLE;
        break;
    }
}

/*----------------------------------------------------------------------------
 * Home-to-endstop sequencer + is_homed cache
 *
 * Backs the CMC-owned surface at 0x3040/1/2 (write axis_home_command=1 to
 * start; read axis_home_status for the live motor state; read axis_is_homed
 * for the "shot recalls allowed" gate). Wraps the motor's own homing at
 * 0x2700:8/9 and the NOT_HOMED bit in 0x2600:1 fault_flags.
 *
 * Two independent state machines share this module:
 *   1. Boot-and-periodic is_homed refresh
 *        On the first tick after startup we kick a one-shot SDO read of
 *        fault_flags so cmc_state has a fresh answer before any recall
 *        attempt. After that, we lean on the manual re-read that happens
 *        at the end of a home sequence — refreshing every N seconds like
 *        bootsync would be overkill (NOT_HOMED only clears via a successful
 *        home procedure or a factory reset, both of which we already know
 *        about locally).
 *   2. Home sequence — states:
 *        IDLE           nothing in progress
 *        WRITING_CMD    SDO write of 0x2700:8=1 in flight to motor
 *        POLLING_STATUS periodic SDO reads of 0x2700:9 waiting for
 *                       DONE / FAILED. re-issued every HOME_POLL_PERIOD_MS.
 *        READING_FAULT  after DONE, SDO-read 0x2600:1 to refresh is_homed
 *        DONE / FAILED  terminal for one wall-clock tick, then IDLE.
 *
 * Interaction with other sequencers: cia402's OD pipeline serialises one
 * SDO at a time. Each state machine here checks its own handle; if
 * cia402_od_*_begin returns INVALID (pipeline busy with bootsync / motor
 * save / proxy write) we just retry next tick. Total home wall-clock is
 * bounded by the motor-side operation (drive-to-stall, typically 1–10 s
 * per ADR-057) plus a few round-trip SDOs on either end.
 *---------------------------------------------------------------------------*/
#define HOME_POLL_PERIOD_MS      200u    /* re-poll home_status every N ms while RUNNING */
#define HOME_OVERALL_TIMEOUT_MS  30000u  /* absolute cap — motor's own homing is <10s typical */

typedef enum {
    HOME_SEQ_IDLE = 0,
    HOME_SEQ_READING_ENCTYPE,  /* one-shot: SDO read motor 0x2500:8 quad_counts_per_rev */
    HOME_SEQ_CLEARING_CMD,     /* SDO write 0x2700:8 = 0 — clear sticky DONE/FAILED */
    HOME_SEQ_WRITING_CMD,      /* SDO write 0x2700:8 = 1 in flight */
    HOME_SEQ_POLLING_STATUS,   /* periodic reads of 0x2700:9 */
    HOME_SEQ_READING_FAULT,    /* post-DONE re-read of 0x2600:1 */
    HOME_SEQ_TERMINAL_DONE,    /* one tick of "done" for logging */
    HOME_SEQ_TERMINAL_FAILED,
} home_seq_state_t;

static home_seq_state_t   s_home_seq_state       = HOME_SEQ_IDLE;
static cia402_od_handle_t s_home_seq_handle      = CIA402_OD_HANDLE_INVALID;
static uint32_t           s_home_seq_start_ms    = 0;    /* wall-clock start of the current sequence */
static uint32_t           s_home_last_poll_ms    = 0;    /* last time we kicked a status read */
static uint8_t            s_home_last_motor_status = 0;  /* mirror of motor 0x2700:9 (MC_IF_HOME_*) */
static bool               s_is_homed             = false; /* fault_flags & NOT_HOMED == 0 */
static bool               s_is_homed_known       = false; /* have we ever successfully read fault_flags? */
static uint32_t           s_last_fault_read_ms   = 0;    /* time_ms of last kick; 0 = never */

/* Encoder-type detection — read motor 0x2500:8 quad_counts_per_rev once at
 * boot. Non-zero = incremental encoder is configured on the motor; zero =
 * absolute (or unconfigured). Result is authoritative and doesn't depend
 * on runtime homing state, so the web UI's "Home to end stop" button stays
 * available even when the axis is currently homed. */
static bool               s_encoder_type_known    = false;
static bool               s_encoder_is_incremental = false;
static uint32_t           s_last_enctype_read_ms  = 0;    /* 0 = never */

/* Wait this long after CMC boot before the first attempt to read anything
 * from the motor. Bootsync uses the same delay to give the motor's SPI
 * link time to come up before we start hammering it with SDOs. */
#define ENCTYPE_BOOT_DELAY_MS 2000u
#define ENCTYPE_RETRY_MS      500u   /* retry cadence until first success */

/* How often we re-read motor fault_flags to refresh is_homed.
 * RETRY_MS: while is_homed is unknown (previous reads failed / never fired).
 *           Short so shot recalls unblock as soon as the motor is up.
 * REFRESH_MS: once we've got a good reading, how often to re-check so a
 *           motor-side change (e.g. operator ran home_command via PC tool
 *           directly, or motor reset mid-session) doesn't leave us stale. */
#define FAULT_READ_RETRY_MS   500u
#define FAULT_READ_REFRESH_MS 5000u

/* Force every motor-OD-touching sub-module back to its IDLE / start state.
 * Called from axis_manager_tick on the rising edge of
 * "cia402_motor_in_bootloader" so the sub-modules don't come back to life
 * mid-request when the motor jumps back to app mode — they'd be holding
 * stale cia402 handles at that point. Placed HERE (after all sub-module
 * statics are declared) — forward-declared prototype near the top of the
 * file so axis_manager_tick can call it. */
static void reset_motor_od_submodules(void)
{
    s_home_seq_state         = HOME_SEQ_IDLE;
    s_home_seq_handle        = CIA402_OD_HANDLE_INVALID;
    s_home_last_motor_status = MC_IF_HOME_IDLE;
    s_bootsync_state         = BOOTSYNC_IDLE;
    s_bootsync_handle        = CIA402_OD_HANDLE_INVALID;
    /* bootsync gets a full restart when motor returns to app — a firmware
     * update may have changed the motor's OD contents, so cached proxy
     * slots must be re-mirrored. */
    s_bootsync_next_slot     = 0;
    s_motor_save_state       = MOTOR_SAVE_IDLE;
    s_motor_save_handle      = CIA402_OD_HANDLE_INVALID;
    s_load_factor_handle     = CIA402_OD_HANDLE_INVALID;
    s_proxy_handle           = CIA402_OD_HANDLE_INVALID;
}

uint8_t axis_manager_get_home_status(void)
{
    /* Report the motor's own status enum. While IDLE we return whatever the
     * last poll saw (either IDLE if we never ran, or DONE/FAILED from the
     * previous run kept as a "sticky" reading). The sequencer state machine
     * runs independently; this is just the last-known motor value. */
    return s_home_last_motor_status;
}

bool axis_manager_is_homed(void)
{
    /* Conservative default: if we haven't successfully read fault_flags
     * yet, treat as NOT homed. Prevents a race at boot where a shot recall
     * arrives before the first SDO landed. */
    return s_is_homed_known && s_is_homed;
}

bool axis_manager_encoder_is_incremental(void)
{
    /* Read at boot from motor 0x2500:8 quad_counts_per_rev — non-zero =
     * incremental, zero = absolute. Independent of runtime homing state
     * so the operator can always re-home an incremental axis even when
     * it's currently reporting is_homed = 1.
     *
     * Conservative fallback: while we haven't successfully read the value
     * yet (first ~2 s after boot, or if the motor SDO isn't responding),
     * return true. Better to show the Home button and have the operator
     * click it needlessly on an absolute-encoder rig than to hide it when
     * it's actually needed. Once we have a real reading it becomes
     * authoritative. */
    return !s_encoder_type_known || s_encoder_is_incremental;
}

/* Body of the axis_manager_stop_op HOMING case (forward-declared near the
 * top of the file). Sends 0x2700:8 = 0 to the motor to abort its homing
 * routine, and forces the CMC-side sequencer back to IDLE so it stops
 * polling. Best-effort: if the SDO slot is currently held by someone else
 * the write silently fails and the motor's own home-timeout eventually
 * unwinds it. */
static void abort_motor_homing_best_effort(void)
{
    uint8_t zero = 0u;
    (void)cia402_od_write_begin(0x2700u, 8u, MC_IF_T_U8, &zero, sizeof(zero));
    s_home_seq_state         = HOME_SEQ_IDLE;
    s_home_last_motor_status = MC_IF_HOME_IDLE;
}

/* Completion detection for the active operation. Runs once per axis_manager
 * tick; clears s_active_op back to NONE when the current op finishes on its
 * own terms.
 *
 * Kept in axis_manager (not cmc_state) so the arbitration logic lives with
 * the state it protects; cmc_state stays a thin protocol adapter that just
 * calls the try_begin/stop primitives. */
#define AXIS_OP_ARRIVAL_GRACE_MS  100u  /* mirrors cmc_state ARRIVAL_GRACE_MS (ADR-033) */
/* Definition matches the forward-declaration near the top of the file. */
static void tick_active_op(void)
{
    switch (s_active_op) {
        case AXIS_OPERATION_NONE:
            return;
        case AXIS_OPERATION_HOMING:
            /* Home sequencer transitions to a terminal state; either DONE
             * or FAILED means the operation is complete. */
            if (s_home_seq_state == HOME_SEQ_TERMINAL_DONE
                || s_home_seq_state == HOME_SEQ_TERMINAL_FAILED
                || s_home_seq_state == HOME_SEQ_IDLE) {
                set_active_op(AXIS_OPERATION_NONE);
            }
            return;
        case AXIS_OPERATION_SHOT_RECALL: {
            /* Motor reports on_target && !moving past the arrival-grace
             * window. Same detector as cmc_state uses to clear s_next_shot. */
            bool motor_moving    = (s_axis.movement_status & MC_IF_MOVE_MOVING)    != 0u;
            bool motor_on_target = (s_axis.movement_status & MC_IF_MOVE_ON_TARGET) != 0u;
            bool grace_active    = (time_elapsed_ms(s_active_op_started_ms) < AXIS_OP_ARRIVAL_GRACE_MS);
            if (motor_on_target && !motor_moving && !grace_active) {
                set_active_op(AXIS_OPERATION_NONE);
            }
            return;
        }
        case AXIS_OPERATION_JOYSTICK: {
            /* Quiescent = stick centred AND motor stopped. Wait
             * JOYSTICK_QUIESCENT_HOLD_MS of continuous quiescence before
             * releasing so a mid-hold flap doesn't churn active_op. */
            bool motor_moving = (s_axis.movement_status & MC_IF_MOVE_MOVING) != 0u;
            bool stick_zero   = (s_axis.joystick_value == 0.0f);
            if (motor_moving || !stick_zero) {
                s_joystick_quiescent_since_ms = 0u;
                return;
            }
            if (s_joystick_quiescent_since_ms == 0u) {
                uint32_t now = time_ms();
                s_joystick_quiescent_since_ms = (now == 0u) ? 1u : now;   /* avoid 0-sentinel */
                return;
            }
            if (time_elapsed_ms(s_joystick_quiescent_since_ms) >= JOYSTICK_QUIESCENT_HOLD_MS) {
                set_active_op(AXIS_OPERATION_NONE);
            }
            return;
        }
        default:
            return;
    }
}

bool axis_manager_request_home(void)
{
    /* Arbitration: HOMING may only start when nothing else is running.
     * axis_manager_try_begin_op enforces the priority rules; if it returns
     * REJECTED the sequencer stays IDLE and the caller (OD write to 0x3040)
     * sees the false return + NOT_READY on the wire. */
    axis_begin_result_t r = axis_manager_try_begin_op(AXIS_OPERATION_HOMING);
    if (r == AXIS_BEGIN_REJECTED) return false;
    /* Also refuse re-entrant home while the sequencer is mid-run — the
     * try_begin above already caught the cross-family case; this catches
     * the pathological "same-family retarget of a home" case. */
    if (s_home_seq_state != HOME_SEQ_IDLE) return false;
    LOG_INFO("axis: HOME start");
    /* Reset the cached motor status so POLLING_STATUS can't short-circuit
     * on a stale DONE/FAILED value from a previous run — otherwise the
     * first read after write=1 might come back reporting the still-latched
     * DONE and we'd skip straight to READING_FAULT without giving the
     * motor time to run. Fresh IDLE means "we haven't heard from motor
     * about this run yet". */
    s_home_last_motor_status = MC_IF_HOME_IDLE;
    s_home_seq_state    = HOME_SEQ_CLEARING_CMD;
    s_home_seq_start_ms = time_ms();
    s_home_last_poll_ms = 0;
    return true;
}

static void tick_home_sequencer(void)
{
    /* --- Periodic fault_flags read (keeps is_homed cache coherent) ---
     * Kicked only while the main sequencer is IDLE — piggybacks on the
     * same READING_FAULT state to consume the reply. Rate depends on
     * whether we have a good reading yet: RETRY_MS (short) while unknown,
     * REFRESH_MS (5 s) once known. Previously this was a one-shot at
     * boot — but if that single attempt failed (transient SDO error,
     * motor still coming up), s_is_homed_known stayed false forever and
     * cmc_state would reject every shot recall for the rest of the
     * session. And even on success, we needed to catch motor-side changes
     * we didn't drive ourselves (PC tool writing motor 0x2700:8 direct,
     * motor mid-session reset, etc.). */
    /* --- One-shot encoder-type read (only while unknown) ---
     * Wait ENCTYPE_BOOT_DELAY_MS after CMC startup so the motor's SPI link
     * has time to come up. After that, retry at ENCTYPE_RETRY_MS until we
     * get a valid reading. Once known, this stops firing. */
    if (s_home_seq_state == HOME_SEQ_IDLE && !s_encoder_type_known) {
        uint32_t now = time_ms();
        if (now >= ENCTYPE_BOOT_DELAY_MS) {
            bool due = (s_last_enctype_read_ms == 0u)
                       || (time_elapsed_ms(s_last_enctype_read_ms) >= ENCTYPE_RETRY_MS);
            if (due) {
                cia402_od_handle_t h = cia402_od_read_begin(0x2500u, 8u, MC_IF_T_F32);
                if (h != CIA402_OD_HANDLE_INVALID) {
                    s_home_seq_handle = h;
                    s_home_seq_state  = HOME_SEQ_READING_ENCTYPE;
                    s_last_enctype_read_ms = now;
                }
            }
        }
    }

    /* --- 0x3043 home_on_boot auto-fire --- Once per CMC boot, once we know
     * the motor uses an incremental encoder (absolute encoders don't need
     * homing — is_homed is always true), fire a home request. Motor-in-BL
     * is already gated at the callsite so we don't need to re-check it.
     * If try_begin rejects (something else running), we retry every tick
     * until it takes — s_boot_home_fired only latches on a successful
     * request_home so a transient rejection doesn't burn the one-shot. */
    static bool s_boot_home_fired = false;
    if (!s_boot_home_fired
        && s_axis.home_on_boot
        && s_encoder_type_known
        && s_encoder_is_incremental
        && s_home_seq_state == HOME_SEQ_IDLE) {
        if (axis_manager_request_home()) {
            LOG_INFO("axis: home_on_boot=1 -> auto-fired home");
            s_boot_home_fired = true;
        }
    }

    if (s_home_seq_state == HOME_SEQ_IDLE) {
        uint32_t period = s_is_homed_known ? FAULT_READ_REFRESH_MS
                                           : FAULT_READ_RETRY_MS;
        bool due = (s_last_fault_read_ms == 0u)
                   || (time_elapsed_ms(s_last_fault_read_ms) >= period);
        if (due) {
            cia402_od_handle_t h = cia402_od_read_begin(0x2600u, 1u, MC_IF_T_U32);
            if (h != CIA402_OD_HANDLE_INVALID) {
                s_home_seq_handle = h;
                s_home_seq_state  = HOME_SEQ_READING_FAULT;
                s_last_fault_read_ms = time_ms();
            }
            /* cia402 busy -> leave timestamp alone, try again next tick. */
        }
    }

    /* --- Sequencer body --- */
    if (s_home_seq_state == HOME_SEQ_IDLE) return;

    /* Absolute timeout — motor should finish or fail within 30 s. */
    if (s_home_seq_state != HOME_SEQ_READING_FAULT
        && time_elapsed_ms(s_home_seq_start_ms) > HOME_OVERALL_TIMEOUT_MS) {
        LOG_ERROR("axis: HOME timeout (%d)", (int)s_home_seq_state);
        s_home_seq_state = HOME_SEQ_TERMINAL_FAILED;
        s_home_last_motor_status = MC_IF_HOME_FAILED;
        return;
    }

    switch (s_home_seq_state) {
    case HOME_SEQ_READING_ENCTYPE: {
        /* Consume the boot-time read of motor 0x2500:8 quad_counts_per_rev.
         * Non-zero -> incremental encoder configured. Zero -> absolute
         * (or unconfigured). Reply size is F32 (4 bytes). */
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_home_seq_handle, &res, buf, &vlen)) return;
        s_home_seq_handle = CIA402_OD_HANDLE_INVALID;
        if (res == MC_IF_OD_OK && vlen >= sizeof(float)) {
            float cpr;
            memcpy(&cpr, buf, sizeof(cpr));
            s_encoder_is_incremental = (cpr != 0.0f);
            s_encoder_type_known     = true;
            LOG_INFO("axis: enc %s (cpr=%d)",
                     s_encoder_is_incremental ? "inc" : "abs",
                     (int)cpr);
        }
        /* On failure, s_encoder_type_known stays false and the retry
         * cadence in the IDLE section above will fire the read again. */
        s_home_seq_state = HOME_SEQ_IDLE;
        break;
    }
    case HOME_SEQ_CLEARING_CMD: {
        /* Write home_command = 0 first to clear any latched DONE/FAILED
         * from a previous run (motor's home_command is edge-triggered per
         * OD contract "0 = idle/abort (also clears done/failed)" — writing
         * 1 while DONE is still latched would be a no-op on the motor
         * side, and the CMC would spin until the overall timeout thinking
         * it was still running). Only after this write completes do we
         * write the actual "1" that arms the run. */
        if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) {
            uint8_t zero = 0u;
            s_home_seq_handle = cia402_od_write_begin(
                0x2700u, 8u, MC_IF_T_U8, &zero, sizeof(zero));
            if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) return;
        }
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_home_seq_handle, &res, buf, &vlen)) return;
        s_home_seq_handle = CIA402_OD_HANDLE_INVALID;
        if (res != MC_IF_OD_OK) {
            LOG_ERROR("axis: HOME clr fail %d", (int)res);
            s_home_seq_state = HOME_SEQ_TERMINAL_FAILED;
            s_home_last_motor_status = MC_IF_HOME_FAILED;
            return;
        }
        s_home_seq_state = HOME_SEQ_WRITING_CMD;
        break;
    }
    case HOME_SEQ_WRITING_CMD: {
        if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) {
            /* Try to issue the write. If cia402 is busy we retry next tick. */
            uint8_t one = 1u;
            s_home_seq_handle = cia402_od_write_begin(
                0x2700u, 8u, MC_IF_T_U8, &one, sizeof(one));
            if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) return;
        }
        /* Wait for reply. */
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_home_seq_handle, &res, buf, &vlen)) return;
        s_home_seq_handle = CIA402_OD_HANDLE_INVALID;
        if (res != MC_IF_OD_OK) {
            LOG_ERROR("axis: HOME wr fail %d", (int)res);
            s_home_seq_state = HOME_SEQ_TERMINAL_FAILED;
            s_home_last_motor_status = MC_IF_HOME_FAILED;
            return;
        }
        s_home_seq_state    = HOME_SEQ_POLLING_STATUS;
        s_home_last_poll_ms = time_ms();
        break;
    }
    case HOME_SEQ_POLLING_STATUS: {
        /* No poll in flight — start a fresh one either right away
         * (first entry) or after HOME_POLL_PERIOD_MS has elapsed. */
        if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) {
            if (time_elapsed_ms(s_home_last_poll_ms) < HOME_POLL_PERIOD_MS
                && time_elapsed_ms(s_home_seq_start_ms) > 100u) {
                return;   /* rate-limit — motor won't have changed yet */
            }
            s_home_seq_handle = cia402_od_read_begin(0x2700u, 9u, MC_IF_T_U8);
            if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) return;
            s_home_last_poll_ms = time_ms();
        }
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_home_seq_handle, &res, buf, &vlen)) return;
        s_home_seq_handle = CIA402_OD_HANDLE_INVALID;
        if (res != MC_IF_OD_OK || vlen == 0) {
            return;   /* stay in POLLING_STATUS, will retry after period */
        }
        uint8_t status = buf[0];
        if (status != s_home_last_motor_status) {
            LOG_INFO("axis: HOME st %u->%u",
                     (unsigned)s_home_last_motor_status, (unsigned)status);
            s_home_last_motor_status = status;
        }
        if (status == MC_IF_HOME_DONE || status == MC_IF_HOME_FAILED) {
            /* Re-read fault_flags so is_homed reflects the current
             * (still-set on FAILED) NOT_HOMED bit. */
            s_home_seq_state = HOME_SEQ_READING_FAULT;
        }
        /* else RUNNING / IDLE — keep polling. */
        break;
    }
    case HOME_SEQ_READING_FAULT: {
        if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) {
            s_home_seq_handle = cia402_od_read_begin(0x2600u, 1u, MC_IF_T_U32);
            if (s_home_seq_handle == CIA402_OD_HANDLE_INVALID) return;
        }
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_home_seq_handle, &res, buf, &vlen)) return;
        s_home_seq_handle = CIA402_OD_HANDLE_INVALID;
        if (res == MC_IF_OD_OK && vlen >= sizeof(uint32_t)) {
            uint32_t ff;
            memcpy(&ff, buf, sizeof(ff));
            bool was_known = s_is_homed_known;
            bool was_homed = s_is_homed;
            s_is_homed = (ff & MC_IF_FAULT_NOT_HOMED) == 0u;
            s_is_homed_known = true;
            if (was_homed != s_is_homed || !was_known) {
                LOG_INFO("axis: is_homed=%d", (int)s_is_homed);
            }
            /* Boot one-shot goes straight to IDLE; sequence terminals log. */
            if (s_home_last_motor_status == MC_IF_HOME_DONE) {
                s_home_seq_state = HOME_SEQ_TERMINAL_DONE;
            } else if (s_home_last_motor_status == MC_IF_HOME_FAILED) {
                s_home_seq_state = HOME_SEQ_TERMINAL_FAILED;
            } else {
                s_home_seq_state = HOME_SEQ_IDLE;
            }
        } else {
            s_home_seq_state = HOME_SEQ_IDLE;
        }
        break;
    }
    case HOME_SEQ_TERMINAL_DONE:
    case HOME_SEQ_TERMINAL_FAILED:
        /* is_homed transition log fires from the READING_FAULT path
         * already; no need for a separate "HOME end" line. */
        s_home_seq_state = HOME_SEQ_IDLE;
        break;
    default:
        s_home_seq_state = HOME_SEQ_IDLE;
        break;
    }
}

/*----------------------------------------------------------------------------
 * Joystick raw + calibration (0x3026-0x302A)
 *---------------------------------------------------------------------------*/

int32_t axis_manager_get_joystick_raw(void) { return s_axis.joystick_raw; }
bool    axis_manager_set_joystick_raw(int32_t v)
{
    s_axis.joystick_raw = v;
    recompute_joystick_value_from_raw();
    return true;
}

int32_t axis_manager_get_joystick_raw_center(void) { return s_axis.joystick_raw_center; }
bool    axis_manager_set_joystick_raw_center(int32_t v)
{
    s_axis.joystick_raw_center = v;
    recompute_joystick_value_from_raw();
    return true;
}

int32_t axis_manager_get_joystick_raw_full_pos(void) { return s_axis.joystick_raw_full_pos; }
bool    axis_manager_set_joystick_raw_full_pos(int32_t v)
{
    s_axis.joystick_raw_full_pos = v;
    recompute_joystick_value_from_raw();
    return true;
}

int32_t axis_manager_get_joystick_raw_full_neg(void) { return s_axis.joystick_raw_full_neg; }
bool    axis_manager_set_joystick_raw_full_neg(int32_t v)
{
    s_axis.joystick_raw_full_neg = v;
    recompute_joystick_value_from_raw();
    return true;
}

uint32_t axis_manager_get_joystick_raw_deadband(void) { return s_axis.joystick_raw_deadband; }
bool     axis_manager_set_joystick_raw_deadband(uint32_t v)
{
    s_axis.joystick_raw_deadband = v;
    recompute_joystick_value_from_raw();
    return true;
}
