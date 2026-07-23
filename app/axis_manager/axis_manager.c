/*
 * app/axis_manager — the axis "orchestrator" for the network MCU.
 *
 * ============================================================================
 * MODULE MAP (2026-07-22 refactor — extracted 3 sub-modules)
 * ============================================================================
 *
 * This file owns the orchestrator concerns: it maintains the s_axis
 * singleton, sequences the tick, backs the public 0x30xx OD entries, drives
 * cia402's cyclic command, and manages the persist blob. Everything that
 * has its own state machine or its own cia402 SDO discipline lives in a
 * peer .c under app/axis_manager/.
 *
 *   Sub-modules (peer files under app/axis_manager/):
 *     home_sequencer.c    Home-to-endstop state machine + encoder-type probe
 *                         + is_homed cache + home_on_boot (0x3043).
 *     motor_od_proxy.c    Dirty-slot writer + bootsync + motor-save state
 *                         machine + load_factor. All four share the cia402
 *                         SDO slot cooperatively.
 *     buttons_jog.c       PB UP/DOWN button poll + JOYSTICK-mode override
 *                         + post-release hold state machine.
 *
 *   In-file sections (this file):
 *     ~120   Constants + helper functions (mode_to_cia402, name lookups)
 *     ~180   Module state (s_axis + op-arbiter statics + setup-sequencer
 *            statics + auto-fault-clear tracker + setup snapshot type)
 *     ~250   Operation arbitration (try_begin_op / stop_op / set_active_op /
 *            tick_active_op — the layer above op_mode that admits at most
 *            one HOMING/SHOT_RECALL/JOYSTICK at a time)
 *     ~400   Scaling helpers (unit conversion, joystick profile, recompute_*)
 *     ~530   Persistence (v6 blob format, apply_persist_blob, capture_persist_blob,
 *            save_to_flash — schema is APPEND-ONLY, see README)
 *     ~640   Bootsync callback (motor_od_proxy notifies us when it read a
 *            CATEGORY-2 limit back from motor flash — we mirror into s_axis)
 *     ~680   Lifecycle (axis_manager_init calls each sub-module's init and
 *            registers the bootsync callback before loading persist)
 *     ~740   Desired-setup snapshot (compute_desired — clamp + scale)
 *     ~780   SDO setup-sequencer (single-in-flight for mode/target/profile
 *            writes to motor 0x6060/6071/607A/607B/6081/6083/6084)
 *     ~960   Cyclic command composition (derive_state + compose_cyclic_cmd)
 *    ~1050   axis_manager_tick + reset_motor_od_submodules
 *    ~1170   State getters (0x30xx RO OD entries)
 *    ~1190   Command latches (enable/quick_stop/clear_fault/start_move/halt)
 *    ~1240   Mode + targets (0x3020..0x302C RW OD entries)
 *    ~1380   Limits (0x3030..0x3033 — writes s_axis mirror + forwards to
 *            motor_od_proxy_write_*; reads return s_axis)
 *    ~1460   Sub-module forwarders (load_factor / vel_accel_* / motor_save /
 *            motor_resync / home / home_on_boot — all thin one-liners into
 *            home_sequencer.c or motor_od_proxy.c)
 *    ~1540   Joystick raw + calibration (0x3026-0x302A)
 *
 * ============================================================================
 * INVARIANTS (see README for the full list)
 * ============================================================================
 *
 *   cia402-slot cooperation. home_sequencer + motor_od_proxy (dirty-slot
 *   writer + bootsync + motor-save + load_factor) + setup_sequencer_tick
 *   all issue cia402_od_*_begin and defer if it returns INVALID. Only one
 *   OD operation is ever in flight at a time. The tick order below in
 *   axis_manager_tick is designed to give reads priority over writes each
 *   cycle so a proxy write can't starve the bootsync-refresh loop.
 *
 *   Reset-on-bootloader-entry. When the motor enters its own bootloader,
 *   reset_motor_od_submodules() is called on the rising edge. It MUST call
 *   the _reset function of every sub-module that holds a cia402 handle.
 *   If you add a new sub-module, add its reset here.
 *
 *   Persist blob is APPEND-ONLY. Never reorder / resize / remove an
 *   existing field. New fields at the tail get value 0 on load from an
 *   older blob; apply_persist_blob's sanitisers promote 0 -> the real
 *   default. persist_load_or_upgrade handles the in-memory upgrade.
 */

#include "axis_manager.h"

#include "buttons_jog.h"
#include "home_sequencer.h"
#include "motor_od_proxy.h"

#include "app/cia402/cia402.h"
#include "app/led_indicator/led_indicator.h"  /* persist-blob co-tenant */
#include "app/log/log.h"
#include "app/persist/persist.h"
#include "bsp/time/time.h"

#include <string.h>
#include <math.h>

/*============================================================================
 * Constants + helpers (name lookups, mode translation)
 *============================================================================*/

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

/* String helpers — used purely for log readability. */
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

/*============================================================================
 * MODULE STATE (SECTION 1) — the s_axis singleton + orchestrator statics
 *
 * s_axis holds every field the OD dispatch reads/writes plus a couple of
 * derived-from-motor values (position_actual_rad, movement_status). Fields
 * that were previously in s_axis but now live in a sub-module:
 *   - load_factor            → motor_od_proxy
 *   - vel_accel_up/dn/jerk   → motor_od_proxy
 *   - home_on_boot           → home_sequencer
 *
 * The velocity/position/accel limits still live here because CMC-side
 * clamping (compute_desired) reads them locally. Their motor-side mirror
 * lives in motor_od_proxy's slot table; setters write both. On bootsync
 * read-back motor_od_proxy calls our bootsync_notify callback to update
 * the local mirror.
 *============================================================================*/

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
     * selected joystick speed profile (NORMAL / MEDIUM / FINE). */
    float            joystick_max_velocity;
    uint8_t          joy_profile;            /* AXIS_JOY_PROFILE_* — CAMERAD JOY_PROFILE_* selects */
    uint8_t          axis_role;              /* 0x3070 — CAMERAD_AXIS_* bitmap value; PAN default */
    float            target_velocity_rad_s;
    float            target_position_rad;
    float            target_time_s;
    float            target_current_a;       /* REQ-0012 — used in AXIS_OP_MODE_TORQUE */
    float            button_current_a;       /* 0x302C — magnitude UP/DOWN buttons apply (TORQUE mode only) */

    /* Joystick raw input + linear-with-deadband calibration (0x3026-0x302A) */
    int32_t          joystick_raw;
    int32_t          joystick_raw_center;
    int32_t          joystick_raw_full_pos;
    int32_t          joystick_raw_full_neg;
    uint32_t         joystick_raw_deadband;

    /* Limits — CMC-side. Setters also fire motor_od_proxy_write_*.
     * Position limits carry the CMC's +/-INFINITY convention (means "unset");
     * conversion to the motor's finite/0-disabled form happens at the write. */
    float            velocity_limit_rad_s;
    float            position_limit_lo_rad;
    float            position_limit_hi_rad;
    float            accel_limit_rad_s2;

    /* Cached from the v4 cyclic status header (REQ-0013/ADR-033). */
    uint16_t         movement_status;

    /* 0x3044 axis_holding_enable — CMC-owned, replaces motor's 0x2300:9.
     * 1 = transition to HOLD on op release, 0 = transition to OFF. Applied
     * uniformly across JOYSTICK / SHOT_RECALL / HOMING exits. See
     * apply_op_release_hold() and tick_active_op. */
    uint8_t          holding_enable;

    /* 0x3045 axis_hold_dwell_ms — JOYSTICK op-release quiescent hold in ms.
     * After both joystick_value = 0 AND motor MOVING cleared, wait this
     * long before firing apply_op_release_hold. Debounces brief flap-back
     * gestures. Range 0..65535; 0 = instant release. Default 200. */
    uint16_t         hold_dwell_ms;
} axis_t;

static axis_t s_axis;

/*----------------------------------------------------------------------------
 * Operation arbitration state
 *
 * s_active_op is the currently-in-flight operation family (or NONE for
 * idle). Only cross-family transitions arbitrate; same-family requests
 * pass through as retarget. Completion detection lives in tick_active_op
 * and clears s_active_op back to NONE.
 *---------------------------------------------------------------------------*/
static axis_operation_t s_active_op                 = AXIS_OPERATION_NONE;
static uint32_t         s_active_op_started_ms      = 0u;   /* time_ms() of NONE->op transition, for logging */
static uint32_t         s_joystick_quiescent_since_ms = 0u; /* 0 = not quiescent; else first tick both stopped */
/* Dwell before releasing the JOYSTICK op moved to s_axis.hold_dwell_ms
 * (0x3045 axis_hold_dwell_ms, PERSIST). Default 200 ms, operator-tunable. */

/*----------------------------------------------------------------------------
 * Auto-fault-clear state (referenced in the tick)
 *
 * If the motor sits in AXIS_STATE_FAULT for AUTO_FAULT_CLEAR_DELAY_MS, we
 * pulse a clear ourselves. Counter exposed RO via OD 0x3014.
 *---------------------------------------------------------------------------*/
#define AUTO_FAULT_CLEAR_DELAY_MS  5000u
static uint32_t s_fault_active_since_ms;
static uint16_t s_auto_fault_clear_count;

/*----------------------------------------------------------------------------
 * Setup snapshot (what the motor MCU has confirmed via OD_WRITE_RESP OK).
 *
 * v3: velocity_setpoint is streamed in the cyclic, not SDO-written, so it's
 * not in this snapshot. joystick_scale also not present — the joystick
 * concept lives entirely on the CMC.
 *---------------------------------------------------------------------------*/
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
 * from CAMERAD panel STOP key (via cmc_state). */
#define HALT_PULSE_CYCLES           3u
static uint8_t s_halt_remaining;

/* FAULT_RESET pulse — same shape and same reason as NEW_SETPOINT / HALT. */
#define FAULT_RESET_PULSE_CYCLES    3u
static uint8_t s_fault_reset_remaining;

/*============================================================================
 * OPERATION ARBITRATION (SECTION 2)
 *
 * The op arbiter is the ONE piece of "which subsystem owns the axis right
 * now" logic. Every operation start goes through try_begin_op — HOMING via
 * home_sequencer_request_home, SHOT_RECALL via cmc_state, JOYSTICK via
 * cmc_state_handle_movement and buttons_jog. Every stop goes through
 * stop_op or set_active_op(NONE).
 *
 * Kept in axis_manager (not extracted) because moving it out would mean
 * op_arbiter would need to know about home_sequencer, buttons_jog, AND
 * cmc_state — more coupling than the extract saves.
 *
 * If you add a new op family: enum entry, try_begin_op is agnostic, but
 * stop_op needs a case for the abort primitive and tick_active_op needs
 * a case for completion detection.
 *============================================================================*/

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
    /* Cross-family conflict — reject. Operator must issue STOP first. */
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
     * shot recall waiting behind the block can proceed on the next tick. */
    switch (s_active_op) {
        case AXIS_OPERATION_HOMING:
            home_sequencer_abort_best_effort();
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

/* Called on every op-family release (JOYSTICK / SHOT_RECALL / HOMING → NONE).
 * Applies the CMC-owned holding_enable rule:
 *   1 → transition op_mode to HOLD  (motor halts + holds at zero velocity)
 *   0 → transition op_mode to OFF   (drive disabled, back-drivable)
 * See 0x3044 axis_holding_enable in Interface/mc_if_od.h and the design
 * rationale in Interface/CHANGELOG.md [5.5.0]. */
static void apply_op_release_hold(axis_operation_t releasing_op)
{
    axis_op_mode_t target = s_axis.holding_enable ? AXIS_OP_MODE_HOLD
                                                  : AXIS_OP_MODE_OFF;
    if (s_axis.op_mode_commanded == target) return;    /* already there */
    LOG_INFO("axis_manager: %s released -> op_mode %s -> %s (holding_enable=%u)",
             op_name(releasing_op),
             axis_mode_name(s_axis.op_mode_commanded),
             axis_mode_name(target),
             (unsigned)s_axis.holding_enable);
    /* Route through the setter so the edge-detect log line + validation
     * fire like any other mode change. */
    (void)axis_manager_set_op_mode((uint8_t)target);
}

/* Completion detection for the active operation. Runs once per axis_manager
 * tick; clears s_active_op back to NONE when the current op finishes on its
 * own terms. */
#define AXIS_OP_ARRIVAL_GRACE_MS  100u  /* mirrors cmc_state ARRIVAL_GRACE_MS (ADR-033) */
static void tick_active_op(void)
{
    switch (s_active_op) {
        case AXIS_OPERATION_NONE:
            return;
        case AXIS_OPERATION_HOMING:
            /* Home sequencer transitions to a terminal state; either DONE
             * or FAILED means the operation is complete. */
            if (home_sequencer_is_terminal_or_idle()) {
                set_active_op(AXIS_OPERATION_NONE);
                apply_op_release_hold(AXIS_OPERATION_HOMING);
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
                apply_op_release_hold(AXIS_OPERATION_SHOT_RECALL);
            }
            return;
        }
        case AXIS_OPERATION_JOYSTICK: {
            /* Quiescent = stick centred AND motor stopped. Wait
             * s_axis.hold_dwell_ms of continuous quiescence before
             * releasing so a mid-hold flap doesn't churn active_op.
             * Dwell is operator-tunable via 0x3045 axis_hold_dwell_ms. */
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
            if (time_elapsed_ms(s_joystick_quiescent_since_ms) >= s_axis.hold_dwell_ms) {
                set_active_op(AXIS_OPERATION_NONE);
                apply_op_release_hold(AXIS_OPERATION_JOYSTICK);
            }
            return;
        }
        default:
            return;
    }
}

/*============================================================================
 * SCALING HELPERS (SECTION 3)
 *
 * clamp_* / to_scaled_* / joystick profile scale / joystick recompute.
 * Kept together because they're all "convert operator/CMC units → wire
 * units" utilities used by compute_desired and setters.
 *============================================================================*/

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
 * over stick sensitivity. */
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
 * value or any cal entry changes. */
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
 * don't drive a streaming velocity. */
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

/*============================================================================
 * PERSISTENCE (SECTION 4)
 *
 * Subset of s_axis (+ home_on_boot from home_sequencer, +led_rgb from
 * led_indicator) that survives reboot. Anything PERSIST-flagged in the
 * CMC OD lives here.
 *
 * APPEND-ONLY. Loads use persist_load_or_upgrade which accepts any
 * on-flash version <= current and zero-fills the appended tail. Zeroed
 * fields are sanitised in apply_persist_blob into their real defaults.
 *
 *   v3 (2026-06-25): removed payload_weight_kg (pre-upgrade-path — v2 rejected).
 *   v4 (2026-06-26): appended 3 bytes of LED indicator colour (R/G/B).
 *   v5 (2026-06-30): appended 1 byte axis_role (0x3070 CAMERAD_AXIS_*).
 *   v6 (2026-07-09): appended 1 byte home_on_boot (0x3043).
 *   v7 (2026-07-22): appended 1 byte holding_enable (0x3044). v6 blobs load
 *                    with the appended byte = 0 → apply-time promotes to
 *                    the real default (1). No re-Save required.
 *   v8 (2026-07-22): appended 2 bytes hold_dwell_ms (0x3045). v7 blobs load
 *                    with the appended U16 = 0 → apply-time promotes to
 *                    the default (200). No re-Save required.
 *============================================================================*/
#define AXIS_PERSIST_VERSION   8u

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
    uint8_t  holding_enable;             /* v7: 0x3044 axis_holding_enable */
    uint16_t hold_dwell_ms;              /* v8: 0x3045 axis_hold_dwell_ms */
} axis_persist_blob_t;

static void apply_persist_blob(const axis_persist_blob_t *b)
{
    /* joystick_max_velocity in the persist blob is legacy — the v4 layout
     * carried it as an operator-settable field. As of 4.8.0 it's derived,
     * so we IGNORE b->joystick_max_velocity on load and recompute after
     * the limit + profile have been applied. */
    s_axis.joystick_raw_center   = b->joystick_raw_center;
    s_axis.joystick_raw_full_pos = b->joystick_raw_full_pos;
    s_axis.joystick_raw_full_neg = b->joystick_raw_full_neg;
    s_axis.joystick_raw_deadband = b->joystick_raw_deadband;
    s_axis.velocity_limit_rad_s  = b->velocity_limit_rad_s;
    s_axis.position_limit_lo_rad = b->position_limit_lo_rad;
    s_axis.position_limit_hi_rad = b->position_limit_hi_rad;
    s_axis.accel_limit_rad_s2    = b->accel_limit_rad_s2;
    /* v5: axis_role. Sanitize on load — an invalid saved value (e.g. 0,
     * or something outside the 8 CAMERAD_AXIS_* bits) falls back to PAN. */
    s_axis.axis_role = b->axis_role;
    if (s_axis.axis_role == 0u) s_axis.axis_role = 0x01u;  /* PAN default */
    /* v6: home_on_boot lives in home_sequencer; setter clamps to 0/1. */
    (void)home_sequencer_set_home_on_boot(b->home_on_boot);
    /* v7: holding_enable. Verbatim copy with 0/1 clamp — an EXPLICIT
     * operator zero (drive OFF after op release) must be respected. The
     * v6→v7 upgrade case where the tail byte was zero-fill (never
     * written by the operator) is handled by axis_manager_init using
     * flash_ver, which promotes to the real default (1) only when we
     * know the blob was actually upgraded. */
    s_axis.holding_enable = (b->holding_enable != 0u) ? 1u : 0u;
    /* v8: hold_dwell_ms. Verbatim copy — an EXPLICIT operator zero
     * (instant release, no dwell) is a legitimate setting. The v7→v8
     * upgrade case where the tail bytes are zero-fill is handled by
     * axis_manager_init using flash_ver, which promotes to the default
     * (200) only when we know the blob was actually upgraded. */
    s_axis.hold_dwell_ms  = b->hold_dwell_ms;
    led_indicator_apply_persist(b->led_rgb);
    recompute_joystick_max_velocity();
}

static void capture_persist_blob(axis_persist_blob_t *b)
{
    /* Write the current derived value into the legacy slot so a v4 reader
     * still gets something meaningful (backwards compat during a rollback). */
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
    b->axis_role      = s_axis.axis_role;
    b->home_on_boot   = home_sequencer_get_home_on_boot();
    b->holding_enable = s_axis.holding_enable;
    b->hold_dwell_ms  = s_axis.hold_dwell_ms;
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

/*============================================================================
 * BOOTSYNC CALLBACK (SECTION 5)
 *
 * motor_od_proxy calls this when it just finished reading a CATEGORY-2 limit
 * from the motor's OD. We update the local s_axis mirror so compute_desired
 * clamps against fresh values. The 0-means-disabled motor convention gets
 * translated back into the CMC's +/-INFINITY convention here for position
 * limits (so the CMC-side clamp_range is permissive when motor has no limit).
 *============================================================================*/
static void on_motor_bootsync(motor_od_proxy_bootsync_id_t which, float motor_v)
{
    switch (which) {
    case MOTOR_OD_PROXY_BOOTSYNC_VEL_LIMIT:
        s_axis.velocity_limit_rad_s = motor_v;
        recompute_joystick_max_velocity();
        break;
    case MOTOR_OD_PROXY_BOOTSYNC_ACCEL_LIMIT:
        s_axis.accel_limit_rad_s2 = motor_v;
        break;
    case MOTOR_OD_PROXY_BOOTSYNC_POS_LIMIT_LO:
        s_axis.position_limit_lo_rad = (motor_v == 0.0f) ? -INFINITY : motor_v;
        break;
    case MOTOR_OD_PROXY_BOOTSYNC_POS_LIMIT_HI:
        s_axis.position_limit_hi_rad = (motor_v == 0.0f) ? +INFINITY : motor_v;
        break;
    default:
        break;
    }
}

/*============================================================================
 * LIFECYCLE (SECTION 6)
 *
 * Init order:
 *   1. Sub-module inits (each puts its own statics at IDLE with sane defaults).
 *   2. Register the bootsync callback so motor_od_proxy can notify us.
 *   3. Zero + populate s_axis with coded factory defaults.
 *   4. Try to load persist — successful upgrade preserves calibration.
 *============================================================================*/
void axis_manager_init(void)
{
    home_sequencer_init();
    motor_od_proxy_init();
    motor_od_proxy_register_bootsync_cb(&on_motor_bootsync);
    buttons_jog_init();

    memset(&s_axis,    0, sizeof(s_axis));
    memset(&s_applied, 0, sizeof(s_applied));
    s_axis.state             = AXIS_STATE_DISABLED;
    s_axis.op_mode_actual    = AXIS_OP_MODE_OFF;
    s_axis.op_mode_commanded = AXIS_OP_MODE_OFF;
    /* Pre-authorise enable at boot. compose_cyclic_cmd gates the ENABLE bit
     * only when mode != OFF, so the drive still sits disabled here (safe:
     * motor spins freely). The moment mode leaves OFF the drive activates. */
    s_axis.enable_latch      = true;

    s_axis.joy_profile            = AXIS_JOY_PROFILE_NORMAL;
    s_axis.axis_role              = 0x01u;                    /* CAMERAD_AXIS_PAN */
    s_axis.holding_enable         = 1u;                       /* default: HOLD after op release; operator can flip to OFF via 0x3044 */
    s_axis.hold_dwell_ms          = 200u;                     /* default: 200 ms quiescent hold before JOYSTICK op releases (0x3045) */
    s_axis.velocity_limit_rad_s   = 10.0f;
    recompute_joystick_max_velocity();   /* derived — needs velocity_limit + profile set first */
    s_axis.position_limit_lo_rad  = -INFINITY;
    s_axis.position_limit_hi_rad  = +INFINITY;
    s_axis.accel_limit_rad_s2     = 100.0f;

    /* Joystick cal defaults: matched to CAMERAD MOVEMENT's signed-int8
     * wire range (-127..+127, 0 = centered). */
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

    /* Attempt to load persisted config from flash. persist_load_or_upgrade
     * accepts any on-flash version <= AXIS_PERSIST_VERSION and zeros the
     * tail — so a v4/v5 blob loads with axis_role/home_on_boot defaulted
     * to 0, then apply_persist_blob sanitises those to real defaults. */
    axis_persist_blob_t blob;
    size_t loaded = 0;
    uint16_t flash_ver = 0;
    if (persist_load_or_upgrade(PERSIST_REGION_CONFIG, &blob, sizeof(blob),
                                AXIS_PERSIST_VERSION, &loaded, &flash_ver)) {
        apply_persist_blob(&blob);
        /* v6 blobs upgraded to v7 have holding_enable zero-filled by the
         * append-tail upgrade path. That zero is NOT an operator setting —
         * it's the absence of one — so promote to the default (1 = HOLD).
         * A blob explicitly saved at v7 with holding_enable=0 preserves
         * the operator's intent verbatim. */
        if (flash_ver < 7u) {
            s_axis.holding_enable = 1u;
        }
        /* v7 blobs upgraded to v8 have hold_dwell_ms zero-filled — that
         * zero would mean "instant release, no debounce," which is a
         * legitimate operator choice for a later save but NOT what we want
         * to inherit from an unset field. Promote to the default (200). */
        if (flash_ver < 8u) {
            s_axis.hold_dwell_ms = 200u;
        }
        if (flash_ver == AXIS_PERSIST_VERSION) {
            LOG_INFO("axis_manager: ready (v%u, persisted config loaded). state=DISABLED",
                     (unsigned)flash_ver);
        } else {
            LOG_INFO("axis_manager: ready (loaded v%u -> v%u in-memory; Save to persist). "
                     "state=DISABLED",
                     (unsigned)flash_ver, (unsigned)AXIS_PERSIST_VERSION);
        }
    } else {
        LOG_INFO("axis_manager: ready (v%u, factory defaults). state=DISABLED",
                 (unsigned)AXIS_PERSIST_VERSION);
    }
}

/*============================================================================
 * DESIRED-SETUP SNAPSHOT (SECTION 7)
 *
 * Convert current s_axis fields into the motor-side scaled values that the
 * setup-sequencer will chase via SDO writes.
 *============================================================================*/
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

    out->target_torque_scaled        = to_scaled_i32(s_axis.target_current_a, MC_IF_CUR_SCALE);
    out->profile_velocity_scaled     = to_scaled_u32(s_axis.velocity_limit_rad_s, MC_IF_VEL_SCALE);
    out->profile_acceleration_scaled = to_scaled_u32(s_axis.accel_limit_rad_s2,   MC_IF_ACC_SCALE);
    out->profile_deceleration_scaled = to_scaled_u32(s_axis.accel_limit_rad_s2,   MC_IF_ACC_SCALE);
}

/*============================================================================
 * SDO SETUP-SEQUENCER (SECTION 8)
 *
 * One-in-flight pipeline that walks the desired-vs-applied diff and issues
 * SDO writes to bring the motor MCU's OD in sync with what we want. Runs
 * every tick; cooperates with motor_od_proxy + home_sequencer via the
 * cia402 slot.
 *============================================================================*/
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
        static uint32_t s_failures = 0;
        if (s_failures < 8u) {
            LOG_WARN("axis_manager: SDO sdo done step=%s FAIL result=0x%02X (will retry)",
                     seq_step_name((int)s_seq_step), (unsigned)res);
            s_failures++;
        }
        return;
    }

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

static seq_step_t start_sdo_write(seq_step_t step, uint16_t idx, uint8_t sub,
                                  MC_IfOdType_t type, const void *data, uint8_t len)
{
    cia402_od_handle_t h = cia402_od_write_begin(idx, sub, type, data, len);
    if (h == CIA402_OD_HANDLE_INVALID) {
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

    if (s_seq_step != SEQ_IDLE) return;

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
}

/*============================================================================
 * CYCLIC COMMAND COMPOSITION (SECTION 9)
 *
 * Every tick: build the 8-byte cyclic command frame that cia402 pushes to
 * the motor MCU next SPI cycle. Contains controlword + velocity_setpoint
 * + command_counter.
 *============================================================================*/
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

    /* Clear-fault: rising edge for a multi-cycle pulse so cia402 OD traffic
     * interleaving doesn't drop the bit before the motor sees it. */
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
        s_new_setpoint_remaining = 0;
        s_halt_remaining         = 0;
        s_fault_reset_remaining  = 0;
        out->controlword         = cw;
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

    /* Operator-driven HALT (from CAMERAD panel STOP key via cmc_state). */
    if (s_halt_remaining > 0) {
        cw |= MC_IF_CW_HALT;
        s_halt_remaining--;
    }

    /* Stream the live velocity demand. */
    if (s_axis.enable_latch && s_axis.state != AXIS_STATE_FAULT) {
        float v = current_velocity_demand_rad_s();
        out->velocity_setpoint = to_scaled_i32(v, MC_IF_VEL_SCALE);
    }

    /* NEW_SETPOINT: fires only when setup is fully applied. */
    if (s_axis.start_move_pulse) {
        if (!setup_sequencer_busy()) {
            s_new_setpoint_remaining = NEW_SETPOINT_PULSE_CYCLES;
            s_axis.start_move_pulse  = false;
            LOG_INFO("axis_manager: NEW_SETPOINT triggered (setup applied, sending pulse for %u cycles)",
                     (unsigned)NEW_SETPOINT_PULSE_CYCLES);
        }
    }
    if (s_new_setpoint_remaining > 0) {
        cw |= MC_IF_CW_NEW_SETPOINT;
        s_new_setpoint_remaining--;
    }

    out->controlword = cw;
}

/*============================================================================
 * TICK (SECTION 10)
 *
 * Reads status → runs sub-modules → drives cyclic command. Every sub-module
 * that touches the cia402 SDO slot lives inside the !motor_in_bl guard;
 * setup_sequencer_tick runs unconditionally because it self-cleans if its
 * writes get NO_OBJECT'd by the bootloader.
 *============================================================================*/

static void reset_motor_od_submodules(void)
{
    /* Called on the rising edge of "motor entered bootloader" — every
     * sub-module that holds a cia402 handle MUST reset here. */
    home_sequencer_reset();
    motor_od_proxy_reset();
}

void axis_manager_tick(void)
{
    /* 1. Pull latest status from cia402 (peek — non-consuming). */
    MC_IfCyclicStatusHeader_t hdr;
    uint8_t blob[MC_IF_TLM_BLOB_MAX];
    uint8_t blob_len = 0;
    if (cia402_peek_cyclic_status(&hdr, blob, &blob_len)) {
        uint16_t       prev_error    = s_axis.error_code;
        axis_state_t   prev_state    = s_axis.state;
        axis_op_mode_t prev_op_mode  = s_axis.op_mode_actual;

        s_axis.error_code     = hdr.error_code;
        s_axis.error_register = (uint8_t)(hdr.error_code >> 8);

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

        /* Auto-fault-clear: 5 s dwell → pulse FAULT_RESET, retry every 5 s
         * while fault persists. */
        if (s_axis.state == AXIS_STATE_FAULT) {
            if (s_fault_active_since_ms == 0u) {
                s_fault_active_since_ms = time_ms();
            } else if (time_elapsed_ms(s_fault_active_since_ms) >= AUTO_FAULT_CLEAR_DELAY_MS) {
                s_axis.clear_fault_pulse = true;
                s_auto_fault_clear_count++;
                s_fault_active_since_ms  = time_ms();
                LOG_WARN("axis_manager: auto-clearing fault (err=0x%04X, count now %u)",
                         (unsigned)s_axis.error_code, (unsigned)s_auto_fault_clear_count);
            }
        } else if (s_fault_active_since_ms != 0u) {
            s_fault_active_since_ms = 0u;
        }
        if (prev_op_mode != s_axis.op_mode_actual) {
            LOG_INFO("axis_manager: op_mode_actual %s -> %s (mode_display=%d)",
                     axis_mode_name(prev_op_mode), axis_mode_name(s_axis.op_mode_actual),
                     (int)hdr.mode_display);
        }

        s_axis.position_actual_rad = (float)hdr.position_actual_scaled * MC_IF_POS_SCALE;

        uint16_t prev_move = s_axis.movement_status;
        s_axis.movement_status = hdr.movement_status;
        if (prev_move != s_axis.movement_status) {
            LOG_INFO("axis_manager: movement_status 0x%04X -> 0x%04X (moving=%d on_target=%d)",
                     (unsigned)prev_move, (unsigned)s_axis.movement_status,
                     (int)((s_axis.movement_status & MC_IF_MOVE_MOVING)    != 0),
                     (int)((s_axis.movement_status & MC_IF_MOVE_ON_TARGET) != 0));
        }
    }

    /* 2. Detect motor-bootloader edge and reset sub-modules on entry. */
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

    /* 3. Drain sub-modules that talk to motor OD. All defer if cia402 slot
     * is busy — cooperation is line-of-sight through the cia402 API. */
    if (!motor_in_bl) {
        motor_od_proxy_tick();     /* load_factor drain, bootsync reads, proxy writes, motor save */
        home_sequencer_tick();     /* home + is_homed refresh + encoder-type + home_on_boot fire */
    }
    tick_active_op();              /* op completion detection (HOMING/SHOT_RECALL/JOYSTICK -> NONE) */

    /* 4. On-board buttons — CMC-local input, always polled. */
    buttons_jog_tick();

    /* 5. Drive the SDO setup-sequencer (single-in-flight for mode/target/profile). */
    setup_sequencer_tick();

    /* 6. Compose and push the cyclic command for the next outbound frame. */
    MC_IfCyclicCommand_t cmd;
    compose_cyclic_cmd(&cmd);
    cia402_set_cyclic_cmd(&cmd);
}

/*============================================================================
 * STATE GETTERS (SECTION 11) — back 0x30xx RO OD entries
 *============================================================================*/

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

/*============================================================================
 * COMMAND LATCHES (SECTION 12)
 *============================================================================*/

bool axis_manager_request_enable(bool enable)
{
    s_axis.enable_latch = enable;
    LOG_INFO("axis_manager: enable=%d", (int)enable);
    return true;
}

bool axis_manager_is_enable_latched(void)
{
    return s_axis.enable_latch;
}

bool axis_manager_request_quick_stop(void)
{
    s_axis.quick_stop_pulse = true;
    LOG_INFO("axis_manager: QUICK_STOP requested (hard decel + disable)");
    return true;
}

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
    return true;
}

/*============================================================================
 * MODE + TARGETS (SECTION 13) — back 0x3020..0x302C RW OD entries
 *============================================================================*/

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
    /* Op arbitration on the raw-jog entry points. Any non-zero deflection
     * is a "jog request" that must arbitrate for the JOYSTICK op family;
     * v=0 is "stop" and always writes through so a jog-then-centre gesture
     * completes normally regardless of arbitration state. See the block
     * comment on axis_manager_set_joystick_raw for the full rationale. */
    if (v != 0.0f) {
        axis_begin_result_t br = axis_manager_try_begin_op(AXIS_OPERATION_JOYSTICK);
        if (br == AXIS_BEGIN_REJECTED) return false;
        if (s_axis.op_mode_commanded != AXIS_OP_MODE_JOYSTICK) {
            (void)axis_manager_set_op_mode((uint8_t)AXIS_OP_MODE_JOYSTICK);
        }
    }
    s_axis.joystick_value = v;
    return true;
}

float axis_manager_get_joystick_max_velocity(void) { return s_axis.joystick_max_velocity; }

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

uint8_t axis_manager_get_holding_enable(void) { return s_axis.holding_enable; }
bool    axis_manager_set_holding_enable(uint8_t v)
{
    uint8_t nv = (v != 0u) ? 1u : 0u;
    if (s_axis.holding_enable != nv) {
        LOG_INFO("axis: holding_enable %u -> %u (op-release will transition to %s)",
                 (unsigned)s_axis.holding_enable, (unsigned)nv,
                 nv ? "HOLD" : "OFF");
    }
    s_axis.holding_enable = nv;
    return true;
}

uint16_t axis_manager_get_hold_dwell_ms(void) { return s_axis.hold_dwell_ms; }
bool     axis_manager_set_hold_dwell_ms(uint16_t v)
{
    if (s_axis.hold_dwell_ms != v) {
        LOG_INFO("axis: hold_dwell_ms %u -> %u",
                 (unsigned)s_axis.hold_dwell_ms, (unsigned)v);
    }
    s_axis.hold_dwell_ms = v;
    return true;
}

uint8_t axis_manager_get_axis_role(void) { return s_axis.axis_role; }
bool    axis_manager_set_axis_role(uint8_t r)
{
    /* Accept only single-bit values matching CAMERAD_AXIS_* (PAN..FADER). */
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

/* REQ-0012 — Commanded current [A] for AXIS_OP_MODE_TORQUE. */
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

/*============================================================================
 * LIMITS (SECTION 14) — back 0x3030..0x3033 RW OD entries
 *
 * The CMC-side value lives in s_axis (used by compute_desired's clamps).
 * The setter also forwards to motor_od_proxy_write_* so the motor's own
 * envelope stays in sync. Position limits carry the +/-INFINITY "unset"
 * convention on the CMC side; finite_or_zero converts to the motor's
 * 0-means-disabled form for the SDO write.
 *============================================================================*/

static float finite_or_zero(float v) { return isfinite(v) ? v : 0.0f; }

float axis_manager_get_velocity_limit(void) { return s_axis.velocity_limit_rad_s; }
bool  axis_manager_set_velocity_limit(float v)
{
    if (v < 0.0f || isnan(v)) return false;
    s_axis.velocity_limit_rad_s = v;
    recompute_joystick_max_velocity();
    return motor_od_proxy_write_vel_limit(v);
}

float axis_manager_get_position_limit_lo(void) { return s_axis.position_limit_lo_rad; }
bool  axis_manager_set_position_limit_lo(float v)
{
    if (isnan(v)) return false;
    s_axis.position_limit_lo_rad = v;
    return motor_od_proxy_write_pos_limit_lo(finite_or_zero(v));
}

float axis_manager_get_position_limit_hi(void) { return s_axis.position_limit_hi_rad; }
bool  axis_manager_set_position_limit_hi(float v)
{
    if (isnan(v)) return false;
    s_axis.position_limit_hi_rad = v;
    return motor_od_proxy_write_pos_limit_hi(finite_or_zero(v));
}

float axis_manager_get_accel_limit(void) { return s_axis.accel_limit_rad_s2; }
bool  axis_manager_set_accel_limit(float v)
{
    if (v < 0.0f || isnan(v)) return false;
    s_axis.accel_limit_rad_s2 = v;
    return motor_od_proxy_write_accel_limit(v);
}

/*============================================================================
 * SUB-MODULE FORWARDERS (SECTION 15)
 *
 * Public API entries that just call through to a sub-module. Kept as thin
 * pass-throughs so external modules don't have to know which sub-module
 * owns which OD entry.
 *============================================================================*/

/* Load factor (motor 0x2300:5, REQ-0014) → motor_od_proxy. */
float axis_manager_get_load_factor(void)         { return motor_od_proxy_get_load_factor(); }
bool  axis_manager_set_load_factor(float v)      { return motor_od_proxy_set_load_factor(v); }

/* Velocity-demand accel ramp (motor 0x2300:6/7/8) → motor_od_proxy. */
float axis_manager_get_vel_accel_up  (void)      { return motor_od_proxy_get_vel_accel_up();   }
float axis_manager_get_vel_accel_dn  (void)      { return motor_od_proxy_get_vel_accel_dn();   }
float axis_manager_get_vel_accel_jerk(void)      { return motor_od_proxy_get_vel_accel_jerk(); }
bool  axis_manager_set_vel_accel_up  (float v)   { return motor_od_proxy_set_vel_accel_up(v);   }
bool  axis_manager_set_vel_accel_dn  (float v)   { return motor_od_proxy_set_vel_accel_dn(v);   }
bool  axis_manager_set_vel_accel_jerk(float v)   { return motor_od_proxy_set_vel_accel_jerk(v); }

/* Motor save + resync (both motor_od_proxy). */
bool  axis_manager_request_motor_save  (void)    { return motor_od_proxy_request_motor_save();  }
void  axis_manager_request_motor_resync(void)    { motor_od_proxy_request_motor_resync();       }

/* Home-to-endstop + is_homed + encoder-type + home_on_boot → home_sequencer. */
bool     axis_manager_request_home           (void)     { return home_sequencer_request_home();       }
uint8_t  axis_manager_get_home_status        (void)     { return home_sequencer_get_home_status();    }
bool     axis_manager_is_homed               (void)     { return home_sequencer_is_homed();           }
bool     axis_manager_encoder_is_incremental (void)     { return home_sequencer_encoder_is_incremental(); }
uint8_t  axis_manager_get_home_on_boot       (void)     { return home_sequencer_get_home_on_boot();   }
bool     axis_manager_set_home_on_boot       (uint8_t v){ return home_sequencer_set_home_on_boot(v);  }

/*============================================================================
 * JOYSTICK RAW + CALIBRATION (SECTION 16) — back 0x3026-0x302A RW OD entries
 *
 * Each setter re-runs the calibration pipeline so joystick_value stays in
 * sync with the current (raw, cal) tuple. Live JOYSTICK-mode motion uses
 * joystick_value directly.
 *============================================================================*/

int32_t axis_manager_get_joystick_raw(void) { return s_axis.joystick_raw; }

/* Op arbitration on the raw-jog entry point.
 *
 * ANY caller of this setter — CAMERAD's cmc_state_handle_movement_scaled,
 * VISCA's cmd_pantilt_drive, buttons_jog, GUI OD-write to 0x3026, PC-tool
 * scripts, whatever — routes through the JOYSTICK op arbiter here. Without
 * this, direct OD writers (like the GUI joystick) bypass the arbiter, so
 * s_active_op never enters JOYSTICK and tick_active_op's release logic
 * never fires — meaning apply_op_release_hold's holding_enable transition
 * doesn't happen when the operator lets go. Users see holding current after
 * a GUI jog even with holding_enable=0. Placing arbitration here makes all
 * three joystick sources equivalent.
 *
 * v = 0 is a "stop" / "centre stick" write and is ALWAYS allowed through
 * without re-arbitrating — a jog-then-centre gesture completes normally,
 * and the quiescent timer in tick_active_op then releases the op naturally.
 *
 * Callers who already arbitrate themselves (cmc_state_handle_movement_scaled,
 * buttons_jog) will hit try_begin_op twice in quick succession — the second
 * returns CONTINUED (same-family) which is a harmless no-op. Removing the
 * caller-side arbitration is a valid follow-up cleanup but not required for
 * correctness. */
bool    axis_manager_set_joystick_raw(int32_t v)
{
    if (v != 0) {
        axis_begin_result_t br = axis_manager_try_begin_op(AXIS_OPERATION_JOYSTICK);
        if (br == AXIS_BEGIN_REJECTED) return false;
        if (s_axis.op_mode_commanded != AXIS_OP_MODE_JOYSTICK) {
            (void)axis_manager_set_op_mode((uint8_t)AXIS_OP_MODE_JOYSTICK);
        }
    }
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
