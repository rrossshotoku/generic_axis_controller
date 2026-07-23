/*
 * app/axis_manager — high-level motor control surface for the network MCU.
 *
 * Owns the data that backs the CMC-side OD entries (0x3000-0x303F per
 * Interface/mc_if_od.h). The OD entries ARE this module's public API:
 * every protocol module (camerad, visca, the PC tool's OD writer, future
 * additions) drives the motor by reading/writing these entries.
 * `axis_manager` translates them into cyclic commands for `cia402` to
 * send to the motor MCU.
 *
 * For v1 (this skeleton): the data model is in place and the getters/
 * setters that back the OD entries work. The actuals (state, position,
 * velocity, error_code) are derived from the latest cia402_peek_cyclic_status.
 * The cyclic-command construction (op_mode + targets -> controlword +
 * targets in MC_IfCyclicCommand_t) is NOT YET WIRED — cia402 still sends
 * its quiescent default. That's the next phase.
 *
 * Layering: depends on cia402 (peek + future cmd push), bsp/time, Interface.
 * Does NOT include any protocol-module headers. Protocol modules call this
 * (or, equivalently, the OD entries it backs) only.
 */

#ifndef APP_AXIS_MANAGER_H
#define APP_AXIS_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#include "Interface/mc_if_od.h"
#include "Interface/mc_if_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Axis state (matches OD 0x3000 and Interface MC_IF_AXIS_STATE_* values). */
typedef enum {
    AXIS_STATE_DISABLED = MC_IF_AXIS_STATE_DISABLED,
    AXIS_STATE_READY    = MC_IF_AXIS_STATE_READY,
    AXIS_STATE_RUNNING  = MC_IF_AXIS_STATE_RUNNING,
    AXIS_STATE_FAULT    = MC_IF_AXIS_STATE_FAULT,
} axis_state_t;

/* Operating mode (matches OD 0x3020 and Interface MC_IF_AXIS_MODE_* values). */
typedef enum {
    AXIS_OP_MODE_OFF              = MC_IF_AXIS_MODE_OFF,
    AXIS_OP_MODE_JOYSTICK         = MC_IF_AXIS_MODE_JOYSTICK,
    AXIS_OP_MODE_PROFILE_VELOCITY = MC_IF_AXIS_MODE_PROFILE_VELOCITY,
    AXIS_OP_MODE_PROFILE_POSITION = MC_IF_AXIS_MODE_PROFILE_POSITION,
    AXIS_OP_MODE_HOLD             = MC_IF_AXIS_MODE_HOLD,
    AXIS_OP_MODE_TORQUE           = MC_IF_AXIS_MODE_TORQUE,
} axis_op_mode_t;

/* Operation-level arbitration state. Layered on top of op_mode: the axis
 * admits at most one cross-family operation at a time. Same-family requests
 * pass through as "retarget" (a second joystick trim updates the demand; a
 * second shot recall retargets the destination). STOP is always accepted
 * (routes through axis_manager_stop_op regardless of state).
 *
 * Design intent: prevents CAMERAD MOVEMENT / shot-recall / home-start
 * requests from overwriting each other on the SDO channel. Example: a
 * MOVEMENT arriving mid-home used to write 0x6060 = PROFILE_VELOCITY,
 * aborting the motor's HOMING trajectory silently. With arbitration it
 * gets rejected + logged instead. */
typedef enum {
    AXIS_OPERATION_NONE        = 0,   /* idle — any op may start */
    AXIS_OPERATION_HOMING      = 1,   /* home-to-endstop in flight */
    AXIS_OPERATION_SHOT_RECALL = 2,   /* PROFILE_POSITION move to a shot in flight */
    AXIS_OPERATION_JOYSTICK    = 3,   /* JOYSTICK demand active (any source) */
} axis_operation_t;

/* Result of axis_manager_try_begin_op — tells the caller whether it needs
 * to do the first-entry setup work (e.g. SDO write of op_mode), just
 * retarget the existing operation, or bail out entirely. */
typedef enum {
    AXIS_BEGIN_STARTED   = 0,   /* transitioned NONE -> op; caller does first-entry setup */
    AXIS_BEGIN_CONTINUED = 1,   /* was already in op; caller just retargets */
    AXIS_BEGIN_REJECTED  = 2,   /* incompatible with active op; caller must bail */
} axis_begin_result_t;

void axis_manager_init(void);
void axis_manager_tick(void);

/* Operation arbitration. Route every "start operation X" entry point through
 * try_begin_op — the return value tells you what to do next. Route every
 * stop (KEY_STOP, disable, quick_stop) through stop_op — it always succeeds
 * and issues the appropriate motor-side stop primitive for the active op. */
axis_operation_t    axis_manager_get_active_op    (void);         /* 0x3006 */
axis_begin_result_t axis_manager_try_begin_op     (axis_operation_t desired);
void                axis_manager_stop_op          (void);

/* Motor-MCU movement_status from the v4 cyclic header (REQ-0013/ADR-033).
 *
 * The motor MCU is authoritative for "is it moving?" and "is it on target?".
 * cmc_state uses these for CAMERAD POLL response status bits (`moving`,
 * `on_shot`). Returns the raw u16 (MC_IF_MOVE_* bitmask) so callers can
 * AND with whatever bits they want. */
uint16_t axis_manager_get_movement_status(void);
bool     axis_manager_is_moving         (void);
bool     axis_manager_is_on_target      (void);

/* Persist the PERSIST-flagged CMC OD entries (joystick cal + motion limits)
 * to the CMC's internal flash via app/persist. Returns true on success.
 * Blocks for ~30 ms during the flash erase/program. Called from cmc_od
 * when the operator writes MC_IF_SAVE_MAGIC to OD 0x3050 cmc_save_config. */
bool axis_manager_save_to_flash(void);

/*----------------------------------------------------------------------------
 * Per-OD-entry accessors. Called by cmc_od to back the dispatch table for
 * 0x3000..0x303F. These mirror the OD layout one-to-one.
 *
 * Setters return false on invalid input (e.g. unknown axis_op_mode value);
 * the caller (cmc_od) translates that into MC_IF_OD_ERR_RANGE.
 *---------------------------------------------------------------------------*/

/* --- 0x3000-0x300F state (RO) --- */
axis_state_t    axis_manager_get_state              (void);    /* 0x3000 */
axis_op_mode_t  axis_manager_get_op_mode_actual     (void);    /* 0x3001 */
float           axis_manager_get_position_actual    (void);    /* 0x3002 rad */
float           axis_manager_get_velocity_actual    (void);    /* 0x3003 rad/s */
uint16_t        axis_manager_get_error_code         (void);    /* 0x3004 */
uint8_t         axis_manager_get_error_register     (void);    /* 0x3005 */
uint16_t        axis_manager_get_auto_fault_clears  (void);    /* 0x3014 — since-boot count of auto-cleared faults */

/* --- Home-to-endstop (0x3040 command, 0x3041 status, 0x3042 is_homed) ---
 * axis_manager runs the sequence against the motor's own homing entries
 * (0x2700:8/9 + fault_flags at 0x2600:1). Shot recalls in cmc_state are
 * gated on axis_manager_is_homed(). */
bool     axis_manager_request_home              (void);        /* 0x3040 write 1 */
uint8_t  axis_manager_get_home_status           (void);        /* 0x3041 — MC_IF_HOME_* mirror */
bool     axis_manager_is_homed                  (void);        /* 0x3042 — fault_flags & NOT_HOMED == 0 */
bool     axis_manager_encoder_is_incremental    (void);        /* true iff NOT_HOMED bit ever observed (incremental encoder) */

/* 0x3043 axis_home_on_boot — persisted flag; when 1, axis_manager fires a
 * home command once per boot as soon as the motor's encoder type is known
 * and reports incremental. Motor-in-bootloader defers the fire (checked
 * again when the motor rejoins the app). */
uint8_t  axis_manager_get_home_on_boot          (void);        /* 0x3043 */
bool     axis_manager_set_home_on_boot          (uint8_t v);   /* 0x3043 (0/1) */

/* 0x3044 axis_holding_enable — CMC-owned idle-behaviour selector.
 * On any op release (JOYSTICK/SHOT_RECALL/HOMING → NONE): 1 → transition
 * to HOLD (motor decelerates via HALT + holds at zero velocity);
 * 0 → transition to OFF (drive fully disabled, back-drivable). Choose 0
 * for actuators with stiction that drift under small residual voltage
 * (drive-off is the only quiet state); choose 1 for anything that needs
 * to resist backdrive. Supersedes motor-owned 0x2300:9 (deprecated). */
uint8_t  axis_manager_get_holding_enable        (void);        /* 0x3044 */
bool     axis_manager_set_holding_enable        (uint8_t v);   /* 0x3044 (0/1) */

/* 0x3045 axis_hold_dwell_ms — how long the JOYSTICK op stays quiescent
 * (joystick_value = 0 AND motor MOVING cleared) before we release it +
 * apply the holding_enable transition. Debounces brief flap-back
 * gestures. Range 0..65535 ms; 0 = instant release. Default 200. */
uint16_t axis_manager_get_hold_dwell_ms         (void);        /* 0x3045 */
bool     axis_manager_set_hold_dwell_ms         (uint16_t v);  /* 0x3045 */

/* --- 0x3010-0x301F commands (write-triggered) --- */
bool  axis_manager_request_enable           (bool enable);  /* 0x3010 write 1/0 */
bool  axis_manager_request_quick_stop       (void);         /* 0x3011 write 1 — hard decel + disable */
bool  axis_manager_request_clear_fault      (void);         /* 0x3012 write 1 */
bool  axis_manager_request_start_move       (void);         /* 0x3013 write 1 — triggers NEW_SETPOINT once SDO setup is fully applied */

/* Operator-driven controlled-stop. Latches a HALT bit in the motor's
 * controlword (CiA-402 bit 8). Motor decelerates per its profile and
 * holds position (still enabled). Cleared automatically on the next
 * axis_manager_request_start_move so subsequent CUT/FADE Just Works.
 * Used by cmc_state_stop_movement (panel STOP keys). */
bool  axis_manager_request_halt             (void);

/* Internal (sub-module use): read the raw enable_latch (what the CMC last
 * commanded, before the motor's own statusword confirms). Distinct from
 * axis_manager_get_state which reflects the motor's PDS state. Used by
 * motor_od_proxy's motor-save sequencer to snapshot operator intent so
 * re-enable after save matches "what operator wanted", not "what motor
 * was reporting" (a mid-fault save should still restore operator intent). */
bool  axis_manager_is_enable_latched        (void);

/* --- 0x3020-0x302F mode + per-mode targets --- */
axis_op_mode_t  axis_manager_get_op_mode             (void);
bool            axis_manager_set_op_mode             (uint8_t v);    /* 0x3020 */

float  axis_manager_get_joystick_value               (void);
bool   axis_manager_set_joystick_value               (float v);      /* 0x3021 */

/* 0x3022 joystick_max_velocity is RO — derived from
 * velocity_limit × joy_profile_scale. CAMERAD JOY_PROFILE_* keypresses
 * flip the profile via axis_manager_set_joy_profile below. */
float   axis_manager_get_joystick_max_velocity       (void);          /* 0x3022 (RO) */
uint8_t axis_manager_get_joy_profile                 (void);
bool    axis_manager_set_joy_profile                 (uint8_t p);     /* 0 NORMAL, 1 MEDIUM, 2 FINE */

/* 0x3070 axis_role — which CAMERAD axis this CMC consumes from MOVEMENT.
 * Value is a CAMERAD_AXIS_* bitmap value (single bit): PAN 0x01 ... FADER 0x80. */
uint8_t axis_manager_get_axis_role                   (void);          /* 0x3070 */
bool    axis_manager_set_axis_role                   (uint8_t r);     /* 0x3070 */

float  axis_manager_get_target_velocity              (void);
bool   axis_manager_set_target_velocity              (float v);      /* 0x3023 */

float  axis_manager_get_target_position              (void);
bool   axis_manager_set_target_position              (float v);      /* 0x3024 */

float  axis_manager_get_target_time                  (void);
bool   axis_manager_set_target_time                  (float v);      /* 0x3025 */

/* Commanded current [A] for AXIS_OP_MODE_TORQUE (REQ-0012). axis_manager
 * SDO-writes motor 0x6071 target_torque = round(amps / MC_IF_CUR_SCALE)
 * via the existing setup-sequencer. Effective only while op_mode_actual
 * is TORQUE on the motor side; written value is also remembered as the
 * setpoint for subsequent mode switches into TORQUE. */
float  axis_manager_get_target_current               (void);
bool   axis_manager_set_target_current               (float amps);   /* 0x302B */

/* On-board UP/DOWN button current magnitude [A]. While AXIS_OP_MODE_TORQUE
 * is active and one of the board's UP_BUTTON / DOWN_BUTTON is held,
 * axis_manager overrides target_current with the signed button current
 * (+ for UP, − for DOWN). On release, target_current goes to 0. Allows
 * a quick local-feel test of the torque loop without needing the PC
 * tool to issue continuous writes. Magnitude only — direction comes
 * from which button. Defaults to 0 A (buttons do nothing until set). */
float  axis_manager_get_button_current               (void);
bool   axis_manager_set_button_current               (float amps);   /* 0x302C */

/* --- 0x3026-0x302A joystick raw input + symmetric calibration --- */
/* Writing axis_joystick_raw triggers normalisation using the four cal entries
 * (center, full_pos, full_neg, deadband) and updates axis_joystick_value. The
 * existing axis_joystick_max_velocity (0x3022) caps the output velocity for
 * both directions (symmetric). */
int32_t  axis_manager_get_joystick_raw               (void);
bool     axis_manager_set_joystick_raw               (int32_t v);    /* 0x3026 */

int32_t  axis_manager_get_joystick_raw_center        (void);
bool     axis_manager_set_joystick_raw_center        (int32_t v);    /* 0x3027 */

int32_t  axis_manager_get_joystick_raw_full_pos      (void);
bool     axis_manager_set_joystick_raw_full_pos      (int32_t v);    /* 0x3028 */

int32_t  axis_manager_get_joystick_raw_full_neg      (void);
bool     axis_manager_set_joystick_raw_full_neg      (int32_t v);    /* 0x3029 */

uint32_t axis_manager_get_joystick_raw_deadband      (void);
bool     axis_manager_set_joystick_raw_deadband      (uint32_t v);   /* 0x302A */

/* --- 0x3030-0x303F limits --- */
float  axis_manager_get_velocity_limit               (void);
bool   axis_manager_set_velocity_limit               (float v);      /* 0x3030 */

float  axis_manager_get_position_limit_lo            (void);
bool   axis_manager_set_position_limit_lo            (float v);      /* 0x3031 */

float  axis_manager_get_position_limit_hi            (void);
bool   axis_manager_set_position_limit_hi            (float v);      /* 0x3032 */

float  axis_manager_get_accel_limit                  (void);
bool   axis_manager_set_accel_limit                  (float v);      /* 0x3033 */

/* --- Load factor (motor-owned 0x2300:5 vel_load_factor, REQ-0014) ---
 * Operator-tunable multiplier on the motor's velocity-loop kp/ki.
 * Range 0.3 (light load) .. 2.0 (heavy load); default 1.0 (no scaling).
 * The setter caches locally AND fires an ad-hoc SDO write to the motor
 * MCU's 0x2300:5; the getter returns the cached value (so the web GET
 * doesn't need an SDO read round-trip per page-load). */
float  axis_manager_get_load_factor                  (void);
bool   axis_manager_set_load_factor                  (float v);

/* --- Velocity-demand accel ramp (motor-owned 0x2300:6/7/8, CHANGELOG [4.5.0]) ---
 * Joystick-feel parameters: the motor slews its PROFILE_VELOCITY demand
 * toward the live setpoint under acceleration caps. accel_up rad/s² while
 * speeding up, accel_dn rad/s² while slowing down; accel_jerk rad/s³
 * eases the acceleration UP to the cap (smoother start, no overshoot at
 * the end since it falls freely). 0 disables that phase (default).
 *
 * Same proxy pattern as load_factor: setter caches locally + fires an
 * ad-hoc SDO write to the motor; getter returns the cache (no SDO read
 * round-trip per web GET). NOT persisted on the CMC side — these are
 * stored in the motor's flash via the motor-save sequencer kicked off by
 * cmc_save_config. */
float  axis_manager_get_vel_accel_up                 (void);
bool   axis_manager_set_vel_accel_up                 (float rad_s2);
float  axis_manager_get_vel_accel_dn                 (void);
bool   axis_manager_set_vel_accel_dn                 (float rad_s2);
float  axis_manager_get_vel_accel_jerk               (void);
bool   axis_manager_set_vel_accel_jerk               (float rad_s3);

/* --- Motor-save sequencer ---
 * Kicks off the disable -> wait -> SDO save -> wait -> re-enable sequence
 * that persists motor-owned settings (load_factor, vel_accel_*, brushed
 * gains, position cal, etc.) to the motor's flash. The motor can only
 * commit a save while the power stage is OFF (ADR-010) so axis_manager
 * brackets the SDO write with disable + re-enable. Re-enable is gated on
 * the pre-save enable state — operators who had the motor intentionally
 * disabled don't suddenly find it re-enabled.
 *
 * Non-blocking: returns immediately, runs across multiple ticks. Total
 * wall-clock ~500 ms (disable settle + flash erase/program + re-enable).
 * Returns false if another save is already in flight. */
bool   axis_manager_request_motor_save               (void);

/* Trigger an immediate re-read of every proxied motor entry into the CMC
 * cache. Used when the operator (or another tool — PC, OD-over-UDP, etc.)
 * may have written motor OD directly without going through axis_manager,
 * leaving the CMC cache stale. Runs across multiple ticks in the
 * background; cache is fully refreshed within ~50 ms typical. No-op if
 * a resync is already in flight. The same state machine also re-fires
 * automatically every 5 s, so manual triggers are an optimisation, not
 * a requirement. */
void   axis_manager_request_motor_resync             (void);

#ifdef __cplusplus
}
#endif

#endif /* APP_AXIS_MANAGER_H */
