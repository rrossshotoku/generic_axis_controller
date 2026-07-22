/*
 * app/axis_manager/buttons_jog — implementation. See header.
 *
 * Post-release hold exit is gated on the motor's own MC_IF_MOVE_MOVING
 * bit in movement_status (cyclic header, REQ-0013). The motor MCU clears
 * MOVING when drive is enabled AND |vel demand| AND |vel meas| are both
 * under ~0.01 rad/s — its own observer is the authoritative judge of
 * "stopped", and it's far more accurate than anything CMC could derive
 * from position deltas on this side. No time cap — we wait as long as
 * the motor takes to decelerate via the JOYSTICK-mode vel_accel_dn ramp.
 */
#include "buttons_jog.h"

#include "axis_manager.h"
#include "app/log/log.h"
#include "bsp/buttons/buttons.h"

#include "Interface/mc_if_protocol.h"

/*----------------------------------------------------------------------------
 * State
 *---------------------------------------------------------------------------*/
static bool           s_own_mode;                 /* true while CMC is forcing JOYSTICK mode from buttons */
static uint8_t        s_prev_mode;                /* op_mode_commanded snapshot from before button engaged */
static bool           s_in_release_hold;          /* true between BOTH-buttons-released and motor-stopped */

/* Local op-mode name helper — module-scope so log lines match the axis
 * manager's naming without needing a public accessor. Duplicated (small)
 * so this module doesn't cross-include axis_manager internals. */
static const char *mode_name(uint8_t m)
{
    switch (m) {
    case AXIS_OP_MODE_OFF:              return "OFF";
    case AXIS_OP_MODE_JOYSTICK:         return "JOYSTICK";
    case AXIS_OP_MODE_PROFILE_VELOCITY: return "PROFILE_VELOCITY";
    case AXIS_OP_MODE_PROFILE_POSITION: return "PROFILE_POSITION";
    case AXIS_OP_MODE_HOLD:             return "HOLD";
    case AXIS_OP_MODE_TORQUE:           return "TORQUE";
    default:                            return "?";
    }
}

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

/*----------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/
void buttons_jog_init(void)
{
    s_own_mode        = false;
    s_prev_mode       = (uint8_t)AXIS_OP_MODE_OFF;
    s_in_release_hold = false;
}

void buttons_jog_tick(void)
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
        s_in_release_hold = false;

        if (!s_own_mode) {
            /* First press — arbitrate through try_begin_op so a mid-home
             * or mid-shot-recall button press doesn't silently abort. If
             * REJECTED, we swallow the button and don't touch mode/value. */
            axis_begin_result_t r = axis_manager_try_begin_op(AXIS_OPERATION_JOYSTICK);
            if (r == AXIS_BEGIN_REJECTED) {
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
             * panel has yet sent SELECT. "Fail-on" semantics apply
             * (we never disable on release; only an explicit 0x3010
             * write or panel timeout drops enable). */
            s_prev_mode = (uint8_t)axis_manager_get_op_mode();
            s_own_mode  = true;
            LOG_INFO("axis_manager: buttons engaged %s (joystick_value = %+.2f, will restore mode %u on release)",
                     (up_held && down_held) ? "BOTH" : (up_held ? "UP" : "DOWN"),
                     (double)new_value, (unsigned)s_prev_mode);
            (void)axis_manager_request_enable(true);
            (void)axis_manager_set_op_mode((uint8_t)AXIS_OP_MODE_JOYSTICK);
        }
        (void)axis_manager_set_joystick_value(new_value);
    } else if (s_own_mode) {
        /* Both released — enter the release-hold instead of restoring the
         * mode immediately. Switching out of JOYSTICK while the motor is
         * still spinning would skip the joystick-mode accel ramp (each
         * other mode has its own velocity-handling semantics: HOLD pulls
         * hard to current position, OFF cuts torque, PROFILE_* uses its
         * own trajectory engine) — that's the audible "sudden stop" the
         * operator was seeing. By forcing joystick_value = 0 and holding
         * JOYSTICK mode, the motor decelerates through the configured
         * vel_accel_dn ramp. We only restore the snapshotted mode once
         * the motor reports MOVING cleared. */
        if (!s_in_release_hold) {
            s_in_release_hold = true;
            LOG_INFO("axis_manager: buttons released -> holding JOYSTICK + joystick_value=0, "
                     "will restore mode %s once motor reports MOVING cleared",
                     mode_name(s_prev_mode));
        }
        /* Force zero every tick so a non-centred physical stick can't
         * keep driving the motor through the hold (operator's hand is
         * off the stick — buttons are the active control surface). */
        (void)axis_manager_set_joystick_value(0.0f);

        if (!axis_manager_is_moving()) {
            LOG_INFO("axis_manager: motor stopped (movement_status=0x%04X) -> restoring mode %s, "
                     "physical stick takes over",
                     (unsigned)axis_manager_get_movement_status(),
                     mode_name(s_prev_mode));
            (void)axis_manager_set_op_mode(s_prev_mode);
            /* Re-derive joystick_value from the physical stick's raw value
             * so we hand back control with no zero-glitch. Public API
             * doesn't expose the recompute directly — writing the raw
             * back to itself has the same effect (setter re-runs the
             * calibration pipeline). */
            (void)axis_manager_set_joystick_raw(axis_manager_get_joystick_raw());
            s_own_mode        = false;
            s_in_release_hold = false;
        }
    }
    /* else: no button activity and no prior ownership — everything passes
     * through normally (physical stick drives joystick_value, PC tool /
     * web drive op_mode_commanded). */
}
