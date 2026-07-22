/*
 * app/axis_manager/buttons_jog — on-board UP/DOWN button poll + JOYSTICK jog.
 *
 * Extracted from axis_manager.c on 2026-07-22 (see axis_manager/README.md
 * "Refactor 2026-07-22" note).
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
 * direction).
 *
 * Ownership model: while neither button is held, op_mode + joystick_value
 * are owned by whoever (PC tool, web, physical stick) was driving them.
 * On the first button press we snapshot op_mode_commanded, switch it to
 * JOYSTICK, and start writing joystick_value each tick. On release of BOTH
 * buttons we start a "release hold" — force joystick_value = 0, keep
 * JOYSTICK mode — until the motor's own MOVING bit clears, then restore
 * the snapshotted op_mode and let the physical stick take over.
 *
 * Cooperates with op_arbiter through axis_manager_try_begin_op(JOYSTICK).
 * A press mid-home / mid-shot-recall is REJECTED and swallowed silently.
 *
 * Not thread-safe (single caller from axis_manager_tick).
 *
 * Layering: depends on axis_manager (mode + joystick setters), bsp/buttons,
 * bsp/time (unused directly but conceptually), Interface (MC_IF_MOVE_*),
 * log. Does NOT include any other axis_manager sub-module.
 */
#ifndef APP_AXIS_MANAGER_BUTTONS_JOG_H
#define APP_AXIS_MANAGER_BUTTONS_JOG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called once from axis_manager_init. Wipes state. */
void buttons_jog_init(void);

/* Called from axis_manager_tick. Regardless of motor-in-bootloader — the
 * buttons are a CMC-local input and are always polled; motor-side effect
 * is deferred by axis_manager itself if the motor isn't responding. */
void buttons_jog_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_AXIS_MANAGER_BUTTONS_JOG_H */
