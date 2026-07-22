/*
 * app/axis_manager/motor_od_proxy — motor-OD write scheduler + bootsync
 *                                   + motor-save sequencer + load_factor.
 *
 * Extracted from axis_manager.c on 2026-07-22 (see axis_manager/README.md
 * "Refactor 2026-07-22" note). Merged four concerns that all share the
 * cia402 single-in-flight SDO slot — keeping them in one file keeps that
 * cooperation line-of-sight.
 *
 * What this module handles:
 *   1. DIRTY-SLOT WRITER — 7-slot round-robin proxy for values the operator
 *      changes on the CMC that must be forwarded to the motor's OD:
 *        SLOT_VEL_ACCEL_UP     motor 0x2300:6
 *        SLOT_VEL_ACCEL_DN     motor 0x2300:7
 *        SLOT_VEL_ACCEL_JERK   motor 0x2300:8
 *        SLOT_VEL_LIMIT        motor 0x2600:4
 *        SLOT_ACCEL_LIMIT      motor 0x2600:5
 *        SLOT_POS_LIMIT_LO     motor 0x2600:6
 *        SLOT_POS_LIMIT_HI     motor 0x2600:7
 *   2. BOOTSYNC — one-shot SDO reads at motor-app first-boot to seed the
 *      slot cache from motor flash, plus periodic auto-resync every 5 s so
 *      PC-tool-direct changes are noticed. Emits per-slot bootsync notifications
 *      via a registered callback so axis_manager can update its own cached
 *      mirrors of the CATEGORY-2 fields (velocity/accel/position limits).
 *   3. MOTOR-SAVE SEQUENCER — disable → SDO write MC_IF_SAVE_MAGIC → re-enable.
 *      Motor MCU can only commit a flash save while the power stage is OFF
 *      (motor ADR-010), so this brackets the SDO with disable/re-enable via
 *      the axis_manager_request_enable() setter.
 *   4. LOAD_FACTOR — dedicated one-shot SDO writer for motor 0x2300:5. Not
 *      in the slot table; not bootsync-tracked (write-only convention). Own
 *      cia402 handle. Cooperates with slot writer + save sequencer via
 *      cia402_od_write_begin returning INVALID.
 *
 * cia402 slot cooperation: all four concerns issue cia402_od_*_begin and
 * defer if it returns INVALID. Because they share ONE cia402 slot, only
 * one operation is ever in flight at a time across all four — the tick
 * order inside motor_od_proxy_tick reflects this. Cooperates externally
 * with home_sequencer (also uses cia402 slot) on the same terms.
 *
 * Ownership split with axis_manager:
 *   - vel_accel_up/dn/jerk (CATEGORY 1) → OWNED here. axis_manager exposes
 *     public getters/setters that forward through.
 *   - vel_limit / accel_limit / pos_limit_lo / pos_limit_hi (CATEGORY 2)
 *     → axis_manager owns the CMC-side cache (with +/-INF convention for
 *     position limits); this module owns the motor-side cache in its slot
 *     table. Setters on axis_manager write BOTH sides. Bootsync updates
 *     the axis_manager mirror through the registered callback.
 *   - load_factor → OWNED here. axis_manager forwards.
 *   - motor_save state machine → OWNED here. Reaches into axis_manager
 *     only via axis_manager_request_enable(bool).
 *
 * Not thread-safe (single caller from axis_manager_tick).
 *
 * Layering: depends on cia402, bsp/time, log, Interface, axis_manager
 * (for request_enable). Does NOT include any other axis_manager sub-module.
 */
#ifndef APP_AXIS_MANAGER_MOTOR_OD_PROXY_H
#define APP_AXIS_MANAGER_MOTOR_OD_PROXY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bootsync notification IDs. Callback (see below) receives one of these to
 * identify which motor-side value was just read back. Only the CATEGORY-2
 * fields raise callbacks — CATEGORY-1 values (vel_accel_*) update this
 * module's own cache and aren't reported outward. */
typedef enum {
    MOTOR_OD_PROXY_BOOTSYNC_VEL_LIMIT = 0,
    MOTOR_OD_PROXY_BOOTSYNC_ACCEL_LIMIT,
    MOTOR_OD_PROXY_BOOTSYNC_POS_LIMIT_LO,
    MOTOR_OD_PROXY_BOOTSYNC_POS_LIMIT_HI,
} motor_od_proxy_bootsync_id_t;

typedef void (*motor_od_proxy_bootsync_cb_t)(motor_od_proxy_bootsync_id_t which,
                                             float motor_value);

/* Called once from axis_manager_init. Wipes all internal state to a
 * clean IDLE and disarms the once-per-boot bootsync log latch. MUST be
 * called before the first tick. */
void motor_od_proxy_init(void);

/* Register the bootsync callback. Called by axis_manager_init AFTER
 * motor_od_proxy_init so this module knows how to notify axis_manager
 * when reads complete. May be NULL to disable notifications. */
void motor_od_proxy_register_bootsync_cb(motor_od_proxy_bootsync_cb_t cb);

/* Called from axis_manager_tick when the motor is NOT in bootloader mode
 * (the guard lives in axis_manager). Order inside is deliberate:
 *   1. drain load_factor SDO
 *   2. bootsync (reads)   — MUST run before proxy writes so a slot that's
 *                            both dirty and being read back doesn't get its
 *                            operator-typed value clobbered.
 *   3. proxy writes       — one dirty slot at a time
 *   4. motor save
 * All four defer if cia402 is busy. */
void motor_od_proxy_tick(void);

/* Called from reset_motor_od_submodules on the rising edge of motor
 * entering the bootloader. Puts every state machine back to IDLE and
 * invalidates every cia402 handle. Restarts bootsync from slot 0 on
 * re-entry so a firmware update on the motor doesn't leave stale cache. */
void motor_od_proxy_reset(void);

/* --- CATEGORY 1: values fully owned here (vel_accel_*) ---------------- */
float motor_od_proxy_get_vel_accel_up   (void);
float motor_od_proxy_get_vel_accel_dn   (void);
float motor_od_proxy_get_vel_accel_jerk (void);
bool  motor_od_proxy_set_vel_accel_up   (float v);   /* validates + marks dirty */
bool  motor_od_proxy_set_vel_accel_dn   (float v);
bool  motor_od_proxy_set_vel_accel_jerk (float v);

/* --- CATEGORY 2: axis_manager owns local cache, we forward to motor ---
 * These setters just mark the slot dirty; axis_manager is responsible for
 * updating its own s_axis mirror in the same call. Values here are in
 * motor-side convention (finite; 0 = disabled for pos limits). */
bool motor_od_proxy_write_vel_limit    (float motor_v);
bool motor_od_proxy_write_accel_limit  (float motor_v);
bool motor_od_proxy_write_pos_limit_lo (float motor_v);
bool motor_od_proxy_write_pos_limit_hi (float motor_v);

/* --- load_factor ------------------------------------------------------ */
float motor_od_proxy_get_load_factor (void);
bool  motor_od_proxy_set_load_factor (float v);       /* validates + fires SDO */

/* --- Motor save + resync --------------------------------------------- */
bool motor_od_proxy_request_motor_save   (void);      /* returns false if busy */
void motor_od_proxy_request_motor_resync (void);      /* restarts bootsync pass */

#ifdef __cplusplus
}
#endif

#endif /* APP_AXIS_MANAGER_MOTOR_OD_PROXY_H */
