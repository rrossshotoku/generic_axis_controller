/*
 * app/cmc_state — CMC operational state owned by the network protocol layer.
 *
 * Path-A scope: just the SELECT/DESELECT/GRAB ownership state needed to keep
 * a CAMERAD panel happy after discovery. Shot table, joystick profile,
 * status bits, controller registry — all defer until they're actually used.
 *
 * Semantics mirrored from the Reduced CMC (uc_camd_interface/app/cmc_state.c
 * lines 136-178):
 *   SELECT   from controller X: grant if nobody selected OR X already owns
 *                                it; deny if a different controller owns it.
 *   DESELECT from controller X: only succeeds if X is the current owner.
 *   GRAB     from controller X: always succeeds, transfers ownership from
 *                                whoever currently has it.
 *
 * Layering: L3 (state). Depends only on stdint/stdbool. No `bsp/`, no
 * `Interface/`, no other `app/` module. Reusable from any caller.
 */

#ifndef APP_CMC_STATE_H
#define APP_CMC_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void cmc_state_init(void);

/* Selection.
 *
 * `controller_no` is the CAMERAD device_no of the controller (the value
 * the panel sends as `return_device_no` in its requests). */

bool     cmc_state_handle_select  (uint32_t controller_no);   /* true = granted */
bool     cmc_state_handle_deselect(uint32_t controller_no);   /* true = succeeded */
void     cmc_state_handle_grab    (uint32_t controller_no);   /* always grants */

/* Force-deselect (CMC initiated): called by controller_mgr when a controller
 * stops POLLing for too long (panel disconnected / network drop). Mirrors
 * the Reduced CMC's cmc_remove_controller -> "if I owned the camera, drop
 * the selection" path. Unlike cmc_state_handle_deselect this does NOT
 * require the caller to be the current owner — the CMC is deciding for
 * itself that the owner is gone. No-op if `controller_no` isn't the
 * current owner. */
void     cmc_state_force_deselect (uint32_t controller_no);

bool     cmc_state_is_selected    (void);
uint32_t cmc_state_selected_by    (void);                     /* 0 if not selected */

/* === Shot table ===========================================================
 *
 * 100 slots. Wire-protocol convention: shot_no is 1-indexed (1..100); the
 * internal array is 0-indexed (0..99). All public APIs here take the
 * **wire-form 1-based shot_no** so the call sites match CAMERAD body fields
 * one-to-one. shot_no==0 means "no shot" in some POLL fields.
 *
 * Per-shot data (Path A: single axis):
 *   - position_rad      (F32) — motor position to recall
 *   - time_to_shot_s    (F32) — default fade time on this shot
 *   - valid             (bool) — has this slot been stored?
 *
 * Persistence: every store/edit operation rewrites the whole shots region
 * to flash (~60 ms blocking, matches Reduced CMC's save-on-every-store).
 * No wear-levelling in v1 — the 10k-cycle-per-page endurance bounds the
 * number of distinct store operations across product lifetime.
 */

#define CMC_MAX_SHOTS 100u

typedef struct {
    bool  valid;
    float position_rad;
    float time_to_shot_s;
} cmc_shot_t;

/* Store the CMC's current motor position as shot `shot_no`. Returns
 * the index that was written (0-based), or -1 on error. Auto-saves the
 * whole shots region to flash. The position recorded is whatever
 * axis_manager_get_position_actual() reports right now. */
int  cmc_state_store_shot(uint32_t shot_no);

/* Same as store_shot but uses current_shot+1 as the target. Returns
 * the new shot number on success or 0 on error. */
uint32_t cmc_state_store_next(void);

/* Set the live time-to-shot (used as default fade duration for the
 * next CUT/FADE that doesn't carry its own). Time in tenths of a
 * second — CAMERAD wire convention. */
void cmc_state_set_time_to_shot_tenths(uint32_t tenths);

/* Issue a move-to-shot command. `is_cut` selects instant (time=0) vs
 * timed (uses the live time_to_shot or the shot's stored time).
 * Drives axis_manager: PROFILE_POSITION + target + start_move trigger.
 * SWOOP is mapped to FADE here per the v1 decision. */
bool cmc_state_move_to_shot(uint32_t shot_no, bool is_cut);

/* Stop any in-progress move (KEY_STOP / KEY_STOP_ALL). Drives
 * axis_manager_request_quick_stop. */
void cmc_state_stop_movement(void);

/* Read accessors for the POLL response and the GUI. */
uint32_t cmc_state_current_shot      (void);
uint32_t cmc_state_next_shot         (void);
uint32_t cmc_state_time_to_shot_tenths(void);
bool     cmc_state_on_shot           (void);
bool     cmc_state_moving            (void);

/* Read a single shot slot. `shot_no` is 1-indexed. Returns false if
 * shot_no is out of range or the slot is invalid (never stored).
 * On success populates *out. */
bool cmc_state_get_shot(uint32_t shot_no, cmc_shot_t *out);

/* Persist the current shot table to flash. Returns true on success.
 * Called automatically after every store; callable manually too (via
 * OD 0x3051 cmc_save_shots) e.g. after a PC-tool batch edit. */
bool cmc_state_save_shots(void);

/* Called periodically by axis_manager (or the main loop) so cmc_state
 * can update its on_shot / moving / time_to_shot countdown from the
 * live motor state. Also runs the joystick watchdog (zero raw if no
 * MOVEMENT message has arrived in JOYSTICK_WATCHDOG_MS).               */
void cmc_state_update_from_motor(void);

/* === Joystick input from a CAMERAD MOVEMENT message =======================
 *
 * Called by controller_mgr each time a MOVEMENT body arrives. `pan` is the
 * signed int8 deflection (CAMERAD wire convention, -128..+127 with 0 =
 * centered). For our single-motor CMC we map the panel's pan axis onto
 * the one motor; other axes (tilt, focus, zoom, x, y, height, fader) are
 * ignored.
 *
 * Side effects:
 *   - writes axis_manager_set_joystick_raw(pan) → triggers the
 *     calibrated normalisation into joystick_value
 *   - if op_mode isn't already JOYSTICK, switches it (so the cyclic
 *     SPI command picks up the new velocity demand on the next cycle)
 *   - stamps a last-seen timestamp for the watchdog
 *
 * If MOVEMENT messages stop arriving for JOYSTICK_WATCHDOG_MS, the
 * watchdog (run from cmc_state_update_from_motor) zeroes the joystick
 * raw value — the motor sees velocity = 0 and stops.
 */
void cmc_state_handle_movement(int8_t pan);

/* === TODO(joystick-profile) ===============================================
 *
 * CAMERAD's KC_JOY_PROFILE_NORMAL/MEDIUM/FINE (0x80/0x81/0x82) currently
 * land in controller_mgr's KEYPRESS_T1 dispatch as a no-op (just logs the
 * keypress). Plan, agreed 2026-06-23, when this becomes priority:
 *
 *   1. Add three CMC-owned OD entries to mc_if_od.h:
 *        0x3022 axis_joystick_max_velocity         (existing; "Normal")
 *        0x302D axis_joystick_max_velocity_medium  (F32 RW PERSIST, new)
 *        0x302E axis_joystick_max_velocity_fine    (F32 RW PERSIST, new)
 *   2. cmc_state tracks `s_joystick_profile` (0=Normal / 1=Medium / 2=Fine);
 *      add cmc_state_set_joystick_profile() / cmc_state_get_joystick_profile().
 *      Profile is NOT persisted — resets to Normal on boot.
 *   3. axis_manager joystick path uses whichever max_velocity entry matches
 *      the active profile when computing the velocity demand.
 *   4. controller_mgr's KEYPRESS_T1 handler calls
 *      cmc_state_set_joystick_profile(0/1/2) for the three KC_JOY_PROFILE_* keys.
 *   5. Web/GUI: expose all three max_velocity values + a read-only "current
 *      profile" indicator.
 *   6. (Optional, Phase B+) When a LIMIT-response handler is implemented,
 *      mirror the profile in bits 24-25 of the status int — matches SW050's
 *      `LimitsU.h:153-154` (ciJoystickProfileMask = 0x03000000, shift=24).
 *      This lets a multi-panel setup keep the displayed profile in sync.
 *
 * Reference: SW050 `CMCapp/CMCControl.h:558 JoystickProfileModifier(...)`
 * (actually implements scaling). Reduced CMC stores the value but doesn't
 * scale anything — see `uc_camd_interface/command_handler.c:373-390`. */

#ifdef __cplusplus
}
#endif

#endif /* APP_CMC_STATE_H */
