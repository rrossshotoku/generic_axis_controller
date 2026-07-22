/*
 * app/cmc_state — CMC operational state owned by the network protocol layer.
 *
 * Session ownership, camera-status bits, shot table, joystick profile.
 * Transport-agnostic: any protocol module (camerad's controller_mgr, a
 * future visca_mgr, the PC tool's OD writer) can drive these primitives.
 * The SELECT / DESELECT / GRAB naming came from CAMERAD but the semantics
 * are generic session-ownership arbitration:
 *   SELECT   from controller X: grant if nobody selected OR X already owns
 *                                it; deny if a different controller owns it.
 *   DESELECT from controller X: only succeeds if X is the current owner.
 *   GRAB     from controller X: always succeeds, transfers ownership from
 *                                whoever currently has it.
 * A transport without an ownership concept (e.g. VISCA) may skip these
 * calls entirely and still use shot / movement / status APIs.
 *
 * Layering: L3 (state). Implementation (cmc_state.c) depends on
 * `app/axis_manager` (motor position + JOYSTICK op), `app/persist` (shot
 * table save/load), `app/log`, and `bsp/time`. Callers include this header
 * only — the DAG stays a tree.
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

/* === Joystick / velocity input from a protocol transport =================
 *
 * Two entry points, both transport-agnostic:
 *
 *   cmc_state_handle_movement_scaled(float v) — the primitive. `v` is
 *   the normalised deflection in [-1.0, +1.0], with 0 = centered. Any
 *   transport translates its own wire form into this range and calls this
 *   directly. VISCA-mgr would divide its magnitude byte by its max range
 *   and choose the sign from the direction byte; a future analog-stick
 *   input would divide raw counts by full-scale.
 *
 *   cmc_state_handle_movement(int8_t deflection) — CAMERAD wrapper. Takes
 *   the raw signed int8 (-128..+127, 0 = centered) from a CAMERAD MOVEMENT
 *   body and forwards to _scaled after normalising by 127. Kept as a thin
 *   wrapper so controller_mgr's dispatch line reads naturally at the
 *   MOVEMENT decode site.
 *
 * Both entry points share the same side effects:
 *   - if op_mode isn't already JOYSTICK, switches it (so the cyclic
 *     SPI command picks up the new velocity demand on the next cycle)
 *   - writes the deflection through axis_manager → joystick_value
 *   - stamps a last-seen timestamp for the watchdog
 *   - REJECTED if a HOMING / SHOT_RECALL op is in flight (arbitration
 *     via axis_manager_try_begin_op) — the call is silently swallowed.
 *
 * If either entry stops being called for JOYSTICK_WATCHDOG_MS, the
 * watchdog (run from cmc_state_update_from_motor) zeroes the joystick
 * raw value — the motor sees velocity = 0 and stops. Transports that
 * send explicit "stop" (like VISCA's Pan-Tilt Stop) can just call
 * handle_movement_scaled(0.0f); the watchdog is defensive, not required.
 *
 * Single-motor CMC: whichever transport supplies the deflection, we
 * apply it to the one motor. axis_role (OD 0x3070) is a CAMERAD-only
 * concept selecting which MOVEMENT byte to consume; other transports
 * (VISCA, direct velocity commands) address one axis at a time and
 * bypass that.
 */
void cmc_state_handle_movement_scaled(float deflection);
void cmc_state_handle_movement       (int8_t deflection);

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
