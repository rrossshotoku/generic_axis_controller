# app/cmc_state

## Purpose
The single source of truth for runtime CMC state. Selection ownership, camera-status bits, CMC-status bits, joystick profile, current/next shot, time-to-shot, shot table.

## Owns
- `cmc_state_t` — one instance, file-static.
- Selection state: `selected`, `selected_by` (controller device number).
- Status bits:
  - `camera_status` (matches SW050 `eCamStatus`: bits for Moving, OnShot, ZOff, OP1-3, PRev, TRev).
  - `cmc_status`   (matches SW050 `eCMCStatus`: bits for Local, Toggle, Ext, etc.). **Moving and OnShot live in camera_status, not cmc_status** (Reduced-CMC mistake to avoid).
- Joystick profile (`jpNormal/jpMedium/jpFine`), settable from CAMERAD Type-2 keypress.
- Current shot, next shot, time-to-shot, current move type.
- Shot table (100 entries), loaded from `config` on boot, saved back on store.

## Does NOT do
- Open any socket or talk to the network.
- Talk to the motor.
- Build or parse any protocol bytes.
- Persist anything directly — delegates to `config`.

## Public API
```c
void   cmc_state_init(void);

/* Selection arbitration */
bool   cmc_select  (uint32_t controller_no);   // returns true if granted
bool   cmc_deselect(uint32_t controller_no);   // returns true if was selected by this controller
void   cmc_grab    (uint32_t controller_no);   // forced take
uint32_t cmc_selected_by(void);                // 0 if none

/* Status */
uint16_t cmc_get_camera_status(void);
uint16_t cmc_get_cmc_status(void);
void     cmc_set_status_bit  (cmc_status_bit_t bit, bool value);

/* Joystick profile */
joy_profile_t cmc_get_joystick_profile(void);
void          cmc_set_joystick_profile(joy_profile_t p);

/* Shot table */
bool   cmc_shot_store (uint16_t shot_no, const cmc_position_t *pos, uint32_t time_tenths);
bool   cmc_shot_recall(uint16_t shot_no, cmc_position_t *out);

/* Shot lifecycle */
void   cmc_begin_move_to_shot(uint16_t shot_no, move_type_t type, uint32_t time_tenths);
void   cmc_movement_finished(void);
```

## Dependencies
- `config` (for persisting shots).
- `motor_ctrl` (called from `cmc_begin_move_to_shot` to actually start motion). Bottom-up arrow — `cmc_state` lives one layer above `motor_ctrl`, so this is the only allowed call into a peer.

## Acceptance criteria
- Only one controller can be selected at a time; a second `cmc_select` while held returns false.
- Status bits round-trip through getters/setters without aliasing camera vs CMC.
- Storing a shot persists it to `config`; the next boot recalls the same position.
- A subscribing module (e.g. `controller_mgr`) reading `cmc_get_camera_status()` from one tick and again from the next sees consistent values (no torn updates — state changes happen atomically in handlers).

## Notes
- No globals exported. Everything goes through the API.
- The Moving and OnShot bits are derived from `motor_ctrl` status, not stored — `cmc_get_camera_status` queries `motor_ctrl` and OR's the bits in. (Avoids the Reduced-CMC duplication where Moving was reported in both camera and cmc status words.)
- Shot 0 is reserved (means "no shot"); valid range is 1..100.
