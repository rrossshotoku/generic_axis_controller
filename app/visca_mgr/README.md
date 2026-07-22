# app/visca_mgr

**STUB — no implementation yet.** Placeholder for VISCA transport + session + dispatch. Exists so `main_loop` can select between CAMERAD (`app/controller_mgr`) and VISCA at boot via `active_protocol` (OD 0x3080) without conditional compilation.

## Selection

Web UI dropdown at the top → `config_set_active_protocol()` → persist to NETWORK region → reboot → `main_loop_init` reads `config_get_active_protocol()` and calls either `controller_mgr_init/tick` or `visca_mgr_init/tick`. See `Interface/mc_if_od.h` `MC_IF_PROTOCOL_*` constants and `Interface/CHANGELOG.md [5.4.0]`.

## What this module will own (once implemented)

- Transport bring-up. Two credible options:
  - **VISCA-over-IP** (UDP port 52381 by default). Reuses `bsp/net`, no new hardware. Recommended first target.
  - **VISCA over RS-232/485.** Needs a UART peripheral brought up at boot; may need a new `bsp/uart` module.
- Address decode. VISCA frames start with `8x` where `x` is the target camera (1-7) or `8` = broadcast. This CMC declares its address once via config (candidate for a future 0x3081 `visca_address` if not folded into `cmc_device_no`).
- Command dispatch. VISCA commands are grouped by category (Camera1 = `81 01 04 xx`, Pan-Tilt = `81 01 06 xx`, etc.). Each command decodes to calls into the shared state:
  - **Pan-Tilt Drive** → `cmc_state_handle_movement_scaled(v)` (velocity from magnitude byte + direction byte)
  - **Pan-Tilt Stop** → `cmc_state_handle_movement_scaled(0.0f)`
  - **Pan-Tilt Absolute Position** → `axis_manager_set_target_position(rad)` + `axis_manager_set_target_velocity(rad/s)` + `axis_manager_set_op_mode(PROFILE_POSITION)` + `axis_manager_request_start_move()`. Bypasses shot table entirely.
  - **Memory Set** → `cmc_state_store_shot(N)` (or `cmc_state_store_shot_with_time(N, seconds)` once added)
  - **Memory Recall** → `cmc_state_move_to_shot(N, is_cut=false)`
  - **Pan-Tilt Home** → `axis_manager_request_home()` or a synthetic "shot 0 = home" recall — decide when implementing
  - **Pan-Tilt Reset** → same as Home for this CMC (re-run home procedure)
  - **Inquiries** (`8x 09 …`) → stateless dispatch table into existing getters
- ACK / completion / error responses.

## What this module will NOT own

- Session ownership (SELECT/DESELECT/GRAB) — VISCA has no such concept. `visca_mgr` can either auto-select on first traffic + deselect on inactivity, or skip ownership arbitration entirely. Design decision at implementation time.
- Codec — belongs in `app/visca/` (mirror of `app/camerad/`).
- Motor calls — go through `axis_manager` and `cmc_state`.

## Layering (planned)

L2, same tier as `controller_mgr`. Deps: `app/visca` (codec), `app/cmc_state`, `app/axis_manager`, `app/config`, `app/log`, `bsp/net` (or `bsp/uart`), `bsp/time`.

## Cross-transport implications

`cmc_state` and `axis_manager` are transport-agnostic. They accept VISCA-driven calls identically to CAMERAD ones. The `handle_movement_scaled(float)` primitive is the shared entry — CAMERAD's `handle_movement(int8_t)` is now a thin wrapper on it. See `app/cmc_state/README.md` for the full list of transport-agnostic APIs.

The precedence in `cmc_state_move_to_shot` uses `s_time_to_shot_tenths` (operator-locked, CAMERAD-only) if set, otherwise the slot's stored `time_to_shot_s`. VISCA-mgr should NOT call `cmc_state_set_time_to_shot_tenths` — it stores per-slot times via the shot-store call and lets the slot's stored value win.

## Files

- `visca_mgr.h` — public API (`init` + `tick`).
- `visca_mgr.c` — stub. Just logs when it's selected as the active protocol.
- `README.md` — this file.
