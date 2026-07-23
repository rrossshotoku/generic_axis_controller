# app/visca_mgr

VISCA transport + session + dispatch. First-pass implementation (2026-07-22) — the small subset of Sony's VISCA protocol needed for a PC-tool bring-up test harness.

Selected at boot via `active_protocol` (OD 0x3080). While VISCA is active, `app/controller_mgr` is compiled in but never ticked. See `Interface/mc_if_od.h` `MC_IF_PROTOCOL_*` and `Interface/CHANGELOG.md [5.4.0]`.

## Supported (first-pass)

### Inquiries (return current state; no side effects)

| Wire bytes | Name | Reply |
|---|---|---|
| `81 09 00 02 FF` | `CAM_VersionInq` | `90 50 vh vl mh ml rh rl 02 FF` — vendor 0xAAAA, model 0x0001, ROM 0x0100 |
| `81 09 06 12 FF` | `Pan_tiltPosInq` | `90 50 0p 0p 0p 0p 00 00 00 00 FF` — pan from `axis_manager_get_position_actual`, tilt=0 (single-axis CMC) |
| `81 09 04 47 FF` | `CAM_ZoomPosInq` | `90 50 00 00 00 00 FF` — always zero |

Position scaling: 1 VISCA unit = 1 milliradian. int16 range ±32.767 rad ≈ ±1877° — plenty.

### Commands (each returns ACK `90 41 FF` then Completion `90 51 FF` on socket 1)

| Wire bytes | Name | Effect |
|---|---|---|
| `81 01 06 01 VV WW YY ZZ FF` | `Pan_tiltDrive` | Pan speed `VV` (1-24), direction `YY` (01=left, 02=right, 03=stop). Tilt bytes `WW`, `ZZ` ignored (single-axis). Calls `cmc_state_handle_movement_scaled(sign·VV/24)`. |
| `81 01 06 01 xx xx 03 03 FF` | `Pan_tiltStop` | Same command with direction=stop → `cmc_state_handle_movement_scaled(0.0f)`. |
| `81 01 06 04 FF` | `Pan_tiltHome` | Runs the endstop homing procedure (`axis_manager_request_home`). See "Semantics note" below. |
| `81 01 04 3F 01 pp FF` | `Memory_Set (Store)` | Store current position at preset `pp` (0..99 → shot 1..100). `cmc_state_store_shot`. Auto-saves to flash. |
| `81 01 04 3F 02 pp FF` | `Memory_Recall` | Fade to preset `pp` using slot's stored `time_to_shot_s` (0 → `FADE_MIN_TIME_S` default). `cmc_state_move_to_shot(shot_no, is_cut=false)`. Rejected if not homed. |
| `81 01 7E 04 3D pp FF` | Sony-ext position-preset mode | No-op ACK — this CMC is position-only anyway. |
| `81 01 7E 04 1B pp FF` | Sony-ext per-preset-settings mode | No-op ACK — every slot already has its own `time_to_shot_s`. |
| `81 01 7E 04 27 pp mm FF` | Sony-ext preset mode select (speed vs duration) | No-op ACK — CMC always uses per-slot duration. |
| `81 01 7E 04 67 pp 0q 0q 0q FF` | Sony-ext preset duration | 12-bit duration in 0.1 s units → `cmc_state_set_shot_time_tenths(pp+1, tenths)`. Sony's documented range 1.0..99.0 s; values outside that log a WARN but are accepted. |

## Semantics note: Pan_tiltHome

Real Sony PTZ cameras interpret `81 01 06 04 FF` as "move to (0, 0) mechanical center." For this CMC we map it to `axis_manager_request_home()` — the endstop homing procedure — because:

1. This is a single-axis motion controller with an endstop, not a PTZ camera with an absolute encoder.
2. Until the encoder is zeroed at the endstop, "position 0" has no physical meaning.
3. After homing, the mechanical zero IS position 0, so the downstream semantics converge for anything that queries position afterward.

If a real "go to 0" is needed alongside, store preset 0 (shot 1) at the desired home position and use Memory Recall.

## Not supported

- **`Pan_tiltReset` (`81 01 06 05 FF`)** — same as Home on this CMC; not distinguished.
- **`Pan_tiltAbsolutePosition` (`81 01 06 02 ...`)** — could be added easily: decode int16 pan/tilt, call `axis_manager_set_target_position` + start_move.
- **`Pan_tiltRelativePosition` (`81 01 06 03 ...`)** — same story, relative to current.
- **`Memory_Reset` (`81 01 04 3F 00 pp FF`)** — no shot-delete primitive in `cmc_state`; would need one.
- **CAM_Zoom** commands (drive, direct) — no zoom on this CMC.
- **Broadcast (`88 ...`)** — parsed, but no broadcast-specific handling; treated as a normal command targeting our address.
- **Sockets 1 vs 2** — real Sony cameras track two concurrent commands. We always reply on socket 1 (single-in-flight behaviour).
- **Cancel class (`81 2S FF`)** — logged, syntax-error reply.

## Transport

- Sony VISCA-over-IP UDP on port 52381.
- 8-byte IP header per packet (payload_type BE, payload_length BE, sequence BE), followed by raw VISCA bytes.
- Command replies are TWO packets (ACK, then Completion), matching real cameras.
- No TCP. No serial.
- Reuses W6100 socket 0 (same physical socket as `controller_mgr`'s CAMERAD POLL). Safe because the two mgrs are mutually exclusive at boot.

## Device address

Sourced from **`cmc_device_no`** in the shared network config (web page: "CMC device #"). CAMERAD also uses this value as its advertised `return_device_no`, so operators only set one number for "this CMC's identity."

VISCA only defines addresses 1..7 (address 8 is broadcast). If `cmc_device_no` is outside that range (the field validates 1..255 for CAMERAD's benefit), VISCA falls back to address 1 with a WARN in the boot log.

Snapshot once at init — changes to `cmc_device_no` require Save + Reboot to take effect for VISCA, matching how every other network-config change behaves.

## Ownership model

- No SELECT/DESELECT/GRAB — VISCA has no such concept. Any VISCA client on the network can command the CMC as long as VISCA is the active protocol. If two clients fight, last-write-wins.
- No inactivity timeout — VISCA-over-IP is stateless UDP; there's no session to time out.
- If contention becomes a real-world problem, auto-select on first traffic + timeout is a small addition.

## Session model

Stateless. Every command gets an immediate ACK + Completion (no in-flight tracking). Long-running commands (preset recall, homing) fire and forget from VISCA's perspective; the caller polls `Pan_tiltPosInq` to detect arrival, or `axis_state`/`axis_is_homed` via a separate channel.

## Extending

To add a new command:

1. **If the shape is new** (new category or command byte), add the case in `dispatch_command` / `dispatch_inquiry` in `visca_mgr.c`.
2. **Codec helpers** (parsers/builders) belong in `app/visca/visca.c` if reusable, otherwise inline in `visca_mgr.c`.
3. **Side-effect** — call `cmc_state_*` or `axis_manager_*`, never touch motor OD directly.
4. **Response** — for inquiries, use `visca_build_inquiry_reply`; for commands, use `send_ack_and_completion` (or `send_error` on rejection).
5. Update the tables above.

To add serial VISCA: create `bsp/uart` for the pins, write a new `service_uart` peer to `service_udp_socket`, keep the codec + dispatch as-is. Codec is transport-agnostic.

## Layering

L2, same tier as `controller_mgr`. Deps: `app/visca` (codec), `app/cmc_state` (transport-agnostic session), `app/axis_manager` (motor surface), `app/config` (identity), `app/log`, `bsp/net`, `bsp/time`. NEVER includes any other `app/` module upward.

## Files

- `visca_mgr.h` — public API (`init` + `tick`).
- `visca_mgr.c` — transport, dispatch, side-effects.
- `README.md` — this file.

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
