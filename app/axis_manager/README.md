# app/axis_manager

The high-level command surface for the motor. Every protocol module that drives the motor (CAMERAD, VISCA, the PC tool's OD writer, future additions) talks to this module — and **only** this module — via the CMC-owned OD entries it backs.

## Files (2026-07-22 refactor)

`axis_manager` used to be one 2800-line file. It's now an orchestrator plus three peer sub-modules, all under this directory.

| File | Purpose | LOC |
|---|---|---|
| `axis_manager.{c,h}` | Orchestrator: singleton, op arbiter, persist blob, cyclic frame, tick, public API, thin forwarders. | ~1550 |
| `home_sequencer.{c,h}` | Home-to-endstop state machine + encoder-type probe + `is_homed` cache + `home_on_boot` (0x3043). | ~470 |
| `motor_od_proxy.{c,h}` | Dirty-slot writer + bootsync + motor-save + load_factor. All four share the cia402 SDO slot. | ~570 |
| `buttons_jog.{c,h}` | PB UP/DOWN button poll + JOYSTICK-mode override + post-release hold. | ~290 |

Public API stays in `axis_manager.h`. External modules (`cmc_od`, `web`, `cmc_state`, `led_indicator`, `controller_mgr`, `debug`, `main_loop`) don't include any sub-module header — the sub-modules are internal to `axis_manager`.

## Sub-module map with responsibilities

**`home_sequencer.c`** owns:
- `s_state` (7-state HOME_SEQ_* machine)
- Encoder-type probe (motor 0x2500:8 quad_counts_per_rev) — one-shot after boot, retries until success.
- `is_homed` cache (from motor 0x2600:1 fault_flags NOT_HOMED bit) — refreshed on end-of-home + periodic 5 s background poll.
- `home_on_boot` — persisted flag; auto-fires `request_home` once per CMC boot when encoder is incremental.
- Cooperates with op arbiter through `axis_manager_try_begin_op(HOMING)`.

**`motor_od_proxy.c`** owns four concerns that share one cia402 slot:
- **7-slot dirty-slot writer** for values the operator changes on the CMC that must be forwarded to the motor's OD:
  - `vel_accel_up/dn/jerk` at motor 0x2300:6/7/8 — CATEGORY 1 (owned here fully)
  - `vel_limit / accel_limit / pos_limit_lo / pos_limit_hi` at motor 0x2600:4/5/6/7 — CATEGORY 2 (axis_manager owns CMC-side mirror)
- **Bootsync** — one-shot SDO reads at motor-app first-boot to seed slot cache from motor flash + periodic auto-resync every 5 s. Emits a per-slot callback so axis_manager can update the CATEGORY-2 mirrors.
- **Motor-save state machine** — disable → SDO write `MC_IF_SAVE_MAGIC` to 0x2800:1 → re-enable. Motor MCU can only commit a save while power stage is OFF (motor ADR-010).
- **load_factor** (motor 0x2300:5, REQ-0014) — dedicated one-shot writer, not in slot table (write-only convention). Own handle. Cooperates with slot writer + save via `cia402_od_write_begin` returning INVALID.

**`buttons_jog.c`** owns:
- PB UP/DOWN GPIO poll (through `bsp/buttons`, debounced there).
- JOYSTICK-mode acquisition through `axis_manager_try_begin_op(JOYSTICK)`.
- Post-release "hold" — force `joystick_value=0` in JOYSTICK mode until motor's MOVING bit clears, then restore snapshotted mode.

**`axis_manager.c`** (orchestrator) owns:
- `s_axis` singleton — every field backing 0x30xx public OD entries.
- **Operation arbiter** — `try_begin_op`, `stop_op`, `tick_active_op`. Kept in the orchestrator because it's touched by every sub-module and by `cmc_state`; extracting it would create more coupling than it saves.
- **Persist blob** (v6) — `apply_persist_blob` / `capture_persist_blob` / `save_to_flash`. Forwards `home_on_boot` byte to home_sequencer, `led_rgb` to led_indicator.
- **Bootsync callback** — receives `motor_od_proxy` notifications and updates the s_axis CATEGORY-2 mirror (velocity/accel/position limits) so `compute_desired`'s clamps track the motor's flash values.
- **Lifecycle** — `axis_manager_init` calls each sub-module's init, registers the bootsync callback, then loads persist.
- **Tick** — reads cyclic status, edge-detects motor bootloader entry (calls `reset_motor_od_submodules`), invokes each sub-module's tick in the correct order, drives the setup sequencer, composes and pushes the cyclic command.
- **Cyclic command composition** + **SDO setup-sequencer** (mode/target/profile writes to motor 0x6060/6071/607A/607B/6081/6083/6084).
- **State getters, command latches, mode+targets, limits, joystick cal, scaling helpers.**

## Invariants — READ BEFORE ADDING A SUB-MODULE

### 1. cia402-slot cooperation

The following functions all issue `cia402_od_read_begin` / `cia402_od_write_begin` / `cia402_od_poll` against a **single in-flight SDO slot**:

- `setup_sequencer_tick` (axis_manager.c) — motor 0x6060/6071/607A/607B/6081/6083/6084
- Inside `motor_od_proxy_tick`:
  - `poll_load_factor_sdo` (motor 0x2300:5)
  - `tick_bootsync` — reads (all 7 proxy slots)
  - `poll_motor_proxy_sdo` — writes (all 7 proxy slots)
  - `tick_motor_save` (motor 0x2800:1)
- `home_sequencer_tick` — motor 0x2500:8 (encoder-type read), 0x2600:1 (fault_flags read), 0x2700:8 (home_command write), 0x2700:9 (home_status read)

**Rule:** at most one operation in flight at any moment. Each caller defers by returning early when `cia402_od_*_begin` returns `CIA402_OD_HANDLE_INVALID`.

**Tick order** inside `axis_manager_tick` (`if (!motor_in_bl)` block):
1. `motor_od_proxy_tick`
   1. `poll_load_factor_sdo` — drains one-shot writes
   2. `tick_bootsync` — reads first so a slot that's both dirty AND being read back doesn't get its operator-typed value clobbered
   3. `poll_motor_proxy_sdo` — writes (round-robin across dirty slots)
   4. `tick_motor_save` — state machine advance
2. `home_sequencer_tick`

Setup sequencer runs **outside** the motor-in-bl guard because it self-cleans if writes get NO_OBJECT'd by the motor bootloader.

### 2. Reset-on-bootloader-entry

When the motor MCU transitions into its own bootloader, `axis_manager_tick` detects the rising edge and calls `reset_motor_od_submodules()` which calls each sub-module's `_reset` function.

**Every sub-module that holds a `cia402_od_handle_t` MUST have a reset call here.** Current list:
- `home_sequencer_reset()`
- `motor_od_proxy_reset()` (handles proxy handle, bootsync handle, save handle, load_factor handle in one shot)

If you add a new sub-module that takes a cia402 handle, add its `_reset` call to `reset_motor_od_submodules` in `axis_manager.c`.

### 3. Persist blob is APPEND-ONLY

The v6 blob layout (see `apply_persist_blob` / `capture_persist_blob` in `axis_manager.c`) may be extended only by:
1. Appending new fields at the tail.
2. Bumping `AXIS_PERSIST_VERSION`.
3. Sanitising any zero-value case in `apply_persist_blob` (older blobs zero-fill the new tail; the sanitiser promotes 0 → the real default). Example: `axis_role == 0 → PAN default`.

**Never** reorder, resize, or remove an existing field. If you need to do any of those, that's a semantic migration — write a per-version migrator function; do not rely on this rule.

`persist_load_or_upgrade` handles the in-memory upgrade automatically — operators do NOT lose config on version bumps.

### 4. Ownership split for CATEGORY-2 limits

`velocity_limit / accel_limit / position_limit_lo / position_limit_hi` are stored in TWO places:
- `s_axis.*` in axis_manager — the CMC-side authoritative value, used by `compute_desired` clamps. Position limits carry `+/-INFINITY` "unset" convention.
- `s_slots[SLOT_*].value` in motor_od_proxy — the motor-side finite value (0 = disabled for pos limits) that gets SDO-written.

Setters (`axis_manager_set_position_limit_lo` etc.) update **both** in one call. Bootsync updates the axis_manager mirror via the registered callback (`on_motor_bootsync` in axis_manager.c). Do not add a getter path that reads only one side.

## If you're adding a new sub-module

Checklist:
1. **Header:** create `app/axis_manager/xxx.h` with `xxx_init(void)`, `xxx_tick(void)`, and any narrow public API.
2. **Init call:** add `xxx_init()` to `axis_manager_init` in the correct order (before persist load if it owns persistable state).
3. **Tick call:** add `xxx_tick()` inside the correct guard in `axis_manager_tick` (inside `if (!motor_in_bl)` if it uses cia402 SDO; outside if it's CMC-local like `buttons_jog`).
4. **Reset:** if it holds a cia402 handle, add `xxx_reset()` to `reset_motor_od_submodules`.
5. **cia402 discipline:** every SDO call must defer if `cia402_od_*_begin` returns INVALID. Don't spin.
6. **Public API forwarders:** if the sub-module backs an OD entry, add a thin one-line forwarder in `axis_manager.c` under "SUB-MODULE FORWARDERS" so external modules don't need to include the sub-module header.
7. **Persist bytes:** if the sub-module owns operator-tunable state that must persist, expose a `get_xxx` / `set_xxx` accessor pair and call them from `capture_persist_blob` / `apply_persist_blob`. Follow the APPEND-ONLY blob rule.
8. **Makefile:** add the `.c` file to `Debug/app/axis_manager/subdir.mk` (four places: `C_SRCS`, `OBJS`, `C_DEPS`, `clean` rule) and `Debug/objects.list`.
9. **This README:** add a row to the "Files" table + a bullet under "Sub-module map" describing what it owns.

## Public API (unchanged by the refactor)

The C API in `axis_manager.h` is a one-to-one mirror of the OD entries. Each entry has a getter and, where it's RW or WO, a setter that returns `bool` (false → invalid input → caller translates to `MC_IF_OD_ERR_RANGE`).

```c
void axis_manager_init(void);
void axis_manager_tick(void);

axis_state_t   axis_manager_get_state(void);
axis_op_mode_t axis_manager_get_op_mode_actual(void);
float          axis_manager_get_position_actual(void);
/* ... etc ... */

bool axis_manager_request_enable(bool enable);
bool axis_manager_request_quick_stop(void);
bool axis_manager_request_clear_fault(void);

bool axis_manager_set_op_mode(uint8_t v);
bool axis_manager_set_joystick_value(float v);
/* ... etc ... */
```

## OD entries this module backs

| Index | Name | Type | Notes |
|---|---|---|---|
| `0x3000` | `axis_state` | U8 RO | DISABLED / READY / RUNNING / FAULT |
| `0x3001` | `axis_op_mode_actual` | U8 RO | echoes commanded once drive is enabled |
| `0x3002` | `axis_position_actual` | F32 RO | rad — from v4 cyclic header (REQ-0013) |
| `0x3003` | `axis_velocity_actual` | F32 RO | rad/s — TODO (telemetry-blob extraction) |
| `0x3004` | `axis_error_code` | U16 RO | mirrored from cyclic status |
| `0x3005` | `axis_error_register` | U8 RO | |
| `0x3006` | `axis_active_operation` | U8 RO | op arbiter state (MC_IF_OP_*) |
| `0x3010` | `axis_enable` | U8 RW | write 1/0; persistent latch |
| `0x3011` | `axis_quick_stop` | U8 WO | write 1 → drop QUICK_STOP + disable |
| `0x3012` | `axis_clear_fault` | U8 WO | write 1 → FAULT_RESET rising edge |
| `0x3013` | `axis_start_move` | U8 WO | write 1 → NEW_SETPOINT once setup applied |
| `0x3014` | `axis_auto_fault_clears` | U16 RO | since-boot count of auto-cleared faults |
| `0x3018` | `cmc_boot_request` | U8 WO | write MC_IF_PROG_START → CMC bootloader |
| `0x3020` | `axis_op_mode` | U8 RW | OFF / JOYSTICK / PROFILE_VELOCITY / PROFILE_POSITION / HOLD / TORQUE |
| `0x3021` | `axis_joystick_value` | F32 RW | -1.0..+1.0; live in JOYSTICK. Auto-updated on `axis_joystick_raw` write |
| `0x3022` | `axis_joystick_max_velocity` | F32 RO | derived from velocity_limit × joy_profile_scale |
| `0x3023` | `axis_target_velocity` | F32 RW | streamed in cyclic `velocity_setpoint` in PROFILE_VELOCITY mode |
| `0x3024` | `axis_target_position` | F32 RW | SDO-mirrored to motor `0x607A` |
| `0x3025` | `axis_target_time` | F32 RW | seconds; SDO-mirrored to motor `0x607B` in ms |
| `0x3026` | `axis_joystick_raw` | I32 RW | raw stick value; CMC normalises using cal entries |
| `0x3027..2A` | `axis_joystick_raw_center/full_pos/neg/deadband` | RW PERSIST | cal |
| `0x3030` | `axis_velocity_limit` | F32 RW | SDO-mirrored via motor_od_proxy to motor `0x2600:4` |
| `0x3031` | `axis_position_limit_lo` | F32 RW | motor `0x2600:6`; +/-INF = unset CMC-side |
| `0x3032` | `axis_position_limit_hi` | F32 RW | motor `0x2600:7`; same |
| `0x3033` | `axis_accel_limit` | F32 RW | motor `0x2600:5` |
| `0x3040` | `axis_home_command` | U8 WO | write 1 → home_sequencer starts |
| `0x3041` | `axis_home_status` | U8 RO | MC_IF_HOME_* mirror from motor |
| `0x3042` | `axis_is_homed` | U8 RO | fault_flags NOT_HOMED == 0 |
| `0x3043` | `axis_home_on_boot` | U8 RW PERSIST | auto-fire home once per boot when incremental |
| `0x3050` | `cmc_save_config` | U8 WO | write MC_IF_SAVE_MAGIC → persist_save |
| `0x3070` | `axis_role` | U8 RW PERSIST | CAMERAD_AXIS_* bitmap; PAN default |

## Layering

`axis_manager` is L3 in the architecture:

```
protocol modules (camerad, visca, ...)  <- L1
              ▼
            cmc_od                       <- L2 (OD dispatcher)
              ▼
         axis_manager                    <- L3 (this module + its sub-modules)
              ▼
            cia402                       <- L4 (codec + SPI cyclic + OD pipeline)
              ▼
       bsp/motor_spi                     <- L5
```

Strict rule: `axis_manager` and its sub-modules include `cia402.h` and the `Interface/` headers; they do NOT include anything from `app/od`, `app/camerad`, `app/visca`, or any other protocol module. Callers come down to it via `cmc_od`.

Sub-modules include `axis_manager.h` (for the op arbiter API) but NOT each other. If two sub-modules need to share something, expose it through `axis_manager` — that keeps the DAG a tree, not a graph.

## Refactor 2026-07-22

Moved from monolithic 2826-line file to orchestrator (~1550) + three sub-modules (~470 + ~570 + ~290). See git log for `d1e259f...` for the details. Rationale + risk analysis lived in the conversation that preceded the change; key decisions preserved here:

- **Op arbiter stayed in axis_manager** (not extracted). Would have created cross-coupling to every sub-module — the "call site" risk was higher than the LOC saved.
- **motor_save + load_factor merged into motor_od_proxy** rather than three separate files. They cooperate via the cia402 slot; keeping them in one file makes the cooperation line-of-sight.
- **Public API unchanged.** Every external call still goes through `axis_manager_*` — sub-modules are internal.
