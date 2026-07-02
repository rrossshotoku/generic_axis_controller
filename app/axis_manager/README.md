# app/axis_manager

The high-level command surface for the motor. Every protocol module that drives the motor (CAMERAD, VISCA, the PC tool's OD writer, future additions) talks to this module — and **only** this module — via the CMC-owned OD entries it backs.

## Owns

The data backing the `MC_IF_OWNER_CMC` block of the OD (`0x3000-0x303F` in `Interface/mc_if_od.h`):

| Range | Purpose |
|---|---|
| `0x3000-0x300F` | State (RO): axis_state, op_mode_actual, position/velocity actual, error code/register |
| `0x3010-0x301F` | Command triggers: enable/disable, quick_stop, clear_fault |
| `0x3020-0x302F` | Op-mode + per-mode targets: joystick value, target velocity, target position+time |
| `0x3030-0x303F` | Per-axis limits (PERSIST): velocity, position lo/hi, acceleration |

Reads on these entries are answered from `axis_manager`'s local state; writes are validated and either stored (targets, limits) or processed as triggers (enable, quick_stop, clear_fault). **No SPI traffic is generated** by an OD access on the CMC-owned range.

## Does NOT do

- Talk SPI to the motor MCU directly — that's `cia402`'s job.
- Know about CAMERAD, VISCA, the PC tool, or any specific protocol. Protocol modules sit above it.
- Hold motor MCU OD entries (`0x1xxx`, `0x2xxx`, `0x6xxx`) — those live on the motor MCU and are routed through `cia402` by the existing OD bridge.

## Public API

The C API in `axis_manager.h` (called by `cmc_od` to back the OD dispatch) is a one-to-one mirror of the OD entries. Each entry has a getter and, where it's RW or WO, a setter that returns `bool` (false → invalid input → caller translates to `MC_IF_OD_ERR_RANGE`).

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

## v3 status (what's wired, what isn't)

Protocol v3 splits the motor command path into **streaming** (cyclic) and **setup** (SDO). The cyclic carries only `controlword`, `joystick_value`, `command_counter`; setup parameters are SDO-written to the motor MCU's OD; moves are triggered by rising-edging `MC_IF_CW_NEW_SETPOINT`.

- ✅ **Data model + accessors** for every CMC-owned OD entry (`0x3000-0x3033`).
- ✅ **Actuals derived from `cia402_peek_cyclic_status`** — `error_code`, `axis_state` from statusword bits.
- ✅ **Setup-sequencer.** `s_applied` snapshot tracks what the motor MCU has confirmed (via OD_WRITE_RESP OK). Each tick, if `desired != applied` for any setup field, axis_manager issues one SDO write (depth-1 queue, walks parameters in fixed priority order). Initial sentinel values force the first sync to write everything. Tracked fields: `mode_of_operation`, `target_position`, `target_position_time_ms`, `profile_velocity`, `profile_acceleration`, `profile_deceleration`. (Note: `target_velocity` is no longer SDO-written in v3 — velocity is streamed in the cyclic `velocity_setpoint` field instead.)
- ✅ **Cyclic-command construction (v3).** Tiny — `controlword` + `velocity_setpoint` (i32 scaled rad/s) + `command_counter`. Translation:
  - `OFF` or `!enable_latch` → `controlword = QUICK_STOP` (idle), `velocity_setpoint = 0`. Motor MCU stays disabled.
  - Enabled + `JOYSTICK` → `controlword = QUICK_STOP | ENABLE`. `velocity_setpoint = joystick_value × joystick_max_velocity` computed locally each cycle and clamped to `velocity_limit`. The motor MCU has no joystick concept — to it, this is just a streaming velocity target in PROFILE_VELOCITY mode.
  - Enabled + `PROFILE_VELOCITY` → `controlword = QUICK_STOP | ENABLE`. `velocity_setpoint = target_velocity_rad_s` clamped to `velocity_limit`, streamed every cycle. Same shape on the wire as JOYSTICK — the difference is just where the value comes from on the CMC side.
  - Enabled + `PROFILE_POSITION` → `controlword = QUICK_STOP | ENABLE`. `velocity_setpoint = 0` (ignored by motor in position mode anyway). Position targets travel via SDO; the operator triggers execution by writing `axis_start_move = 1` (OD `0x3013`), which pulses `NEW_SETPOINT` for a few cycles once the sequencer is idle.
  - `HOLD` → `controlword = QUICK_STOP | ENABLE | HALT`, `velocity_setpoint = 0`. Motor MCU holds current position.
  - `quick_stop` pulse: clears `enable_latch`, drops `QUICK_STOP` bit for one cycle, abandons any pending `start_move` trigger.
  - `clear_fault` pulse: rising-edge `FAULT_RESET` for one cycle.
  - `FAULT` state: neutral (`velocity_setpoint = 0`) until `clear_fault` is pulsed.
- ✅ **Start-move trigger** (OD `0x3013`). PROFILE_POSITION only — velocity modes don't need a trigger. The pulse is deferred until the setup sequencer is idle, so the motor MCU never sees `NEW_SETPOINT` before its stored setup matches the LCMC's desired values. Held high for `NEW_SETPOINT_PULSE_CYCLES = 3` cycles for robustness against single dropped cyclic frames.
- ⚠ **Actuals from telemetry blob** (position_actual, velocity_actual) — not yet extracted from the mapped blob. Needs map-aware unpacking using the active `map_version`.

## OD entries this module backs

| Index | Name | Type | Notes |
|---|---|---|---|
| `0x3000` | `axis_state` | U8 RO | DISABLED / READY / RUNNING / FAULT |
| `0x3001` | `axis_op_mode_actual` | U8 RO | echoes commanded once drive is enabled |
| `0x3002` | `axis_position_actual` | F32 RO | rad — TODO (telemetry-blob extraction) |
| `0x3003` | `axis_velocity_actual` | F32 RO | rad/s — TODO |
| `0x3004` | `axis_error_code` | U16 RO | mirrored from cyclic status |
| `0x3005` | `axis_error_register` | U8 RO | |
| `0x3010` | `axis_enable` | U8 RW | write 1/0; persistent latch |
| `0x3011` | `axis_quick_stop` | U8 WO | write 1 → drop QUICK_STOP + disable |
| `0x3012` | `axis_clear_fault` | U8 WO | write 1 → FAULT_RESET rising edge |
| `0x3013` | `axis_start_move` | U8 WO | write 1 → NEW_SETPOINT once setup applied |
| `0x3020` | `axis_op_mode` | U8 RW | OFF / JOYSTICK / PROFILE_VELOCITY / PROFILE_POSITION / HOLD |
| `0x3021` | `axis_joystick_value` | F32 RW | -1.0..+1.0; live in JOYSTICK. Updated automatically when `axis_joystick_raw` is written; can also be written directly to bypass the raw-side cal. LCMC-only — never reaches the motor MCU |
| `0x3022` | `axis_joystick_max_velocity` | F32 RW PERSIST | rad/s at full stick (symmetric — applies in both directions). LCMC-only |
| `0x3023` | `axis_target_velocity` | F32 RW | streamed in cyclic `velocity_setpoint` when in PROFILE_VELOCITY mode (NOT SDO-mirrored) |
| `0x3024` | `axis_target_position` | F32 RW | SDO-mirrored to motor `0x607A` |
| `0x3025` | `axis_target_time` | F32 RW | seconds; converted to ms and SDO-mirrored to motor `0x607B target_position_time_ms` |
| `0x3026` | `axis_joystick_raw` | I32 RW | raw stick value from a protocol module; LCMC normalises into `axis_joystick_value` using the cal entries below |
| `0x3027` | `axis_joystick_raw_center` | I32 RW PERSIST | rest position (raw units). Default `0` |
| `0x3028` | `axis_joystick_raw_full_pos` | I32 RW PERSIST | raw value at full positive deflection. Default `+32767` |
| `0x3029` | `axis_joystick_raw_full_neg` | I32 RW PERSIST | raw value at full negative deflection. Default `-32767` |
| `0x302A` | `axis_joystick_raw_deadband` | U32 RW PERSIST | ± around center → output 0. Default `0` |
| `0x3030` | `axis_velocity_limit` | F32 RW | SDO-mirrored to motor `0x6081 profile_velocity` |
| `0x3031` | `axis_position_limit_lo` | F32 RW | enforced LCMC-side (clamps target_position before SDO) |
| `0x3032` | `axis_position_limit_hi` | F32 RW | same |
| `0x3033` | `axis_accel_limit` | F32 RW | SDO-mirrored to motor `0x6083` and `0x6084` (symmetric accel/decel) |

## Typical control flow

**PROFILE_POSITION shot recall:**
1. Protocol module writes `axis_op_mode = PROFILE_POSITION`, `axis_target_position = X`, `axis_target_time = Y` to the CMC OD. Each write returns immediately (CMC-local).
2. Over the next few cycles, axis_manager fires SDO writes for mode (`0x6060`), target_position (`0x607A`), target_position_time_ms (`0x607B`), profile_velocity (`0x6081`), profile_acceleration (`0x6083`), profile_deceleration (`0x6084`). Each is one cyclic round-trip; total ~6 cycles ≈ 6 ms.
3. Protocol module writes `axis_start_move = 1`. axis_manager waits until the sequencer is idle (all setup applied), then pulses NEW_SETPOINT for 3 cycles.
4. Motor MCU's trajectory engine executes; statusword reports TARGET_REACHED when done.

**Joystick mode (or PROFILE_VELOCITY — same shape on the wire):**
1. Protocol module writes `axis_joystick_max_velocity = X`, `axis_op_mode = JOYSTICK`, `axis_enable = 1` (or for PROFILE_VELOCITY, writes `axis_target_velocity` and `axis_op_mode = PROFILE_VELOCITY`).
2. axis_manager SDO-writes `mode_of_operation = 3 (PROFILE_VELOCITY)` if not already set. No `joystick_scale` write — the motor MCU has no joystick concept.
3. Each subsequent cyclic frame carries `velocity_setpoint = joystick_value × joystick_max_velocity` (JOYSTICK) or `velocity_setpoint = target_velocity_rad_s` (PROFILE_VELOCITY), in scaled wire units (×`MC_IF_VEL_SCALE`). The motor MCU follows it live.
4. No NEW_SETPOINT trigger — velocity modes don't need one.

## Layering

`axis_manager` is L3 in the architecture:

```
protocol modules (camerad, visca, ...)  <- L1
              ▼
            cmc_od                       <- L2 (OD dispatcher)
              ▼
         axis_manager                    <- L3 (this module)
              ▼
            cia402                       <- L4 (codec + SPI cyclic + OD pipeline)
              ▼
       bsp/motor_spi                     <- L5
```

Strict rule: `axis_manager` includes `cia402.h` and the Interface headers; it does NOT include anything from `app/od`, `app/camerad`, `app/visca`, or any other protocol module. Callers come down to it via `cmc_od`.

## Future expansion

- **Actuals extraction from the telemetry blob**. The motor MCU sends mapped PDOs as a packed blob per the active `0x2A00` map. axis_manager needs to know that map and pull `0x6064 position_actual` / `0x606C velocity_actual` (scaled CiA-402 ints) out of the blob.
- **State-machine refinement.** Today the controlword bits we use (ENABLE, QUICK_STOP, HALT, FAULT_RESET) are a simplified set; real CiA-402 has a fuller PDS state machine (Ready to Switch On / Switched On / Operation Enabled / etc.). Worth folding in if the motor MCU ever needs us to walk that explicitly.
- **Persistence** for `0x3030-0x303F` once `bsp/flash` lands.
- **Multi-axis** support — add axis 1 at `0x3100-0x31FF`, axis 2 at `0x3200-0x32FF`. The accessor API would gain an `axis` parameter; the dispatch in `cmc_od` would index into a `s_axis[N]` array.
