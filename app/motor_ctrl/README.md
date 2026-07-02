# app/motor_ctrl

> **The motor MCU's interface is defined by `Interface/`.** This module *uses* that interface via `app/cia402`. It implements CMC-side *policy* (which CiA-402 mode we use, when to enable, how to plan a trajectory). The wire-format contract is not redefined here.

## Purpose
High-level motor API and trajectory generator. Translates CMC-level intents ("jog at velocity V", "move to position P over T ms", "stop", "fault-reset") into the per-cycle `CYCLIC_CMD` fields passed to `app/cia402`, which packs and sends them over SPI.

## Owns
- The trajectory generator: trapezoidal velocity profile (accel ramp → cruise → decel ramp).
- The velocity setpoint streaming during a fade — emits a new target velocity at ~25 ms cadence until the move completes.
- The cached snapshot of motor state read back from CiA 402 (`statusword`, actual position, actual velocity, fault flags).
- The "is the motor moving / on shot" derived flags consumed by `cmc_state`.

## Does NOT do
- Hold the CiA 402 state machine — that's `cia402`.
- Encode SDO bytes — `cia402` does.
- Talk SPI directly — `cia402` calls `bsp/motor_spi`.

## Public API
```c
void motor_ctrl_init(void);
void motor_ctrl_tick(void);     // streams velocity setpoints, polls status

/* Lifecycle */
motor_status_t motor_enable(void);            // walk to CiA 402 Operation Enabled
motor_status_t motor_disable(void);           // back to Switch On Disabled
motor_status_t motor_fault_reset(void);

/* Motion */
motor_status_t motor_jog(int16_t velocity_counts_per_sec);     // direct PV
motor_status_t motor_move_to(int32_t target, uint32_t duration_ms); // CMC trajectory
motor_status_t motor_stop(void);                                 // ramp to zero

/* Status (cached, refreshed each tick) */
bool     motor_is_moving(void);
bool     motor_is_on_shot(void);
int32_t  motor_actual_position(void);
uint16_t motor_statusword(void);
uint32_t motor_fault_flags(void);
```

## Dependencies
- `cia402` (drive state machine + SDO read/write).
- `od`     (reads motor limits and acceleration/deceleration parameters as OD entries).
- `config` (for soft-limit clipping of target positions).

## Acceptance criteria
- `motor_enable()` walks the drive from Switch On Disabled → Ready to Switch On → Switched On → Operation Enabled, with timeouts on each step.
- `motor_jog(0)` returns the drive to standstill within one acceleration period.
- `motor_move_to(target, T)` produces a trapezoidal velocity profile such that actual position reaches target within ±tolerance at time T.
- A new `motor_move_to` or `motor_jog` call during a fade cleanly cancels the in-progress trajectory.
- Soft limits are respected: targets outside the configured per-axis bounds are clipped (and logged).
- Profile velocity setpoint updates happen at ≤25 ms intervals during a fade — verifiable on logic analyser via SPI traffic.

## Notes
- **Policy default: Profile Velocity** (CiA-402 mode `3`). CMC streams velocity setpoints during a fade. The Interface contract (`MC_IfCyclicCommand_t` in `Interface/mc_if_protocol.h`) carries `mode_of_operation` + all three targets per cycle, so switching to Profile Position or Cyclic Sync Position is a single-field change here, not a protocol change. If the policy ever shifts (e.g. for shot recall it would be cleaner to set Profile Position and let the motor MCU run the trajectory), update *this README* and the affected code; `cia402` and `Interface/` need no change.
- Targets are sent as **scaled integers** per the `MC_IF_*_SCALE` constants in `Interface/mc_if_od.h` — position in 1e-5 rad, velocity in 1e-3 rad/s. Trajectory generation works in those units throughout.
- "On shot" means actual position is within tolerance of the last commanded target *and* not currently fading. Tolerance is an OD entry (motor-side), settable via the web through the OD bridge.
- Failure modes (timeout entering `MC_IF_NODE_RUNNING`, `statusword` reports fault, SPI timeout) propagate through `motor_status_t` and are logged.
- This module is the only place that uses `cia402_set_*` to compose `CYCLIC_CMD`. Other modules (e.g. `app/od` for arbitrary OD reads/writes initiated by a PC) interact with the motor MCU via `cia402_od_*` instead.
