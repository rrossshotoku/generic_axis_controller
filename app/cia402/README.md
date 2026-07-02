# app/cia402

> **Wire format authority: `Interface/mc_if_protocol.h`** and `Interface/INTERFACE_SPEC.md`. **OD types and results: `Interface/mc_if_od.h`.** This module consumes those headers; it does not redefine constants from them. Any contract change is an Interface change (bumps `MC_IF_PROTOCOL_VERSION`) and must be flagged before editing.

## Purpose

Host-side counterpart of the CiA-402 state machine that runs on the motor MCU. Three responsibilities:

1. **Frame codec.** Build/validate the 64-byte SPI frames per `MC_IfFrameHeader_t` + `MC_IfFrameFooter_t`, with CRC-16/Modbus on header and payload separately.
2. **Cyclic exchange.** Drive a continuous 1 kHz `CYCLIC_CMD` / `CYCLIC_STATUS` exchange via `bsp/motor_spi`. The slave's command-timeout dead-man (30 ms per Interface) requires the master to keep feeding `CYCLIC_CMD`.
3. **OD pipeline.** Carry acyclic OD reads/writes on the same SPI transactions: `OD_READ/WRITE_REQ` preempts one cyclic tick; the matching `OD_READ/WRITE_RESP` (correlated by sequence number) arrives on a later tick.

## Owns

- The two 64-byte working buffers (`s_tx`, `s_rx`).
- The current `MC_IfCyclicCommand_t` state. **Protocol v3** shrank this to just `controlword`, `velocity_setpoint` (i32 scaled rad/s), `command_counter` — 10 bytes. Setup parameters (mode, position target, position-target time, profile_*) are SDO-only; they don't appear in the cyclic command. Written by `app/axis_manager` each tick via `cia402_set_cyclic_cmd()`. `cia402` owns `command_counter` and overwrites it on every send.
- `command_counter` and `sequence` — both incremented per frame.
- The latest received `MC_IfCyclicStatusHeader_t` + telemetry blob. Exposed to upper layers via `cia402_take_cyclic_status()`.
- One in-flight OD request (depth 1). Phase 6 hardening may widen.
- Diagnostic counters (`cia402_stats_t`).

## Does NOT do

- Decide motor policy (mode, targets, when to enable) — that's `app/motor_ctrl`.
- Talk SPI directly — calls `bsp/motor_spi` for transfers.
- Hold or define OD entries — the OD lives on the motor MCU.
- Call upward into other app modules. `app/od` polls `cia402_take_cyclic_status()` and reads via `cia402_od_*()`; the call direction is always downward.

## Public API

See `cia402.h`. Summary:

```c
void cia402_init(void);
void cia402_tick(void);    /* call from main_loop every pass; rate-limits internally to 1 kHz */

cia402_od_handle_t cia402_od_read_begin (uint16_t idx, uint8_t sub, MC_IfOdType_t type);
cia402_od_handle_t cia402_od_write_begin(uint16_t idx, uint8_t sub, MC_IfOdType_t type,
                                         const void *data, uint8_t len);
bool               cia402_od_poll(cia402_od_handle_t h,
                                  MC_IfOdResult_t *out_result,
                                  void *out_data, uint8_t *out_len);

bool cia402_take_cyclic_status(MC_IfCyclicStatusHeader_t *out_hdr,
                               uint8_t *out_blob, uint8_t *out_blob_len);

void cia402_get_stats(cia402_stats_t *out);
```

## Dependencies

- `Interface/mc_if_protocol.h`, `Interface/mc_if_od.h` (frozen contract).
- `bsp/motor_spi` — one 64-byte full-duplex transfer per cyclic tick.
- `bsp/time` — 1 kHz rate-limit + OD response timeout.

No `app/*` includes — preserves the downward-only dependency rule.

## Acceptance criteria

- `cia402_init` resets state and calls `motor_spi_init`. Returns with cyclic counter at 0, sequence at 0, no OD request pending, no fresh status.
- Each successful `cia402_tick` (post 1 ms cadence check) produces exactly one valid 64-byte TX frame on SPI3: sync `0xA55A`, version 1, correct `message_type`, payload length matching the payload struct, correct CRC-16/Modbus on header and payload, padded to 64.
- Continuous `cia402_tick` invocations at >1 kHz produce SPI frames at the 1 kHz rate-limit (not faster).
- The rate-limit is **drift-tolerant**: after a main-loop stall we send one frame and resume cadence — no catch-up burst onto SPI3. Long-term cadence drifts forward by the total stall time; `cia402_stats_t` exposes `max_cycle_gap_ms`, `late_cycles` and `total_drift_ms` for diagnosis.
- `cia402_od_read_begin` followed by repeated `cia402_od_poll` returns true within `OD_RESPONSE_TIMEOUT_MS` (30 ms) with the motor MCU's result. The 30 ms ceiling is chosen to fit inside the PC GUI's 50 ms retransmit window — slower responses would let the GUI retransmit before the in-flight request completes, which the CMC used to mis-report as a queue-full error.
- Out-of-order OD response, response with wrong sequence, or motor-side `ERROR` referencing the in-flight sequence all result in the OD request completing with a meaningful `MC_IfOdResult_t`.
- Bad-CRC / bad-sync / bad-version frames received are silently discarded; `cia402_stats_t` counters increment. The cyclic exchange continues.
- After a successful `motor_spi_transfer`, if the RX frame is `CYCLIC_STATUS`, `cia402_take_cyclic_status` returns true exactly once with that frame's header and blob; subsequent calls return false until the next status arrives.

## Notes

- **Cyclic command source (v3).** `app/axis_manager` calls `cia402_set_cyclic_cmd()` on every `axis_manager_tick` to push the streaming command (controlword + joystick_value). `cia402` sends it on the next 1 kHz transmission. Before axis_manager runs for the first time the struct is zero-initialised, so the very first frames after boot are quiescent — motor MCU treats them as "no command". Setup parameters (mode, targets, profile params, joystick scale) are NOT in the cyclic; axis_manager SDO-writes them via `cia402_od_write_begin()` and triggers execution via the `MC_IF_CW_NEW_SETPOINT` bit in the cyclic controlword.
- **Rate-limit cadence**: ms-resolution from `bsp/time`. Each tick of `main_loop` calls `cia402_tick`, which returns immediately unless ≥1 ms has elapsed since the last SPI transfer. Uses a **drift-tolerant** pattern (sets `s_last_tick_ms = now` after each send), so a stall in `main_loop` causes one delayed frame, not a back-to-back burst — important because the motor MCU's slave SPI DMA rearm path cannot absorb bursts (see `Interface/REQUESTS.md` REQ-0007). The trade-off is that long-term cadence drifts forward by the total stall time; `cia402_stats_t` surfaces the worst-case gap, the count of late cycles, and the accumulated drift so the drift can be monitored. At 1 kHz × 64 B at 6 MHz SPI ≈ 85 µs of CPU per second = ~8.5% — acceptable in polled mode.
- **OD request priority**: a queued OD request preempts the next cyclic frame. Only one cyclic frame is skipped per OD round-trip (request + response = 2 frames). This satisfies the 30 ms command-timeout window.
- **Frame integrity**: CRC-16/Modbus (poly 0xA001 reflected, init 0xFFFF, no final XOR) over header bytes [0..7] and over payload bytes separately. Implemented bit-by-bit; swap to a table if profile demands.
- **Layering**: this module is L4 in the architecture catalogue. Upper layers (`app/od`, `app/motor_ctrl`) call down; `cia402` calls only into `bsp/`. No `app/*` includes here.
