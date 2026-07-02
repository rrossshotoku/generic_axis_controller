# bsp/motor_spi

> **The frame size and SPI configuration are owned by `Interface/INTERFACE_SPEC.md`.** This module *implements* the master side of that contract over SPI3 on the STM32G431.

## Purpose
Run the SPI master that exchanges one **fixed 64-byte full-duplex frame** with the motor MCU per transaction. The frame contents are entirely the concern of `app/cia402`; this module only owns the SPI transfer itself, the NSS line, and a small request queue so the OD network port (`app/od`) can interleave OD reads/writes alongside the cyclic exchange driven by `app/motor_ctrl`.

## Owns
- Init of SPI3 + chip-select GPIO for the motor MCU. SPI mode 0 (CPOL=0, CPHA=0), 8-bit, MSB-first per `INTERFACE_SPEC.md §1`. Clock ≤ ~6 MHz initially (raise after bring-up, see Open Points).
- One active transaction at a time. NSS is asserted, exactly 64 bytes are clocked in both directions, NSS is deasserted. **NSS edges are the slave's frame-boundary sync** — never split a 64-byte frame across multiple `motor_spi_transfer` calls.
- A small request queue (depth ~4 initially) so `app/od` can submit OD requests for transmission on the next available transaction without disturbing `app/motor_ctrl`'s cyclic cadence. Each queued request is a 64-byte buffer (already framed by `cia402`); the queue is FIFO.

## Does NOT do
- Interpret payload bytes. The 64 bytes are an opaque `MC_IfFrame_t` to this module.
- Compute CRCs. Done by `app/cia402`.
- Decide when to send cyclic vs. OD. `app/motor_ctrl` calls the cyclic path each cycle; `app/od` enqueues OD requests as a side band; the queue arbitrates.
- Recover the motor MCU on protocol error. CRC validation lives in `app/cia402`; safe-state behaviour lives on the motor MCU itself.

## Public API
```c
void motor_spi_init(void);

/* Perform one 64-byte full-duplex transaction.
 * tx and rx may be the same buffer or different. Blocks with a bounded
 * timeout (default 5 ms). Returns 0 on success, <0 on error/timeout. */
int32_t motor_spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t timeout_ms);

/* Queue a 64-byte frame for transmission on the next opportunity. The
 * cyclic caller (motor_ctrl) checks the queue before composing its own
 * CYCLIC_CMD; if the queue is non-empty, the queued frame is sent
 * instead and the cyclic frame is deferred by one tick. Used by app/od
 * to pipeline OD_READ_REQ / OD_WRITE_REQ onto the SPI link.
 *
 * Returns true if accepted, false if the queue is full. */
bool motor_spi_queue_oneshot(const uint8_t buf[64]);

/* Non-blocking peek: how many one-shots are waiting? Used by motor_ctrl
 * to decide whether to defer its cyclic frame this tick. */
uint8_t motor_spi_queue_depth(void);
```

## Dependencies
- `Drivers/STM32G4xx_HAL_Driver` (SPI3 HAL; DMA mode preferred — see notes).

## Acceptance criteria
- A 64-byte transfer to a connected motor MCU round-trips correctly: bytes clocked out arrive intact at the slave; bytes clocked in are read back without overrun.
- A 5 ms timeout returns a negative result rather than blocking forever.
- CS is deasserted on timeout, error, and successful completion — never left low.
- Queue overflow is detected and reported (`motor_spi_queue_oneshot` returns false); the cyclic channel is unaffected.
- Sustained 1 kHz cyclic transfer with occasional one-shot OD requests runs without dropping frames (verified on logic analyser).

## Notes
- **DMA recommended.** The reference uc_camd_interface project uses DMA on the W6100 SPI for throughput; we're using polled on the W6100 because the load is light. The motor SPI is the opposite case — 1 kHz × 64 bytes = 64 KB/s sustained, with a tight per-cycle budget. Polled HAL_SPI_TransmitReceive at 6 MHz would take ~85 µs per frame busy-waiting. DMA frees the CPU; on completion (RxCplt callback) the next cycle's prep work can begin. Plan to use HAL_SPI_TransmitReceive_DMA from day one for this bus.
- **No clock gap between bytes** within a transaction — the slave's DMA expects a continuous 64-byte stream per NSS-low window.
- **NSS hardware vs software**: the slave's DMA re-syncs on the NSS edge, so the master must drive NSS as a real GPIO per-transaction (not HW NSS, which the STM32 SPI peripheral toggles per-byte). Configure NSS as a software-driven GPIO, asserted before SPI start and deasserted after the last byte clocks out.
- **Speed ramp-up.** Per `INTERFACE_SPEC.md §1` open points: start at ~6 MHz, raise once bring-up is stable.
- **Queue policy.** Drop-newest on overflow is the safe choice for the OD side (an unsendable OD read is reported back to the caller). The cyclic channel is never queued — it composes a fresh `CYCLIC_CMD` every tick from `app/motor_ctrl` state.
