# bsp/time

## Purpose
Monotonic millisecond tick. Sole time source for the entire codebase. No other module calls `HAL_GetTick()` directly.

## Owns
- A monotonic millisecond counter backed by SysTick (or a hardware timer if SysTick is reserved later).
- Wrap-safe elapsed-time arithmetic.

## Does NOT do
- Wall-clock time or date. No RTC.
- Microsecond-resolution timing. Use a hardware timer directly if you ever need µs precision (motor PID lives on a different MCU, so the CMC doesn't need it).

## Public API
```c
void     time_init(void);
uint32_t time_ms(void);                                  /* monotonic, wraps every 49.7 days */
uint32_t time_elapsed_ms(uint32_t since);                /* wrap-safe (since must be from time_ms) */
bool     time_after(uint32_t a, uint32_t b);             /* a > b in wrap-safe arithmetic */
```

## Dependencies
- `Drivers/STM32G4xx_HAL_Driver` (SysTick or TIM).

## Acceptance criteria
- `time_ms()` increments by 1 every millisecond, ±1 ms.
- `time_elapsed_ms(start)` returns the correct elapsed time across a wrap.
- A 49.7-day soak test never produces a non-monotonic return value.

## Notes
- Overrides STM32 HAL's `HAL_GetTick` so HAL timeouts use this time source. Implementation provides `uint32_t HAL_GetTick(void) { return time_ms(); }`.
- HAL tick priority (NVIC) must be the lowest application IRQ priority so user IRQs don't delay timekeeping — confirm in `stm32g4xx_hal_msp.c`.
- Watchdog kick cadence in `main_loop` reads `time_ms()`; if this stops ticking the watchdog will reset the device (which is the desired behaviour).
