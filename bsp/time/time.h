/*
 * bsp/time — monotonic millisecond tick.
 *
 * Sole time source for the codebase. No other module calls HAL_GetTick directly.
 * See bsp/time/README.md for the contract.
 */

#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void     time_init(void);
uint32_t time_ms(void);
uint32_t time_elapsed_ms(uint32_t since);
bool     time_after(uint32_t a, uint32_t b);

/* Microsecond-resolution monotonic counter, backed by the Cortex-M4 DWT
 * cycle counter (enabled in time_init). Resolution = 1 us. Wraps every
 * ~71 min at 170 MHz core — uint32 subtraction is wrap-safe so
 * time_elapsed_us(since) is correct for deltas < ~35 min. Use for
 * pacing where ms granularity isn't enough (e.g. 1 kHz cyclic loop). */
uint32_t time_us(void);
uint32_t time_elapsed_us(uint32_t since);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIME_H */
