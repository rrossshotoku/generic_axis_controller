/*
 * bsp/time — monotonic millisecond tick backed by SysTick.
 *
 * SysTick is configured by CubeMX in HAL_Init to fire at 1 kHz, calling
 * HAL_IncTick each interrupt. We override HAL_GetTick (declared weak in
 * stm32g4xx_hal.c) so the entire HAL — and our app — share one time source.
 */

#include "time.h"
#include "stm32g4xx_hal.h"

void time_init(void)
{
    /* SysTick was started by HAL_Init() before main_loop_init runs.
     * We additionally enable the DWT cycle counter so time_us() works.
     * Idempotent — bsp/profile (if it runs later) won't undo this.
     * Both modules touch the same registers; either order is fine. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT      = 0;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t time_ms(void)
{
    return HAL_GetTick();
}

uint32_t time_elapsed_ms(uint32_t since)
{
    /* uint32 subtraction is wrap-safe across the 49.7-day rollover. */
    return time_ms() - since;
}

bool time_after(uint32_t a, uint32_t b)
{
    /* True if a is later than b under wrap-safe arithmetic.
     * Treats a "distance" of more than 2^31 as wrap-around. */
    return (int32_t)(a - b) > 0;
}

uint32_t time_us(void)
{
    /* DWT->CYCCNT runs at SystemCoreClock Hz. Convert cycles -> us by
     * dividing by (Hz / 1e6). On a 170 MHz core that's /170. */
    uint64_t cyc_per_us = SystemCoreClock / 1000000u;
    if (cyc_per_us == 0u) cyc_per_us = 1u;
    return (uint32_t)((uint64_t)DWT->CYCCNT / cyc_per_us);
}

uint32_t time_elapsed_us(uint32_t since)
{
    return time_us() - since;
}
