/*
 * bsp/leds — see leds.h for the contract.
 */

#include "leds.h"

#include "stm32g4xx_hal.h"
#include "tim.h"            /* htim1 extern from CubeMX-generated Core/Src/tim.c */

/* TIM1 period set in MX_TIM1_Init (Core/Src/tim.c). If that constant
 * changes in CubeMX, this must match — otherwise set_rgb(255,...) won't
 * reach 100% duty. Asserted at init. */
#define LEDS_TIM_PERIOD  4250u

static uint32_t scale_8bit_to_period(uint8_t v)
{
    /* Map 0..255 → 0..LEDS_TIM_PERIOD with rounding. v=0 -> 0,
     * v=255 -> exactly LEDS_TIM_PERIOD (full duty). */
    return ((uint32_t)v * LEDS_TIM_PERIOD + 127u) / 255u;
}

bool bsp_leds_init(void)
{
    /* Sanity: if CubeMX changes the period, our scaling is wrong.
     * Fail loud rather than silently mis-scale. */
    if (htim1.Init.Period != LEDS_TIM_PERIOD) {
        return false;
    }

    /* Start at 0% on all three channels so an unset LED doesn't flash
     * during init before led_indicator imposes its boot pattern. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) return false;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) return false;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) return false;
    return true;
}

void bsp_leds_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, scale_8bit_to_period(r));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, scale_8bit_to_period(g));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, scale_8bit_to_period(b));
}
