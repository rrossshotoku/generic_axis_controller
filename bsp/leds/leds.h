/*
 * bsp/leds — RGB LED PWM driver.
 *
 * Hardware: STM32G431RB Nucleo with an external RGB LED on PC0/PC1/PC2,
 * each driven by a TIM1 PWM channel:
 *   PC0  TIM1_CH1  R
 *   PC1  TIM1_CH2  G
 *   PC2  TIM1_CH3  B
 *
 * TIM1 is configured by CubeMX (Core/Src/tim.c MX_TIM1_Init) at ~40 kHz
 * with a 12-bit period (4250). bsp_leds_init starts the three channels;
 * bsp_leds_set_rgb takes 8-bit per channel and scales internally.
 *
 * Layering: this is the ONLY place that talks to TIM1's PWM channels.
 * App-layer modules (led_indicator) call set_rgb only — they MUST NOT
 * include stm32g4xx_hal.h or main.h to drive the timer directly.
 */

#ifndef BSP_LEDS_H
#define BSP_LEDS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start TIM1 PWM on all three channels at 0% duty (LEDs off). Called
 * once from main_loop_init AFTER MX_TIM1_Init. Returns false if HAL
 * failed to start any channel — caller can log and continue (LEDs just
 * won't light). */
bool bsp_leds_init(void);

/* Set the RGB output, 0..255 per channel. 0 = off, 255 = full brightness.
 * Linear in PWM duty (not gamma-corrected — perceptual brightness is the
 * caller's concern if it matters). Returns immediately; PWM hardware
 * latches the new compare value on the next timer reload. */
void bsp_leds_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LEDS_H */
