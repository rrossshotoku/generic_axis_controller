/*
 * bsp/buttons — debounced read of the on-board UP/DOWN push-buttons.
 *
 * Hardware: STM32G431RB Nucleo, PB1 (DOWN) and PB2 (UP), pull-down enabled
 * in CubeMX so a press reads HIGH. GPIO mode + pin init lives in
 * Core/Src/gpio.c (CubeMX-generated); this module just reads + debounces.
 *
 * Layering: this is the ONLY place in the firmware that talks to the
 * button GPIOs. App-layer modules (axis_manager etc.) must call the
 * accessors here — they MUST NOT include stm32g4xx_hal.h or main.h to
 * read these pins directly.
 *
 * Debounce: 3 consecutive same-direction samples to flip the debounced
 * state. Both press and release are debounced (~3 ms at a 1 kHz call
 * rate — short enough to be imperceptible, long enough to kill mechanical
 * bounce that would otherwise queue spurious SDO writes).
 */

#ifndef BSP_BUTTONS_H
#define BSP_BUTTONS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* No-op today (CubeMX-generated gpio.c already initialises the pins),
 * but kept as the explicit init hook so future button-related setup
 * lands in one obvious place. */
void bsp_buttons_init(void);

/* Sample the GPIOs and advance the debounce filters. Must be called
 * from the main loop every iteration (or every tick of whatever calls
 * pressed_up/pressed_down) — the filter cadence is tied to call rate.
 * At a 1 kHz loop rate the debounce window is ~3 ms. */
void bsp_buttons_tick(void);

/* Debounced state — true while the button is firmly held down. */
bool bsp_buttons_up_pressed  (void);
bool bsp_buttons_down_pressed(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUTTONS_H */
