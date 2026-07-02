/*
 * bsp/buttons — see buttons.h for the contract.
 */

#include "buttons.h"

#include "stm32g4xx_hal.h"
#include "main.h"               /* CubeMX-generated UP_BUTTON_* / DOWN_BUTTON_* */

#define BUTTON_DEBOUNCE_SAMPLES  3u

static uint8_t s_up_filter;        /* saturating counter, 0..BUTTON_DEBOUNCE_SAMPLES */
static uint8_t s_down_filter;
static bool    s_up_pressed;       /* debounced output */
static bool    s_down_pressed;

void bsp_buttons_init(void)
{
    /* GPIO pin init lives in CubeMX-generated Core/Src/gpio.c. Nothing
     * to do here yet — the function exists so the dependency is explicit
     * in main_loop_init and a future board with extra button setup has
     * an obvious place to land. */
    s_up_filter    = 0;
    s_down_filter  = 0;
    s_up_pressed   = false;
    s_down_pressed = false;
}

void bsp_buttons_tick(void)
{
    bool up_now   = (HAL_GPIO_ReadPin(UP_BUTTON_GPIO_Port,   UP_BUTTON_Pin)   == GPIO_PIN_SET);
    bool down_now = (HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET);

    /* Saturating accumulators: count up while pressed, snap to 0 when not.
     * Flipping the debounced state at saturation gives a clean N-sample
     * hold both directions; the reset-to-0 on a single LOW sample makes
     * release fast (good for a "let go" safety feel). */
    s_up_filter   = up_now   ? (uint8_t)((s_up_filter   < BUTTON_DEBOUNCE_SAMPLES) ? s_up_filter   + 1u : BUTTON_DEBOUNCE_SAMPLES) : 0u;
    s_down_filter = down_now ? (uint8_t)((s_down_filter < BUTTON_DEBOUNCE_SAMPLES) ? s_down_filter + 1u : BUTTON_DEBOUNCE_SAMPLES) : 0u;

    s_up_pressed   = (s_up_filter   >= BUTTON_DEBOUNCE_SAMPLES);
    s_down_pressed = (s_down_filter >= BUTTON_DEBOUNCE_SAMPLES);
}

bool bsp_buttons_up_pressed  (void) { return s_up_pressed;   }
bool bsp_buttons_down_pressed(void) { return s_down_pressed; }
