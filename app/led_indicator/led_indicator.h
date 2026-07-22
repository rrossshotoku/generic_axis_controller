/*
 * app/led_indicator — RGB LED status pattern controller.
 *
 * Owns the small state machine that drives bsp/leds. Doesn't talk to the
 * timer directly; just decides the brightness scalar each tick and asks
 * bsp_leds to render (configured_rgb × brightness).
 *
 * Patterns:
 *   FAULT_RED     Solid full-brightness RED whenever axis_manager reports
 *                 AXIS_STATE_FAULT (CiA-402 FAULT bit from the cyclic
 *                 status header). Overrides operator colour + every other
 *                 pattern for unambiguous "attention needed" signalling.
 *   BOOT_SOLID    Solid ON at full configured colour for a few seconds
 *                 after boot, so the operator sees the unit is alive.
 *   NET_FLASH     3 quick flashes (off-on-off-on-off-on-off) on a network
 *                 link transition to UP. One-shot.
 *   BREATHING     Ramp up over 1 s, ramp down over 1 s, repeat. Triggered
 *                 while the motor reports moving.
 *   SOLID         Configured colour at full brightness. Default once boot
 *                 indication finishes and the motor is idle.
 *
 * Priority (high to low):
 *   1. FAULT_RED (while axis state == FAULT)
 *   2. NET_FLASH (one-shot, can interrupt boot or solid)
 *   3. BREATHING (while motor moving)
 *   4. BOOT_SOLID (during the boot indicator window)
 *   5. SOLID
 *
 * Colour is operator-tunable via OD 0x3060 (R), 0x3061 (G), 0x3062 (B) and
 * persisted in the axis_persist blob (AXIS_PERSIST_VERSION bumped 3 -> 4
 * when the LED bytes were added).
 */

#ifndef APP_LED_INDICATOR_H
#define APP_LED_INDICATOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void led_indicator_init(void);
void led_indicator_tick(void);

/* Operator-facing colour (OD 0x3060/61/62). The actual rendered colour
 * is (configured × current_brightness) where current_brightness comes
 * from the pattern (breathing ramp, flash on/off, solid 1.0, etc.). */
uint8_t led_indicator_get_r(void);
uint8_t led_indicator_get_g(void);
uint8_t led_indicator_get_b(void);
bool    led_indicator_set_r(uint8_t v);
bool    led_indicator_set_g(uint8_t v);
bool    led_indicator_set_b(uint8_t v);

/* External trigger — called by the network module (or wherever link-state
 * is detected) when the W6100 transitions from DOWN to UP. Queues a
 * 3-flash sequence; harmless to call again before the sequence finishes
 * (it just restarts). */
void led_indicator_signal_link_up(void);

/* Persist helpers — used by axis_manager's persist save/load so the LED
 * colour rides in the same blob as the other operator tunables (no
 * second persist region needed). axis_persist serialises the three U8s
 * directly. */
void led_indicator_capture_persist(uint8_t out_rgb[3]);
void led_indicator_apply_persist  (const uint8_t in_rgb[3]);

#ifdef __cplusplus
}
#endif

#endif /* APP_LED_INDICATOR_H */
