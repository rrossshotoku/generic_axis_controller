# app/led_indicator

> **Wire authority:** none — CMC-local. **Hardware authority:** `bsp/leds` drives the TIM1 PWM channels.

## Purpose

State/motion-aware policy layer for the on-board RGB status LED. Owns the operator-configurable colour (persisted with `axis_manager`'s config blob) and the render patterns: solid at rest, breathing during motion, 3× flash on network link-up.

## Owns

- The RGB colour (operator-tunable via `axis_manager` OD entries + web UI, persisted). Default magenta so a fresh unit is visibly alive rather than dark.
- The three visual patterns:
  1. **Boot solid** — full brightness for `BOOT_SOLID_MS` (3 s) after `led_indicator_init` so the operator sees "powered up."
  2. **Link-up flash** — three 100 ms ON / 100 ms OFF cycles on the rising edge of `net_link_up()`.
  3. **Idle vs motion** — solid at full brightness when the axis is at rest; perceptually-exponential breathing (squared triangle 0 → 255 → 0 over 1 s) while `axis_manager_is_moving()` returns true. Motion-stop finishes the current pulse before returning to solid — no jump.

## Does NOT do

- Talk directly to the timer registers. Uses `bsp_leds_set_rgb()`.
- Store the colour to flash. `led_indicator_capture_persist` / `_apply_persist` are called by `axis_manager` when it (de)serialises its config blob.
- Fault indication. Faults are surfaced through the diagnostics OD + web UI, not the LED.

## Public API

```c
void    led_indicator_init(void);
void    led_indicator_tick(void);

/* Colour accessors backing OD entries. */
uint8_t led_indicator_get_r(void);
bool    led_indicator_set_r(uint8_t v);
/* ...same for g, b... */

/* Persist blob co-tenants (axis_manager owns the blob). */
void    led_indicator_capture_persist(uint8_t out_rgb[3]);
void    led_indicator_apply_persist(const uint8_t in_rgb[3]);

/* External trigger — e.g. cmc_state signalling something worth flashing. */
void    led_indicator_signal_link_up(void);
```
