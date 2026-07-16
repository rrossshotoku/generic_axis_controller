/*
 * app/led_indicator — see led_indicator.h for the contract.
 */

#include "led_indicator.h"

#include "app/axis_manager/axis_manager.h"
#include "app/log/log.h"
#include "bsp/leds/leds.h"
#include "bsp/net/net.h"
#include "bsp/time/time.h"

/*----------------------------------------------------------------------------
 * Configuration
 *---------------------------------------------------------------------------*/

/* Boot-solid indicator window — how long after boot the LED stays solid
 * at full colour before falling back to normal SOLID/BREATHING. Long
 * enough that the operator can see "yes, it powered up". */
#define BOOT_SOLID_MS         3000u

/* Breathing — perceptually-exponential ramp up + down, repeating. 1000 ms
 * up + 1000 ms down = 2 s full cycle. Brightness is computed each tick as
 * a fraction of the cycle position then squared for the natural "breathing"
 * feel — see breath_brightness. */
#define BREATH_UP_MS          1000u
#define BREATH_DOWN_MS        1000u
#define BREATH_CYCLE_MS       (BREATH_UP_MS + BREATH_DOWN_MS)

/* Network link-up flash: three blinks. ON_MS bright + OFF_MS dark per
 * blink. ~600 ms total (3 × 200 ms) — quick enough to feel responsive
 * but visible. */
#define FLASH_ON_MS           100u
#define FLASH_OFF_MS          100u
#define FLASH_CYCLES          3u
#define FLASH_TOTAL_MS        (FLASH_CYCLES * (FLASH_ON_MS + FLASH_OFF_MS))

/*----------------------------------------------------------------------------
 * State
 *---------------------------------------------------------------------------*/

/* Operator-configured colour (OD-writable, persisted). Default magenta
 * so an unconfigured unit is visibly "alive" rather than dark; operator
 * picks their own colour via the GUI on first commissioning. */
static uint8_t s_r = 128;
static uint8_t s_g = 0;
static uint8_t s_b = 128;

static uint32_t s_boot_ms;             /* time_ms() at led_indicator_init */
static bool     s_link_was_up;         /* edge detection for link transitions */
static uint32_t s_flash_started_ms;    /* 0 = no flash sequence active */
static bool     s_prev_moving;         /* edge detect for motion stop */
/* Settle-out phase: when motion stops mid-pulse we keep breathing until
 * the next brightness peak so the LED never jumps from a dim value up
 * to solid-on. Zero = not settling. */
static uint32_t s_settle_started_ms;
static uint32_t s_settle_duration_ms;

/*----------------------------------------------------------------------------
 * Brightness computation per pattern
 *---------------------------------------------------------------------------*/

/* Squared-triangle ramp 0 -> 255 -> 0 over BREATH_CYCLE_MS. Approximates
 * gamma ≈ 2 correction — matches the human eye's log-scale response so
 * the pulse feels "natural" rather than mechanical: dim end lingers,
 * bright end punches. Integer maths only (no sin/lookup), squaring in
 * uint32 (255*255 = 65025) then dividing back to 8-bit. */
static uint8_t breath_brightness(uint32_t now_ms)
{
    uint32_t phase = now_ms % BREATH_CYCLE_MS;
    uint32_t linear;
    if (phase < BREATH_UP_MS) {
        linear = (phase * 255u) / BREATH_UP_MS;
    } else {
        uint32_t down = phase - BREATH_UP_MS;
        linear = 255u - ((down * 255u) / BREATH_DOWN_MS);
    }
    return (uint8_t)((linear * linear) / 255u);
}

/* In a flash sequence: ON for FLASH_ON_MS, OFF for FLASH_OFF_MS, repeated.
 * Returns 0 (off) or 255 (full). Caller is responsible for stopping the
 * sequence once FLASH_TOTAL_MS has elapsed. */
static uint8_t flash_brightness(uint32_t elapsed_ms)
{
    uint32_t cycle = elapsed_ms % (FLASH_ON_MS + FLASH_OFF_MS);
    return (cycle < FLASH_ON_MS) ? 255u : 0u;
}

/*----------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

void led_indicator_init(void)
{
    s_boot_ms          = time_ms();
    s_link_was_up      = false;
    s_flash_started_ms = 0u;
    /* Initial render — drive the configured colour at boot brightness
     * immediately so there's no dark gap before the first tick. */
    bsp_leds_set_rgb(s_r, s_g, s_b);
    LOG_INFO("led_indicator: ready (boot colour R=%u G=%u B=%u)",
             (unsigned)s_r, (unsigned)s_g, (unsigned)s_b);
}

void led_indicator_tick(void)
{
    uint32_t now = time_ms();

    /* Detect link-up edge from bsp/net. net_link_up() is cheap (reads a
     * cached PHY status). On a 0->1 transition queue the flash. */
    bool link_now = net_link_up();
    if (link_now && !s_link_was_up) {
        s_flash_started_ms = (now == 0u) ? 1u : now;   /* avoid 0-sentinel collision */
        LOG_INFO("led_indicator: network link up — flashing %ux", (unsigned)FLASH_CYCLES);
    }
    s_link_was_up = link_now;

    /* Resolve which pattern is active this tick (priority order in header). */
    uint8_t brightness;

    if (s_flash_started_ms != 0u) {
        uint32_t elapsed = now - s_flash_started_ms;
        if (elapsed >= FLASH_TOTAL_MS) {
            s_flash_started_ms = 0u;   /* sequence done; fall through to lower priority */
        } else {
            brightness = flash_brightness(elapsed);
            bsp_leds_set_rgb((uint8_t)((uint32_t)s_r * brightness / 255u),
                             (uint8_t)((uint32_t)s_g * brightness / 255u),
                             (uint8_t)((uint32_t)s_b * brightness / 255u));
            return;
        }
    }

    bool moving = axis_manager_is_moving();
    if (moving) {
        /* Motion resumed (or ongoing) — cancel any settle-out and pulse. */
        s_settle_duration_ms = 0u;
        brightness = breath_brightness(now);
    } else if (s_prev_moving) {
        /* Motion just stopped this tick. Compute how long until the next
         * brightness peak (phase == BREATH_UP_MS) so we finish the pulse
         * gracefully instead of snapping to solid-on. */
        uint32_t phase = now % BREATH_CYCLE_MS;
        uint32_t ms_to_peak;
        if (phase < BREATH_UP_MS) {
            /* Currently on the way up — finish the ascent. */
            ms_to_peak = BREATH_UP_MS - phase;
        } else {
            /* Currently on the way down — complete descent then next ascent. */
            ms_to_peak = (BREATH_CYCLE_MS - phase) + BREATH_UP_MS;
        }
        s_settle_started_ms  = now;
        s_settle_duration_ms = ms_to_peak;
        brightness = breath_brightness(now);
    } else if (s_settle_duration_ms != 0u
            && time_elapsed_ms(s_settle_started_ms) < s_settle_duration_ms) {
        /* Still finishing the post-motion pulse. */
        brightness = breath_brightness(now);
    } else if ((now - s_boot_ms) < BOOT_SOLID_MS) {
        /* Boot indicator window — solid full on. */
        s_settle_duration_ms = 0u;
        brightness = 255u;
    } else {
        /* Idle steady state — solid full on at configured colour. The
         * configured colour itself is what dims things if the operator
         * wants a quieter indicator. */
        s_settle_duration_ms = 0u;
        brightness = 255u;
    }
    s_prev_moving = moving;

    bsp_leds_set_rgb((uint8_t)((uint32_t)s_r * brightness / 255u),
                     (uint8_t)((uint32_t)s_g * brightness / 255u),
                     (uint8_t)((uint32_t)s_b * brightness / 255u));
}

/*----------------------------------------------------------------------------
 * Operator-facing accessors (back the OD entries)
 *---------------------------------------------------------------------------*/

uint8_t led_indicator_get_r(void) { return s_r; }
uint8_t led_indicator_get_g(void) { return s_g; }
uint8_t led_indicator_get_b(void) { return s_b; }

bool led_indicator_set_r(uint8_t v)
{
    if (s_r != v) LOG_INFO("led_indicator: R = %u", (unsigned)v);
    s_r = v;
    return true;
}
bool led_indicator_set_g(uint8_t v)
{
    if (s_g != v) LOG_INFO("led_indicator: G = %u", (unsigned)v);
    s_g = v;
    return true;
}
bool led_indicator_set_b(uint8_t v)
{
    if (s_b != v) LOG_INFO("led_indicator: B = %u", (unsigned)v);
    s_b = v;
    return true;
}

/*----------------------------------------------------------------------------
 * External trigger + persist hooks
 *---------------------------------------------------------------------------*/

void led_indicator_signal_link_up(void)
{
    uint32_t now = time_ms();
    s_flash_started_ms = (now == 0u) ? 1u : now;
}

void led_indicator_capture_persist(uint8_t out_rgb[3])
{
    out_rgb[0] = s_r;
    out_rgb[1] = s_g;
    out_rgb[2] = s_b;
}

void led_indicator_apply_persist(const uint8_t in_rgb[3])
{
    s_r = in_rgb[0];
    s_g = in_rgb[1];
    s_b = in_rgb[2];
}
