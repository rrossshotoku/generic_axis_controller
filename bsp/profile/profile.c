/*
 * bsp/profile — see header.
 */

#include "profile.h"

#include "app/log/log.h"
#include "stm32g4xx_hal.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Per-slot accumulator. Reset every reporting window (~1 s).
 *---------------------------------------------------------------------------*/

typedef struct {
    uint32_t begin_cyc;   /* set by profile_begin; consumed by profile_end */
    uint32_t count;       /* number of begin/end pairs this window */
    uint64_t sum_cyc;     /* total cycles spent in this slot this window */
    uint32_t max_cyc;     /* worst single call this window */
} slot_t;

static slot_t s_slots[PROFILE_SLOT_COUNT];

/* Inter-iteration period tracking. */
static uint32_t s_last_iter_cyc;    /* DWT->CYCCNT at last profile_tick */
static uint32_t s_iter_count;
static uint64_t s_iter_sum_cyc;
static uint32_t s_iter_max_cyc;

/* Reporting cadence: emit roughly once per second based on cumulative
 * loop period. Driven off the same CYCCNT so we don't need bsp/time. */
static uint32_t s_window_start_cyc;
#define REPORT_WINDOW_CYCLES  (170u * 1000u * 1000u)   /* ~1 s at 170 MHz */

static const char *slot_name[PROFILE_SLOT_COUNT] = {
    [PROFILE_LOG]            = "log",
    [PROFILE_AXIS_MGR]       = "axis_mgr",
    [PROFILE_CIA402]         = "cia402",
    [PROFILE_OD]             = "od",
    [PROFILE_CMC_STATE]      = "cmc_state",
    [PROFILE_CONTROLLER_MGR] = "ctrl_mgr",
    [PROFILE_WEB]            = "web",
    [PROFILE_DEBUG]          = "debug",
    [PROFILE_HEARTBEAT]      = "heartbeat",
};

/*----------------------------------------------------------------------------
 * DWT setup (one-time)
 *---------------------------------------------------------------------------*/

void profile_init(void)
{
    /* Enable the trace + debug block, then the cycle counter. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT      = 0;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;

    memset(s_slots, 0, sizeof(s_slots));
    s_last_iter_cyc    = DWT->CYCCNT;
    s_window_start_cyc = s_last_iter_cyc;
    s_iter_count       = 0;
    s_iter_sum_cyc     = 0;
    s_iter_max_cyc     = 0;

    LOG_INFO("profile: DWT cycle counter enabled (SystemCoreClock=%lu)",
             (unsigned long)SystemCoreClock);
}

/*----------------------------------------------------------------------------
 * Section bracketing
 *---------------------------------------------------------------------------*/

void profile_begin(profile_slot_t s)
{
    if ((unsigned)s >= PROFILE_SLOT_COUNT) return;
    s_slots[s].begin_cyc = DWT->CYCCNT;
}

void profile_end(profile_slot_t s)
{
    if ((unsigned)s >= PROFILE_SLOT_COUNT) return;
    uint32_t now   = DWT->CYCCNT;
    uint32_t delta = now - s_slots[s].begin_cyc;     /* unsigned wraps cleanly */
    s_slots[s].count++;
    s_slots[s].sum_cyc += delta;
    if (delta > s_slots[s].max_cyc) s_slots[s].max_cyc = delta;
}

/*----------------------------------------------------------------------------
 * Per-iteration tracking + reporting
 *---------------------------------------------------------------------------*/

static uint32_t cyc_to_us(uint64_t cyc)
{
    /* SystemCoreClock is Hz; cycles / (Hz / 1e6) = microseconds. Divide
     * before to keep us in uint32 range for any sane window. */
    uint64_t hz_per_us = SystemCoreClock / 1000000u;
    if (hz_per_us == 0u) hz_per_us = 1u;
    return (uint32_t)(cyc / hz_per_us);
}

static void emit_report(void)
{
    /* Loop period summary (worst-case is what hurts cyclic-rate stability). */
    uint32_t avg_period_us = (s_iter_count > 0)
                             ? cyc_to_us(s_iter_sum_cyc / s_iter_count)
                             : 0u;
    uint32_t max_period_us = cyc_to_us(s_iter_max_cyc);
    uint32_t avg_hz = avg_period_us ? (1000000u / avg_period_us) : 0u;
    uint32_t min_hz = max_period_us ? (1000000u / max_period_us) : 0u;

    LOG_INFO("profile: loop avg=%lu us max=%lu us  -> avg=%lu Hz min=%lu Hz  (n=%lu)",
             (unsigned long)avg_period_us, (unsigned long)max_period_us,
             (unsigned long)avg_hz,        (unsigned long)min_hz,
             (unsigned long)s_iter_count);

    /* Per-slot breakdown — one log line per slot that actually ran in this
     * window so cheap silent slots don't pollute the output. */
    for (size_t i = 0; i < PROFILE_SLOT_COUNT; i++) {
        if (s_slots[i].count == 0) continue;
        uint32_t avg_us = cyc_to_us(s_slots[i].sum_cyc / s_slots[i].count);
        uint32_t max_us = cyc_to_us(s_slots[i].max_cyc);
        LOG_INFO("profile:   %-12s n=%-5lu avg=%lu us max=%lu us",
                 slot_name[i],
                 (unsigned long)s_slots[i].count,
                 (unsigned long)avg_us,
                 (unsigned long)max_us);
    }

    /* Reset window. */
    memset(s_slots, 0, sizeof(s_slots));
    s_iter_count   = 0;
    s_iter_sum_cyc = 0;
    s_iter_max_cyc = 0;
}

void profile_tick(void)
{
    uint32_t now    = DWT->CYCCNT;
    uint32_t period = now - s_last_iter_cyc;
    s_last_iter_cyc = now;
    s_iter_count++;
    s_iter_sum_cyc += period;
    if (period > s_iter_max_cyc) s_iter_max_cyc = period;

    if ((uint32_t)(now - s_window_start_cyc) >= REPORT_WINDOW_CYCLES) {
        emit_report();
        s_window_start_cyc = now;
    }
}
