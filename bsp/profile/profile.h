/*
 * bsp/profile — section timing using the Cortex-M4 DWT cycle counter.
 *
 * Free-running cycle counter at `DWT->CYCCNT` (1 cycle at 170 MHz = ~5.88 ns).
 * `profile_begin(slot)` records the start cycle; `profile_end(slot)` records
 * the delta. Each slot accumulates count + sum + max for one reporting
 * window. `profile_tick()` measures the inter-iteration period and, once
 * per second, dumps a one-line summary to LOG_INFO.
 *
 * Cost per begin/end pair: one register read + one subtraction (~10 cycles
 * ≈ 60 ns) — negligible against a ~1 ms loop.
 *
 * Wrap-around: DWT->CYCCNT is uint32_t, wraps every ~25 s at 170 MHz, but
 * we only ever take `(end - begin)` deltas of a few microseconds, so the
 * unsigned subtraction handles wrap naturally. The inter-iteration period
 * has the same property as long as the loop runs faster than ~25 s, which
 * it always does.
 */

#ifndef BSP_PROFILE_H
#define BSP_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PROFILE_LOG = 0,
    PROFILE_AXIS_MGR,
    PROFILE_CIA402,
    PROFILE_OD,
    PROFILE_CMC_STATE,
    PROFILE_CONTROLLER_MGR,
    PROFILE_WEB,
    PROFILE_DEBUG,
    PROFILE_HEARTBEAT,
    PROFILE_SLOT_COUNT
} profile_slot_t;

void profile_init(void);

/* Bracket the code under measurement. Safe to nest different slots; do not
 * call begin/end out-of-order for the same slot. */
void profile_begin(profile_slot_t s);
void profile_end  (profile_slot_t s);

/* Called once per main-loop iteration. Records the period since the last
 * call (i.e. the loop iteration period) and emits a LOG_INFO summary every
 * ~1 s with avg/max per slot, plus the achievable Hz from the loop period. */
void profile_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PROFILE_H */
