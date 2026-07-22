/*
 * app/axis_manager/motor_od_proxy — implementation. See header for the
 * responsibility split, ownership rules, and cia402-slot cooperation.
 */
#include "motor_od_proxy.h"

#include "axis_manager.h"                /* axis_manager_request_enable */
#include "app/cia402/cia402.h"
#include "app/log/log.h"
#include "bsp/time/time.h"

#include <math.h>
#include <string.h>

/*----------------------------------------------------------------------------
 * Slot table (7 slots) — order MUST match motor_proxy_slot_id_t below.
 *---------------------------------------------------------------------------*/
#define PROXY_SLOT_VEL_ACCEL_UP    (0u)
#define PROXY_SLOT_VEL_ACCEL_DN    (1u)
#define PROXY_SLOT_VEL_ACCEL_JERK  (2u)
#define PROXY_SLOT_VEL_LIMIT       (3u)
#define PROXY_SLOT_ACCEL_LIMIT     (4u)
#define PROXY_SLOT_POS_LIMIT_LO    (5u)
#define PROXY_SLOT_POS_LIMIT_HI    (6u)
#define PROXY_SLOT_COUNT           (7u)

typedef struct {
    uint16_t    idx;
    uint8_t     sub;
    const char *name;
    float       value;
    bool        dirty;
} motor_proxy_slot_t;

static motor_proxy_slot_t s_slots[PROXY_SLOT_COUNT] = {
    [PROXY_SLOT_VEL_ACCEL_UP]   = { 0x2300u, 6u, "vel_accel_up",    0.0f, false },
    [PROXY_SLOT_VEL_ACCEL_DN]   = { 0x2300u, 7u, "vel_accel_dn",    0.0f, false },
    [PROXY_SLOT_VEL_ACCEL_JERK] = { 0x2300u, 8u, "vel_accel_jerk",  0.0f, false },
    [PROXY_SLOT_VEL_LIMIT]      = { 0x2600u, 4u, "max_velocity",    0.0f, false },
    [PROXY_SLOT_ACCEL_LIMIT]    = { 0x2600u, 5u, "max_accel",       0.0f, false },
    [PROXY_SLOT_POS_LIMIT_LO]   = { 0x2600u, 6u, "pos_limit_lo",    0.0f, false },
    [PROXY_SLOT_POS_LIMIT_HI]   = { 0x2600u, 7u, "pos_limit_hi",    0.0f, false },
};

/*----------------------------------------------------------------------------
 * Proxy write in-flight tracking. At most one SDO active at a time.
 *---------------------------------------------------------------------------*/
static cia402_od_handle_t s_proxy_handle       = CIA402_OD_HANDLE_INVALID;
static bool               s_proxy_pending      = false;
static uint32_t           s_proxy_inflight_slot = 0;

/*----------------------------------------------------------------------------
 * Bootsync (SDO-read every slot into cache) state
 *---------------------------------------------------------------------------*/
typedef enum {
    BOOTSYNC_IDLE = 0,
    BOOTSYNC_BUSY,
    BOOTSYNC_DONE,
} bootsync_state_t;

static bootsync_state_t   s_bootsync_state         = BOOTSYNC_IDLE;
static cia402_od_handle_t s_bootsync_handle        = CIA402_OD_HANDLE_INVALID;
static uint32_t           s_bootsync_inflight_slot = 0;
static uint32_t           s_bootsync_next_slot     = 0;
static uint32_t           s_bootsync_start_ms      = 0;
static uint32_t           s_bootsync_last_done_ms  = 0;
static bool               s_bootsync_first_done_logged = false;

#define BOOTSYNC_KICKOFF_DELAY_MS  500u    /* let cia402 + motor settle before pinging */
#define BOOTSYNC_RESYNC_PERIOD_MS  5000u   /* re-mirror motor's OD every N ms */

static motor_od_proxy_bootsync_cb_t s_bootsync_cb = NULL;

/*----------------------------------------------------------------------------
 * Motor-save sequencer state
 *---------------------------------------------------------------------------*/
#define MOTOR_SAVE_DISABLE_MS         250u
#define MOTOR_SAVE_OVERALL_TIMEOUT_MS 5000u

typedef enum {
    MOTOR_SAVE_IDLE = 0,
    MOTOR_SAVE_DISABLING,
    MOTOR_SAVE_WRITING_SDO,
    MOTOR_SAVE_REENABLE,
} motor_save_state_t;

static motor_save_state_t s_save_state      = MOTOR_SAVE_IDLE;
static uint32_t           s_save_phase_ms   = 0;
static uint32_t           s_save_start_ms   = 0;
static bool               s_save_was_enabled = false;
static cia402_od_handle_t s_save_handle     = CIA402_OD_HANDLE_INVALID;

/*----------------------------------------------------------------------------
 * Load-factor (dedicated one-shot SDO writer, not in slot table)
 *---------------------------------------------------------------------------*/
#define MOTOR_OD_VEL_LOAD_FACTOR_IDX  (0x2300u)
#define MOTOR_OD_VEL_LOAD_FACTOR_SUB  (5u)

static float              s_load_factor        = 1.0f;    /* boot default = no scaling */
static cia402_od_handle_t s_load_factor_handle = CIA402_OD_HANDLE_INVALID;
static bool               s_load_factor_pending = false;

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/
void motor_od_proxy_init(void)
{
    for (uint32_t i = 0; i < PROXY_SLOT_COUNT; ++i) {
        s_slots[i].value = 0.0f;
        s_slots[i].dirty = false;
    }
    s_proxy_handle       = CIA402_OD_HANDLE_INVALID;
    s_proxy_pending      = false;
    s_proxy_inflight_slot = 0;

    s_bootsync_state         = BOOTSYNC_IDLE;
    s_bootsync_handle        = CIA402_OD_HANDLE_INVALID;
    s_bootsync_inflight_slot = 0;
    s_bootsync_next_slot     = 0;
    s_bootsync_start_ms      = 0;
    s_bootsync_last_done_ms  = 0;
    s_bootsync_first_done_logged = false;

    s_save_state      = MOTOR_SAVE_IDLE;
    s_save_phase_ms   = 0;
    s_save_start_ms   = 0;
    s_save_was_enabled = false;
    s_save_handle     = CIA402_OD_HANDLE_INVALID;

    s_load_factor         = 1.0f;
    s_load_factor_handle  = CIA402_OD_HANDLE_INVALID;
    s_load_factor_pending = false;

    s_bootsync_cb = NULL;
}

void motor_od_proxy_register_bootsync_cb(motor_od_proxy_bootsync_cb_t cb)
{
    s_bootsync_cb = cb;
}

void motor_od_proxy_reset(void)
{
    /* Motor entered the bootloader. Force IDLE and drop every handle so
     * we don't leave sub-modules holding stale cia402 handles.
     * Bootsync restarts from slot 0 so the CMC cache re-mirrors the motor's
     * OD after any potential firmware update while it was in the bootloader. */
    s_bootsync_state         = BOOTSYNC_IDLE;
    s_bootsync_handle        = CIA402_OD_HANDLE_INVALID;
    s_bootsync_next_slot     = 0;
    s_save_state             = MOTOR_SAVE_IDLE;
    s_save_handle            = CIA402_OD_HANDLE_INVALID;
    s_load_factor_handle     = CIA402_OD_HANDLE_INVALID;
    s_load_factor_pending    = false;
    s_proxy_handle           = CIA402_OD_HANDLE_INVALID;
    s_proxy_pending          = false;
    /* Slot dirty flags are DELIBERATELY not cleared — an operator write
     * that was mid-flight when the motor entered bootloader still deserves
     * to reach the motor once it rejoins the app. */
}

/*----------------------------------------------------------------------------
 * CATEGORY 1 accessors (vel_accel_*)
 *---------------------------------------------------------------------------*/
float motor_od_proxy_get_vel_accel_up  (void) { return s_slots[PROXY_SLOT_VEL_ACCEL_UP  ].value; }
float motor_od_proxy_get_vel_accel_dn  (void) { return s_slots[PROXY_SLOT_VEL_ACCEL_DN  ].value; }
float motor_od_proxy_get_vel_accel_jerk(void) { return s_slots[PROXY_SLOT_VEL_ACCEL_JERK].value; }

static bool set_slot_f32(uint32_t slot_idx, float v)
{
    if (slot_idx >= PROXY_SLOT_COUNT) return false;
    motor_proxy_slot_t *s = &s_slots[slot_idx];
    s->value = v;
    s->dirty = true;
    LOG_INFO("axis_manager: %s queued = %d/1000 (motor 0x%04X:%u)",
             s->name, (int)(v * 1000.0f),
             (unsigned)s->idx, (unsigned)s->sub);
    return true;
}

bool motor_od_proxy_set_vel_accel_up(float v)
{
    if (isnan(v) || v < 0.0f) return false;
    return set_slot_f32(PROXY_SLOT_VEL_ACCEL_UP, v);
}
bool motor_od_proxy_set_vel_accel_dn(float v)
{
    if (isnan(v) || v < 0.0f) return false;
    return set_slot_f32(PROXY_SLOT_VEL_ACCEL_DN, v);
}
bool motor_od_proxy_set_vel_accel_jerk(float v)
{
    if (isnan(v) || v < 0.0f) return false;
    return set_slot_f32(PROXY_SLOT_VEL_ACCEL_JERK, v);
}

/*----------------------------------------------------------------------------
 * CATEGORY 2 write-through (axis_manager owns local mirror)
 *---------------------------------------------------------------------------*/
bool motor_od_proxy_write_vel_limit   (float motor_v) { return set_slot_f32(PROXY_SLOT_VEL_LIMIT,     motor_v); }
bool motor_od_proxy_write_accel_limit (float motor_v) { return set_slot_f32(PROXY_SLOT_ACCEL_LIMIT,   motor_v); }
bool motor_od_proxy_write_pos_limit_lo(float motor_v) { return set_slot_f32(PROXY_SLOT_POS_LIMIT_LO,  motor_v); }
bool motor_od_proxy_write_pos_limit_hi(float motor_v) { return set_slot_f32(PROXY_SLOT_POS_LIMIT_HI,  motor_v); }

/*----------------------------------------------------------------------------
 * Load-factor
 *---------------------------------------------------------------------------*/
float motor_od_proxy_get_load_factor(void) { return s_load_factor; }

bool motor_od_proxy_set_load_factor(float v)
{
    if (isnan(v) || v < 0.3f || v > 2.0f) {
        LOG_WARN("axis_manager: set_load_factor REJECTED v=%d/1000 (need 0.3..2.0)",
                 (int)(v * 1000.0f));
        return false;
    }
    s_load_factor = v;

    /* Don't try to issue a new SDO if we still have one in flight — we'd
     * just overwrite our own handle and leak the previous OD_COMPLETE slot.
     * The tick path drains the existing one first; the operator can re-slide
     * afterwards if they were too fast. */
    if (s_load_factor_pending) {
        LOG_INFO("axis_manager: load_factor cached %d/1000 (previous SDO still in flight; will not chain)",
                 (int)(v * 1000.0f));
        return true;
    }

    cia402_od_handle_t h = cia402_od_write_begin(
        MOTOR_OD_VEL_LOAD_FACTOR_IDX, MOTOR_OD_VEL_LOAD_FACTOR_SUB,
        MC_IF_T_F32, &s_load_factor, sizeof(float));
    if (h == CIA402_OD_HANDLE_INVALID) {
        LOG_WARN("axis_manager: load_factor=%d/1000 cached, cia402 OD pipeline "
                 "busy — write not issued. Re-slide once it frees up.",
                 (int)(v * 1000.0f));
        return true;     /* cache still good; just no motor-side effect this time */
    }
    s_load_factor_handle  = h;
    s_load_factor_pending = true;
    LOG_INFO("axis_manager: load_factor -> %d/1000 (SDO write queued to motor 0x2300:5, handle=%d)",
             (int)(v * 1000.0f), (int)h);
    return true;
}

/* Drain the load-factor SDO write so cia402's OD pipeline doesn't stay
 * in OD_COMPLETE. Called from motor_od_proxy_tick. */
static void poll_load_factor_sdo(void)
{
    if (!s_load_factor_pending) return;
    MC_IfOdResult_t res = MC_IF_OD_OK;
    uint8_t buf[8] = {0};
    uint8_t vlen   = sizeof(buf);
    if (!cia402_od_poll(s_load_factor_handle, &res, buf, &vlen)) {
        return;  /* still pending — try again next tick */
    }
    if (res == MC_IF_OD_OK) {
        LOG_INFO("axis_manager: load_factor SDO -> motor 0x2300:5 OK (=%d/1000)",
                 (int)(s_load_factor * 1000.0f));
    } else {
        LOG_WARN("axis_manager: load_factor SDO -> motor 0x2300:5 FAIL result=0x%02X "
                 "(motor MCU may not yet implement REQ-0014)", (unsigned)res);
    }
    s_load_factor_handle  = CIA402_OD_HANDLE_INVALID;
    s_load_factor_pending = false;
}

/*----------------------------------------------------------------------------
 * Proxy writer — drain in-flight, then start next dirty slot
 *---------------------------------------------------------------------------*/
static void poll_motor_proxy_sdo(void)
{
    /* Step 1: drain any in-flight SDO completion. */
    if (s_proxy_pending) {
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_proxy_handle, &res, buf, &vlen)) return;
        motor_proxy_slot_t *inflight = &s_slots[s_proxy_inflight_slot];
        if (res == MC_IF_OD_OK) {
            LOG_INFO("axis_manager: %s SDO -> motor 0x%04X:%u OK",
                     inflight->name, (unsigned)inflight->idx, (unsigned)inflight->sub);
        } else {
            LOG_WARN("axis_manager: %s SDO -> motor 0x%04X:%u FAIL result=0x%02X",
                     inflight->name, (unsigned)inflight->idx, (unsigned)inflight->sub,
                     (unsigned)res);
        }
        s_proxy_handle  = CIA402_OD_HANDLE_INVALID;
        s_proxy_pending = false;
        /* Fall through — pipeline is now free, can start next dirty slot. */
    }

    /* Step 2: find next dirty slot and start its SDO. Round-robin from
     * the last in-flight slot so heavy traffic on one entry doesn't
     * starve the others. */
    for (uint32_t i = 0; i < PROXY_SLOT_COUNT; ++i) {
        uint32_t slot_idx = (s_proxy_inflight_slot + 1u + i) % PROXY_SLOT_COUNT;
        motor_proxy_slot_t *s = &s_slots[slot_idx];
        if (!s->dirty) continue;
        cia402_od_handle_t h = cia402_od_write_begin(
            s->idx, s->sub, MC_IF_T_F32, &s->value, sizeof(float));
        if (h == CIA402_OD_HANDLE_INVALID) {
            /* cia402 still busy (e.g. load_factor or setup_sequencer write
             * in flight). Try again next tick. */
            return;
        }
        s->dirty               = false;
        s_proxy_handle         = h;
        s_proxy_pending        = true;
        s_proxy_inflight_slot  = slot_idx;
        LOG_INFO("axis_manager: %s -> motor 0x%04X:%u = %d/1000 (SDO h=%d)",
                 s->name, (unsigned)s->idx, (unsigned)s->sub,
                 (int)(s->value * 1000.0f), (int)h);
        return;
    }
}

/*----------------------------------------------------------------------------
 * Bootsync — one-shot read each slot at motor-app first-boot, then
 * periodic auto-resync every 5 s.
 *---------------------------------------------------------------------------*/
void motor_od_proxy_request_motor_resync(void)
{
    if (s_bootsync_state != BOOTSYNC_DONE) return;  /* already running */
    s_bootsync_state     = BOOTSYNC_IDLE;
    s_bootsync_next_slot = 0;
    /* Skip the kickoff delay on restart — only the first-ever boot needs
     * it (motor was still settling). After that, motor is up. */
    s_bootsync_start_ms  = time_ms() - BOOTSYNC_KICKOFF_DELAY_MS;
}

/* Notify axis_manager (if registered) that a CATEGORY-2 slot just got a
 * fresh read-back. axis_manager updates its own s_axis mirror to match. */
static void notify_bootsync(uint32_t slot_idx, float v)
{
    if (s_bootsync_cb == NULL) return;
    switch (slot_idx) {
    case PROXY_SLOT_VEL_LIMIT:      s_bootsync_cb(MOTOR_OD_PROXY_BOOTSYNC_VEL_LIMIT,     v); break;
    case PROXY_SLOT_ACCEL_LIMIT:    s_bootsync_cb(MOTOR_OD_PROXY_BOOTSYNC_ACCEL_LIMIT,   v); break;
    case PROXY_SLOT_POS_LIMIT_LO:   s_bootsync_cb(MOTOR_OD_PROXY_BOOTSYNC_POS_LIMIT_LO,  v); break;
    case PROXY_SLOT_POS_LIMIT_HI:   s_bootsync_cb(MOTOR_OD_PROXY_BOOTSYNC_POS_LIMIT_HI,  v); break;
    default: break;   /* CATEGORY 1 slots (vel_accel_*) live entirely here */
    }
}

static void tick_bootsync(void)
{
    /* Auto-restart every BOOTSYNC_RESYNC_PERIOD_MS once the previous pass
     * completed. Lets the CMC cache track motor-side changes without any
     * external trigger. */
    if (s_bootsync_state == BOOTSYNC_DONE) {
        if (time_elapsed_ms(s_bootsync_last_done_ms) >= BOOTSYNC_RESYNC_PERIOD_MS) {
            motor_od_proxy_request_motor_resync();
        } else {
            return;
        }
    }

    /* Wait a beat after init before kicking off — the motor MCU's SPI
     * link is still settling and the first SDO often gets dropped if
     * fired too early. Once we've started, the state machine drives. */
    if (s_bootsync_state == BOOTSYNC_IDLE) {
        if (s_bootsync_start_ms == 0u) s_bootsync_start_ms = time_ms();
        if (time_elapsed_ms(s_bootsync_start_ms) < BOOTSYNC_KICKOFF_DELAY_MS) return;
    }

    /* Drain completed read, if any. */
    if (s_bootsync_state == BOOTSYNC_BUSY) {
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_bootsync_handle, &res, buf, &vlen)) return;

        motor_proxy_slot_t *s = &s_slots[s_bootsync_inflight_slot];
        if (res == MC_IF_OD_OK && vlen == sizeof(float)) {
            float v;
            memcpy(&v, buf, sizeof(float));
            /* Don't clobber the cache if the operator already typed a new
             * value (slot was marked dirty before we got here). Otherwise
             * adopt the motor's flash value into slot cache + notify axis_manager. */
            if (!s->dirty) {
                bool changed = (s->value != v);
                s->value = v;
                notify_bootsync(s_bootsync_inflight_slot, v);
                if (changed) {
                    LOG_INFO("axis_manager: motor-resync %s = %d/1000 (from motor 0x%04X:%u)",
                             s->name, (int)(v * 1000.0f),
                             (unsigned)s->idx, (unsigned)s->sub);
                }
            } else {
                LOG_INFO("axis_manager: motor-resync %s skipped (already dirty — operator set value first)",
                         s->name);
            }
        } else {
            LOG_WARN("axis_manager: bootsync %s read FAIL result=0x%02X vlen=%u",
                     s->name, (unsigned)res, (unsigned)vlen);
        }
        s_bootsync_handle = CIA402_OD_HANDLE_INVALID;
        s_bootsync_state  = BOOTSYNC_IDLE;   /* fall through to start next */
    }

    /* Issue next read if there's one outstanding. */
    if (s_bootsync_next_slot >= PROXY_SLOT_COUNT) {
        if (s_bootsync_state != BOOTSYNC_DONE) {
            s_bootsync_state = BOOTSYNC_DONE;
            s_bootsync_last_done_ms = time_ms();
            if (!s_bootsync_first_done_logged) {
                s_bootsync_first_done_logged = true;
                LOG_INFO("axis_manager: bootsync complete — motor values mirrored into CMC cache (will auto-resync every %u ms)",
                         (unsigned)BOOTSYNC_RESYNC_PERIOD_MS);
            }
        }
        return;
    }
    motor_proxy_slot_t *s = &s_slots[s_bootsync_next_slot];
    cia402_od_handle_t h = cia402_od_read_begin(s->idx, s->sub, MC_IF_T_F32);
    if (h == CIA402_OD_HANDLE_INVALID) return;  /* cia402 busy — retry next tick */
    s_bootsync_handle        = h;
    s_bootsync_inflight_slot = s_bootsync_next_slot;
    s_bootsync_state         = BOOTSYNC_BUSY;
    s_bootsync_next_slot++;
}

/*----------------------------------------------------------------------------
 * Motor-save sequencer
 *---------------------------------------------------------------------------*/
bool motor_od_proxy_request_motor_save(void)
{
    if (s_save_state != MOTOR_SAVE_IDLE) {
        LOG_WARN("axis_manager: motor save requested but sequencer already busy (state=%d)",
                 (int)s_save_state);
        return false;
    }
    /* Snapshot operator intent BEFORE disabling so re-enable knows whether
     * the operator had the motor intentionally disabled. We read the raw
     * enable_latch (what CMC last commanded) — NOT the derived state —
     * because a mid-fault save should still restore intent when the fault
     * clears. This module doesn't own s_axis.enable_latch (lives in
     * axis_manager); the internal getter exposes it. */
    s_save_was_enabled = axis_manager_is_enable_latched();
    s_save_start_ms    = time_ms();
    s_save_phase_ms    = s_save_start_ms;

    LOG_INFO("axis_manager: motor save START (was_enabled=%d) — disabling for %u ms",
             (int)s_save_was_enabled, (unsigned)MOTOR_SAVE_DISABLE_MS);
    (void)axis_manager_request_enable(false);
    s_save_state = MOTOR_SAVE_DISABLING;
    return true;
}

static void tick_motor_save(void)
{
    if (s_save_state == MOTOR_SAVE_IDLE) return;

    if (time_elapsed_ms(s_save_start_ms) > MOTOR_SAVE_OVERALL_TIMEOUT_MS) {
        LOG_ERROR("axis_manager: motor save TIMEOUT after %u ms (state=%d) — aborting, restoring enable=%d",
                  (unsigned)MOTOR_SAVE_OVERALL_TIMEOUT_MS,
                  (int)s_save_state, (int)s_save_was_enabled);
        (void)axis_manager_request_enable(s_save_was_enabled);
        s_save_state  = MOTOR_SAVE_IDLE;
        s_save_handle = CIA402_OD_HANDLE_INVALID;
        return;
    }

    switch (s_save_state) {
    case MOTOR_SAVE_DISABLING: {
        if (time_elapsed_ms(s_save_phase_ms) < MOTOR_SAVE_DISABLE_MS) return;
        /* Disable settle window elapsed — start the SDO write. If the
         * cia402 OD pipeline is busy with something else (e.g. a
         * load_factor or vel_accel proxy), try again next tick. */
        uint16_t magic = MC_IF_SAVE_MAGIC;
        cia402_od_handle_t h = cia402_od_write_begin(
            0x2800u, 1u, MC_IF_T_U16, &magic, sizeof(magic));
        if (h == CIA402_OD_HANDLE_INVALID) return;
        s_save_handle   = h;
        s_save_state    = MOTOR_SAVE_WRITING_SDO;
        s_save_phase_ms = time_ms();
        LOG_INFO("axis_manager: motor save WRITE 0x2800:1 = 0x%04X (SDO h=%d)",
                 (unsigned)MC_IF_SAVE_MAGIC, (int)h);
        break;
    }
    case MOTOR_SAVE_WRITING_SDO: {
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_save_handle, &res, buf, &vlen)) return;
        if (res == MC_IF_OD_OK) {
            LOG_INFO("axis_manager: motor save SDO accepted (motor wrote to flash)");
        } else {
            LOG_WARN("axis_manager: motor save SDO FAIL result=0x%02X "
                     "(motor refused — drive may still be enabled?)", (unsigned)res);
        }
        s_save_handle = CIA402_OD_HANDLE_INVALID;
        s_save_state  = MOTOR_SAVE_REENABLE;
        s_save_phase_ms = time_ms();
        break;
    }
    case MOTOR_SAVE_REENABLE: {
        /* Brief grace period before re-enabling, so the motor's
         * statusword has time to stabilise after the save. */
        if (time_elapsed_ms(s_save_phase_ms) < 50u) return;
        if (s_save_was_enabled) {
            (void)axis_manager_request_enable(true);
            LOG_INFO("axis_manager: motor save DONE — re-enabling axis (was enabled pre-save)");
        } else {
            LOG_INFO("axis_manager: motor save DONE — leaving disabled (was disabled pre-save)");
        }
        s_save_state = MOTOR_SAVE_IDLE;
        break;
    }
    default:
        s_save_state = MOTOR_SAVE_IDLE;
        break;
    }
}

/*----------------------------------------------------------------------------
 * Tick — order matters (see header for rationale)
 *---------------------------------------------------------------------------*/
void motor_od_proxy_tick(void)
{
    poll_load_factor_sdo();
    tick_bootsync();       /* reads first (no-op once complete) */
    poll_motor_proxy_sdo(); /* writes second (round-robin across dirty slots) */
    tick_motor_save();
}
