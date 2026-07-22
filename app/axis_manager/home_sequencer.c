/*
 * app/axis_manager/home_sequencer — implementation. See header.
 *
 * See axis_manager/README.md for the sub-module map + cia402-slot
 * cooperation invariant + reset checklist.
 */
#include "home_sequencer.h"

#include "axis_manager.h"                /* try_begin_op (arbitration) */
#include "app/cia402/cia402.h"
#include "app/log/log.h"
#include "bsp/time/time.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Tunables
 *---------------------------------------------------------------------------*/
#define HOME_POLL_PERIOD_MS      200u    /* re-poll home_status every N ms while RUNNING */
#define HOME_OVERALL_TIMEOUT_MS  30000u  /* absolute cap — motor's own homing is <10s typical */

/* Wait this long after CMC boot before the first attempt to read anything
 * from the motor. Bootsync uses the same delay to give the motor's SPI
 * link time to come up before we start hammering it with SDOs. */
#define ENCTYPE_BOOT_DELAY_MS 2000u
#define ENCTYPE_RETRY_MS      500u   /* retry cadence until first success */

/* How often we re-read motor fault_flags to refresh is_homed.
 * RETRY_MS: while is_homed is unknown (previous reads failed / never fired).
 *           Short so shot recalls unblock as soon as the motor is up.
 * REFRESH_MS: once we've got a good reading, how often to re-check so a
 *           motor-side change (e.g. operator ran home_command via PC tool
 *           directly, or motor reset mid-session) doesn't leave us stale. */
#define FAULT_READ_RETRY_MS   500u
#define FAULT_READ_REFRESH_MS 5000u

/*----------------------------------------------------------------------------
 * State
 *---------------------------------------------------------------------------*/
typedef enum {
    HOME_SEQ_IDLE = 0,
    HOME_SEQ_READING_ENCTYPE,  /* one-shot: SDO read motor 0x2500:8 quad_counts_per_rev */
    HOME_SEQ_CLEARING_CMD,     /* SDO write 0x2700:8 = 0 — clear sticky DONE/FAILED */
    HOME_SEQ_WRITING_CMD,      /* SDO write 0x2700:8 = 1 in flight */
    HOME_SEQ_POLLING_STATUS,   /* periodic reads of 0x2700:9 */
    HOME_SEQ_READING_FAULT,    /* post-DONE re-read of 0x2600:1 */
    HOME_SEQ_TERMINAL_DONE,    /* one tick of "done" for logging */
    HOME_SEQ_TERMINAL_FAILED,
} home_seq_state_t;

static home_seq_state_t   s_state       = HOME_SEQ_IDLE;
static cia402_od_handle_t s_handle      = CIA402_OD_HANDLE_INVALID;
static uint32_t           s_start_ms    = 0;    /* wall-clock start of the current sequence */
static uint32_t           s_last_poll_ms = 0;   /* last time we kicked a status read */
static uint8_t            s_last_motor_status = 0;  /* mirror of motor 0x2700:9 (MC_IF_HOME_*) */
static bool               s_is_homed        = false; /* fault_flags & NOT_HOMED == 0 */
static bool               s_is_homed_known  = false; /* have we ever successfully read fault_flags? */
static uint32_t           s_last_fault_read_ms = 0;  /* time_ms of last kick; 0 = never */

static bool               s_encoder_type_known    = false;
static bool               s_encoder_is_incremental = false;
static uint32_t           s_last_enctype_read_ms  = 0;   /* 0 = never */

/* Persisted operator setting (0x3043). See header. */
static uint8_t            s_home_on_boot   = 0u;
static bool               s_boot_home_fired = false;

/*----------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

void home_sequencer_init(void)
{
    s_state       = HOME_SEQ_IDLE;
    s_handle      = CIA402_OD_HANDLE_INVALID;
    s_start_ms    = 0;
    s_last_poll_ms = 0;
    s_last_motor_status = 0;
    s_is_homed        = false;
    s_is_homed_known  = false;
    s_last_fault_read_ms = 0;
    s_encoder_type_known    = false;
    s_encoder_is_incremental = false;
    s_last_enctype_read_ms  = 0;
    s_home_on_boot    = 0u;
    s_boot_home_fired = false;
}

void home_sequencer_reset(void)
{
    /* Motor-in-bootloader rising edge. Force the state machine to IDLE and
     * release the cia402 handle. Deliberately preserve s_is_homed_known —
     * a bootloader cycle doesn't change whether the axis is homed; when
     * the motor rejoins the app we'll re-verify via the periodic refresh
     * without blocking shot recalls in the meantime. */
    s_state             = HOME_SEQ_IDLE;
    s_handle            = CIA402_OD_HANDLE_INVALID;
    s_last_motor_status = MC_IF_HOME_IDLE;
    /* s_boot_home_fired NOT reset — one-per-CMC-boot, not one-per-motor-reboot.
     * A firmware update on the motor shouldn't cause the auto-home to fire
     * again unless the CMC itself reboots. */
}

uint8_t home_sequencer_get_home_status(void)
{
    return s_last_motor_status;
}

bool home_sequencer_is_homed(void)
{
    return s_is_homed_known && s_is_homed;
}

bool home_sequencer_encoder_is_incremental(void)
{
    return !s_encoder_type_known || s_encoder_is_incremental;
}

uint8_t home_sequencer_get_home_on_boot(void) { return s_home_on_boot; }
bool    home_sequencer_set_home_on_boot(uint8_t v)
{
    uint8_t nv = (v != 0u) ? 1u : 0u;
    if (s_home_on_boot != nv) {
        LOG_INFO("axis: home_on_boot %u -> %u", (unsigned)s_home_on_boot, (unsigned)nv);
    }
    s_home_on_boot = nv;
    return true;
}

bool home_sequencer_is_terminal_or_idle(void)
{
    return s_state == HOME_SEQ_IDLE
        || s_state == HOME_SEQ_TERMINAL_DONE
        || s_state == HOME_SEQ_TERMINAL_FAILED;
}

void home_sequencer_abort_best_effort(void)
{
    uint8_t zero = 0u;
    (void)cia402_od_write_begin(0x2700u, 8u, MC_IF_T_U8, &zero, sizeof(zero));
    s_state             = HOME_SEQ_IDLE;
    s_last_motor_status = MC_IF_HOME_IDLE;
}

bool home_sequencer_request_home(void)
{
    /* Arbitration: HOMING may only start when nothing else is running.
     * axis_manager_try_begin_op enforces the priority rules; if it returns
     * REJECTED the sequencer stays IDLE and the caller (OD write to 0x3040)
     * sees the false return + NOT_READY on the wire. */
    axis_begin_result_t r = axis_manager_try_begin_op(AXIS_OPERATION_HOMING);
    if (r == AXIS_BEGIN_REJECTED) return false;
    /* Also refuse re-entrant home while the sequencer is mid-run — the
     * try_begin above already caught the cross-family case; this catches
     * the pathological "same-family retarget of a home" case. */
    if (s_state != HOME_SEQ_IDLE) return false;
    LOG_INFO("axis: HOME start");
    /* Reset the cached motor status so POLLING_STATUS can't short-circuit
     * on a stale DONE/FAILED value from a previous run — otherwise the
     * first read after write=1 might come back reporting the still-latched
     * DONE and we'd skip straight to READING_FAULT without giving the
     * motor time to run. Fresh IDLE means "we haven't heard from motor
     * about this run yet". */
    s_last_motor_status = MC_IF_HOME_IDLE;
    s_state    = HOME_SEQ_CLEARING_CMD;
    s_start_ms = time_ms();
    s_last_poll_ms = 0;
    return true;
}

/*----------------------------------------------------------------------------
 * Tick
 *---------------------------------------------------------------------------*/

void home_sequencer_tick(void)
{
    /* --- One-shot encoder-type read (only while unknown) ---
     * Wait ENCTYPE_BOOT_DELAY_MS after CMC startup so the motor's SPI link
     * has time to come up. After that, retry at ENCTYPE_RETRY_MS until we
     * get a valid reading. Once known, this stops firing. */
    if (s_state == HOME_SEQ_IDLE && !s_encoder_type_known) {
        uint32_t now = time_ms();
        if (now >= ENCTYPE_BOOT_DELAY_MS) {
            bool due = (s_last_enctype_read_ms == 0u)
                       || (time_elapsed_ms(s_last_enctype_read_ms) >= ENCTYPE_RETRY_MS);
            if (due) {
                cia402_od_handle_t h = cia402_od_read_begin(0x2500u, 8u, MC_IF_T_F32);
                if (h != CIA402_OD_HANDLE_INVALID) {
                    s_handle = h;
                    s_state  = HOME_SEQ_READING_ENCTYPE;
                    s_last_enctype_read_ms = now;
                }
            }
        }
    }

    /* --- 0x3043 home_on_boot auto-fire --- Once per CMC boot, once we know
     * the motor uses an incremental encoder (absolute encoders don't need
     * homing — is_homed is always true), fire a home request. Motor-in-BL
     * is already gated at the callsite so we don't need to re-check it.
     * If try_begin rejects (something else running), we retry every tick
     * until it takes — s_boot_home_fired only latches on a successful
     * request_home so a transient rejection doesn't burn the one-shot. */
    if (!s_boot_home_fired
        && s_home_on_boot
        && s_encoder_type_known
        && s_encoder_is_incremental
        && s_state == HOME_SEQ_IDLE) {
        if (home_sequencer_request_home()) {
            LOG_INFO("axis: home_on_boot=1 -> auto-fired home");
            s_boot_home_fired = true;
        }
    }

    /* --- Periodic fault_flags read (keeps is_homed cache coherent) ---
     * Kicked only while the main sequencer is IDLE — piggybacks on the
     * same READING_FAULT state to consume the reply. Rate depends on
     * whether we have a good reading yet: RETRY_MS (short) while unknown,
     * REFRESH_MS (5 s) once known. */
    if (s_state == HOME_SEQ_IDLE) {
        uint32_t period = s_is_homed_known ? FAULT_READ_REFRESH_MS
                                           : FAULT_READ_RETRY_MS;
        bool due = (s_last_fault_read_ms == 0u)
                   || (time_elapsed_ms(s_last_fault_read_ms) >= period);
        if (due) {
            cia402_od_handle_t h = cia402_od_read_begin(0x2600u, 1u, MC_IF_T_U32);
            if (h != CIA402_OD_HANDLE_INVALID) {
                s_handle = h;
                s_state  = HOME_SEQ_READING_FAULT;
                s_last_fault_read_ms = time_ms();
            }
            /* cia402 busy -> leave timestamp alone, try again next tick. */
        }
    }

    /* --- Sequencer body --- */
    if (s_state == HOME_SEQ_IDLE) return;

    /* Absolute timeout — motor should finish or fail within 30 s. */
    if (s_state != HOME_SEQ_READING_FAULT
        && time_elapsed_ms(s_start_ms) > HOME_OVERALL_TIMEOUT_MS) {
        LOG_ERROR("axis: HOME timeout (%d)", (int)s_state);
        s_state = HOME_SEQ_TERMINAL_FAILED;
        s_last_motor_status = MC_IF_HOME_FAILED;
        return;
    }

    switch (s_state) {
    case HOME_SEQ_READING_ENCTYPE: {
        /* Consume the boot-time read of motor 0x2500:8 quad_counts_per_rev.
         * Non-zero -> incremental encoder configured. Zero -> absolute
         * (or unconfigured). Reply size is F32 (4 bytes). */
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_handle, &res, buf, &vlen)) return;
        s_handle = CIA402_OD_HANDLE_INVALID;
        if (res == MC_IF_OD_OK && vlen >= sizeof(float)) {
            float cpr;
            memcpy(&cpr, buf, sizeof(cpr));
            s_encoder_is_incremental = (cpr != 0.0f);
            s_encoder_type_known     = true;
            LOG_INFO("axis: enc %s (cpr=%d)",
                     s_encoder_is_incremental ? "inc" : "abs",
                     (int)cpr);
        }
        /* On failure, s_encoder_type_known stays false and the retry
         * cadence in the IDLE section above will fire the read again. */
        s_state = HOME_SEQ_IDLE;
        break;
    }
    case HOME_SEQ_CLEARING_CMD: {
        /* Write home_command = 0 first to clear any latched DONE/FAILED
         * from a previous run (motor's home_command is edge-triggered per
         * OD contract "0 = idle/abort (also clears done/failed)" — writing
         * 1 while DONE is still latched would be a no-op on the motor
         * side, and the CMC would spin until the overall timeout thinking
         * it was still running). Only after this write completes do we
         * write the actual "1" that arms the run. */
        if (s_handle == CIA402_OD_HANDLE_INVALID) {
            uint8_t zero = 0u;
            s_handle = cia402_od_write_begin(
                0x2700u, 8u, MC_IF_T_U8, &zero, sizeof(zero));
            if (s_handle == CIA402_OD_HANDLE_INVALID) return;
        }
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_handle, &res, buf, &vlen)) return;
        s_handle = CIA402_OD_HANDLE_INVALID;
        if (res != MC_IF_OD_OK) {
            LOG_ERROR("axis: HOME clr fail %d", (int)res);
            s_state = HOME_SEQ_TERMINAL_FAILED;
            s_last_motor_status = MC_IF_HOME_FAILED;
            return;
        }
        s_state = HOME_SEQ_WRITING_CMD;
        break;
    }
    case HOME_SEQ_WRITING_CMD: {
        if (s_handle == CIA402_OD_HANDLE_INVALID) {
            /* Try to issue the write. If cia402 is busy we retry next tick. */
            uint8_t one = 1u;
            s_handle = cia402_od_write_begin(
                0x2700u, 8u, MC_IF_T_U8, &one, sizeof(one));
            if (s_handle == CIA402_OD_HANDLE_INVALID) return;
        }
        /* Wait for reply. */
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_handle, &res, buf, &vlen)) return;
        s_handle = CIA402_OD_HANDLE_INVALID;
        if (res != MC_IF_OD_OK) {
            LOG_ERROR("axis: HOME wr fail %d", (int)res);
            s_state = HOME_SEQ_TERMINAL_FAILED;
            s_last_motor_status = MC_IF_HOME_FAILED;
            return;
        }
        s_state    = HOME_SEQ_POLLING_STATUS;
        s_last_poll_ms = time_ms();
        break;
    }
    case HOME_SEQ_POLLING_STATUS: {
        /* No poll in flight — start a fresh one either right away
         * (first entry) or after HOME_POLL_PERIOD_MS has elapsed. */
        if (s_handle == CIA402_OD_HANDLE_INVALID) {
            if (time_elapsed_ms(s_last_poll_ms) < HOME_POLL_PERIOD_MS
                && time_elapsed_ms(s_start_ms) > 100u) {
                return;   /* rate-limit — motor won't have changed yet */
            }
            s_handle = cia402_od_read_begin(0x2700u, 9u, MC_IF_T_U8);
            if (s_handle == CIA402_OD_HANDLE_INVALID) return;
            s_last_poll_ms = time_ms();
        }
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_handle, &res, buf, &vlen)) return;
        s_handle = CIA402_OD_HANDLE_INVALID;
        if (res != MC_IF_OD_OK || vlen == 0) {
            return;   /* stay in POLLING_STATUS, will retry after period */
        }
        uint8_t status = buf[0];
        if (status != s_last_motor_status) {
            LOG_INFO("axis: HOME st %u->%u",
                     (unsigned)s_last_motor_status, (unsigned)status);
            s_last_motor_status = status;
        }
        if (status == MC_IF_HOME_DONE || status == MC_IF_HOME_FAILED) {
            /* Re-read fault_flags so is_homed reflects the current
             * (still-set on FAILED) NOT_HOMED bit. */
            s_state = HOME_SEQ_READING_FAULT;
        }
        /* else RUNNING / IDLE — keep polling. */
        break;
    }
    case HOME_SEQ_READING_FAULT: {
        if (s_handle == CIA402_OD_HANDLE_INVALID) {
            s_handle = cia402_od_read_begin(0x2600u, 1u, MC_IF_T_U32);
            if (s_handle == CIA402_OD_HANDLE_INVALID) return;
        }
        MC_IfOdResult_t res = MC_IF_OD_OK;
        uint8_t buf[8] = {0};
        uint8_t vlen   = sizeof(buf);
        if (!cia402_od_poll(s_handle, &res, buf, &vlen)) return;
        s_handle = CIA402_OD_HANDLE_INVALID;
        if (res == MC_IF_OD_OK && vlen >= sizeof(uint32_t)) {
            uint32_t ff;
            memcpy(&ff, buf, sizeof(ff));
            bool was_known = s_is_homed_known;
            bool was_homed = s_is_homed;
            s_is_homed = (ff & MC_IF_FAULT_NOT_HOMED) == 0u;
            s_is_homed_known = true;
            if (was_homed != s_is_homed || !was_known) {
                LOG_INFO("axis: is_homed=%d", (int)s_is_homed);
            }
            /* Boot one-shot goes straight to IDLE; sequence terminals log. */
            if (s_last_motor_status == MC_IF_HOME_DONE) {
                s_state = HOME_SEQ_TERMINAL_DONE;
            } else if (s_last_motor_status == MC_IF_HOME_FAILED) {
                s_state = HOME_SEQ_TERMINAL_FAILED;
            } else {
                s_state = HOME_SEQ_IDLE;
            }
        } else {
            s_state = HOME_SEQ_IDLE;
        }
        break;
    }
    case HOME_SEQ_TERMINAL_DONE:
    case HOME_SEQ_TERMINAL_FAILED:
        /* is_homed transition log fires from the READING_FAULT path
         * already; no need for a separate "HOME end" line. */
        s_state = HOME_SEQ_IDLE;
        break;
    default:
        s_state = HOME_SEQ_IDLE;
        break;
    }
}
