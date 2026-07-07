/*
 * app/cmc_state — Path A minimum: ownership state for CAMERAD SELECT/
 * DESELECT/GRAB. See cmc_state.h for the contract.
 */

#include "cmc_state.h"

#include "app/axis_manager/axis_manager.h"
#include "app/log/log.h"
#include "app/persist/persist.h"
#include "bsp/time/time.h"

#include <math.h>
#include <string.h>

/*----------------------------------------------------------------------------
 * Module state
 *---------------------------------------------------------------------------*/

static bool       s_selected;
static uint32_t   s_selected_by;

static cmc_shot_t s_shots[CMC_MAX_SHOTS];

/* Live shot tracking — surfaces in POLL response so panels can show
 * "shot 5, fading, 2.3 s remaining". */
static uint32_t   s_current_shot;            /* 0 = none; else 1..100 */
static uint32_t   s_next_shot;               /* 0 = none; the target of an in-progress move */
static uint32_t   s_time_to_shot_tenths;     /* live remaining or operator-set default */
static bool       s_moving;
static bool       s_on_shot;

/* After move_to_shot is called, the motor MCU's on_target bit is STALE for
 * a few ms — it's still reporting "at the previous shot's position" because
 * it hasn't yet processed the SDO writes + NEW_SETPOINT for the new move.
 * If cmc_state_update_from_motor runs in that window and sees on_target=true,
 * it would falsely declare "arrived at shot N" and clear s_next_shot, so
 * the real arrival a few seconds later is missed entirely.
 *
 * The grace period suppresses arrival detection for ARRIVAL_GRACE_MS after
 * a move request. By then the motor MCU has definitely seen NEW_SETPOINT
 * and either started moving (on_target=0) or completed an instant move
 * (on_target=1 means we really did arrive). 100 ms covers the worst-case
 * SDO setup + cyclic frame round-trip with headroom. */
#define ARRIVAL_GRACE_MS  100u
static uint32_t   s_move_started_ms;

/* Move-acknowledgment watchdog. After a move request the motor MCU should
 * transition to moving=1 within a few cycles (NEW_SETPOINT processing +
 * trajectory startup). If we never see moving=1 within MOVE_ACK_WATCHDOG_MS
 * the move was dropped — most often because:
 *   - cia402 OD pipeline was stuck (one SDO never completed → busy check
 *     keeps NEW_SETPOINT suppressed),
 *   - motor MCU was in FAULT or DISABLED so it rejected the trigger,
 *   - controlword ENABLE bit wasn't set (operator disabled mid-move).
 * Logged once per move so the operator sees "MISSED MOVE" appear in the
 * log within ~500 ms instead of having to spot the absence of a movement
 * transition. Reset on every new move request. */
#define MOVE_ACK_WATCHDOG_MS  500u
static bool       s_move_ack_seen;     /* true once motor reports moving=1 since last move_to_shot */
static bool       s_move_ack_warned;   /* true after we've logged the WARN — don't spam */

/* Joystick watchdog — track when we last received a CAMERAD MOVEMENT
 * message. If silence exceeds JOYSTICK_WATCHDOG_MS we zero the joystick
 * so a hard-disconnected panel doesn't leave the motor running.
 *
 * 500 ms (was 100 ms). Observed panels burst-send MOVEMENT at ~25 ms
 * nominal but with occasional 200 ms silences (Wireshark capture3 on
 * 2026-07-03 confirms). A 100 ms watchdog fires on those silences and
 * causes the motor to decelerate to zero then reaccelerate when trims
 * resume — the operator sees "start-stop" motion. 500 ms survives any
 * plausible panel/network hiccup while still stopping the axis within
 * half a second of a genuine disconnect. Panel-released state still
 * arrives as an explicit Pan=0 MOVEMENT frame (not gated by this).
 * raw value so the motor stops, rather than holding the last commanded
 * deflection forever (panel disconnect / network drop safety). */
#define JOYSTICK_WATCHDOG_MS  500u
static uint32_t   s_last_movement_ms;
static bool       s_movement_active;        /* true between first MOVEMENT and the watchdog firing */

/* Forward decls — defined further down (in the persistence + shot sections). */
static bool       load_shots_from_flash(void);

void cmc_state_init(void)
{
    s_selected            = false;
    s_selected_by         = 0;
    s_current_shot        = 0u;
    s_next_shot           = 0u;
    s_time_to_shot_tenths = 0u;
    s_moving              = false;
    s_on_shot             = false;

    /* Try to load the persisted shot table. On any failure (uninitialised
     * flash, CRC mismatch, version bump) all slots stay invalid and the
     * operator starts from an empty table. */
    memset(s_shots, 0, sizeof(s_shots));
    (void)load_shots_from_flash();

    LOG_INFO("cmc_state: ready");
}

bool cmc_state_handle_select(uint32_t controller_no)
{
    if (s_selected && s_selected_by != controller_no) {
        LOG_INFO("cmc_state: SELECT from ctrl=%lu DENIED (owned by ctrl=%lu)",
                 (unsigned long)controller_no, (unsigned long)s_selected_by);
        return false;
    }
    s_selected    = true;
    s_selected_by = controller_no;

    /* Auto-enable the axis on SELECT. From the operator's perspective, the
     * panel pressing SELECT means "I'm claiming this camera and intend to
     * use it" — they expect subsequent CUT/FADE/MOVEMENT to actually move
     * the motor without first having to mess with the PC tool to write
     * 0x3010 = 1. Fail-on semantics: once enabled, stay enabled even on
     * DESELECT or panel timeout — the only way to disable is an explicit
     * OD write (0x3010 = 0) from PC tool / web UI. */
    (void)axis_manager_request_enable(true);

    LOG_INFO("cmc_state: SELECTED by ctrl=%lu (axis auto-enabled)",
             (unsigned long)controller_no);
    return true;
}

bool cmc_state_handle_deselect(uint32_t controller_no)
{
    if (!s_selected || s_selected_by != controller_no) {
        LOG_WARN("cmc_state: DESELECT from ctrl=%lu ignored (selected=%d, by=%lu)",
                 (unsigned long)controller_no, (int)s_selected,
                 (unsigned long)s_selected_by);
        return false;
    }
    s_selected    = false;
    s_selected_by = 0;
    LOG_INFO("cmc_state: DESELECTED by ctrl=%lu", (unsigned long)controller_no);
    return true;
}

void cmc_state_handle_grab(uint32_t controller_no)
{
    if (s_selected && s_selected_by != controller_no) {
        LOG_INFO("cmc_state: GRAB ctrl=%lu took ownership from ctrl=%lu",
                 (unsigned long)controller_no, (unsigned long)s_selected_by);
    } else {
        LOG_INFO("cmc_state: GRAB by ctrl=%lu", (unsigned long)controller_no);
    }
    s_selected    = true;
    s_selected_by = controller_no;
    /* Same auto-enable policy as SELECT — see cmc_state_handle_select. */
    (void)axis_manager_request_enable(true);
}

void cmc_state_force_deselect(uint32_t controller_no)
{
    /* Only clear if this controller is actually the current owner.
     * Otherwise the call is a no-op (the timeout fired for a controller
     * that wasn't selected anyway — fine). */
    if (s_selected && s_selected_by == controller_no) {
        s_selected    = false;
        s_selected_by = 0;

        /* Mirror of the auto-enable in cmc_state_handle_select: when the
         * owning panel disappears (5s POLL silence), drop axis_enable so
         * the motor goes cold. This is a ONE-SHOT write — we set
         * enable=0 here and the PC tool can re-enable any time afterwards
         * by writing 0x3010 = 1. The naturally-one-shot behaviour falls
         * out of controller_mgr's evict_controller wiping the slot
         * (s_ctrl_a.in_use=false), so the timeout check early-returns on
         * subsequent ticks and we won't keep re-asserting enable=0. */
        (void)axis_manager_request_enable(false);

        LOG_INFO("cmc_state: FORCE-DESELECT ctrl=%lu (controller_mgr timeout; axis auto-disabled)",
                 (unsigned long)controller_no);
    }
}

bool     cmc_state_is_selected(void)  { return s_selected;    }
uint32_t cmc_state_selected_by(void)  { return s_selected_by; }

uint32_t cmc_state_current_shot       (void) { return s_current_shot; }
uint32_t cmc_state_next_shot          (void) { return s_next_shot; }
uint32_t cmc_state_time_to_shot_tenths(void) { return s_time_to_shot_tenths; }
bool     cmc_state_on_shot            (void) { return s_on_shot; }
bool     cmc_state_moving             (void) { return s_moving; }

/*----------------------------------------------------------------------------
 * Shot persistence
 *
 * On-flash blob = 16-byte header + 100 × 12-byte slot. Schema version 2;
 * version 1 was the Path-A placeholder (incompatible — rejected by persist).
 *---------------------------------------------------------------------------*/

#define SHOTS_PERSIST_VERSION   2u

typedef struct __attribute__((packed)) {
    uint32_t magic;            /* SHOTS_BLOB_MAGIC; sanity beyond persist's */
    uint32_t shot_count;       /* always CMC_MAX_SHOTS (100) for now */
    uint32_t reserved[2];      /* pad to 16 byte header */
    struct __attribute__((packed)) {
        uint8_t  valid;        /* 0/1 */
        uint8_t  pad[3];
        float    position_rad;
        float    time_to_shot_s;
    } slots[CMC_MAX_SHOTS];    /* 100 × 12 = 1200 bytes */
} shots_persist_blob_t;
#define SHOTS_BLOB_MAGIC   0x53484F54u   /* 'SHOT' */

_Static_assert(sizeof(((shots_persist_blob_t *)0)->slots[0]) == 12,
               "shot slot must be 12 bytes");
_Static_assert(sizeof(shots_persist_blob_t) == 16 + 12 * CMC_MAX_SHOTS,
               "shots blob layout drift");

static bool load_shots_from_flash(void)
{
    shots_persist_blob_t blob;
    size_t got = 0;
    if (!persist_load(PERSIST_REGION_SHOTS, &blob, sizeof(blob),
                      SHOTS_PERSIST_VERSION, &got)) {
        return false;
    }
    if (got != sizeof(blob) || blob.magic != SHOTS_BLOB_MAGIC
                            || blob.shot_count != CMC_MAX_SHOTS) {
        LOG_WARN("cmc_state: shots blob layout mismatch — ignoring");
        return false;
    }
    uint32_t loaded = 0;
    for (uint32_t i = 0; i < CMC_MAX_SHOTS; i++) {
        s_shots[i].valid          = (blob.slots[i].valid != 0);
        s_shots[i].position_rad   = blob.slots[i].position_rad;
        s_shots[i].time_to_shot_s = blob.slots[i].time_to_shot_s;
        if (s_shots[i].valid) loaded++;
    }
    LOG_INFO("cmc_state: shots loaded from flash (%lu/%u valid)",
             (unsigned long)loaded, (unsigned)CMC_MAX_SHOTS);
    return true;
}

bool cmc_state_save_shots(void)
{
    shots_persist_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic      = SHOTS_BLOB_MAGIC;
    blob.shot_count = CMC_MAX_SHOTS;
    for (uint32_t i = 0; i < CMC_MAX_SHOTS; i++) {
        blob.slots[i].valid          = s_shots[i].valid ? 1 : 0;
        blob.slots[i].position_rad   = s_shots[i].position_rad;
        blob.slots[i].time_to_shot_s = s_shots[i].time_to_shot_s;
    }
    bool ok = persist_save(PERSIST_REGION_SHOTS, &blob, sizeof(blob),
                           SHOTS_PERSIST_VERSION);
    if (!ok) LOG_ERROR("cmc_state: shots save FAILED");
    return ok;
}

/*----------------------------------------------------------------------------
 * Shot operations
 *
 * shot_no over the wire is 1-indexed; internal array is 0-indexed.
 * `idx = shot_no - 1`. Caller-side validation rejects shot_no == 0 and
 * shot_no > CMC_MAX_SHOTS.
 *---------------------------------------------------------------------------*/

static bool valid_shot_no(uint32_t shot_no)
{
    return shot_no >= 1u && shot_no <= CMC_MAX_SHOTS;
}

int cmc_state_store_shot(uint32_t shot_no)
{
    if (!valid_shot_no(shot_no)) {
        LOG_WARN("cmc_state: store_shot bad shot_no=%lu", (unsigned long)shot_no);
        return -1;
    }
    uint32_t idx = shot_no - 1u;
    float pos = axis_manager_get_position_actual();
    /* The "current default time-to-shot" the operator most recently set
     * via KEY_STORE_TIME_TO_SHOT is captured into the slot too. Tenths
     * → seconds. If never set, this is 0. */
    float t_s = (float)s_time_to_shot_tenths * 0.1f;

    s_shots[idx].valid          = true;
    s_shots[idx].position_rad   = pos;
    s_shots[idx].time_to_shot_s = t_s;
    s_current_shot = shot_no;

    /* Integer-scaled (mrad, ms) — nano-libc's printf has %f disabled by
     * default in this build, so %f silently prints nothing. Using ints
     * also makes the values grep-friendly. */
    LOG_INFO("cmc_state: STORE shot %lu pos=%ld mrad t=%ld ms",
             (unsigned long)shot_no,
             (long)lroundf(pos * 1000.0f),
             (long)lroundf(t_s * 1000.0f));

    /* Save-on-every-store per the v1 decision. The user accepts the
     * flash-wear cost in exchange for never losing a stored shot. */
    (void)cmc_state_save_shots();
    return (int)idx;
}

uint32_t cmc_state_store_next(void)
{
    uint32_t next = (s_current_shot > 0u && s_current_shot < CMC_MAX_SHOTS)
                    ? s_current_shot + 1u
                    : 1u;
    if (cmc_state_store_shot(next) >= 0) return next;
    return 0u;
}

void cmc_state_set_time_to_shot_tenths(uint32_t tenths)
{
    s_time_to_shot_tenths = tenths;
    LOG_INFO("cmc_state: set time_to_shot = %lu tenths (=%lu ms)",
             (unsigned long)tenths, (unsigned long)(tenths * 100u));
}

/* Minimum move duration for FADE. Matches Reduced CMC cmc_state.c:363-365:
 * even when the operator-set tenths is zero AND the shot has no stored time,
 * a 100 ms floor avoids asking the motor to do an unbounded-velocity step.
 * CUT skips this floor — it intentionally requests time=0 (motor MCU
 * interprets as "as fast as the velocity/accel limits allow"). */
#define FADE_MIN_TIME_S   0.100f

/* === The "lock-in then fade" sequence (Reduced CMC pattern) =================
 *
 * Two-step CAMERAD flow:
 *
 *   1) KEY_STORE_TIME_TO_SHOT (0xA1) — operator presses a duration on the
 *      panel (e.g. "3.0"). Panel sends KP_T1 {key=0xA1, value=30 tenths}.
 *      controller_mgr.c::handle_keypress_t1 → cmc_state_set_time_to_shot_tenths
 *      → s_time_to_shot_tenths = 30. The value is LOCKED IN cmc_state and
 *      persists across calls until either (a) the next STORE_TIME_TO_SHOT
 *      overwrites it, or (b) a move consumes it (see below).
 *
 *   2) KEY_FADE / KEY_CUT (0x09/0x08/...) — operator presses the move key
 *      with a shot number. controller_mgr.c::handle_keypress_t1 →
 *      cmc_state_move_to_shot(shot_no, is_cut). This function:
 *        - looks up the shot's stored position,
 *        - picks the duration: operator-locked > shot-stored > 0,
 *        - drives axis_manager (PROFILE_POSITION + target_position +
 *          target_time + NEW_SETPOINT) which the cia402 sequencer hands
 *          off to the motor MCU as SDO writes + a cyclic-command bit,
 *        - re-stores the chosen duration into s_time_to_shot_tenths so
 *          the POLL response can show it.
 *
 * The motor MCU receives `0x607B target_position_time_ms` (uint32 ms) as
 * part of the PROFILE_POSITION SDO setup, then sees the NEW_SETPOINT bit
 * flip on the cyclic controlword, then runs its own trajectory generator
 * over that duration. The CMC is only responsible for the lock-in,
 * lookup, and dispatch — no motion math here.
 */
bool cmc_state_move_to_shot(uint32_t shot_no, bool is_cut)
{
    if (!valid_shot_no(shot_no)) {
        LOG_WARN("cmc_state: move_to_shot bad shot_no=%lu", (unsigned long)shot_no);
        return false;
    }
    uint32_t idx = shot_no - 1u;
    if (!s_shots[idx].valid) {
        LOG_WARN("cmc_state: move_to_shot %lu — slot empty", (unsigned long)shot_no);
        return false;
    }
    /* Gate shot recalls on the motor's homed state (CHANGELOG 4.9.0).
     * position_actual on an un-homed incremental encoder is meaningless as
     * an absolute reference, so recalling a stored position could send the
     * rig anywhere. Operator must run the home-to-endstop procedure via
     * axis_manager (OD 0x3040 = 1) first. axis_manager_is_homed() reports
     * false conservatively until fault_flags has been successfully read
     * at least once — protects the very first recall attempt at boot. */
    if (!axis_manager_is_homed()) {
        LOG_WARN("cmc_state: shot %lu rejected: not homed", (unsigned long)shot_no);
        return false;
    }

    /* Operation arbitration. STARTED = fresh recall (first-entry path below
     * writes op_mode + NEW_SETPOINT). CONTINUED = retarget of an in-flight
     * recall (operator changed their mind mid-fade). REJECTED = mid-home or
     * mid-joystick — swallow the recall, operator must STOP first. */
    axis_begin_result_t br = axis_manager_try_begin_op(AXIS_OPERATION_SHOT_RECALL);
    if (br == AXIS_BEGIN_REJECTED) {
        LOG_WARN("cmc_state: shot %lu rejected: another operation in flight", (unsigned long)shot_no);
        return false;
    }

    float target = s_shots[idx].position_rad;

    /* Pick the duration: matches Reduced CMC cmc_state.c:358-365 precedence. */
    float t_s;
    if (is_cut) {
        t_s = 0.0f;                                              /* instant */
    } else if (s_time_to_shot_tenths > 0u) {
        t_s = (float)s_time_to_shot_tenths * 0.1f;               /* operator-locked */
    } else if (s_shots[idx].time_to_shot_s > 0.0f) {
        t_s = s_shots[idx].time_to_shot_s;                       /* shot's stored time */
    } else {
        t_s = FADE_MIN_TIME_S;                                   /* defensive minimum */
    }
    if (!is_cut && t_s < FADE_MIN_TIME_S) {
        t_s = FADE_MIN_TIME_S;
    }

    axis_manager_set_op_mode(AXIS_OP_MODE_PROFILE_POSITION);
    axis_manager_set_target_position(target);
    axis_manager_set_target_time(t_s);
    bool ok = axis_manager_request_start_move();

    if (ok) {
        s_next_shot              = shot_no;
        s_moving                 = true;
        s_on_shot                = false;
        s_time_to_shot_tenths    = (uint32_t)lroundf(t_s * 10.0f);
        s_move_started_ms        = time_ms();     /* arrival-grace + ack-watchdog start */
        s_move_ack_seen          = false;
        s_move_ack_warned        = false;
        LOG_INFO("cmc_state: %s to shot %lu target=%ld mrad t=%lu ms",
                 is_cut ? "CUT" : "FADE",
                 (unsigned long)shot_no,
                 (long)lroundf(target * 1000.0f),
                 (unsigned long)lroundf(t_s * 1000.0f));
    } else {
        LOG_WARN("cmc_state: move_to_shot %lu — start_move rejected",
                 (unsigned long)shot_no);
    }
    return ok;
}

void cmc_state_stop_movement(void)
{
    /* Panel STOP keys → CiA-402 HALT (controlled stop + hold position).
     * Was previously quick_stop (hard decel + disable) which left the
     * motor cold. HALT keeps the motor enabled so the operator's next
     * CUT/FADE just works without a re-enable step. The HALT latch is
     * cleared automatically inside axis_manager_request_start_move on
     * the next move command. For emergency-stop, use OD 0x3011 directly
     * (still wired to axis_manager_request_quick_stop). */
    (void)axis_manager_request_halt();
    axis_manager_stop_op();  /* clear operation arbitration; forces active_op -> NONE */
    s_moving    = false;
    s_next_shot = 0u;
    LOG_INFO("cmc_state: STOP (HALT asserted to motor)");
}

bool cmc_state_get_shot(uint32_t shot_no, cmc_shot_t *out)
{
    if (!out || !valid_shot_no(shot_no)) return false;
    uint32_t idx = shot_no - 1u;
    if (!s_shots[idx].valid) return false;
    *out = s_shots[idx];
    return true;
}

/*----------------------------------------------------------------------------
 * Live-state update — derive moving/on_shot/time_to_shot from the motor's
 * actual position vs. the target. Coarse for Path A; Phase B will use the
 * motor MCU's trajectory_time_remaining_ms (REQ-0011) when it lands.
 *---------------------------------------------------------------------------*/

/* (ON_SHOT_TOLERANCE_RAD removed — the position-based on-target check has
 * been replaced by the motor MCU's MC_IF_MOVE_ON_TARGET bit, exposed via
 * axis_manager_is_on_target(). See REQ-0013 / v4 cyclic header.) */

/*----------------------------------------------------------------------------
 * Joystick / MOVEMENT input
 *
 * Auto-switches op_mode to JOYSTICK so the cyclic SPI command carries the
 * resulting velocity demand. axis_manager already does the raw→normalised
 * → target_velocity translation via the cal entries (0x3026-302A) and the
 * joystick_max_velocity cap (0x3022).
 *---------------------------------------------------------------------------*/

void cmc_state_handle_movement(int8_t pan)
{
    /* Operation arbitration (via axis_manager). CONTINUED = we're already in
     * JOYSTICK; the trim retargets. STARTED = new joystick session; caller
     * will drop through and set op_mode + joystick_raw. REJECTED = mid-home
     * or mid-shot-recall; swallow the trim entirely.
     *
     * We deliberately do NOT stamp s_last_movement_ms while rejecting —
     * the watchdog stays passive, which is correct: there's nothing to
     * watchdog because no joystick input is being honoured. Logging is
     * suppressed to keep the log clean during multi-second fades (panels
     * send MOVEMENT at ~25 ms intervals — 120 rejection lines per 3 s
     * would dominate the log). */
    axis_begin_result_t br = axis_manager_try_begin_op(AXIS_OPERATION_JOYSTICK);
    if (br == AXIS_BEGIN_REJECTED) {
        return;
    }

    /* Stamp last-seen BEFORE the write so a tick interleaved with the
     * receive still sees a valid window. */
    s_last_movement_ms = time_ms();
    s_movement_active  = true;

    /* Auto-switch op_mode to JOYSTICK on any (non-rejected) movement.
     * After a fade/cut completes, the next MOVEMENT brings the operator
     * back into live control automatically. The first call after a mode
     * change pays one SDO write to 0x6060 on the motor MCU (~few ms);
     * subsequent calls in the same mode are free. */
    if (axis_manager_get_op_mode() != AXIS_OP_MODE_JOYSTICK) {
        (void)axis_manager_set_op_mode((uint8_t)AXIS_OP_MODE_JOYSTICK);
    }

    /* Single-motor CMC: only the pan axis maps to our one motor. Other
     * MOVEMENT axes (tilt, focus, zoom, etc.) are ignored. */
    (void)axis_manager_set_joystick_raw((int32_t)pan);
}

void cmc_state_update_from_motor(void)
{
    /* Joystick watchdog: if MOVEMENT messages have stopped, zero the raw
     * so the motor sees velocity=0 and stops. Stale-deflection-held-
     * forever is the failure mode this avoids. */
    if (s_movement_active && time_elapsed_ms(s_last_movement_ms) > JOYSTICK_WATCHDOG_MS) {
        (void)axis_manager_set_joystick_raw(0);
        s_movement_active = false;
        LOG_INFO("cmc_state: joystick watchdog fired (no MOVEMENT in %u ms) — zeroed",
                 (unsigned)JOYSTICK_WATCHDOG_MS);
    }

    /* moving/on_target are now read directly from the motor MCU's
     * cyclic-status header (REQ-0013/ADR-033). The motor MCU is the
     * authoritative source: it knows when the trajectory generator is
     * still working, when the position-loop is settled, and the
     * appropriate hysteresis — far better than a CMC-side tolerance
     * compare against a possibly-stale position reading. */
    bool motor_moving    = axis_manager_is_moving();
    bool motor_on_target = axis_manager_is_on_target();

    /* Move-acknowledgment watchdog (paired with s_move_started_ms set in
     * cmc_state_move_to_shot). If a shot move was requested and we never
     * see motor_moving=1 within MOVE_ACK_WATCHDOG_MS, fire ONE WARN so
     * the operator notices in the log even if they weren't watching.
     * Latching motor_moving=true into s_move_ack_seen lets us also detect
     * the "started moving but never reached on_target" case (separately —
     * arrival logic above takes care of it when it does arrive). */
    if (s_next_shot != 0u && s_shots[s_next_shot - 1u].valid) {
        if (motor_moving) s_move_ack_seen = true;
        if (!s_move_ack_seen && !s_move_ack_warned
            && time_elapsed_ms(s_move_started_ms) > MOVE_ACK_WATCHDOG_MS) {
            LOG_WARN("cmc_state: MISSED MOVE — shot %lu requested %lu ms ago but motor never started moving "
                     "(check earlier log: NEW_SETPOINT triggered? SDO writes complete? motor enabled / faulted?)",
                     (unsigned long)s_next_shot,
                     (unsigned long)time_elapsed_ms(s_move_started_ms));
            s_move_ack_warned = true;
        }
    }

    if (s_next_shot == 0u || !s_shots[s_next_shot - 1u].valid) {
        /* No commanded shot — moving might still be true (e.g. operator
         * is on the joystick). Track moving directly from the motor;
         * leave on_shot/current_shot alone (they're sticky to the last
         * arrived-at shot). */
        s_moving = motor_moving;
        return;
    }

    /* A shot is in progress (s_next_shot != 0). Arrival = motor reports
     * on_target AND not still moving — BUT only after the arrival-grace
     * period has elapsed. Motor MCU's on_target bit is stale during the
     * window between move_to_shot returning and NEW_SETPOINT actually
     * reaching the motor (it still shows the PREVIOUS shot's "at target"
     * state). Trusting it too early causes false-positive arrivals that
     * clear s_next_shot and orphan the actual move. */
    bool grace_active = (time_elapsed_ms(s_move_started_ms) < ARRIVAL_GRACE_MS);

    if (motor_on_target && !motor_moving && !grace_active) {
        if (s_moving) {
            LOG_INFO("cmc_state: arrived at shot %lu (motor on_target)",
                     (unsigned long)s_next_shot);
        }
        s_moving       = false;
        s_on_shot      = true;
        s_current_shot = s_next_shot;
        s_next_shot    = 0u;
        s_time_to_shot_tenths = 0u;
    } else {
        /* Either still moving, or in the post-move grace window. Keep
         * s_moving true until we actually arrive — panel UI shows the
         * "moving" indicator continuously from press through arrival,
         * including the brief grace window where motor MCU hasn't yet
         * cleared its on_target bit from the previous shot. */
        s_moving  = true;
        s_on_shot = false;
        /* Time-to-shot countdown still not implemented (would need the
         * live ETA from REQ-0011 trajectory_time_remaining_ms). */
    }
}

