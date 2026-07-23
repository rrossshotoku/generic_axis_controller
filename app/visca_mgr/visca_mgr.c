/*
 * app/visca_mgr — implementation. See visca_mgr.h for scope.
 *
 * Structure:
 *   - init: open UDP :52381, cache identity constants.
 *   - tick: recvfrom → parse IP header → parse VISCA frame → dispatch:
 *       inquiry: build inquiry reply → send
 *       command: apply side-effect, build ACK + Completion → send
 *   - dispatch_inquiry / dispatch_command split by (class, category, command).
 *
 * Position scaling (VISCA int16 <-> radians):
 *   VISCA position is a signed 16-bit that different cameras interpret
 *   differently. For this CMC we pick 1 unit = 1 milliradian, so int16
 *   range ±32.767 rad ≈ ±1877° covers any conceivable pan sweep. The
 *   PC tool decodes the same way.
 *
 * Response transmission:
 *   For commands we send TWO IP packets (ACK, then Completion), matching
 *   real Sony behaviour so a strict controller sees the shape it expects.
 *   Both share the same sequence_number. If net_sendto fails, we drop
 *   silently (log-throttled) — retransmit is the client's job in UDP.
 */
#include "visca_mgr.h"

#include "app/visca/visca.h"
#include "app/axis_manager/axis_manager.h"
#include "app/cmc_state/cmc_state.h"
#include "app/config/config.h"
#include "app/log/log.h"
#include "bsp/net/net.h"
#include "bsp/time/time.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Socket + port
 *
 * Reuses the same physical W6100 socket 0 as controller_mgr's POLL_SOCKET.
 * SAFE because main_loop dispatches to exactly one of controller_mgr /
 * visca_mgr at boot — they never coexist. If that invariant ever changes
 * (would require deinit paths neither mgr has today), this sharing needs
 * revisiting.
 *---------------------------------------------------------------------------*/
#define VISCA_UDP_SOCKET   ((net_sock_t)0)
#define VISCA_UDP_PORT     ((uint16_t)52381)

/*----------------------------------------------------------------------------
 * Identity constants (CAM_VersionInq response)
 *
 * These aren't a real Sony vendor ID. Picked so operators can identify
 * the CMC in logs. Feel free to bump ROM version on releases.
 *---------------------------------------------------------------------------*/
#define VISCA_VENDOR_ID     (0xAAAAu)
#define VISCA_MODEL_ID      (0x0001u)     /* generic axis controller, model 1 */
#define VISCA_ROM_VERSION   (0x0100u)     /* v1.0 — bump on protocol changes */
#define VISCA_SOCKET_COUNT  (0x02u)       /* we don't actually queue commands; matches Sony convention */

/*----------------------------------------------------------------------------
 * Position scale
 *
 * 1 VISCA position unit = 1 milliradian. int16 range ±32.767 rad ≈ ±1877°.
 *---------------------------------------------------------------------------*/
#define VISCA_POS_UNITS_PER_RAD  (1000.0f)

/*----------------------------------------------------------------------------
 * VISCA device address (1..7). Sourced from config's cmc_device_no (which
 * is also what CAMERAD advertises as return_device_no) so operators only
 * have to set one number for "this CMC's identity." VISCA restricts the
 * address to 1..7; values outside that range get clamped to 1 with a
 * WARN log so it's obvious in the boot trace.
 *
 * Snapshot once at init — changes require Save + Reboot, matching how
 * the rest of the network config behaves.
 *---------------------------------------------------------------------------*/
static uint8_t s_our_address = 1u;   /* set in visca_mgr_init from config */

/*----------------------------------------------------------------------------
 * Axis role mapping
 *
 * The CMC's axis_role (OD 0x3070, set on the web page) picks which physical
 * axis this hardware represents. CAMERAD uses this to select one byte from
 * an 8-axis MOVEMENT frame; VISCA doesn't pack axes that way, so we use
 * axis_role to decide which VISCA command byte to consume:
 *
 *   PAN   → Pan_tiltDrive uses VV+YY (pan speed + direction, ÷24)
 *           Pan_tiltPosInq puts motor position in pan nibbles, tilt=0
 *   TILT  → Pan_tiltDrive uses WW+ZZ (tilt speed + direction, ÷20)
 *           Pan_tiltPosInq puts motor position in tilt nibbles, pan=0
 *
 * Other roles (ZOOM/FOCUS/X/Y/HEIGHT/FADER) don't map onto Pan_tiltDrive.
 * VISCA's per-axis commands for those (CAM_Zoom, CAM_Focus, direct
 * position variants) are not yet implemented — until they are, any
 * Pan_tiltDrive that arrives while axis_role is one of those roles is
 * rejected with NOT_EXECUTABLE and logged. ZOOM specifically will pick
 * up CAM_ZoomPosInq once we add it; today CAM_ZoomPosInq is always 0.
 *
 * Values mirror CAMERAD_AXIS_* / mc_if_od.h axis_role bitmap. Defined
 * locally so we don't include the CAMERAD codec header from a VISCA
 * transport module.
 *---------------------------------------------------------------------------*/
#define VISCA_ROLE_PAN     (0x01u)
#define VISCA_ROLE_TILT    (0x02u)
#define VISCA_ROLE_ZOOM    (0x04u)
#define VISCA_ROLE_FOCUS   (0x08u)
#define VISCA_ROLE_X       (0x10u)
#define VISCA_ROLE_Y       (0x20u)
#define VISCA_ROLE_HEIGHT  (0x40u)
#define VISCA_ROLE_FADER   (0x80u)

/* Snapshot at init; changes require Save + Reboot. */
static uint8_t s_axis_role = VISCA_ROLE_PAN;

/*----------------------------------------------------------------------------
 * Command socket used in ACK/Completion. Real Sony cameras rotate between
 * sockets 1 and 2 for concurrent commands; we always use 1 (single-in-flight
 * behaviour, described in visca_mgr.h "session model").
 *---------------------------------------------------------------------------*/
#define VISCA_TX_SOCKET     (1u)

/*----------------------------------------------------------------------------
 * Buffers
 *---------------------------------------------------------------------------*/
#define VISCA_RX_BUF_SIZE   (64u)    /* well over any real command */
#define VISCA_TX_BUF_SIZE   (64u)

static uint8_t s_rx_buf[VISCA_RX_BUF_SIZE];
static uint8_t s_tx_buf[VISCA_TX_BUF_SIZE];

/* Log-throttle counter for send failures — don't spam if the link is down. */
static uint32_t s_send_fail_log_count = 0;

/*----------------------------------------------------------------------------
 * Send helpers
 *---------------------------------------------------------------------------*/

/* Wrap a raw VISCA payload in an IP header and send it back to `peer`.
 * `visca_payload` points into s_tx_buf starting at offset VISCA_IP_HEADER_SIZE;
 * we assemble in place so we don't need a second buffer. Returns true on
 * successful net_sendto; false on error (logged, throttled). */
static bool send_visca_reply(const net_addr_t *peer, uint16_t payload_type,
                             uint32_t sequence, size_t visca_len)
{
    visca_ip_build_header(s_tx_buf, payload_type,
                          (uint16_t)visca_len, sequence);
    size_t total = VISCA_IP_HEADER_SIZE + visca_len;
    int32_t sent = net_sendto(VISCA_UDP_SOCKET, peer, s_tx_buf, total);
    if (sent < (int32_t)total) {
        if (s_send_fail_log_count < 8u) {
            s_send_fail_log_count++;
            LOG_WARN("visca_mgr: net_sendto returned %ld (needed %u)",
                     (long)sent, (unsigned)total);
        }
        return false;
    }
    return true;
}

/* Convenience: build an ACK, send it back. */
static void send_ack(const net_addr_t *peer, uint32_t sequence)
{
    size_t len = visca_build_ack(&s_tx_buf[VISCA_IP_HEADER_SIZE],
                                 VISCA_TX_BUF_SIZE - VISCA_IP_HEADER_SIZE,
                                 s_our_address, VISCA_TX_SOCKET);
    if (len == 0u) return;
    (void)send_visca_reply(peer, VISCA_IP_TYPE_REPLY, sequence, len);
}

/* Convenience: build a Completion, send it back. */
static void send_completion(const net_addr_t *peer, uint32_t sequence)
{
    size_t len = visca_build_completion(&s_tx_buf[VISCA_IP_HEADER_SIZE],
                                        VISCA_TX_BUF_SIZE - VISCA_IP_HEADER_SIZE,
                                        s_our_address, VISCA_TX_SOCKET);
    if (len == 0u) return;
    (void)send_visca_reply(peer, VISCA_IP_TYPE_REPLY, sequence, len);
}

/* Convenience: send an error reply (uses socket 0 for uncorrelated errors
 * per Sony convention when we can't attribute to a specific in-flight cmd). */
static void send_error(const net_addr_t *peer, uint32_t sequence,
                       uint8_t socket, uint8_t err_type)
{
    size_t len = visca_build_error(&s_tx_buf[VISCA_IP_HEADER_SIZE],
                                   VISCA_TX_BUF_SIZE - VISCA_IP_HEADER_SIZE,
                                   s_our_address, socket, err_type);
    if (len == 0u) return;
    (void)send_visca_reply(peer, VISCA_IP_TYPE_REPLY, sequence, len);
}

/* Convenience: send a two-frame ACK + Completion pair (in that order) as
 * two separate UDP packets. Matches real Sony camera behaviour on Command-
 * class frames. */
static void send_ack_and_completion(const net_addr_t *peer, uint32_t sequence)
{
    send_ack(peer, sequence);
    send_completion(peer, sequence);
}

/*----------------------------------------------------------------------------
 * Inquiry handlers
 *
 * Each returns true if it wrote a reply (and it was sent), false if the
 * inquiry wasn't recognised. Wrong inquiries fall through to the parent
 * dispatch which sends a syntax error.
 *---------------------------------------------------------------------------*/

static bool inq_version(const net_addr_t *peer, uint32_t sequence)
{
    /* CAM_VersionInq: reply is 7 bytes vendor/model/rom/socket_count. */
    uint8_t body[7];
    visca_pack_version_body(body, VISCA_VENDOR_ID, VISCA_MODEL_ID,
                            VISCA_ROM_VERSION, VISCA_SOCKET_COUNT);
    size_t len = visca_build_inquiry_reply(&s_tx_buf[VISCA_IP_HEADER_SIZE],
                                           VISCA_TX_BUF_SIZE - VISCA_IP_HEADER_SIZE,
                                           s_our_address, body, sizeof(body));
    if (len == 0u) return false;
    return send_visca_reply(peer, VISCA_IP_TYPE_REPLY, sequence, len);
}

/* Convert a radian value to a VISCA int16 with saturation. */
static int16_t rad_to_visca_pos(float rad)
{
    float v = rad * VISCA_POS_UNITS_PER_RAD;
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

static bool inq_pantilt_pos(const net_addr_t *peer, uint32_t sequence)
{
    /* Pan_tiltPosInq: 8 bytes (4 nibbles pan + 4 nibbles tilt).
     * Single-axis CMC: motor position lands in the pan OR tilt nibbles
     * based on axis_role. If the role isn't PAN or TILT, both fields
     * are zero — a VISCA client asking about pan/tilt when we represent
     * a zoom / focus / etc. axis gets a truthful "not pointing anywhere". */
    int16_t motor_pos = rad_to_visca_pos(axis_manager_get_position_actual());
    int16_t pan  = 0;
    int16_t tilt = 0;
    if      (s_axis_role == VISCA_ROLE_PAN)  pan  = motor_pos;
    else if (s_axis_role == VISCA_ROLE_TILT) tilt = motor_pos;
    /* other roles: both stay 0 */
    uint8_t body[8];
    visca_pack_pantilt_pos_body(body, pan, tilt);
    size_t len = visca_build_inquiry_reply(&s_tx_buf[VISCA_IP_HEADER_SIZE],
                                           VISCA_TX_BUF_SIZE - VISCA_IP_HEADER_SIZE,
                                           s_our_address, body, sizeof(body));
    if (len == 0u) return false;
    return send_visca_reply(peer, VISCA_IP_TYPE_REPLY, sequence, len);
}

static bool inq_zoom_pos(const net_addr_t *peer, uint32_t sequence)
{
    /* CAM_ZoomPosInq: 4 bytes (4 nibbles zoom). If axis_role is ZOOM
     * report motor position; otherwise 0. Note we do NOT accept CAM_Zoom
     * drive commands yet, so a ZOOM-role client can read back position
     * but has no way to command motion via VISCA today. Add the drive
     * commands (CAM_Zoom Standard 0x07, Direct 0x47) when needed. */
    int16_t zoom = 0;
    if (s_axis_role == VISCA_ROLE_ZOOM) {
        zoom = rad_to_visca_pos(axis_manager_get_position_actual());
    }
    uint8_t body[4];
    visca_pack_zoom_pos_body(body, zoom);
    size_t len = visca_build_inquiry_reply(&s_tx_buf[VISCA_IP_HEADER_SIZE],
                                           VISCA_TX_BUF_SIZE - VISCA_IP_HEADER_SIZE,
                                           s_our_address, body, sizeof(body));
    if (len == 0u) return false;
    return send_visca_reply(peer, VISCA_IP_TYPE_REPLY, sequence, len);
}

/*----------------------------------------------------------------------------
 * Command handlers
 *
 * Each returns true if it recognised the command (and either succeeded or
 * intentionally rejected with an error reply). Unrecognised commands fall
 * through to the dispatcher which sends a syntax error.
 *---------------------------------------------------------------------------*/

/* Pan_tiltDrive: 81 01 06 01 VV WW YY ZZ FF
 *   VV = pan speed (0x01..0x18 = 1..24)
 *   WW = tilt speed (0x01..0x14 = 1..20)
 *   YY = pan direction (0x01=left, 0x02=right, 0x03=stop)
 *   ZZ = tilt direction (0x01=up, 0x02=down, 0x03=stop)
 * frame->body carries [VV WW YY ZZ] (4 bytes).
 *
 * axis_role picks which byte pair we consume — PAN uses VV/YY, TILT uses
 * WW/ZZ. Every other role has no Pan_tiltDrive mapping (VISCA has separate
 * CAM_Zoom/CAM_Focus/etc. commands for those) so we reject with
 * NOT_EXECUTABLE + log once per boot. */
static bool cmd_pantilt_drive(const visca_frame_t *frame,
                              const net_addr_t *peer, uint32_t sequence)
{
    if (frame->body_len < 4u) {
        send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
        return true;
    }
    uint8_t vv = frame->body[0];   /* pan speed  */
    uint8_t ww = frame->body[1];   /* tilt speed */
    uint8_t yy = frame->body[2];   /* pan direction  */
    uint8_t zz = frame->body[3];   /* tilt direction */

    /* Pick the (speed, direction, max_speed) triple based on axis_role. */
    uint8_t speed_byte;
    uint8_t dir_byte;
    uint8_t max_speed;
    if (s_axis_role == VISCA_ROLE_PAN) {
        speed_byte = vv;
        dir_byte   = yy;
        max_speed  = 24u;          /* VISCA pan range 0x01..0x18 */
    } else if (s_axis_role == VISCA_ROLE_TILT) {
        speed_byte = ww;
        dir_byte   = zz;
        max_speed  = 20u;          /* VISCA tilt range 0x01..0x14 */
    } else {
        /* Role has no VISCA Pan_tiltDrive mapping. Log once per boot to
         * make the mismatch visible; subsequent rejections stay silent. */
        static bool s_logged_role_mismatch = false;
        if (!s_logged_role_mismatch) {
            s_logged_role_mismatch = true;
            LOG_WARN("visca_mgr: Pan_tiltDrive received but axis_role=0x%02X isn't PAN/TILT — rejecting",
                     (unsigned)s_axis_role);
        }
        send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
        return true;
    }

    /* Direction 0x03 = Stop regardless of speed byte. Any other value:
     * 0x01 = decrement (left / down) = negative deflection;
     * 0x02 = increment (right / up)  = positive deflection.
     * Speed is clamped to 1..max_speed; 0 is invalid, values above the
     * per-axis maximum are treated as the maximum. */
    float deflection;
    if (dir_byte == 0x03u) {
        deflection = 0.0f;
    } else {
        if (speed_byte == 0u)         speed_byte = 1u;
        if (speed_byte > max_speed)   speed_byte = max_speed;
        float mag = (float)speed_byte / (float)max_speed;
        if (dir_byte == 0x01u)      deflection = -mag;   /* left / down */
        else if (dir_byte == 0x02u) deflection =  mag;   /* right / up */
        else {
            /* Unknown direction byte — reject rather than pick a side. */
            send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
            return true;
        }
    }
    cmc_state_handle_movement_scaled(deflection);
    send_ack_and_completion(peer, sequence);
    return true;
}

/* Pan_tiltHome: 81 01 06 04 FF
 *
 * Real Sony cameras interpret this as "return to (0,0) mechanical center."
 * For this CMC we map it to axis_manager_request_home() (run the endstop
 * homing procedure) because that's what a rig operator actually needs on
 * an incremental axis — and once homed, the mechanical zero IS position 0
 * so the semantics converge for anything downstream. If a real "go to 0"
 * is needed later, add a synthetic shot at position 0 and recall it via
 * Memory Recall.
 *
 * Rejects with NOT_EXECUTABLE if arbitration fails (something already in
 * flight — join / joystick / another home). */
static bool cmd_pantilt_home(const visca_frame_t *frame,
                             const net_addr_t *peer, uint32_t sequence)
{
    (void)frame;
    bool ok = axis_manager_request_home();
    if (!ok) {
        send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
        return true;
    }
    send_ack_and_completion(peer, sequence);
    return true;
}

/* Memory commands: 81 01 04 3F sub pp FF
 *   sub = 0x00 Reset (not implemented, syntax-error)
 *         0x01 Set     (store current pos to preset pp)
 *         0x02 Recall  (move to preset pp)
 * Preset pp is 0-indexed. cmc_state's shot table is 1-indexed; we add 1
 * and clamp pp to 0..99 to fit CMC_MAX_SHOTS = 100. */
static bool cmd_memory(const visca_frame_t *frame,
                       const net_addr_t *peer, uint32_t sequence)
{
    if (frame->body_len < 2u) {
        send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
        return true;
    }
    uint8_t sub = frame->body[0];
    uint8_t pp  = frame->body[1];
    if (pp >= 100u) {
        send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
        return true;
    }
    uint32_t shot_no = (uint32_t)pp + 1u;   /* 0-indexed preset → 1-indexed shot */

    if (sub == 0x01u) {
        /* Memory Set — store current position. cmc_state persists to
         * flash inside the store (60 ms blocking, matches CAMERAD). */
        int result = cmc_state_store_shot(shot_no);
        if (result < 0) {
            send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
            return true;
        }
        send_ack_and_completion(peer, sequence);
        return true;
    } else if (sub == 0x02u) {
        /* Memory Recall — fade using slot's stored time_to_shot_s. If the
         * slot is empty or the axis isn't homed, cmc_state_move_to_shot
         * rejects with false. */
        bool ok = cmc_state_move_to_shot(shot_no, /*is_cut=*/false);
        if (!ok) {
            send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
            return true;
        }
        send_ack_and_completion(peer, sequence);
        return true;
    } else if (sub == 0x00u) {
        /* Memory Reset (clear preset) — not implemented. */
        send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
        return true;
    }
    send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
    return true;
}

/* Sony vendor-extension block (`81 01 7E 04 xx ...`) — per Sony's
 * "VISCA Command List v4" (pro.sony/s3/2022/09/03065933/VISCA_Command_List_v4.pdf).
 *
 * frame->body carries the extended payload starting from the xx byte:
 *   body[0]         = extended sub-command
 *   body[1..N-1]    = sub-command-specific parameters
 *
 * Sub-commands we care about (all others fall through to syntax-error):
 *
 *   3D pp        Position-preset mode enable (pp=01 on, 00 off).
 *                Sony cameras restrict recall to PTZF-position-only when
 *                enabled. This CMC only has position anyway → no-op ACK.
 *
 *   1B pp        Separate-settings-per-preset mode enable.
 *                On real cameras: each preset can carry its own duration
 *                / speed / mode. On this CMC every slot ALWAYS has its
 *                own time_to_shot_s → no-op ACK.
 *
 *   27 pp mm     Set preset pp to speed-mode (mm=00) or duration-mode (mm=01).
 *                We use per-slot stored time for every recall regardless →
 *                no-op ACK. If mm is neither 00 nor 01, syntax-error.
 *
 *   67 pp 0q 0q 0q
 *                Assign duration to preset pp. The three nibble bytes
 *                pack a 12-bit unsigned value in 0.1 s units (Sony docs
 *                specify range 0x00A..0x3DE = 1.0..99.0 s). We accept
 *                the full 12-bit range 0..4095 (= 0..409.5 s) since a
 *                rig may want longer / shorter moves than a PTZ camera;
 *                a WARN log flags values outside the Sony window.
 *                → cmc_state_set_shot_time_tenths(pp+1, tenths).
 */
static bool cmd_sony_ext(const visca_frame_t *frame,
                         const net_addr_t *peer, uint32_t sequence)
{
    if (frame->body_len < 1u) {
        send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
        return true;
    }
    uint8_t sub = frame->body[0];

    if (sub == 0x3Du) {
        /* Position-preset mode. body: [3D pp]. Accept 00/01, ACK, no-op. */
        if (frame->body_len < 2u) { send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX); return true; }
        LOG_INFO("visca_mgr: Sony-ext position-preset mode = %u (no-op — CMC always position-only)",
                 (unsigned)frame->body[1]);
        send_ack_and_completion(peer, sequence);
        return true;
    }
    if (sub == 0x1Bu) {
        /* Separate-settings-per-preset. body: [1B pp]. Accept, ACK, no-op. */
        if (frame->body_len < 2u) { send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX); return true; }
        LOG_INFO("visca_mgr: Sony-ext per-preset-settings = %u (no-op — CMC always per-slot)",
                 (unsigned)frame->body[1]);
        send_ack_and_completion(peer, sequence);
        return true;
    }
    if (sub == 0x27u) {
        /* Preset speed/duration mode select. body: [27 pp mm].
         * mm=00 speed mode, mm=01 duration mode. We use per-slot time
         * always, so both are effectively duration mode → ACK either way. */
        if (frame->body_len < 3u) { send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX); return true; }
        uint8_t pp = frame->body[1];
        uint8_t mm = frame->body[2];
        if (mm != 0x00u && mm != 0x01u) {
            send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
            return true;
        }
        LOG_INFO("visca_mgr: Sony-ext preset %u mode=%s (no-op — CMC always uses slot time)",
                 (unsigned)pp, (mm == 0x01u) ? "duration" : "speed");
        send_ack_and_completion(peer, sequence);
        return true;
    }
    if (sub == 0x67u) {
        /* Assign duration to preset. body: [67 pp 0q 0q 0q]. */
        if (frame->body_len < 5u) { send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX); return true; }
        uint8_t pp = frame->body[1];
        if (pp >= 100u) {
            send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
            return true;
        }
        /* Reassemble 12-bit duration from three low-nibble bytes MSN first. */
        uint16_t tenths = (uint16_t)(((frame->body[2] & 0x0Fu) << 8)
                                   | ((frame->body[3] & 0x0Fu) << 4)
                                   |  (frame->body[4] & 0x0Fu));
        /* Sony spec range: 0x00A..0x3DE (1.0..99.0 s). Values outside
         * this window are technically out-of-spec — accept them anyway
         * for rig flexibility but log a WARN so the operator can see it. */
        if (tenths < 0x00Au || tenths > 0x3DEu) {
            LOG_WARN("visca_mgr: preset %u duration=%u tenths (%.1fs) outside Sony range 1.0..99.0s — accepting",
                     (unsigned)pp, (unsigned)tenths, (double)tenths * 0.1);
        }
        uint32_t shot_no = (uint32_t)pp + 1u;
        bool ok = cmc_state_set_shot_time_tenths(shot_no, (uint32_t)tenths);
        if (!ok) {
            /* Empty slot or out-of-range shot_no. Sony convention: NOT_EXECUTABLE. */
            send_error(peer, sequence, VISCA_TX_SOCKET, VISCA_ERR_NOT_EXECUTABLE);
            return true;
        }
        send_ack_and_completion(peer, sequence);
        return true;
    }

    /* Unknown Sony-ext sub-command. */
    LOG_INFO("visca_mgr: unknown Sony-ext sub=0x%02X", (unsigned)sub);
    send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
    return true;
}

/*----------------------------------------------------------------------------
 * Dispatch
 *---------------------------------------------------------------------------*/

static void dispatch_inquiry(const visca_frame_t *frame,
                             const net_addr_t *peer, uint32_t sequence)
{
    /* Inquiries are keyed on (category, command). */
    if (frame->category == 0x00u && frame->command == 0x02u) {
        (void)inq_version(peer, sequence);
        return;
    }
    if (frame->category == 0x06u && frame->command == 0x12u) {
        (void)inq_pantilt_pos(peer, sequence);
        return;
    }
    if (frame->category == 0x04u && frame->command == 0x47u) {
        (void)inq_zoom_pos(peer, sequence);
        return;
    }
    /* Unknown inquiry — Sony convention returns Syntax Error. */
    LOG_INFO("visca_mgr: unknown inquiry cat=0x%02X cmd=0x%02X",
             (unsigned)frame->category, (unsigned)frame->command);
    send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
}

static void dispatch_command(const visca_frame_t *frame,
                             const net_addr_t *peer, uint32_t sequence)
{
    /* Commands: key on (category, command) — some (Memory) further
     * discriminate on frame->body[0] (Set / Recall / Reset). */
    if (frame->category == 0x06u && frame->command == 0x01u) {
        (void)cmd_pantilt_drive(frame, peer, sequence);
        return;
    }
    if (frame->category == 0x06u && frame->command == 0x04u) {
        (void)cmd_pantilt_home(frame, peer, sequence);
        return;
    }
    if (frame->category == 0x04u && frame->command == 0x3Fu) {
        (void)cmd_memory(frame, peer, sequence);
        return;
    }
    /* Sony vendor-extension block (7E 04 xx). Preset-mode + per-preset
     * duration commands live here. */
    if (frame->category == 0x7Eu && frame->command == 0x04u) {
        (void)cmd_sony_ext(frame, peer, sequence);
        return;
    }
    /* Unknown command. */
    LOG_INFO("visca_mgr: unknown command cat=0x%02X cmd=0x%02X",
             (unsigned)frame->category, (unsigned)frame->command);
    send_error(peer, sequence, 0u, VISCA_ERR_SYNTAX);
}

/*----------------------------------------------------------------------------
 * Packet service (one call per visca_mgr_tick)
 *---------------------------------------------------------------------------*/

static void service_udp_socket(void)
{
    net_addr_t from;
    int32_t n = net_recvfrom(VISCA_UDP_SOCKET, &from, s_rx_buf, sizeof(s_rx_buf));
    if (n <= 0) return;

    /* Parse Sony IP header first. */
    if ((size_t)n < VISCA_IP_HEADER_SIZE) {
        LOG_INFO("visca_mgr: rx runt (%ld B) — dropping", (long)n);
        return;
    }
    visca_ip_header_t hdr;
    if (!visca_ip_parse_header(s_rx_buf, (size_t)n, &hdr)) {
        LOG_INFO("visca_mgr: rx header parse failed");
        return;
    }
    /* Sanity: declared payload length must fit in what we received. */
    size_t vpay_len = (size_t)n - VISCA_IP_HEADER_SIZE;
    if (hdr.payload_length > vpay_len) {
        LOG_INFO("visca_mgr: rx payload_length=%u exceeds available %u",
                 (unsigned)hdr.payload_length, (unsigned)vpay_len);
        send_error(&from, hdr.sequence_number, 0u, VISCA_ERR_SYNTAX);
        return;
    }
    const uint8_t *vpay = &s_rx_buf[VISCA_IP_HEADER_SIZE];

    /* Only VISCA command/inquiry payload types are dispatched to the
     * VISCA parser. Control-command type 0x0200 (sequence reset etc.)
     * gets a stub reply so a Sony-compliant client doesn't wedge waiting.
     * Everything else is dropped with a log line. */
    if (hdr.payload_type == VISCA_IP_TYPE_CTRL_CMD) {
        /* Reply with control-reply payload type, empty body, echoing seq. */
        visca_ip_build_header(s_tx_buf, VISCA_IP_TYPE_CTRL_REPLY,
                              0u, hdr.sequence_number);
        (void)net_sendto(VISCA_UDP_SOCKET, &from, s_tx_buf, VISCA_IP_HEADER_SIZE);
        return;
    }
    if (hdr.payload_type != VISCA_IP_TYPE_COMMAND
     && hdr.payload_type != VISCA_IP_TYPE_INQUIRY) {
        LOG_INFO("visca_mgr: rx payload_type=0x%04X ignored",
                 (unsigned)hdr.payload_type);
        return;
    }

    /* Parse the raw VISCA payload. */
    visca_frame_t frame;
    if (!visca_parse(vpay, hdr.payload_length, &frame)) {
        LOG_INFO("visca_mgr: rx VISCA parse failed (payload_len=%u)",
                 (unsigned)hdr.payload_length);
        send_error(&from, hdr.sequence_number, 0u, VISCA_ERR_SYNTAX);
        return;
    }

    /* Address filter: accept our address AND broadcast (8).
     * Silent drop for anyone else — real Sony cameras behave the same way. */
    if (frame.destination != s_our_address && frame.destination != 8u) {
        return;
    }

    if (frame.cls == VISCA_CLASS_INQUIRY) {
        dispatch_inquiry(&frame, &from, hdr.sequence_number);
    } else if (frame.cls == VISCA_CLASS_COMMAND) {
        dispatch_command(&frame, &from, hdr.sequence_number);
    } else {
        /* Cancel or something odd — Sony returns a Cancel-Ack on the socket.
         * We just log + syntax-error. */
        LOG_INFO("visca_mgr: rx unhandled class=0x%02X", (unsigned)frame.cls);
        send_error(&from, hdr.sequence_number, 0u, VISCA_ERR_SYNTAX);
    }
}

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

void visca_mgr_init(void)
{
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    s_send_fail_log_count = 0;

    /* Derive VISCA device address from the shared cmc_device_no config
     * field (also used by CAMERAD as return_device_no). VISCA is defined
     * for addresses 1..7 only; clamp anything outside that range to 1
     * with a WARN so the boot trace makes the discrepancy obvious. */
    uint32_t configured = config_get_network()->cmc_device_no;
    if (configured >= 1u && configured <= 7u) {
        s_our_address = (uint8_t)configured;
    } else {
        s_our_address = 1u;
        LOG_WARN("visca_mgr: cmc_device_no=%lu is outside VISCA range 1..7 — using address 1",
                 (unsigned long)configured);
    }

    /* Snapshot axis_role. Only PAN and TILT have a matching VISCA drive
     * command shape (Pan_tiltDrive with VV+YY vs WW+ZZ). ZOOM currently
     * only reads back position via CAM_ZoomPosInq; other roles have no
     * VISCA command implemented yet. Requests that don't match get an
     * INFO log + NOT_EXECUTABLE reply (see cmd_pantilt_drive). */
    s_axis_role = axis_manager_get_axis_role();
    const char *role_name = "?";
    switch (s_axis_role) {
    case VISCA_ROLE_PAN:    role_name = "PAN";    break;
    case VISCA_ROLE_TILT:   role_name = "TILT";   break;
    case VISCA_ROLE_ZOOM:   role_name = "ZOOM";   break;
    case VISCA_ROLE_FOCUS:  role_name = "FOCUS";  break;
    case VISCA_ROLE_X:      role_name = "X";      break;
    case VISCA_ROLE_Y:      role_name = "Y";      break;
    case VISCA_ROLE_HEIGHT: role_name = "HEIGHT"; break;
    case VISCA_ROLE_FADER:  role_name = "FADER";  break;
    default: break;
    }
    if (s_axis_role != VISCA_ROLE_PAN && s_axis_role != VISCA_ROLE_TILT) {
        LOG_WARN("visca_mgr: axis_role=0x%02X (%s) has no VISCA Pan_tiltDrive mapping — "
                 "drive commands will be rejected until CAM_Zoom / CAM_Focus / etc. "
                 "are implemented", (unsigned)s_axis_role, role_name);
    }

    if (!net_open(VISCA_UDP_SOCKET, NET_PROTO_UDP, VISCA_UDP_PORT, false)) {
        LOG_ERROR("visca_mgr: net_open(UDP :%u) failed — VISCA offline",
                  (unsigned)VISCA_UDP_PORT);
        return;
    }
    LOG_INFO("visca_mgr: ready (VISCA-over-IP UDP :%u, addr=%u, vendor=0x%04X model=0x%04X rom=0x%04X)",
             (unsigned)VISCA_UDP_PORT, (unsigned)s_our_address,
             (unsigned)VISCA_VENDOR_ID, (unsigned)VISCA_MODEL_ID,
             (unsigned)VISCA_ROM_VERSION);
}

void visca_mgr_tick(void)
{
    /* One packet per tick keeps the drain rate proportional to the loop
     * rate without introducing internal backlog. If VISCA traffic is
     * bursty and floods the socket buffer, W6100 drops from its side. */
    service_udp_socket();
}
