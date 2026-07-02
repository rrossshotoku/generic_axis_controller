/*
 * app/controller_mgr — Path A (final): structure copied from the Reduced
 * CMC (uc_camd_interface).
 *
 * Three sockets active:
 *   - UDP listen socket on udp_poll_port (30002) — receives POLL discovery
 *     broadcasts from controllers. (app_init.c:97-103)
 *   - TCP listen socket on tcp_camerad_port (30001 — SW050 LISTENPORT1) —
 *     accepts inbound TCP from the controller. The panel uses this for
 *     command messages (SELECT / KEYPRESS / MOVEMENT). (app_init.c:106-112)
 *   - One outbound TCP socket per controller — opened on first POLL,
 *     connects from CMC to controller's return_port (typically 30001 on
 *     the panel side). All responses (POLL response + others) flow over
 *     this socket. (command_handler.c::ensure_controller_connection)
 *
 * Sequence (mirroring command_handler.c::cmd_handle_poll line 110-154):
 *   1. POLL UDP arrives.
 *   2. Extract controller IP + return_port from POLL header.
 *   3. ensure_controller_connection:
 *      - If no outbound socket yet, open one (TCP, ephemeral local port).
 *      - Issue connect to controller's return_port.
 *      - Wait for ESTABLISHED (poll-based, non-blocking on our side).
 *   4. Once connected, send POLL response over the outbound TCP.
 *   5. (Inbound TCP listener stays up so panel can push commands —
 *      Phase B will dispatch those.)
 */

#include "controller_mgr.h"

#include "app/axis_manager/axis_manager.h"   /* JOY_PROFILE keypresses -> axis_manager_set_joy_profile */
#include "app/camerad/camerad.h"
#include "app/cmc_state/cmc_state.h"
#include "app/config/config.h"
#include "app/log/log.h"
#include "bsp/net/net.h"
#include "bsp/time/time.h"

#include <string.h>

/* Socket allocation per architecture.md §10.1:
 *   0 = CAMERAD POLL UDP listener
 *   1 = CAMERAD TCP inbound listener (panel-initiated TCP for commands)
 *   2 = Controller A outbound TCP (CMC-initiated, for responses)
 *
 *
 * Two-panel (2026-06-26): slot 3 is now CMC's listen B for panel B
 * inbound TCP. Outbound TCP stays single slot (2), shared between panels
 * via close-after-send round-robin. Each panel has its own listen slot
 * + its own rx buffer; POLLs route by source IP to the matching slot. */
#define POLL_SOCKET         ((net_sock_t)0)
#define TCP_LISTEN_A_SOCKET ((net_sock_t)1)
#define CTRL_OUT_TCP_SOCKET ((net_sock_t)2)
#define TCP_LISTEN_B_SOCKET ((net_sock_t)3)
/* Back-compat alias: old code referred to the listen socket as
 * TCP_LISTEN_SOCKET. Keep the name in case downstream tools depend on it,
 * but new code should use the _A / _B forms explicitly. */
#define TCP_LISTEN_SOCKET   TCP_LISTEN_A_SOCKET

#define RX_BUF_SIZE         CAMERAD_MAX_FRAME_SIZE

/* TCP listener has its own larger rx buffer + persistent-state byte count
 * so it can implement a proper streaming parser (TCP doesn't guarantee
 * frames arrive whole — bytes can split arbitrarily across reads). 1 KB
 * fits ~9 maximally-sized CAMERAD frames, plenty of slack for any
 * realistic panel TX rate. */
#define TCP_RX_BUF_SIZE     1024u
#define TX_BUF_SIZE         128u
#define CONNECT_TIMEOUT_MS  2000u
/* Ephemeral local port for the (shared) outbound TCP slot — predictable
 * per slot (matches Reduced CMC pal_net.c:337 `ephemeral_port = 10000 + sock`). */
#define OUTBOUND_LOCAL_PORT (10000u + (uint16_t)CTRL_OUT_TCP_SOCKET)

#define CMC_CONTROLLERS     2u   /* W6100 socket budget allows exactly this many */

/*----------------------------------------------------------------------------
 * Per-controller record (Path A: one slot)
 *---------------------------------------------------------------------------*/

typedef enum {
    CONN_IDLE = 0,        /* outbound socket closed; nothing pending */
    CONN_CONNECTING,      /* connect() issued; waiting for ESTABLISHED */
    CONN_ESTABLISHED,
} conn_state_t;

typedef struct {
    /* Slot metadata — established at init from config (panel_a_/panel_b_ ip+port).
     * `enabled` = listener actually opened (operator filled in expected IP). */
    bool             enabled;
    uint8_t          expected_ip[4];          /* matches POLL source against this; 0.0.0.0 = disabled */
    uint16_t         tcp_listen_port;         /* our listen port for this panel */
    net_sock_t       listen_sock;             /* slot 1 (panel A) or slot 3 (panel B) */
    uint8_t          tcp_rx_buf[TCP_RX_BUF_SIZE];
    size_t           tcp_rx_pending;
    bool             listener_up_prev;        /* edge-detect for log */

    /* Discovered panel — populated on first POLL whose source IP matches. */
    bool             in_use;
    uint32_t         device_no;
    uint32_t         device_type;
    char             ip_str[16];
    uint16_t         return_port;             /* panel's listen port (from POLL header), for CMC's outbound TCP */

    /* Shared-outbound state. Multiple controllers share CTRL_OUT_TCP_SOCKET
     * — at most one is the current owner. `conn` tracks the owner's view. */
    conn_state_t     conn;
    uint32_t         connect_start_ms;

    uint32_t         last_poll_ms;            /* heartbeat for timeout */
    bool             pending_poll_response;
    camerad_header_t pending_poll_request;
} controller_t;

/* If a controller stops POLLing for this long, the CMC considers it gone:
 * outbound TCP closed, slot dropped, and if it owned the camera the
 * selection is force-cleared. Matches Reduced CMC's PROTOCOL_TIMEOUT_MS.
 * Panels normally POLL at ~1 Hz; 5 missed POLLs is unambiguously "gone". */
#define CONTROLLER_TIMEOUT_MS  5000u

/* Per-controller state. Index 0 = panel A (slot 1 listen), index 1 = panel B
 * (slot 3 listen). The rx buffer + tcp_rx_pending live INSIDE controller_t
 * now (previously file-scope statics), since the streaming parser needs
 * per-slot state. */
static controller_t s_ctrls[CMC_CONTROLLERS];

/* Outbound TCP slot 2 is shared. s_outbound_owner_idx is the s_ctrls[]
 * index currently using it (-1 = idle). Multiplexing: when the owner
 * finishes its POLL response, we close the slot, release ownership, and
 * the next service_outbound_tcp pass may pick the other panel. */
#define OUTBOUND_OWNER_NONE  (-1)
static int                      s_outbound_owner_idx = OUTBOUND_OWNER_NONE;

static bool                     s_inited = false;
static controller_mgr_stats_t   s_stats;
static uint8_t                  s_rx_buf[RX_BUF_SIZE];
static uint8_t                  s_tx_buf[TX_BUF_SIZE];

/* Back-compat — old code referenced s_ctrl_a as a singular field.
 * A few diagnostic paths still grep for it; map to slot 0 explicitly. */
#define s_ctrl_a  (s_ctrls[0])

/*----------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/

static void format_our_ip(char out[16])
{
    const network_cfg_t *cfg = config_get_network();
    camerad_format_ip(cfg->ip, out);
}

/*----------------------------------------------------------------------------
 * Response builders.
 *
 * Reused for POLL response, SELECT response, GRAB response — all three are
 * the same body shape (S 22 B / T 14 B) per SW050's eCMCMessageLengths;
 * only the response opcode (echoed in the header's msg_command) differs.
 * Body fields come from app/cmc_state so the panel sees real selection
 * state across responses.
 *
 * DESELECT response is a different shape (5 B body) — separate builder.
 *
 * Advertise tcp_camerad_port as return_port (matches Reduced CMC). The
 * outbound TCP we initiate (CMC → panel's return_port) carries the response.
 *---------------------------------------------------------------------------*/

/* Build a POLL-shaped response body (S 22 B / T 14 B). Returns total frame
 * size written into out; 0 if the device type isn't a recognised
 * controller. msg_command is the opcode to echo (POLL, SELECT, GRAB). */
static uint16_t build_pollshape_response(const camerad_header_t *req,
                                         uint32_t msg_command,
                                         uint8_t *out)
{
    char our_ip[16];
    format_our_ip(our_ip);
    const network_cfg_t *cfg = config_get_network();

    /* Body fields shared between S and T. All come from cmc_state so the
     * panel sees a coherent live picture: who selected the camera, which
     * shot is current/next, whether the motor is moving, etc. */
    uint8_t  camera_selected = cmc_state_is_selected() ? 1u : 0u;
    int32_t  controller_no   = (int32_t)cmc_state_selected_by();
    uint32_t current_shot    = cmc_state_current_shot();
    uint32_t next_shot       = cmc_state_next_shot();
    uint32_t ttshot_tenths   = cmc_state_time_to_shot_tenths();

    /* Two separate status fields per SW050 (CameraToolsU.h):
     *
     * `camera_status` (eCamStatus) — per-camera/motion state. cmsMoving=0x01,
     * cmsOnShot=0x02 are the bits panels render as "moving"/"on shot"
     * indicators. The Reduced CMC's cmc_state.c:228 confusingly wrote
     * these into the cmc_status field at 0x02/0x04 — that's NOT what
     * panels actually read; SW050 panels look in camera_status.
     *
     * `cmc_status` (eCMCStatus) — CMC-mode state. csRemote=0x00 (we are
     * a network-only CMC, never local), csConnectionOK=0x20 set when a
     * controller is selected. */
    uint16_t camera_status = 0;
    if (cmc_state_moving())  camera_status |= CAMERAD_CAM_MOVING;
    if (cmc_state_on_shot()) camera_status |= CAMERAD_CAM_ON_SHOT;

    uint16_t cmc_status = 0;
    if (cmc_state_is_selected()) cmc_status |= CAMERAD_CMC_CONNECTION_OK;

    /* move_type the panel renders as the action icon. CUT shows "instant",
     * FADE shows "fading", NONE = idle. We don't track CUT-vs-FADE
     * separately yet (cmc_state could expose this later); for now
     * report FADING during any active move, NONE when idle. Good enough
     * for the UI to flip the indicator. */
    uint8_t move_type = cmc_state_moving() ? (uint8_t)CAMERAD_MOVE_FADE
                                           : (uint8_t)CAMERAD_MOVE_NONE;

    if (camerad_dev_is_s_type(req->return_device)) {
        camerad_build_response_header(
            out, req, msg_command,
            (uint16_t)sizeof(camerad_poll_resp_s_t),
            our_ip, cfg->tcp_camerad_port, cfg->cmc_device_no);

        camerad_poll_resp_s_t body;
        memset(&body, 0, sizeof(body));
        body.camera_selected = camera_selected;
        body.controller_no   = controller_no;
        body.camera_status   = camera_status;
        body.cmc_status      = cmc_status;
        body.time_to_shot    = (int32_t)ttshot_tenths;
        body.shot_no         = (int32_t)current_shot;
        body.next_shot_no    = (int32_t)next_shot;
        body.move_type       = move_type;
        memcpy(out + CAMERAD_HEADER_SIZE, &body, sizeof(body));
        return (uint16_t)(CAMERAD_HEADER_SIZE + sizeof(body));

    } else if (camerad_dev_is_t_type(req->return_device)) {
        camerad_build_response_header(
            out, req, msg_command,
            (uint16_t)sizeof(camerad_poll_resp_t_t),
            our_ip, cfg->tcp_camerad_port, cfg->cmc_device_no);

        camerad_poll_resp_t_t body;
        memset(&body, 0, sizeof(body));
        body.camera_selected = camera_selected;
        body.controller_no   = controller_no;
        body.camera_status   = camera_status;
        body.cmc_status      = cmc_status;
        body.time_to_shot    = (int32_t)ttshot_tenths;
        body.move_type       = move_type;
        memcpy(out + CAMERAD_HEADER_SIZE, &body, sizeof(body));
        return (uint16_t)(CAMERAD_HEADER_SIZE + sizeof(body));

    } else {
        s_stats.poll_rejected_dev++;
        return 0;
    }
}

/* DESELECT response — 5-byte body: camera_selected, controller_no.
 * Sent whether the deselect was granted or denied. If granted,
 * camera_selected=false; if denied (someone else owns), camera_selected=true
 * and controller_no = whoever currently owns. (Mirrors Reduced CMC
 * command_handler.c::cmd_send_deselect_response.) */
static uint16_t build_deselect_response(const camerad_header_t *req,
                                        uint8_t *out)
{
    char our_ip[16];
    format_our_ip(our_ip);
    const network_cfg_t *cfg = config_get_network();

    camerad_build_response_header(
        out, req, (uint32_t)CAMERAD_MSG_DESELECT,
        (uint16_t)sizeof(camerad_deselect_resp_t),
        our_ip, cfg->tcp_camerad_port, cfg->cmc_device_no);

    camerad_deselect_resp_t body;
    body.camera_selected = cmc_state_is_selected() ? 1u : 0u;
    body.controller_no   = (int32_t)cmc_state_selected_by();
    memcpy(out + CAMERAD_HEADER_SIZE, &body, sizeof(body));
    return (uint16_t)(CAMERAD_HEADER_SIZE + sizeof(body));
}

/* Backwards-compat wrapper for the outbound TCP path that always sends
 * a POLL-shaped response with command=POLL. */
static uint16_t build_poll_response(const camerad_header_t *req)
{
    if (camerad_dev_is_s_type(req->return_device)) s_stats.poll_responded_s++;
    else if (camerad_dev_is_t_type(req->return_device)) s_stats.poll_responded_t++;
    return build_pollshape_response(req, (uint32_t)CAMERAD_MSG_POLL, s_tx_buf);
}

/*----------------------------------------------------------------------------
 * Outbound TCP connection per controller
 *
 * Mirrors command_handler.c::ensure_controller_connection (line 49-87).
 *---------------------------------------------------------------------------*/

static void start_outbound_tcp(controller_t *c)
{
    /* Always close-and-reopen — TCP socket must be in CLOSED state. */
    net_close(CTRL_OUT_TCP_SOCKET);

    if (!net_open(CTRL_OUT_TCP_SOCKET, NET_PROTO_TCP, OUTBOUND_LOCAL_PORT, false)) {
        LOG_WARN("ctrl_mgr: net_open(TCP outbound) failed");
        c->conn = CONN_IDLE;
        return;
    }

    net_addr_t peer;
    memset(&peer, 0, sizeof(peer));
    if (!camerad_parse_ip(c->ip_str, peer.addr)) {
        LOG_WARN("ctrl_mgr: unparseable controller IP '%s'", c->ip_str);
        net_close(CTRL_OUT_TCP_SOCKET);
        c->conn = CONN_IDLE;
        return;
    }
    peer.port = c->return_port;

    if (!net_tcp_connect(CTRL_OUT_TCP_SOCKET, &peer)) {
        LOG_WARN("ctrl_mgr: net_tcp_connect %s:%u failed",
                 c->ip_str, (unsigned)c->return_port);
        net_close(CTRL_OUT_TCP_SOCKET);
        c->conn = CONN_IDLE;
        return;
    }

    c->conn             = CONN_CONNECTING;
    c->connect_start_ms = time_ms();
    s_stats.tcp_outbound_connects++;
    LOG_INFO("ctrl_mgr: TCP outbound connect started -> %s:%u (dev=%lu type=%lu)",
             c->ip_str, (unsigned)c->return_port,
             (unsigned long)c->device_no, (unsigned long)c->device_type);
}

/* Release the shared outbound slot back to the idle pool. Closes the W6100
 * socket and clears the per-controller conn state for the current owner. */
static void release_outbound(void)
{
    net_close(CTRL_OUT_TCP_SOCKET);
    if (s_outbound_owner_idx != OUTBOUND_OWNER_NONE) {
        s_ctrls[s_outbound_owner_idx].conn = CONN_IDLE;
    }
    s_outbound_owner_idx = OUTBOUND_OWNER_NONE;
}

/* Service the single shared outbound TCP slot. Round-robin: at most one
 * controller owns CTRL_OUT_TCP_SOCKET at any time. When the owner finishes
 * its POLL response we close + release so the other panel can acquire it
 * on the next tick. Idle while nobody has pending responses. */
static void service_outbound_tcp(void)
{
    /* Acquire phase — if idle and someone needs a push, open for them. */
    if (s_outbound_owner_idx == OUTBOUND_OWNER_NONE) {
        for (uint32_t k = 0; k < CMC_CONTROLLERS; ++k) {
            controller_t *c = &s_ctrls[k];
            if (c->in_use && c->pending_poll_response) {
                s_outbound_owner_idx = (int)k;
                start_outbound_tcp(c);
                break;
            }
        }
        if (s_outbound_owner_idx == OUTBOUND_OWNER_NONE) {
            return;  /* nothing to do */
        }
    }

    /* Service phase — drive the owner's state machine. */
    controller_t *c = &s_ctrls[s_outbound_owner_idx];
    net_tcp_state_t st = net_tcp_state(CTRL_OUT_TCP_SOCKET);

    if (c->conn == CONN_CONNECTING) {
        if (st == NET_TCP_ESTABLISHED) {
            c->conn = CONN_ESTABLISHED;
            LOG_INFO("ctrl_mgr: TCP outbound established -> %s:%u",
                     c->ip_str, (unsigned)c->return_port);
        } else if (time_elapsed_ms(c->connect_start_ms) > CONNECT_TIMEOUT_MS) {
            LOG_WARN("ctrl_mgr: TCP outbound connect timeout -> %s:%u",
                     c->ip_str, (unsigned)c->return_port);
            s_stats.tcp_outbound_failures++;
            release_outbound();
            return;
        }
    }

    if (c->conn == CONN_ESTABLISHED && c->pending_poll_response) {
        uint16_t len = build_poll_response(&c->pending_poll_request);
        if (len > 0) {
            int32_t sent = net_send(CTRL_OUT_TCP_SOCKET, s_tx_buf, len);
            if (sent < 0 || (uint32_t)sent != len) {
                LOG_WARN("ctrl_mgr: outbound TCP send failed (rc=%ld)", (long)sent);
                s_stats.poll_send_errors++;
                release_outbound();
                return;
            }
            /* POLL response sent OK — bump per-opcode counter so it mirrors
             * the other handlers. */
            s_stats.tx_ok[CAMERAD_MSG_POLL]++;
        }
        c->pending_poll_response = false;
        /* Close immediately so the OTHER controller can acquire the slot on
         * the next tick. Trades a TCP handshake per POLL response for fair
         * two-panel multiplexing on a single shared outbound slot. */
        release_outbound();
        return;
    }

    if ((c->conn == CONN_ESTABLISHED) &&
        (st == NET_TCP_CLOSED || st == NET_TCP_CLOSE_WAIT)) {
        LOG_INFO("ctrl_mgr: TCP outbound closed by peer -> %s:%u",
                 c->ip_str, (unsigned)c->return_port);
        release_outbound();
    }
}

/*----------------------------------------------------------------------------
 * Inbound TCP command dispatch
 *
 * The panel sends SELECT/DESELECT/GRAB (and later KEYPRESS/MOVEMENT/etc.)
 * over the inbound TCP. Responses go back over the outbound TCP that
 * controller_mgr opens to the panel — that's how the Reduced CMC works
 * (cmd_send_poll_response uses controller->send_socket, the outbound).
 *
 * For Path A we only handle SELECT/DESELECT/GRAB — enough to keep the
 * panel UI green. Other commands are silently dropped until Phase B.
 *---------------------------------------------------------------------------*/

/* Set by service_tcp_listener_slot before dispatching a command, so that
 * the response handlers (handle_select / handle_grab / etc.) know which
 * inbound TCP socket to write the response back down. TCP is bidirectional
 * — the panel's command arrived ESTABLISHED on this slot, and our reply
 * goes back down the same slot. NO_SOCK = not currently dispatching
 * (e.g. POLL responses use the shared outbound, not this path). */
#define DISPATCH_NO_SOCK  ((net_sock_t)0xFF)
static net_sock_t s_dispatch_inbound_sock = DISPATCH_NO_SOCK;

/* Send a response (already in s_tx_buf) back down the inbound TCP socket
 * the triggering command arrived on. Previously this used the per-panel
 * outbound TCP slot, but the slot is now shared and transient (close-
 * after-POLL-response) — sending responses down the inbound is the
 * natural CAMERAD pattern and avoids the multiplexing problem entirely.
 * Drops the response if there's no active dispatch context (the caller
 * should only invoke this from inside handle_command -> handle_*). */
static void send_response_via_outbound(uint16_t len, uint32_t opcode)
{
    if (s_dispatch_inbound_sock == DISPATCH_NO_SOCK) return;
    int32_t sent = net_send(s_dispatch_inbound_sock, s_tx_buf, len);
    if (sent < 0 || (uint32_t)sent != len) {
        LOG_WARN("ctrl_mgr: response TCP send failed (rc=%ld)", (long)sent);
        s_stats.poll_send_errors++;
    } else if (opcode < CTRL_MGR_OPCODES) {
        s_stats.tx_ok[opcode]++;
    }
}

static void handle_select(const camerad_header_t *req)
{
    uint32_t requester = req->return_device_no;
    bool granted = cmc_state_handle_select(requester);
    uint16_t len;
    uint32_t resp_op;
    if (granted) {
        /* SELECT granted — respond with POLL-shaped body, command echoed
         * as SELECT, camera_selected=true, controller_no=requester. */
        len = build_pollshape_response(req, (uint32_t)CAMERAD_MSG_SELECT, s_tx_buf);
        resp_op = (uint32_t)CAMERAD_MSG_SELECT;
    } else {
        /* SELECT denied — respond with DESELECT-shaped body carrying the
         * current owner's controller_no (mirrors Reduced CMC pattern). */
        len = build_deselect_response(req, s_tx_buf);
        resp_op = (uint32_t)CAMERAD_MSG_DESELECT;
    }
    if (len > 0) send_response_via_outbound(len, resp_op);
}

static void handle_deselect(const camerad_header_t *req)
{
    (void)cmc_state_handle_deselect(req->return_device_no);
    uint16_t len = build_deselect_response(req, s_tx_buf);
    if (len > 0) send_response_via_outbound(len, (uint32_t)CAMERAD_MSG_DESELECT);
}

static void handle_grab(const camerad_header_t *req)
{
    cmc_state_handle_grab(req->return_device_no);
    /* GRAB always grants — respond with POLL-shaped body, command=GRAB. */
    uint16_t len = build_pollshape_response(req, (uint32_t)CAMERAD_MSG_GRAB, s_tx_buf);
    if (len > 0) send_response_via_outbound(len, (uint32_t)CAMERAD_MSG_GRAB);
}

/*----------------------------------------------------------------------------
 * KEYPRESS dispatch
 *
 * KEYPRESS_T1 body = 5 bytes (camerad_keypress_t1_t): {u8 key_code, i32 value}.
 *   For shot keys, `value` is the 1-based shot_no.
 *   For STORE_TIME_TO_SHOT, `value` is the fade time in tenths of a second.
 *
 * KEYPRESS_T2 body = 2 bytes (camerad_keypress_t2_t): {u8 key_code, u8 status}.
 *   Used for STOP / STOP_ALL and similar zero-argument actions.
 *
 * After dispatching the action to cmc_state, we send back a POLL-shaped
 * response carrying the live state — that's what the Reduced CMC does for
 * KEYPRESS responses (see uc_camd_interface command_handler.c::
 * cmd_send_keypress1_response). The panel uses it to update its UI:
 *   - camera_selected / controller_no — unchanged (operator owns it)
 *   - cmc_status bits — moving/on_shot flip as the move progresses
 *   - current_shot / next_shot / time_to_shot — reflect the just-issued
 *     action immediately, so the panel can display "going to shot 5".
 *---------------------------------------------------------------------------*/

static void send_keypress_response(const camerad_header_t *req, uint32_t echoed_cmd)
{
    uint16_t len = build_pollshape_response(req, echoed_cmd, s_tx_buf);
    if (len > 0) send_response_via_outbound(len, echoed_cmd);
}

/* Human-readable CAMERAD key-code name for the dispatch log. Only the
 * codes we actually handle are listed; everything else shows as "?". */
static const char *camerad_kc_name(uint8_t kc)
{
    switch ((camerad_key_code_t)kc) {
    case CAMERAD_KC_STORE_SHOT:         return "STORE_SHOT";
    case CAMERAD_KC_STORE_NEXT:         return "STORE_NEXT";
    case CAMERAD_KC_SWOOP:              return "SWOOP";
    case CAMERAD_KC_CUT:                return "CUT";
    case CAMERAD_KC_FADE:               return "FADE";
    case CAMERAD_KC_FADE_CUE:           return "FADE_CUE";
    case CAMERAD_KC_CUT_CUE:            return "CUT_CUE";
    case CAMERAD_KC_PRELOAD:            return "PRELOAD";
    case CAMERAD_KC_SWOOP_TO:           return "SWOOP_TO";
    case CAMERAD_KC_CUT_TO:             return "CUT_TO";
    case CAMERAD_KC_FADE_TO:            return "FADE_TO";
    case CAMERAD_KC_JOY_PROFILE_NORMAL: return "JOY_PROFILE_NORMAL";
    case CAMERAD_KC_JOY_PROFILE_MEDIUM: return "JOY_PROFILE_MEDIUM";
    case CAMERAD_KC_JOY_PROFILE_FINE:   return "JOY_PROFILE_FINE";
    case CAMERAD_KC_STORE_TIME_TO_SHOT: return "STORE_TIME_TO_SHOT";
    case CAMERAD_KC_STOP:               return "STOP";
    case CAMERAD_KC_STOP_ALL:           return "STOP_ALL";
    default:                            return "?";
    }
}

static void handle_keypress_t1(const camerad_header_t *req,
                               const uint8_t *body, size_t body_len)
{
    if (body_len < sizeof(camerad_keypress_t1_t)) {
        LOG_WARN("ctrl_mgr: KEYPRESS_T1 body too short (%u)", (unsigned)body_len);
        return;
    }
    camerad_keypress_t1_t kp;
    /* Body is packed LE so a direct memcpy is correct on Cortex-M. */
    memcpy(&kp, body, sizeof(kp));

    LOG_INFO("ctrl_mgr: KEYPRESS_T1 key=%s (0x%02X) value=%ld",
             camerad_kc_name(kp.key_code), (unsigned)kp.key_code, (long)kp.value);

    switch ((camerad_key_code_t)kp.key_code) {
    case CAMERAD_KC_STORE_SHOT:
        (void)cmc_state_store_shot((uint32_t)kp.value);
        break;
    case CAMERAD_KC_STORE_NEXT:
        (void)cmc_state_store_next();
        break;

    case CAMERAD_KC_CUT:
    case CAMERAD_KC_CUT_TO:
    case CAMERAD_KC_CUT_CUE:
        (void)cmc_state_move_to_shot((uint32_t)kp.value, /*is_cut*/true);
        break;

    /* FADE and SWOOP share the same dispatch: a timed move to the shot's
     * stored position, using the operator-locked time_to_shot (or the
     * shot's stored time if none locked). Per the v1 decision the SWOOP
     * "custom curve" profile is NOT implemented — the panel still sends
     * the SWOOP key code, we just execute a plain FADE. The panel sees
     * a fading-indicator either way (POLL response reports
     * CAMERAD_MOVE_FADE for any active move). */
    case CAMERAD_KC_FADE:
    case CAMERAD_KC_FADE_TO:
    case CAMERAD_KC_FADE_CUE:
    case CAMERAD_KC_SWOOP:
    case CAMERAD_KC_SWOOP_TO:
        (void)cmc_state_move_to_shot((uint32_t)kp.value, /*is_cut*/false);
        break;

    case CAMERAD_KC_STORE_TIME_TO_SHOT:
        cmc_state_set_time_to_shot_tenths((uint32_t)kp.value);
        break;

    case CAMERAD_KC_STOP:
        cmc_state_stop_movement();
        break;

    case CAMERAD_KC_PRELOAD:
        /* T-screen "preload" — operator selected a shot in the UI but
         * hasn't pressed move yet. Reduced CMC just acks; we do the same
         * (no state change, response carries current state). */
        break;

    case CAMERAD_KC_JOY_PROFILE_NORMAL:
    case CAMERAD_KC_JOY_PROFILE_MEDIUM:
    case CAMERAD_KC_JOY_PROFILE_FINE: {
        /* Map CAMERAD's three-way selector onto axis_manager's joy_profile.
         * axis_manager scales joystick_max_velocity = velocity_limit ×
         * (1.0 | 0.5 | 0.15) so the operator's full-stick output shrinks
         * with the profile — CAMERAD's fine/medium/normal without the
         * motor MCU needing to know anything about profiles. */
        uint8_t p =
            (kp.key_code == CAMERAD_KC_JOY_PROFILE_FINE)   ? 2u :
            (kp.key_code == CAMERAD_KC_JOY_PROFILE_MEDIUM) ? 1u : 0u;
        (void)axis_manager_set_joy_profile(p);
        break;
    }

    default:
        LOG_INFO("ctrl_mgr: KEYPRESS_T1 unhandled key 0x%02X (ignored, will still ack)",
                 (unsigned)kp.key_code);
        break;
    }

    send_keypress_response(req, (uint32_t)CAMERAD_MSG_KEYPRESS_T1);
}

static void handle_keypress_t2(const camerad_header_t *req,
                               const uint8_t *body, size_t body_len)
{
    if (body_len < sizeof(camerad_keypress_t2_t)) {
        LOG_WARN("ctrl_mgr: KEYPRESS_T2 body too short (%u)", (unsigned)body_len);
        return;
    }
    camerad_keypress_t2_t kp;
    memcpy(&kp, body, sizeof(kp));

    LOG_INFO("ctrl_mgr: KEYPRESS_T2 key=0x%02X status=%u",
             (unsigned)kp.key_code, (unsigned)kp.status);

    switch ((camerad_key_code_t)kp.key_code) {
    case CAMERAD_KC_STOP:
    case CAMERAD_KC_STOP_ALL:
        cmc_state_stop_movement();
        break;
    default:
        /* Limits / toggles / other KP2 keys not handled in v1. */
        break;
    }
    send_keypress_response(req, (uint32_t)CAMERAD_MSG_KEYPRESS_T2);
}

/* MOVEMENT body (9 bytes, camerad_movement_t): {u8 axis_bitmap, i8 pan,
 * i8 tilt, i8 zoom, i8 focus, i8 x, i8 y, i8 height, i8 fader}. Sent
 * by the controller at its joystick rate (~25 ms while the stick is
 * deflected, plus a final zero on release). No response expected
 * (camerad.h:264 — "No ACK from CMC"). */
static void handle_movement(const camerad_header_t *req,
                            const uint8_t *body, size_t body_len)
{
    (void)req;
    if (body_len < sizeof(camerad_movement_t)) {
        LOG_WARN("ctrl_mgr: MOVEMENT body too short (%u)", (unsigned)body_len);
        return;
    }
    camerad_movement_t mv;
    memcpy(&mv, body, sizeof(mv));

    /* Single-motor CMC: only the pan axis is wired. axis_bitmap tells us
     * which fields the panel actually populated; we still accept pan=0
     * frames (operator released the stick) to drive the motor to a stop
     * — that's distinct from "no MOVEMENT at all" (handled by the
     * cmc_state watchdog). */
    cmc_state_handle_movement(mv.pan);
}

/* Human-readable CAMERAD opcode name for the dispatch log. Only the
 * opcodes we actually receive are listed; unknowns show as "?". */
static const char *camerad_msg_name(uint32_t op)
{
    switch ((camerad_msg_t)op) {
    case CAMERAD_MSG_POLL:          return "POLL";
    case CAMERAD_MSG_SELECT:        return "SELECT";
    case CAMERAD_MSG_DESELECT:      return "DESELECT";
    case CAMERAD_MSG_GRAB:          return "GRAB";
    case CAMERAD_MSG_KEYPRESS_T1:   return "KEYPRESS_T1";
    case CAMERAD_MSG_KEYPRESS_T2:   return "KEYPRESS_T2";
    case CAMERAD_MSG_KEYPRESS_T3:   return "KEYPRESS_T3";
    case CAMERAD_MSG_MOVEMENT:      return "MOVEMENT";
    case CAMERAD_MSG_LIMIT:         return "LIMIT";
    case CAMERAD_MSG_POSITION_REQ:  return "POSITION_REQ";
    default:                        return "?";
    }
}

static void handle_command(const camerad_header_t *req,
                           const uint8_t *body, size_t body_len)
{
    /* Stamp rx_ok per opcode before dispatch so the count includes the
     * call even if the handler bails. Unknown opcodes go to the default
     * branch and increment rx_unknown_opcode instead. */
    uint32_t op = req->msg_command;
    if (op < CTRL_MGR_OPCODES) s_stats.rx_ok[op]++;

    /* MOVEMENT arrives at ~25-40 fps even when the stick is centred — too
     * noisy for the dispatch log. Everything else is operator-driven and
     * low-rate; log it so the shot-recall chain is traceable end-to-end. */
    if (op != (uint32_t)CAMERAD_MSG_MOVEMENT) {
        LOG_INFO("ctrl_mgr: RX %s from ctrl=%lu (msg_id=%lu)",
                 camerad_msg_name(op),
                 (unsigned long)req->return_device_no,
                 (unsigned long)req->message_id);
    }

    switch ((camerad_msg_t)op) {
    case CAMERAD_MSG_SELECT:        handle_select     (req);                 break;
    case CAMERAD_MSG_DESELECT:      handle_deselect   (req);                 break;
    case CAMERAD_MSG_GRAB:          handle_grab       (req);                 break;
    case CAMERAD_MSG_KEYPRESS_T1:   handle_keypress_t1(req, body, body_len); break;
    case CAMERAD_MSG_KEYPRESS_T2:   handle_keypress_t2(req, body, body_len); break;
    case CAMERAD_MSG_MOVEMENT:      handle_movement   (req, body, body_len); break;
    /* Phase B+: KEYPRESS_T3, LIMIT, POSITION_REQ, LEARN_ID_REQ — silently
     * dropped. Stats keep the rx_ok[] count for them so a test that
     * sends those opcodes can see they arrived even though we ignore them. */
    default:
        s_stats.rx_unknown_opcode++;
        if (op < CTRL_MGR_OPCODES) s_stats.rx_ok[op]--;  /* undo the speculative bump above */
        break;
    }
}

/*----------------------------------------------------------------------------
 * Inbound TCP listener (panel-initiated TCP for commands)
 *
 * Mirrors Reduced CMC's app_init.c:106-112.
 *---------------------------------------------------------------------------*/

static void open_tcp_listen_for(controller_t *c)
{
    if (!c->enabled) return;
    if (!net_tcp_reopen_listen(c->listen_sock, c->tcp_listen_port)) {
        LOG_ERROR("ctrl_mgr: failed to (re)open TCP listen slot %u on port %u",
                  (unsigned)c->listen_sock, (unsigned)c->tcp_listen_port);
    }
}

/* Per-slot listener service. Each controller has its OWN listen socket
 * (slot 1 or slot 3), its OWN streaming-parser rx buffer, and its OWN
 * edge-detect for log lines. Called once per controller per tick. */
static void service_tcp_listener_slot(controller_t *c)
{
    if (!c->enabled) return;

    net_tcp_state_t st = net_tcp_state(c->listen_sock);
    bool is_up = (st == NET_TCP_ESTABLISHED);

    if (is_up && !c->listener_up_prev) {
        s_stats.tcp_listener_accepts++;
        LOG_INFO("ctrl_mgr: TCP listener inbound up on slot %u port %u (panel %u.%u.%u.%u)",
                 (unsigned)c->listen_sock, (unsigned)c->tcp_listen_port,
                 c->expected_ip[0], c->expected_ip[1],
                 c->expected_ip[2], c->expected_ip[3]);
    } else if (!is_up && c->listener_up_prev) {
        s_stats.tcp_listener_closes++;
        LOG_INFO("ctrl_mgr: TCP listener inbound down slot %u (state=%d)",
                 (unsigned)c->listen_sock, (int)st);
    }
    c->listener_up_prev = is_up;

    if (st == NET_TCP_CLOSED || st == NET_TCP_CLOSE_WAIT) {
        c->tcp_rx_pending = 0;
        open_tcp_listen_for(c);
        return;
    }

    if (!is_up) return;

    if (c->tcp_rx_pending < sizeof(c->tcp_rx_buf)) {
        int32_t n = net_recv(c->listen_sock,
                             c->tcp_rx_buf + c->tcp_rx_pending,
                             (int32_t)(sizeof(c->tcp_rx_buf) - c->tcp_rx_pending));
        if (n > 0) {
            c->tcp_rx_pending += (size_t)n;
        }
    }

    if (c->tcp_rx_pending > s_stats.rx_buf_high_water) {
        s_stats.rx_buf_high_water = (uint32_t)c->tcp_rx_pending;
    }

    size_t offset = 0;
    while (c->tcp_rx_pending - offset >= CAMERAD_HEADER_SIZE) {
        camerad_header_t req;
        if (!camerad_parse_header(c->tcp_rx_buf + offset,
                                  c->tcp_rx_pending - offset, &req)) {
            s_stats.rx_parse_fail++;
            c->tcp_rx_pending = 0;
            return;
        }
        if (req.message_length > c->tcp_rx_pending - offset) {
            break;  /* body not all here yet */
        }
        const uint8_t *body     = c->tcp_rx_buf + offset + CAMERAD_HEADER_SIZE;
        size_t         body_len = (size_t)req.message_length - CAMERAD_HEADER_SIZE;
        /* Set dispatch context so handle_select / handle_grab / etc. send
         * their response back down THIS inbound socket. Cleared before
         * the next loop iteration so a stray send_response_via_outbound
         * outside dispatch can't accidentally hit a wrong socket. */
        s_dispatch_inbound_sock = c->listen_sock;
        handle_command(&req, body, body_len);
        s_dispatch_inbound_sock = DISPATCH_NO_SOCK;
        offset += req.message_length;
    }

    if (offset > 0) {
        if (offset < c->tcp_rx_pending) {
            memmove(c->tcp_rx_buf, c->tcp_rx_buf + offset,
                    c->tcp_rx_pending - offset);
        }
        c->tcp_rx_pending -= offset;
    }

    if (c->tcp_rx_pending >= sizeof(c->tcp_rx_buf)) {
        s_stats.rx_truncated++;
        c->tcp_rx_pending = 0;
    }
}

static void service_tcp_listener(void)
{
    for (uint32_t i = 0; i < CMC_CONTROLLERS; ++i) {
        service_tcp_listener_slot(&s_ctrls[i]);
    }
}

/*----------------------------------------------------------------------------
 * Slot routing — match the POLL's SOURCE IP against the operator-configured
 * panel_a_ip / panel_b_ip. Strict mode: POLLs from unconfigured IPs are
 * dropped. The slot's enabled flag was set at init based on whether the
 * operator provided a non-zero IP for that slot.
 *---------------------------------------------------------------------------*/

static controller_t *find_slot_for_source_ip(const uint8_t src_ip[4])
{
    for (uint32_t i = 0; i < CMC_CONTROLLERS; ++i) {
        controller_t *c = &s_ctrls[i];
        if (c->enabled && memcmp(c->expected_ip, src_ip, 4) == 0) {
            return c;
        }
    }
    return NULL;
}

/*----------------------------------------------------------------------------
 * POLL dispatch — route per-IP, update the matching controller's state,
 * queue a POLL response for service_outbound_tcp to send.
 *---------------------------------------------------------------------------*/

static void handle_poll(const camerad_header_t *req, const uint8_t src_ip[4])
{
    controller_t *c = find_slot_for_source_ip(src_ip);
    if (c == NULL) {
        /* Strict mode: drop POLLs from IPs that don't match either slot. */
        s_stats.poll_rejected_dev++;
        if (s_stats.poll_rejected_dev < 8u) {
            LOG_INFO("ctrl_mgr: POLL from %u.%u.%u.%u dropped (no slot configured for this IP)",
                     src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
        }
        return;
    }

    /* First POLL on this slot — promote in_use and snapshot the panel's
     * identity. Subsequent POLLs from the same IP just refresh the
     * heartbeat + return_port (panel ephemeral port can change). */
    if (!c->in_use) {
        c->in_use      = true;
        c->device_no   = req->return_device_no;
        c->device_type = req->return_device;
        camerad_format_ip((uint8_t *)src_ip, c->ip_str);
        LOG_INFO("ctrl_mgr: panel slot %u acquired by ctrl=%lu type=%lu ip=%s",
                 (unsigned)c->listen_sock, (unsigned long)c->device_no,
                 (unsigned long)c->device_type, c->ip_str);
    }
    c->device_type    = req->return_device;
    c->return_port    = (uint16_t)req->return_port;
    c->last_poll_ms   = time_ms();
    c->pending_poll_request  = *req;
    c->pending_poll_response = true;
}

/*----------------------------------------------------------------------------
 * UDP POLL socket service
 *---------------------------------------------------------------------------*/

static void service_poll_socket(void)
{
    net_addr_t from;
    int32_t n = net_recvfrom(POLL_SOCKET, &from, s_rx_buf, sizeof(s_rx_buf));
    if (n <= 0) return;

    if ((size_t)n >= sizeof(s_rx_buf)) s_stats.rx_truncated++;

    camerad_header_t req;
    if (!camerad_parse_header(s_rx_buf, (size_t)n, &req)) {
        s_stats.poll_rejected_hdr++;
        s_stats.rx_parse_fail++;
        return;
    }
    if ((int32_t)req.message_length != n) {
        s_stats.poll_rejected_hdr++;
        s_stats.rx_parse_fail++;
        return;
    }
    if ((camerad_msg_t)req.msg_command != CAMERAD_MSG_POLL) return;

    s_stats.poll_received++;
    s_stats.rx_ok[CAMERAD_MSG_POLL]++;

    handle_poll(&req, from.addr);
}

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

/* Helper: is this 4-byte IP all zeros? Used to decide whether a slot is
 * "operator-configured" — strict mode means a 0.0.0.0 expected_ip leaves
 * the listen socket closed and POLLs from any IP are dropped from that
 * slot (they may still match the other slot). */
static bool ip_is_blank(const uint8_t ip[4])
{
    return (ip[0] | ip[1] | ip[2] | ip[3]) == 0u;
}

void controller_mgr_init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_ctrls,  0, sizeof(s_ctrls));
    s_outbound_owner_idx = OUTBOUND_OWNER_NONE;

    const network_cfg_t *cfg = config_get_network();
    uint16_t udp_port = (cfg->udp_poll_port != 0) ? cfg->udp_poll_port : 30002u;

    /* Configure slot metadata. Listener socket numbers + ports are fixed
     * by the (chip, design) — only the expected_ip varies per deployment. */
    s_ctrls[0].listen_sock     = TCP_LISTEN_A_SOCKET;
    s_ctrls[0].tcp_listen_port = (cfg->tcp_camerad_port != 0) ? cfg->tcp_camerad_port : 30001u;
    memcpy(s_ctrls[0].expected_ip, cfg->panel_a_ip, 4);
    s_ctrls[0].enabled = !ip_is_blank(s_ctrls[0].expected_ip);

    s_ctrls[1].listen_sock     = TCP_LISTEN_B_SOCKET;
    s_ctrls[1].tcp_listen_port = (cfg->panel_b_port != 0) ? cfg->panel_b_port : 30004u;
    memcpy(s_ctrls[1].expected_ip, cfg->panel_b_ip, 4);
    s_ctrls[1].enabled = !ip_is_blank(s_ctrls[1].expected_ip);

    if (!net_open(POLL_SOCKET, NET_PROTO_UDP, udp_port, false)) {
        LOG_ERROR("ctrl_mgr: failed to open POLL UDP %u", (unsigned)udp_port);
        return;
    }

    /* Open each enabled listener. A disabled slot stays in CLOSED state on
     * its W6100 socket — no resources consumed, no packets accepted. */
    for (uint32_t i = 0; i < CMC_CONTROLLERS; ++i) {
        controller_t *c = &s_ctrls[i];
        if (!c->enabled) {
            LOG_INFO("ctrl_mgr: panel %c slot disabled (no expected IP configured)",
                     (char)('A' + i));
            continue;
        }
        if (!net_open(c->listen_sock, NET_PROTO_TCP, c->tcp_listen_port, true)) {
            LOG_ERROR("ctrl_mgr: failed to open TCP listen on slot %u port %u",
                      (unsigned)c->listen_sock, (unsigned)c->tcp_listen_port);
            c->enabled = false;
        } else {
            LOG_INFO("ctrl_mgr: panel %c listen up on :%u (expect ip=%u.%u.%u.%u)",
                     (char)('A' + i), (unsigned)c->tcp_listen_port,
                     c->expected_ip[0], c->expected_ip[1],
                     c->expected_ip[2], c->expected_ip[3]);
        }
    }

    s_inited = true;
    LOG_INFO("ctrl_mgr: ready. POLL UDP=%u (device_no=%lu, type=%u)",
             (unsigned)udp_port,
             (unsigned long)cfg->cmc_device_no,
             (unsigned)CAMERAD_DEV_CMC_BLDC);
}

/* (Periodic stats dump removed — was generating ~12 log lines every 10s
 * and drowning out the event trace. Counters still update internally
 * and are exposed via GET /api/stats for tools that want them.) */

/* Drop the per-panel discovered state (in_use, device info, last_poll_ms)
 * and force-clear any selection this controller held. PRESERVES the slot's
 * config (enabled, expected_ip, listen_sock, tcp_listen_port) so the
 * listener stays open ready for the panel to come back. Also releases the
 * shared outbound slot if this controller owned it. */
static void evict_controller(controller_t *c, uint32_t idx)
{
    uint32_t dev = c->device_no;
    LOG_INFO("ctrl_mgr: panel %c (ctrl=%lu) timed out (no POLL for %u ms) — evicting",
             (char)('A' + idx), (unsigned long)dev,
             (unsigned)CONTROLLER_TIMEOUT_MS);
    cmc_state_force_deselect(dev);
    if (s_outbound_owner_idx == (int)idx) {
        release_outbound();
    }
    /* Wipe the dynamic part; keep listener config + rx buffer intact. */
    c->in_use                = false;
    c->device_no             = 0;
    c->device_type           = 0;
    c->ip_str[0]             = '\0';
    c->return_port           = 0;
    c->conn                  = CONN_IDLE;
    c->connect_start_ms      = 0;
    c->last_poll_ms          = 0;
    c->pending_poll_response = false;
    memset(&c->pending_poll_request, 0, sizeof(c->pending_poll_request));
    /* tcp_rx_pending preserved — partial frame from this panel would have
     * been mid-stream, but listener will reset it on its own state path. */
}

static void check_controller_timeouts(void)
{
    for (uint32_t i = 0; i < CMC_CONTROLLERS; ++i) {
        controller_t *c = &s_ctrls[i];
        if (!c->in_use) continue;
        if (time_elapsed_ms(c->last_poll_ms) > CONTROLLER_TIMEOUT_MS) {
            evict_controller(c, i);
        }
    }
}

void controller_mgr_tick(void)
{
    if (!s_inited) return;
    service_poll_socket();
    service_outbound_tcp();
    service_tcp_listener();
    check_controller_timeouts();
}

/*----------------------------------------------------------------------------
 * Stats
 *---------------------------------------------------------------------------*/

void controller_mgr_get_stats(controller_mgr_stats_t *out)
{
    if (out) *out = s_stats;
}
