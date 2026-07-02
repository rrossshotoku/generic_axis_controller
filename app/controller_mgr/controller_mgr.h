/*
 * app/controller_mgr — CAMERAD controller socket + dispatch.
 *
 * Owns the network-facing side of CAMERAD:
 *   - UDP poll listener on port 30002 (CAMERAD_PORT_POLL_DEFAULT)
 *   - per-controller TCP sessions (max 2, Phase B)
 *   - dispatch table: incoming CAMERAD opcode → handler
 *   - response sender (S/T poll responses, keypress responses, etc.)
 *   - GRAB arbitration (Phase B — depends on app/cmc_state)
 *
 * v1 (Path A — minimal): UDP poll listener + hardcoded POLL response so a
 * CAMERAD-style panel can discover the LCMC on the network. TCP sessions,
 * SELECT/GRAB, keypress, movement, shot recall — all Phase B.
 *
 * Layering: L2 (top-level service). Depends on:
 *   - app/camerad      (codec — parse / build CAMERAD frames)
 *   - app/config       (our IP, our device number, poll port)
 *   - app/log          (diagnostics)
 *   - bsp/net          (W6100 sockets)
 *   - bsp/time         (timeouts — Phase B)
 *
 * Does NOT include any other app/ module from above (no upward
 * dependency into app/main_loop, app/debug). Phase B will add app/cmc_state
 * and app/axis_manager downward dependencies.
 */

#ifndef APP_CONTROLLER_MGR_H
#define APP_CONTROLLER_MGR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void controller_mgr_init(void);
void controller_mgr_tick(void);

/* Diagnostic counters surfaced via app/debug and the /api/stats web endpoint.
 *
 * The legacy poll_* counters are kept for backward-compat with anything that
 * already references them. The per-opcode rx_ok[]/tx_ok[] arrays are the
 * canonical source for the robustness tests — indexed by camerad_msg_t
 * (POLL=1, SELECT=2, ... POSITION_REQ=11, LIMIT=10, etc.). Array size 16
 * leaves headroom for all current CAMERAD opcodes; index 0 is unused. */
#define CTRL_MGR_OPCODES  16

typedef struct {
    /* Legacy POLL-only counters (kept for backward compat). */
    uint32_t poll_received;
    uint32_t poll_responded_s;
    uint32_t poll_responded_t;
    uint32_t poll_rejected_hdr;
    uint32_t poll_rejected_dev;
    uint32_t poll_send_errors;

    /* Per-opcode counters (canonical for robustness tests). */
    uint32_t rx_ok            [CTRL_MGR_OPCODES];   /* parsed + dispatched   */
    uint32_t tx_ok            [CTRL_MGR_OPCODES];   /* response sent for op  */
    uint32_t rx_parse_fail;                          /* bad header / length   */
    uint32_t rx_unknown_opcode;                      /* parsed, opcode unhand */
    uint32_t rx_truncated;                           /* > buffer size         */

    /* Transport-level (TCP listener, TCP outbound). */
    uint32_t tcp_listener_accepts;                   /* inbound TCP ESTABLISH */
    uint32_t tcp_listener_closes;                    /* inbound TCP closed    */
    uint32_t tcp_outbound_connects;                  /* CMC-initiated TCP     */
    uint32_t tcp_outbound_failures;                  /* outbound connect t/o  */

    /* Buffer pressure. */
    uint32_t rx_buf_high_water;                      /* peak inbound TCP buf  */
} controller_mgr_stats_t;

void controller_mgr_get_stats(controller_mgr_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROLLER_MGR_H */
