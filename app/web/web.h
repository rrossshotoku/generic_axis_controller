/*
 * app/web — embedded HTTP server for the CMC configuration page.
 *
 * Listens on socket 7 (per architecture.md §10.1) at config.http_port
 * (default 80). One concurrent request at a time — a panel-style config
 * tool needs no more.
 *
 * Endpoints:
 *   GET  /              -> serves the embedded HTML page
 *   GET  /api/config    -> returns JSON of current state (network +
 *                          limits + joystick + payload + status)
 *   POST /api/config    -> applies values (RAM only)
 *   POST /api/save      -> commits axis_manager config + network to flash
 *   POST /api/reboot    -> NVIC reset (after responding)
 *
 * Apply-then-Save UX:
 *   The web page sends individual section changes as POST /api/config
 *   bodies that contain only the changed sub-objects. The CMC applies
 *   them to RAM immediately. Operator then clicks "Save to flash" which
 *   triggers POST /api/save. For an IP-address change, the operator
 *   clicks "Save + Reboot" — the page POSTs /api/save then /api/reboot.
 *
 * No JSON library — small hand-rolled parser/builder for the fixed
 * schema below. No auth (v1, LAN-only). HTTP/1.0 style: one request per
 * connection, Connection: close.
 *
 * Layering: L4 (orchestrator-ish — calls into config + axis_manager
 * and the bsp directly). Depends on app/config, app/axis_manager,
 * bsp/net, bsp/sys, bsp/time, bsp/identity.
 */

#ifndef APP_WEB_H
#define APP_WEB_H

#ifdef __cplusplus
extern "C" {
#endif

void web_init(void);
void web_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_WEB_H */
