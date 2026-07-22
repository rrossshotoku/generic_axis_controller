/*
 * app/visca_mgr — VISCA transport + session + dispatch.
 *
 * STUB (2026-07-22). Init logs "visca_mgr: stub — no VISCA implementation
 * yet". Tick is a no-op. The module exists so main_loop can dispatch on
 * the active_protocol flag (0x3080) without conditional-compilation.
 *
 * When implemented, this module owns the network-facing side of VISCA:
 *   - VISCA-over-IP UDP listener (port 52381 by default) OR serial UART
 *     bring-up, depending on the deployment. Decide at implementation time
 *     — the interface here (init + tick) stays the same either way.
 *   - Address-based dispatch (VISCA frames start with `8x` where `x` is the
 *     target camera 1-7 or 8=broadcast).
 *   - Command decode → cmc_state / axis_manager calls (Pan-Tilt Drive,
 *     Memory Set / Recall, Absolute Position, Home, Reset, inquiries).
 *   - ACK / completion / error responses back to the sender.
 *
 * Compiled in on every build so main_loop can pick between CAMERAD and
 * VISCA at boot without conditional compilation. Only one runs — the
 * inactive module's flash cost is ~10-15 KB when fully implemented.
 *
 * Layering: same as controller_mgr — L2, depends on:
 *   - app/visca         (codec, once implemented)
 *   - app/cmc_state     (session / shots / status — transport-agnostic)
 *   - app/axis_manager  (joy_profile, joystick_raw, homing)
 *   - app/config        (device address, ports)
 *   - app/log           (diagnostics)
 *   - bsp/net           (sockets)   OR   bsp/uart (serial, TBD)
 *   - bsp/time          (timeouts)
 *
 * Does NOT include any other app/ module upward (no main_loop, debug).
 */
#ifndef APP_VISCA_MGR_H
#define APP_VISCA_MGR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Called once from main_loop_init IFF config_get_active_protocol()
 * returns MC_IF_PROTOCOL_VISCA. Currently just logs the stub state. */
void visca_mgr_init(void);

/* Called every loop iteration IFF the CMC booted with VISCA active.
 * Currently no-op. */
void visca_mgr_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISCA_MGR_H */
