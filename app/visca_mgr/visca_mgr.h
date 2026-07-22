/*
 * app/visca_mgr — VISCA transport + session + dispatch.
 *
 * First-pass implementation (2026-07-22). Supports the small subset the
 * PC tool needs for bring-up:
 *
 *   Inquiries:
 *     - CAM_VersionInq      → static vendor/model/rom identifier
 *     - Pan_tiltPosInq      → current axis_manager position → int16 pan, 0 tilt
 *     - CAM_ZoomPosInq      → always 0 (no zoom on this CMC)
 *
 *   Commands:
 *     - Pan_tiltDrive       → cmc_state_handle_movement_scaled (speed 1-24)
 *     - Pan_tiltStop        → cmc_state_handle_movement_scaled(0)
 *                              (Stop is Pan_tiltDrive with direction=03)
 *     - Pan_tiltHome        → axis_manager_request_home
 *     - Memory Set (store)  → cmc_state_store_shot
 *     - Memory Recall       → cmc_state_move_to_shot (fade, uses stored time)
 *
 * Transport: VISCA over UDP port 52381 (Sony's VISCA-over-IP standard,
 * 8-byte IP header + VISCA payload). No TCP. No serial. When/if serial
 * is added, the app/visca codec is reusable — only this transport layer
 * needs to change.
 *
 * Session model: STATELESS. Every command gets an immediate ACK +
 * Completion (no in-flight tracking). Long-running commands (preset
 * recall, homing) fire and forget from VISCA's perspective; the caller
 * polls Pan_tiltPosInq to detect arrival. Real Sony cameras track two
 * concurrent commands via sockets; we skip that until a real need shows
 * up.
 *
 * Ownership: VISCA has no SELECT/DESELECT/GRAB concept. This module does
 * NOT call cmc_state_handle_select — any VISCA client can command the
 * CMC as long as VISCA is the active protocol. If two VISCA clients
 * fight, last-write-wins; add auto-select if that becomes a problem.
 *
 * Layering: L2. Depends on:
 *   - app/visca         (codec)
 *   - app/cmc_state     (handle_movement_scaled, store_shot, move_to_shot)
 *   - app/axis_manager  (get_position_actual, request_home)
 *   - app/config        (VISCA device address, our IP for socket bring-up)
 *   - app/log
 *   - bsp/net
 *   - bsp/time
 */
#ifndef APP_VISCA_MGR_H
#define APP_VISCA_MGR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Called once from main_loop_init IFF config_get_active_protocol()
 * returns MC_IF_PROTOCOL_VISCA. Opens the UDP listen socket on port
 * VISCA_UDP_PORT (52381). Failure logs an error; the module still
 * ticks (no-op) so the rest of the system keeps working. */
void visca_mgr_init(void);

/* Called every loop iteration IFF the CMC booted with VISCA active.
 * Drains one inbound packet per tick, dispatches, sends any responses. */
void visca_mgr_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISCA_MGR_H */
