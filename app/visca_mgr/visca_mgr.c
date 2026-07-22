/*
 * app/visca_mgr — implementation. STUB (2026-07-22).
 *
 * See visca_mgr.h for the intent. When you begin the real implementation:
 *   1. Add app/visca/ (codec — bytes ↔ structs, stateless).
 *   2. Decide transport: VISCA-over-IP (UDP 52381) uses bsp/net; serial
 *      uses a new bsp/uart module.
 *   3. Wire dispatch: on inbound frame, decode → call cmc_state_* /
 *      axis_manager_* as appropriate (see app/controller_mgr for the
 *      CAMERAD analogue).
 *   4. Emit ACK / completion / error back over the same channel.
 *   5. Register any inactivity/keepalive logic if VISCA-over-IP.
 *   6. Add diagnostic counters (visca_mgr_get_stats) matching the
 *      shape debug.c consumes.
 */
#include "visca_mgr.h"

#include "app/log/log.h"

void visca_mgr_init(void)
{
    LOG_INFO("visca_mgr: stub — no VISCA implementation yet. "
             "Boot dispatched here because config.active_protocol=VISCA. "
             "Set 0x3080=0 (CAMERAD) + Save + Reboot to switch back.");
}

void visca_mgr_tick(void)
{
    /* No-op until implemented. */
}
