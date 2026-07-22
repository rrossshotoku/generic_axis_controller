/*
 * app/main_loop — Phase 0b orchestrator.
 *
 * Init order (top of file = first to run):
 *   1. bsp/time     — timekeeping
 *   2. app/log      — RAM ring buffer ready to capture init messages
 *   3. app/config   — defaults populated
 *   4. log port set — log_tick can now know where to listen
 *   5. bsp/net      — W6100 brought up; static IP applied
 *
 * Tick order, repeated forever:
 *   1. log_tick()                            — service TCP log socket
 *   2. heartbeat (LED toggle, log every 1 s)
 *
 * Phase 1+ will add controller_mgr, web, motor_ctrl, od, cia402 ticks here.
 *
 * NOTE: a hardware watchdog (IWDG) was originally part of this skeleton
 * but has been removed pending a deliberate enable in the .ioc + a unified
 * init path. See Documentation/architecture.md §7.1 for the placeholder.
 */

#include "main_loop.h"

#include "app/log/log.h"
#include "app/config/config.h"
#include "app/cia402/cia402.h"
#include "app/axis_manager/axis_manager.h"
#include "app/boot_meta/boot_meta.h"
#include "app/od/od.h"
#include "app/cmc_state/cmc_state.h"
#include "app/controller_mgr/controller_mgr.h"
#include "app/visca_mgr/visca_mgr.h"
#include "app/led_indicator/led_indicator.h"
#include "app/persist/persist.h"
#include "app/web/web.h"
#include "app/debug/debug.h"

#include "Interface/mc_if_od.h"    /* MC_IF_PROTOCOL_* */
#include "bsp/buttons/buttons.h"
#include "bsp/leds/leds.h"
#include "bsp/time/time.h"
#include "bsp/net/net.h"

#include "stm32g4xx_hal.h"
#include "main.h"               /* CubeMX-generated DEBUG_LED_* macros */

/* Heartbeat: blink LED + INFO log once per second. */
#define HEARTBEAT_PERIOD_MS  1000

/* Active protocol snapshot. Set once at init from config; the tick uses
 * this cached value so an OD write to 0x3080 mid-session doesn't hot-swap
 * (the operator must Save + Reboot for the change to take effect).
 * Sockets/UARTs the mgrs own are brought up in their _init and would
 * conflict if two mgrs ran simultaneously. */
static uint8_t s_active_protocol;

void main_loop_init(void)
{
    time_init();
    log_init();
    config_init();

    LOG_INFO("Lightweight CMC boot");

    const network_cfg_t *net = config_get_network();
    LOG_INFO("node_id=%u ip=%u.%u.%u.%u",
             config_get_node_id(),
             net->ip[0], net->ip[1], net->ip[2], net->ip[3]);

    log_set_tcp_port(net->log_tcp_port);

    /* Bring up the W6100. On failure, the device still ticks (heartbeat
     * + watchdog) so we can spot the failure in the log ring under the
     * debugger. The log TCP socket simply never opens, and log_tick
     * keeps retrying its prerequisites. */
    if (!net_init(net->mac, net->ip, net->netmask, net->gateway)) {
        LOG_ERROR("net_init failed — continuing without network");
    }

    /* Phase 4 cia402 stub + Phase 5 OD network bridge. cia402 currently
     * returns NOT_READY for every OD request — that's exactly what we
     * want for now, so a PC tool can exercise the full network path. */
    /* persist must come BEFORE axis_manager_init so axis_manager can read
     * its config blob from flash during its own init. */
    bsp_buttons_init();
    /* LEDs must be initialised BEFORE persist_init/axis_manager_init so the
     * LED driver is ready to render whatever colour axis_manager loads from
     * flash on first tick. Failure to start (period mismatch with CubeMX)
     * is logged but non-fatal — system continues with LEDs dark. */
    if (!bsp_leds_init()) {
        LOG_ERROR("bsp_leds_init failed (TIM1 period mismatch?) — LEDs disabled");
    }
    persist_init();
    /* boot_meta reads the persistent "stay in bootloader" flag from the
     * BOOT persist region. Placed immediately after persist_init so the
     * flag's boot-time state is snapshotted before anything else runs
     * (subsequent modules may want to log against it). */
    boot_meta_init();
    cia402_init();
    axis_manager_init();
    led_indicator_init();
    od_init();
    cmc_state_init();
    /* Protocol dispatch. Snapshot the persisted selection ONCE at boot
     * and init only the module that will actually run. web_init still
     * runs regardless — the operator uses the web UI to change the
     * selection and Save + Reboot to apply. */
    s_active_protocol = config_get_active_protocol();
    switch (s_active_protocol) {
    case MC_IF_PROTOCOL_VISCA:
        LOG_INFO("main_loop: protocol=VISCA (initialising visca_mgr)");
        visca_mgr_init();
        break;
    case MC_IF_PROTOCOL_CAMERAD:
    default:
        LOG_INFO("main_loop: protocol=CAMERAD (initialising controller_mgr)");
        controller_mgr_init();
        break;
    }
    web_init();
    debug_init();
    /* (profile module retired from per-tick use; bsp/time owns DWT init.) */
}

void main_loop_run(void)
{
    uint32_t last_beat = time_ms();
    uint32_t beats = 0;

    for (;;) {
        log_tick();
        /* Advance button debounce filters every loop iteration so any
         * consumer (currently axis_manager) sees a fresh debounced state.
         * Cheap (two GPIO reads + counter updates); independent of who
         * actually consumes the state. */
        bsp_buttons_tick();
        /* axis_manager must run BEFORE cia402 so each cycle's outbound SPI
         * frame carries the freshest cmd. axis_manager peeks status (stale
         * by ~1 ms from the previous cycle's RX), composes a new cmd, and
         * cia402_tick then transmits it on this cycle. */
        axis_manager_tick();
        cia402_tick();
        od_tick();
        /* cmc_state refreshes its on_shot / moving flags from the live
         * motor position; cheap and harmless to call every loop. */
        cmc_state_update_from_motor();
        /* Only the mgr matching the boot-time protocol snapshot runs.
         * The other one is compiled in but never ticked. */
        switch (s_active_protocol) {
        case MC_IF_PROTOCOL_VISCA:   visca_mgr_tick();      break;
        case MC_IF_PROTOCOL_CAMERAD:
        default:                     controller_mgr_tick(); break;
        }
        web_tick();
        /* led_indicator_tick reads motor moving state from axis_manager and
         * link state from bsp/net, then drives bsp_leds. Cheap (~µs). */
        led_indicator_tick();
        /* boot_meta_tick clears the "stay in bootloader" flag after
         * BOOT_META_HEALTHY_MS of no-fault runtime. No-op on subsequent
         * ticks once cleared, and a no-op every tick when the flag was
         * already CLEAR at boot. */
        boot_meta_tick();
        debug_tick();

        if (time_elapsed_ms(last_beat) >= HEARTBEAT_PERIOD_MS) {
            last_beat += HEARTBEAT_PERIOD_MS;
            beats++;
            g_debug.system.heartbeat_count = beats;
            HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
            /* (No periodic LOG_INFO — the debug log is now event-driven so
             * the request/dispatch/SDO trace for shot recalls isn't drowned
             * out by 1Hz heartbeat noise. The LED still blinks as a visual
             * "alive" indicator and g_debug.system.heartbeat_count remains
             * inspectable via the debugger.) */
        }

        /* No sleep — the orchestrator polls. */
    }
}
