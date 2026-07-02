/*
 * app/config — persistent settings.
 *
 * Phase 0a: RAM-only. Defaults applied at init; setters mutate the RAM
 * struct. Phase 0b/4 will back this with bsp/flash for persistence.
 *
 * See app/config/README.md for the contract.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_AUTH_USER_MAX   16
#define CONFIG_AUTH_HASH_LEN   32          /* SHA-256 */
#define CONFIG_AUTH_SALT_LEN   16

typedef struct {
    uint8_t  mac[6];
    uint8_t  ip[4];
    uint8_t  netmask[4];
    uint8_t  gateway[4];
    uint16_t udp_poll_port;       /* CAMERAD poll, default 30002 */
    /* Two-panel IP-pinned routing: each slot has its own TCP listen port +
     * an expected panel IP. POLL from panel_a_ip -> respond with panel_a_port;
     * POLL from panel_b_ip -> respond with panel_b_port. Strict mode —
     * panel_X_ip == 0.0.0.0 means "slot disabled, no listener opened, POLLs
     * from any IP that doesn't match the OTHER slot are dropped". Slot A
     * keeps the historical tcp_camerad_port field name for back-compat with
     * existing call sites; slot B is brand-new. */
    uint16_t tcp_camerad_port;    /* Panel A TCP listen, default 30001 (matches SW050 LISTENPORT1) — advertised as return_port in POLL responses to panel_a_ip */
    uint8_t  panel_a_ip[4];       /* Expected source IP for panel A's POLLs; 0.0.0.0 = slot A disabled */
    uint16_t panel_b_port;        /* Panel B TCP listen; 0 = slot B disabled */
    uint8_t  panel_b_ip[4];       /* Expected source IP for panel B's POLLs; 0.0.0.0 = slot B disabled */
    uint16_t http_port;           /* default 80 */
    uint16_t od_udp_port;         /* OD access UDP, default 5000 per Interface/NETWORK_UDP_SPEC.md.
                                   * Telemetry sits at od_udp_port + 1 (= 5001 by default). */
    uint16_t log_tcp_port;        /* log socket, default 30200 */
    uint32_t cmc_device_no;       /* this CMC's CAMERAD device number */
} network_cfg_t;

#define MOTOR_AXIS_COUNT 1   /* Phase 0a: one pan axis. Extend later. */

typedef struct {
    int32_t low_count;
    int32_t high_count;
} axis_limit_t;

typedef struct {
    axis_limit_t axis[MOTOR_AXIS_COUNT];
} motor_limits_t;

typedef struct {
    char     username[CONFIG_AUTH_USER_MAX];
    uint8_t  pass_hash[CONFIG_AUTH_HASH_LEN];
    uint8_t  salt[CONFIG_AUTH_SALT_LEN];
    bool     default_password;   /* true until the user has changed it */
} auth_cfg_t;

void                   config_init(void);                          /* applies defaults */

const network_cfg_t  * config_get_network(void);
bool                   config_set_network(const network_cfg_t *cfg);

const motor_limits_t * config_get_limits(void);
bool                   config_set_limits(const motor_limits_t *lim);

const auth_cfg_t     * config_get_auth(void);
bool                   config_set_auth_password(const char *new_password);

uint8_t                config_get_node_id(void);
bool                   config_set_node_id(uint8_t node_id);

/* Persist the operator-tunable subset of the network config (IP, netmask,
 * gateway, cmc_device_no) to flash via app/persist's NETWORK region.
 * Returns true on success. Blocks for ~30 ms during flash erase/program.
 * Called from app/web's /api/save handler. Other network fields (ports,
 * MAC) are not persisted — ports are fixed wire conventions and MAC is
 * derived from the STM32 UID at boot. */
bool                   config_save_network_to_flash(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
