/*
 * app/config — Phase 0a: RAM-only defaults.
 *
 * No flash backing yet. Settings are reset to defaults on every boot.
 * Phase 4 will wire this through bsp/flash for persistence.
 */

#include "config.h"
#include "app/log/log.h"
#include "app/persist/persist.h"
#include "bsp/identity/identity.h"
#include "Interface/mc_if_od.h"    /* MC_IF_PROTOCOL_* */
#include <string.h>

static network_cfg_t  s_network;
static motor_limits_t s_limits;
static auth_cfg_t     s_auth;
static uint8_t        s_node_id;

/* On-flash network blob. Only the operator-tunable subset is persisted.
 * Bump NETWORK_PERSIST_VERSION on any layout change so stale blobs are
 * rejected by persist_load (caller falls back to coded defaults).
 *
 * v2 (2026-06-26): added panel_a_ip + panel_b_port + panel_b_ip for the
 * two-panel IP-pinned routing scheme (controller_mgr opens up to two TCP
 * listeners, one per (port, expected-source-IP) pair). Boards with a v1
 * blob in flash fall through to coded defaults at next boot — operator
 * must re-Save once after upgrade to recover their custom IP. Struct
 * grew 32 -> 48 B; persist region (2 KB) has plenty of room. */
#define NETWORK_PERSIST_VERSION  2u
#define NETWORK_PERSIST_MAGIC    0x4E455457u    /* "NETW" little-endian */

typedef struct __attribute__((packed)) {
    uint32_t magic;             /*  4 */
    uint8_t  ip[4];             /*  4 */
    uint8_t  netmask[4];        /*  4 */
    uint8_t  gateway[4];        /*  4 */
    uint32_t cmc_device_no;     /*  4 — advertised to panels as return_device_no */
    uint32_t tcp_camerad_port;  /*  4 — panel A listen port, advertised as return_port to panel_a_ip */
    uint8_t  panel_a_ip[4];     /*  4 — expected source IP for panel A; 0.0.0.0 = slot disabled */
    uint32_t panel_b_port;      /*  4 — panel B listen port, advertised as return_port to panel_b_ip; 0 = slot disabled */
    uint8_t  panel_b_ip[4];     /*  4 — expected source IP for panel B; 0.0.0.0 = slot disabled */
    /* Repurposed reserved[0] as active_protocol (0x3080). Value is MC_IF_PROTOCOL_*
     * in the low byte, zero elsewhere. Old v2 blobs written before this field
     * existed already had reserved[0] = 0 → loads as MC_IF_PROTOCOL_CAMERAD,
     * which is also the default, so no version bump or migration needed. If
     * we ever need this to mean something else, THEN bump v3 with a real
     * migrator. */
    uint32_t active_protocol;   /*  4 — MC_IF_PROTOCOL_* (was reserved[0]) */
    uint32_t reserved[2];       /*  8 — room for two more u32s without another version bump */
} network_persist_blob_t;       /* total: 48 */

_Static_assert(sizeof(network_persist_blob_t) == 48,
               "network_persist_blob_t layout drift");

void config_init(void)
{
    /* Defaults. MAC is derived per-unit by bsp/identity (today: from the
     * STM32 UID; later: from an SPI-connected identity device). IP is a
     * recognisable placeholder until the web UI sets it. */
    memset(&s_network, 0, sizeof(s_network));
    identity_get_mac(s_network.mac);
    s_network.ip[0]      = 192; s_network.ip[1]      =   1; s_network.ip[2]      = 0; s_network.ip[3]      = 100;
    s_network.netmask[0] = 255; s_network.netmask[1] = 255; s_network.netmask[2] = 255; s_network.netmask[3] = 0;
    s_network.gateway[0] = 192; s_network.gateway[1] =   1; s_network.gateway[2] = 0; s_network.gateway[3] = 1;
    s_network.udp_poll_port    = 30002;
    s_network.tcp_camerad_port = 30001;     /* Panel A default port (matches SW050 LISTENPORT1) */
    s_network.panel_b_port     = 30004;     /* Panel B default port (different from A so two listen slots can coexist on different W6100 sockets) */
    /* Both panel IPs default to 0.0.0.0 — strict mode means neither listener
     * comes up until the operator configures real IPs via the web UI. This is
     * deliberate (Option A): a fresh board with no config doesn't open any
     * CAMERAD inbound sockets, so no random network device can connect.
     * Operator workflow: web -> Network -> set panel IPs -> Save -> Reboot. */
    memset(s_network.panel_a_ip, 0, sizeof(s_network.panel_a_ip));
    memset(s_network.panel_b_ip, 0, sizeof(s_network.panel_b_ip));
    s_network.http_port        = 80;
    s_network.od_udp_port      = 5000;
    s_network.log_tcp_port     = 30200;
    s_network.cmc_device_no    = 1;
    s_network.active_protocol  = MC_IF_PROTOCOL_CAMERAD;   /* default until operator changes it via 0x3080 + Save + Reboot */

    /* Try to overlay the operator-tunable fields from flash. On any
     * failure (uninitialised region, CRC mismatch, version bump) the
     * defaults set above stand. The fields outside the persisted set
     * (ports, MAC) always come from the defaults — they are wire
     * conventions, not operator settings. */
    network_persist_blob_t blob;
    size_t got = 0;
    if (persist_load(PERSIST_REGION_NETWORK, &blob, sizeof(blob),
                     NETWORK_PERSIST_VERSION, &got)
        && got == sizeof(blob)
        && blob.magic == NETWORK_PERSIST_MAGIC) {
        memcpy(s_network.ip,      blob.ip,      4);
        memcpy(s_network.netmask, blob.netmask, 4);
        memcpy(s_network.gateway, blob.gateway, 4);
        s_network.cmc_device_no = blob.cmc_device_no;
        if (blob.tcp_camerad_port != 0u && blob.tcp_camerad_port <= 0xFFFFu) {
            s_network.tcp_camerad_port = (uint16_t)blob.tcp_camerad_port;
        }
        memcpy(s_network.panel_a_ip, blob.panel_a_ip, 4);
        if (blob.panel_b_port <= 0xFFFFu) {
            s_network.panel_b_port = (uint16_t)blob.panel_b_port;
        }
        memcpy(s_network.panel_b_ip, blob.panel_b_ip, 4);
        /* active_protocol was written into what used to be reserved[0].
         * Range-clamp on load so a corrupt value can't crash main_loop's
         * dispatch switch — unknown values fall back to CAMERAD. */
        if (blob.active_protocol < MC_IF_PROTOCOL_COUNT) {
            s_network.active_protocol = (uint8_t)blob.active_protocol;
        } else {
            LOG_WARN("config: unknown active_protocol=%lu on flash — using CAMERAD",
                     (unsigned long)blob.active_protocol);
            s_network.active_protocol = MC_IF_PROTOCOL_CAMERAD;
        }
        LOG_INFO("config: network loaded from flash (ip=%u.%u.%u.%u dev=%lu)",
                 s_network.ip[0], s_network.ip[1], s_network.ip[2], s_network.ip[3],
                 (unsigned long)s_network.cmc_device_no);
        LOG_INFO("config: panel A: ip=%u.%u.%u.%u port=%u  panel B: ip=%u.%u.%u.%u port=%u",
                 s_network.panel_a_ip[0], s_network.panel_a_ip[1],
                 s_network.panel_a_ip[2], s_network.panel_a_ip[3],
                 (unsigned)s_network.tcp_camerad_port,
                 s_network.panel_b_ip[0], s_network.panel_b_ip[1],
                 s_network.panel_b_ip[2], s_network.panel_b_ip[3],
                 (unsigned)s_network.panel_b_port);
        LOG_INFO("config: active_protocol=%u (%s)",
                 (unsigned)s_network.active_protocol,
                 (s_network.active_protocol == MC_IF_PROTOCOL_VISCA) ? "VISCA" : "CAMERAD");
    } else {
        LOG_INFO("config: network using factory defaults (active_protocol=CAMERAD)");
    }

    memset(&s_limits, 0, sizeof(s_limits));
    /* Default to "no limit" by setting a wide range; motor_ctrl will treat
     * a low==high pair as "disabled". Tighten per-axis from the web. */
    for (size_t i = 0; i < MOTOR_AXIS_COUNT; i++) {
        s_limits.axis[i].low_count  = -2147483647;
        s_limits.axis[i].high_count =  2147483647;
    }

    memset(&s_auth, 0, sizeof(s_auth));
    /* Factory default — flagged so the device can refuse production
     * features until the user has set a real password. */
    strncpy(s_auth.username, "admin", sizeof(s_auth.username) - 1);
    s_auth.default_password = true;

    s_node_id = 1;
}

const network_cfg_t * config_get_network(void) { return &s_network; }

bool config_set_network(const network_cfg_t *cfg)
{
    if (!cfg) return false;
    s_network = *cfg;
    return true;
}

const motor_limits_t * config_get_limits(void) { return &s_limits; }

bool config_set_limits(const motor_limits_t *lim)
{
    if (!lim) return false;
    s_limits = *lim;
    return true;
}

const auth_cfg_t * config_get_auth(void) { return &s_auth; }

bool config_set_auth_password(const char *new_password)
{
    if (!new_password || new_password[0] == '\0') return false;
    /* Phase 0a stub: store the literal string in the hash slot. Phase 4
     * replaces this with a real SHA-256(salt || password). */
    size_t n = strlen(new_password);
    if (n > sizeof(s_auth.pass_hash)) n = sizeof(s_auth.pass_hash);
    memset(s_auth.pass_hash, 0, sizeof(s_auth.pass_hash));
    memcpy(s_auth.pass_hash, new_password, n);
    s_auth.default_password = false;
    return true;
}

uint8_t config_get_node_id(void)            { return s_node_id; }

bool config_set_node_id(uint8_t node_id)
{
    /* CANopen valid range 1..127 */
    if (node_id < 1 || node_id > 127) return false;
    s_node_id = node_id;
    return true;
}

uint8_t config_get_active_protocol(void) { return s_network.active_protocol; }

bool config_set_active_protocol(uint8_t protocol)
{
    if (protocol >= MC_IF_PROTOCOL_COUNT) return false;
    if (s_network.active_protocol != protocol) {
        LOG_INFO("config: active_protocol staged %u -> %u (applies on reboot)",
                 (unsigned)s_network.active_protocol, (unsigned)protocol);
    }
    s_network.active_protocol = protocol;
    return true;
}

bool config_save_network_to_flash(void)
{
    network_persist_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic         = NETWORK_PERSIST_MAGIC;
    memcpy(blob.ip,      s_network.ip,      4);
    memcpy(blob.netmask, s_network.netmask, 4);
    memcpy(blob.gateway, s_network.gateway, 4);
    blob.cmc_device_no    = s_network.cmc_device_no;
    blob.tcp_camerad_port = (uint32_t)s_network.tcp_camerad_port;
    memcpy(blob.panel_a_ip, s_network.panel_a_ip, 4);
    blob.panel_b_port     = (uint32_t)s_network.panel_b_port;
    memcpy(blob.panel_b_ip, s_network.panel_b_ip, 4);
    blob.active_protocol  = (uint32_t)s_network.active_protocol;

    bool ok = persist_save(PERSIST_REGION_NETWORK, &blob, sizeof(blob),
                           NETWORK_PERSIST_VERSION);
    if (ok) {
        LOG_INFO("config: network saved to flash (ip=%u.%u.%u.%u dev=%lu)",
                 s_network.ip[0], s_network.ip[1], s_network.ip[2], s_network.ip[3],
                 (unsigned long)s_network.cmc_device_no);
        LOG_INFO("config: panel A: ip=%u.%u.%u.%u port=%u  panel B: ip=%u.%u.%u.%u port=%u",
                 s_network.panel_a_ip[0], s_network.panel_a_ip[1],
                 s_network.panel_a_ip[2], s_network.panel_a_ip[3],
                 (unsigned)s_network.tcp_camerad_port,
                 s_network.panel_b_ip[0], s_network.panel_b_ip[1],
                 s_network.panel_b_ip[2], s_network.panel_b_ip[3],
                 (unsigned)s_network.panel_b_port);
    } else {
        LOG_ERROR("config: network save FAILED");
    }
    return ok;
}
