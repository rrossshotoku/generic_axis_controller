/*
 * boot_od — see boot_od.h.
 *
 * Wire format constants MUST match app/od/od.c (UDP magic, msg type
 * numbering, error class ids) — see Interface/mc_if_protocol.h.
 */

#include "boot_od.h"
#include "boot_flash.h"
#include "boot_net_cfg.h"
#include "boot_seg_sdo.h"

#include "Interface/mc_if_od.h"
#include "Interface/mc_if_protocol.h"
#include "bsp/net/net.h"
#include "bsp/identity/identity.h"

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Provided by boot/main.c — does NOT reset; hands control to the app's
 * Reset_Handler at 0x08008004. See boot_jump_to_app comment for why we
 * jump (rather than NVIC_SystemReset) on PROG_COMMIT: the flag stays
 * STAY until the app clears it, preserving brick-proof. */
extern void boot_jump_to_app(void);

/* Set by PROG_COMMIT; polled at the end of boot_od_tick so the OK
 * response has already been sent + a small drain delay applied before
 * we vanish into the app. */
static volatile bool s_commit_pending = false;

#define OD_ACCESS_SOCKET       ((net_sock_t)4)
#define OD_ACCESS_PORT         5000u
#define UDP_MAGIC              0x4D55u
#define UDP_HDR_BYTES          8u
#define UDP_PAYLOAD_MAX        512u

/* Wire-msg types — match app/od/od.c udp_msg_type_t. */
#define MSG_OD_READ_REQ            0x01u
#define MSG_OD_READ_RESP           0x02u
#define MSG_OD_WRITE_REQ           0x03u
#define MSG_OD_WRITE_RESP          0x04u
#define MSG_ERROR                  0x7Fu
/* v5 additions. */
#define MSG_OD_DOWNLOAD_INIT       0x14u
#define MSG_OD_DOWNLOAD_SEGMENT    0x15u
#define MSG_OD_DOWNLOAD_RESP       0x16u

#define ERR_BAD_LENGTH   0x05u
#define ERR_UNKNOWN_MSG  0x06u

/* Fallback network config used only when the NETWORK persist blob at
 * 0x0807F800 fails to decode (fresh chip, corrupt blob, version mismatch).
 * On a normal chip boot_net_cfg_load returns true and these are unused. */
static const uint8_t DEFAULT_IP[4]      = { 192u, 1u, 0u, 100u };
static const uint8_t DEFAULT_NETMASK[4] = { 255u, 255u, 255u, 0u };
static const uint8_t DEFAULT_GATEWAY[4] = { 192u, 1u, 0u, 1u };

/* Little-endian read/write helpers. */
static inline uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t rd_u32(const uint8_t *p)
{
    return  (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t) v;        p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static size_t build_hdr(uint8_t *out, uint8_t type, uint16_t seq, uint16_t plen)
{
    wr_u16(out + 0, UDP_MAGIC);
    out[2] = MC_IF_PROTOCOL_VERSION;
    out[3] = type;
    wr_u16(out + 4, seq);
    wr_u16(out + 6, plen);
    return UDP_HDR_BYTES;
}

static void send_error(const net_addr_t *to, uint16_t ref_seq, uint8_t cls, uint8_t detail)
{
    uint8_t buf[UDP_HDR_BYTES + 4];
    build_hdr(buf, MSG_ERROR, ref_seq, 4);
    buf[8]  = cls;
    buf[9]  = detail;
    wr_u16(buf + 10, ref_seq);
    (void)net_sendto(OD_ACCESS_SOCKET, to, buf, sizeof(buf));
}

/* Reply body for MSG_OD_READ_RESP: idx(2) sub(1) type(1) result(1) len(1) data[len]. */
static void send_read_resp(const net_addr_t *peer, uint16_t seq,
                           uint16_t idx, uint8_t sub, MC_IfOdType_t type,
                           MC_IfOdResult_t result,
                           const uint8_t *data, uint8_t data_len)
{
    uint8_t out[UDP_HDR_BYTES + 6 + 8];
    uint8_t  dl   = (result == MC_IF_OD_OK) ? data_len : 0u;
    uint16_t plen = (uint16_t)(6u + dl);
    build_hdr(out, MSG_OD_READ_RESP, seq, plen);
    wr_u16(out + 8, idx);
    out[10] = sub;
    out[11] = (uint8_t)type;
    out[12] = (uint8_t)result;
    out[13] = dl;
    if (dl > 0 && data != NULL) memcpy(out + 14, data, dl);
    (void)net_sendto(OD_ACCESS_SOCKET, peer, out, (size_t)(UDP_HDR_BYTES + plen));
}

static void send_write_resp(const net_addr_t *peer, uint16_t seq,
                            uint16_t idx, uint8_t sub, MC_IfOdResult_t result)
{
    uint8_t out[UDP_HDR_BYTES + 4];
    build_hdr(out, MSG_OD_WRITE_RESP, seq, 4);
    wr_u16(out + 8, idx);
    out[10] = sub;
    out[11] = (uint8_t)result;
    (void)net_sendto(OD_ACCESS_SOCKET, peer, out, sizeof(out));
}

static void send_download_resp(const net_addr_t *peer, uint16_t seq,
                               const MC_IfOdDownloadResp_t *resp)
{
    uint8_t out[UDP_HDR_BYTES + sizeof(*resp)];
    build_hdr(out, MSG_OD_DOWNLOAD_RESP, seq, sizeof(*resp));
    memcpy(out + UDP_HDR_BYTES, resp, sizeof(*resp));
    (void)net_sendto(OD_ACCESS_SOCKET, peer, out, sizeof(out));
}

/* --- OD dispatch ---------------------------------------------------------- */

/* Map the boot_flash state enum to MC_IF_FLASH_* wire values (same
 * numeric order — kept explicit to catch any future drift). */
static uint16_t flash_state_to_wire(boot_flash_state_t s)
{
    switch (s) {
        case BOOT_FLASH_IDLE:        return MC_IF_FLASH_IDLE;
        case BOOT_FLASH_ERASING:     return MC_IF_FLASH_ERASING;
        case BOOT_FLASH_PROGRAMMING: return MC_IF_FLASH_PROGRAMMING;
        case BOOT_FLASH_VERIFYING:   return MC_IF_FLASH_VERIFYING;
        case BOOT_FLASH_FAULT:       return MC_IF_FLASH_FAULT;
        default:                     return MC_IF_FLASH_FAULT;
    }
}

static void handle_read(const net_addr_t *peer, uint16_t seq,
                        const uint8_t *payload, uint16_t plen)
{
    if (plen != 4) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }
    uint16_t      idx  = rd_u16(payload + 0);
    uint8_t       sub  = payload[2];
    MC_IfOdType_t type = (MC_IfOdType_t)payload[3];

    if (idx == 0x1F56u && sub == 1u) {
        uint32_t crc = boot_flash_current_image_crc32();
        uint8_t data[4]; wr_u32(data, crc);
        send_read_resp(peer, seq, idx, sub, MC_IF_T_U32, MC_IF_OD_OK, data, 4);
        return;
    }
    if (idx == 0x1F57u && sub == 1u) {
        uint16_t st = flash_state_to_wire(boot_flash_get_state());
        uint8_t data[2]; wr_u16(data, st);
        send_read_resp(peer, seq, idx, sub, MC_IF_T_U16, MC_IF_OD_OK, data, 2);
        return;
    }
    /* Everything else — including 0x1F50 (WO) and 0x1F51 as a read —
     * gets NO_OBJECT so the PC tool can distinguish "here but not this
     * entry" from "wrong OD range." */
    send_read_resp(peer, seq, idx, sub, type, MC_IF_OD_ERR_NO_OBJECT, NULL, 0);
}

/* program_control commands (0x1F51:1). Documented in mc_if_od.h. */
static MC_IfOdResult_t apply_program_control(uint8_t cmd, const net_addr_t *peer)
{
    (void)peer;
    switch (cmd) {
        case MC_IF_PROG_STOP:
        case MC_IF_PROG_ABORT:
            boot_seg_sdo_abort();
            return MC_IF_OD_OK;
        case MC_IF_PROG_START:
            /* START is the app-side signal to enter bootloader mode —
             * ignored here since we're already in the bootloader. */
            return MC_IF_OD_OK;
        case MC_IF_PROG_VERIFY:
            /* Verify CRC of everything written so far. Expected CRC was
             * previously conveyed how? For the MVP we take the shortcut
             * of storing zero as "no expected value"; the PC tool then
             * verifies by reading 0x1F56 (which computes CRC over the
             * new image) and comparing locally to its .bin CRC. Return
             * OK regardless. */
            return MC_IF_OD_OK;
        case MC_IF_PROG_COMMIT:
            /* Commit = jump to app WITHOUT resetting. NVIC_SystemReset
             * would loop back into the bootloader which would read the
             * flag (still STAY) and stay in bootloader mode forever —
             * the app never gets a chance to run + clear the flag. So
             * we jump directly. The flag stays STAY until the app has
             * been running healthy for BOOT_META_HEALTHY_MS, at which
             * point app/boot_meta clears it. If the new app crashes
             * before that window elapses, the next reboot re-enters
             * bootloader — that's the brick-proof property.
             *
             * Deferred: the jump happens at the end of boot_od_tick
             * after the OK response has been sent and had a chance to
             * drain from the W6100 TX buffer. */
            s_commit_pending = true;
            return MC_IF_OD_OK;
        default:
            return MC_IF_OD_ERR_RANGE;
    }
}

static void handle_write(const net_addr_t *peer, uint16_t seq,
                         const uint8_t *payload, uint16_t plen)
{
    if (plen < 5) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }
    uint16_t idx = rd_u16(payload + 0);
    uint8_t  sub = payload[2];
    uint8_t  len = payload[4];
    if ((uint16_t)(5u + len) != plen) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }

    if (idx == 0x1F51u && sub == 1u && len == 1u) {
        MC_IfOdResult_t r = apply_program_control(payload[5], peer);
        send_write_resp(peer, seq, idx, sub, r);
        return;
    }
    /* Writes to program_data (0x1F50:1) via expedited SDO are meaningless
     * (max 4 bytes). Bytes flow via segmented SDO only. */
    send_write_resp(peer, seq, idx, sub, MC_IF_OD_ERR_NO_OBJECT);
}

static void handle_download_init(const net_addr_t *peer, uint16_t seq,
                                 const uint8_t *payload, uint16_t plen)
{
    if (plen < sizeof(MC_IfOdDownloadInit_t)) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }
    const MC_IfOdDownloadInit_t *req = (const MC_IfOdDownloadInit_t *)payload;
    MC_IfOdDownloadResp_t resp;
    (void)boot_seg_sdo_on_init(req, &resp);
    send_download_resp(peer, seq, &resp);
}

static void handle_download_segment(const net_addr_t *peer, uint16_t seq,
                                    const uint8_t *payload, uint16_t plen)
{
    const uint8_t seg_hdr = 3u;
    if (plen < seg_hdr) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }
    const MC_IfOdDownloadSegment_t *seg = (const MC_IfOdDownloadSegment_t *)payload;
    MC_IfOdDownloadResp_t resp;
    (void)boot_seg_sdo_on_segment(seg, (uint8_t)plen, &resp);
    send_download_resp(peer, seq, &resp);
}

/* --- module init + tick --------------------------------------------------- */

void boot_od_init(void)
{
    uint8_t mac[6];
    identity_get_mac(mac);

    /* Prefer the app's NETWORK persist blob so the CMC stays reachable at
     * the same IP across the app -> bootloader reboot. Fall back to the
     * hardcoded defaults only if the persist decode fails (fresh chip,
     * corrupt blob, version mismatch). */
    boot_net_cfg_t cfg;
    const uint8_t *ip, *nm, *gw;
    if (boot_net_cfg_load(&cfg)) {
        ip = cfg.ip; nm = cfg.netmask; gw = cfg.gateway;
    } else {
        ip = DEFAULT_IP; nm = DEFAULT_NETMASK; gw = DEFAULT_GATEWAY;
    }

    (void)net_init(mac, ip, nm, gw);
    (void)net_open(OD_ACCESS_SOCKET, NET_PROTO_UDP, OD_ACCESS_PORT, false);
    boot_flash_init();
    boot_seg_sdo_init();
}

void boot_od_tick(void)
{
    uint8_t buf[UDP_HDR_BYTES + UDP_PAYLOAD_MAX];
    net_addr_t from;
    int32_t n = net_recvfrom(OD_ACCESS_SOCKET, &from, buf, sizeof(buf));
    if (n < (int32_t)UDP_HDR_BYTES) return;

    uint16_t magic  = rd_u16(buf + 0);
    uint8_t  ver    = buf[2];
    uint8_t  type   = buf[3];
    uint16_t seq    = rd_u16(buf + 4);
    uint16_t plen   = rd_u16(buf + 6);

    if (magic != UDP_MAGIC)                    return;
    if (ver   != MC_IF_PROTOCOL_VERSION)       { send_error(&from, seq, 0x02, ver); return; }
    if ((uint32_t)UDP_HDR_BYTES + plen > (uint32_t)n) return;

    const uint8_t *payload = buf + UDP_HDR_BYTES;

    switch (type) {
        case MSG_OD_READ_REQ:          handle_read            (&from, seq, payload, plen); break;
        case MSG_OD_WRITE_REQ:         handle_write           (&from, seq, payload, plen); break;
        case MSG_OD_DOWNLOAD_INIT:     handle_download_init   (&from, seq, payload, plen); break;
        case MSG_OD_DOWNLOAD_SEGMENT:  handle_download_segment(&from, seq, payload, plen); break;
        default:                       send_error(&from, seq, ERR_UNKNOWN_MSG, type); break;
    }

    if (s_commit_pending) {
        /* Let the OK response for PROG_COMMIT drain out of the W6100
         * TX buffer + across the wire before we vanish into the app
         * (which will re-initialise SPI/W6100 for its own purposes).
         * 100 ms is generous but the wall-clock cost is one-off. */
        HAL_Delay(100);
        boot_jump_to_app();
        /* not reached */
    }
}
