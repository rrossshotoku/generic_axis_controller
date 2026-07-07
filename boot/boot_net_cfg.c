/*
 * boot_net_cfg — see boot_net_cfg.h.
 *
 * Same "duplicate the format to keep the bootloader independent" pattern
 * as boot_flag.c. Persist header + CRC32 must match app/persist/persist.c;
 * network_persist_blob_t must match app/config/config.c.
 */

#include "boot_net_cfg.h"

#include <stdint.h>
#include <string.h>

/* Must match app/persist/persist.c. */
#define PERSIST_MAGIC              0x54535250u  /* "PRST" LE */

/* Must match app/config/config.c. */
#define NETWORK_PERSIST_VERSION    2u
#define NETWORK_PERSIST_MAGIC      0x4E455457u  /* "NETW" LE */
#define NETWORK_PERSIST_ADDR       0x0807F800u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t payload_size_bytes;
    uint32_t crc32;
} persist_header_t;

_Static_assert(sizeof(persist_header_t) == 16, "persist header must be 16 B");

typedef struct __attribute__((packed)) {
    uint32_t magic;             /* 4  — "NETW" */
    uint8_t  ip[4];             /* 4  */
    uint8_t  netmask[4];        /* 4  */
    uint8_t  gateway[4];        /* 4  */
    uint32_t cmc_device_no;     /* 4  */
    uint32_t tcp_camerad_port;  /* 4  */
    uint8_t  panel_a_ip[4];     /* 4  */
    uint32_t panel_b_port;      /* 4  */
    uint8_t  panel_b_ip[4];     /* 4  */
    uint32_t reserved[3];       /* 12 */
} network_persist_blob_t;

_Static_assert(sizeof(network_persist_blob_t) == 48, "network blob layout drift");

/* Same bytewise CRC32 as boot_flag.c / app/persist/persist.c. */
static uint32_t crc32_calc(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1u));
        }
    }
    return ~crc;
}

bool boot_net_cfg_load(boot_net_cfg_t *out)
{
    if (out == NULL) return false;

    const persist_header_t *hdr = (const persist_header_t *)NETWORK_PERSIST_ADDR;
    if (hdr->magic != PERSIST_MAGIC)                                return false;
    if (hdr->version != NETWORK_PERSIST_VERSION)                    return false;
    if (hdr->payload_size_bytes != sizeof(network_persist_blob_t))  return false;

    const uint8_t *payload_bytes = (const uint8_t *)(NETWORK_PERSIST_ADDR + sizeof(persist_header_t));
    if (crc32_calc(payload_bytes, hdr->payload_size_bytes) != hdr->crc32) return false;

    network_persist_blob_t blob;
    memcpy(&blob, payload_bytes, sizeof(blob));
    if (blob.magic != NETWORK_PERSIST_MAGIC) return false;

    memcpy(out->ip,      blob.ip,      4);
    memcpy(out->netmask, blob.netmask, 4);
    memcpy(out->gateway, blob.gateway, 4);
    return true;
}
