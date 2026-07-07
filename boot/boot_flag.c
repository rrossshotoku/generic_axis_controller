/*
 * boot_flag — see boot_flag.h. Reads the persistent flag without linking
 * app/persist (which would drag axis_manager etc. into the bootloader).
 * The persist header format is duplicated here — keep in sync with
 * app/persist/persist.c if the header layout ever changes.
 */

#include "boot_flag.h"

#include <stdint.h>
#include <string.h>

/* Must match app/persist/persist.c PERSIST_MAGIC + persist_header_t. */
#define PERSIST_MAGIC              0x54535250u  /* "PRST" LE */
#define BOOT_META_STAY_MAGIC       0xB007107Du  /* see app/boot_meta.h */
#define BOOT_META_BLOB_VERSION     1u

#define BOOT_PERSIST_BASE_ADDR     0x0807D800u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t payload_size_bytes;
    uint32_t crc32;
} persist_header_t;

_Static_assert(sizeof(persist_header_t) == 16, "persist header must be 16 B");

/* Same bytewise CRC32 as app/persist/persist.c. Duplicated to keep the
 * bootloader self-contained; no perf concern given a 4-byte payload. */
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

bool boot_flag_is_stay(void)
{
    const persist_header_t *hdr =
        (const persist_header_t *)BOOT_PERSIST_BASE_ADDR;
    if (hdr->magic != PERSIST_MAGIC)                 return false;
    if (hdr->version != BOOT_META_BLOB_VERSION)      return false;
    if (hdr->payload_size_bytes != sizeof(uint32_t)) return false;

    const uint8_t *payload = (const uint8_t *)(BOOT_PERSIST_BASE_ADDR + sizeof(persist_header_t));
    uint32_t crc = crc32_calc(payload, sizeof(uint32_t));
    if (crc != hdr->crc32) return false;

    uint32_t magic;
    memcpy(&magic, payload, sizeof(magic));
    return magic == BOOT_META_STAY_MAGIC;
}
