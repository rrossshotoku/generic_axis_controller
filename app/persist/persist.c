/*
 * app/persist — non-volatile storage for CMC config + shots. See header.
 *
 * Layout matches the linker script's FLASH region cap (STM32G474RETX_FLASH.ld
 * LENGTH = 0x7E000, i.e. code stops at 0x0807E000). Persist sits above the
 * cap, in the last 8 KB of the G474RE's 512 KB flash — Bank 2 pages 252..255
 * in bsp/flash's linear numbering.
 *
 * Region addresses (must match STM32G474RETX_FLASH.ld's FLASH LENGTH cap):
 *   PERSIST_REGION_BOOT    : 0x0807D800 .. 0x0807DFFF  (linear page 251,     2 KB)
 *   PERSIST_REGION_SHOTS   : 0x0807E000 .. 0x0807EFFF  (linear pages 252-253, 4 KB)
 *   PERSIST_REGION_CONFIG  : 0x0807F000 .. 0x0807F7FF  (linear page 254,     2 KB)
 *   PERSIST_REGION_NETWORK : 0x0807F800 .. 0x0807FFFF  (linear page 255,     2 KB)
 *
 * Moved to the top of flash on the G431->G474 port so the app code region
 * (which grew past the old mid-flash persist addresses) can't overlap them
 * — the previous layout at 0x0801E000..0x0801FFFF was mid-Bank1 on the G474,
 * so a persist_save could erase active code and brick the unit until reflash.
 */

#include "persist.h"

#include "bsp/flash/flash.h"
#include "app/log/log.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Region map
 *---------------------------------------------------------------------------*/

typedef struct {
    uint32_t base_addr;
    uint32_t base_page;
    uint32_t num_pages;
    uint32_t max_payload;
} region_info_t;

static const region_info_t s_regions[PERSIST_REGION_COUNT] = {
    [PERSIST_REGION_CONFIG] = {
        .base_addr   = 0x0807F000u,
        .base_page   = 254u,
        .num_pages   = 1u,                  /* 2 KB; axis_manager config */
        .max_payload = PERSIST_CONFIG_MAX_BYTES,
    },
    [PERSIST_REGION_SHOTS]  = {
        .base_addr   = 0x0807E000u,
        .base_page   = 252u,
        .num_pages   = 2u,                  /* 4 KB; shot table */
        .max_payload = PERSIST_SHOTS_MAX_BYTES,
    },
    [PERSIST_REGION_NETWORK] = {
        .base_addr   = 0x0807F800u,
        .base_page   = 255u,
        .num_pages   = 1u,                  /* 2 KB; network config (IP etc) */
        .max_payload = PERSIST_NETWORK_MAX_BYTES,
    },
    [PERSIST_REGION_BOOT] = {
        .base_addr   = 0x0807D800u,
        .base_page   = 251u,
        .num_pages   = 1u,                  /* 2 KB; boot_meta stay-in-bootloader flag */
        .max_payload = PERSIST_BOOT_MAX_BYTES,
    },
};

/*----------------------------------------------------------------------------
 * On-flash header
 *---------------------------------------------------------------------------*/

#define PERSIST_MAGIC  0x54535250u   /* "PRST" little-endian */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t payload_size_bytes;
    uint32_t crc32;
} persist_header_t;

_Static_assert(sizeof(persist_header_t) == 16, "persist_header_t must be 16 bytes");

/*----------------------------------------------------------------------------
 * Simple CRC32 (bytewise IEEE 802.3, poly 0xEDB88320). Not the fastest
 * but no table required and good enough for small payloads. We use it
 * only for validation, not authentication.
 *---------------------------------------------------------------------------*/

static uint32_t crc32_calc(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1u));
        }
    }
    return ~crc;
}

/*----------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

void persist_init(void)
{
    /* Stateless for now. Logged so the boot trace shows we're alive. */
    LOG_INFO("persist: ready (config @0x%08lX, shots @0x%08lX)",
             (unsigned long)s_regions[PERSIST_REGION_CONFIG].base_addr,
             (unsigned long)s_regions[PERSIST_REGION_SHOTS ].base_addr);
}

bool persist_load(persist_region_t region,
                  void *out, size_t out_cap, uint16_t version,
                  size_t *out_size)
{
    if (region >= PERSIST_REGION_COUNT || out == NULL) return false;
    const region_info_t *r = &s_regions[region];

    const persist_header_t *hdr = (const persist_header_t *)r->base_addr;

    if (hdr->magic != PERSIST_MAGIC) {
        LOG_INFO("persist: region %u uninitialised (magic=0x%08lX)",
                 (unsigned)region, (unsigned long)hdr->magic);
        return false;
    }
    if (hdr->version != version) {
        LOG_WARN("persist: region %u version mismatch (on-flash=%u, expected=%u)",
                 (unsigned)region, (unsigned)hdr->version, (unsigned)version);
        return false;
    }
    if (hdr->payload_size_bytes == 0 || hdr->payload_size_bytes > r->max_payload) {
        LOG_WARN("persist: region %u bad payload size (%lu, max %lu)",
                 (unsigned)region, (unsigned long)hdr->payload_size_bytes,
                 (unsigned long)r->max_payload);
        return false;
    }
    if (hdr->payload_size_bytes > out_cap) {
        LOG_WARN("persist: region %u caller buffer too small (need %lu, got %lu)",
                 (unsigned)region, (unsigned long)hdr->payload_size_bytes,
                 (unsigned long)out_cap);
        return false;
    }

    const uint8_t *payload = (const uint8_t *)(r->base_addr + sizeof(persist_header_t));
    uint32_t calc_crc = crc32_calc(payload, hdr->payload_size_bytes);
    if (calc_crc != hdr->crc32) {
        LOG_WARN("persist: region %u CRC mismatch (on-flash=0x%08lX, calc=0x%08lX)",
                 (unsigned)region, (unsigned long)hdr->crc32, (unsigned long)calc_crc);
        return false;
    }

    memcpy(out, payload, hdr->payload_size_bytes);
    if (out_size) *out_size = hdr->payload_size_bytes;
    LOG_INFO("persist: region %u loaded (%lu B, v%u)",
             (unsigned)region, (unsigned long)hdr->payload_size_bytes,
             (unsigned)version);
    return true;
}

/* Pad a payload size up to the next 8-byte boundary so flash_program_buf
 * can write it (G4 only programs dword-aligned). */
static size_t round_up_dword(size_t n) { return (n + 7u) & ~(size_t)7u; }

bool persist_save(persist_region_t region,
                  const void *payload, size_t size, uint16_t version)
{
    if (region >= PERSIST_REGION_COUNT || payload == NULL) return false;
    const region_info_t *r = &s_regions[region];

    if (size == 0 || size > r->max_payload) {
        LOG_ERROR("persist: save region %u — bad size %lu (max %lu)",
                  (unsigned)region, (unsigned long)size,
                  (unsigned long)r->max_payload);
        return false;
    }

    /* Header. */
    persist_header_t hdr;
    hdr.magic              = PERSIST_MAGIC;
    hdr.version            = version;
    hdr.reserved           = 0;
    hdr.payload_size_bytes = (uint32_t)size;
    hdr.crc32              = crc32_calc(payload, size);

    /* Pad payload size up to dword (G4 program granularity). The trailing
     * bytes beyond `size` end up as 0xFF (un-programmed flash) — that's
     * fine because the header records the real size and the CRC is over
     * exactly `size` bytes. */
    size_t prog_size_payload = round_up_dword(size);

    if (!flash_unlock()) {
        LOG_ERROR("persist: flash_unlock failed");
        return false;
    }

    bool ok = true;
    for (uint32_t i = 0; i < r->num_pages && ok; i++) {
        if (!flash_erase_page(r->base_page + i)) {
            LOG_ERROR("persist: erase page %lu failed",
                      (unsigned long)(r->base_page + i));
            ok = false;
        }
    }

    if (ok) {
        /* Program header first. */
        if (!flash_program_buf(r->base_addr, &hdr, sizeof(hdr))) {
            LOG_ERROR("persist: header write failed");
            ok = false;
        }
    }
    if (ok && prog_size_payload > 0) {
        /* Program the payload (we need an intermediate aligned buffer if
         * size != prog_size_payload, but most payloads end on a dword. We
         * handle the partial tail by staging into a small stack buffer.) */
        size_t full = size & ~(size_t)7u;
        if (full > 0) {
            if (!flash_program_buf(r->base_addr + sizeof(hdr),
                                   payload, full)) {
                LOG_ERROR("persist: payload (aligned part) write failed");
                ok = false;
            }
        }
        if (ok && full < size) {
            uint8_t tail[8];
            memset(tail, 0xFF, sizeof(tail));   /* match erased-state */
            memcpy(tail, (const uint8_t *)payload + full, size - full);
            if (!flash_program_dword(r->base_addr + sizeof(hdr) + full,
                                     *(const uint64_t *)tail)) {
                LOG_ERROR("persist: payload tail write failed");
                ok = false;
            }
        }
    }

    flash_lock();

    if (ok) {
        LOG_INFO("persist: region %u saved (%lu B, v%u)",
                 (unsigned)region, (unsigned long)size, (unsigned)version);
    }
    return ok;
}
