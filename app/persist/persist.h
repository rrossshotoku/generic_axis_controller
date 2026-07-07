/*
 * app/persist — non-volatile storage for CMC config + shots.
 *
 * Sits on top of bsp/flash. Owns two flash regions reserved at the top
 * of the chip's flash (see STM32G431RBTX_FLASH.ld):
 *   region 0 (PERSIST_REGION_CONFIG) — config, 1 page (2 KB) at 0x0801F000
 *   region 1 (PERSIST_REGION_SHOTS)  — shots,  2 pages (4 KB) at 0x0801E000
 *
 * Region layout:
 *   [ persist_header_t (16 B) ][ payload (size B, zero-padded to dword) ]
 *
 *   persist_header_t {
 *       magic     = "PRST" (0x54535250 LE)
 *       version   = caller-defined (lets caller bump and reject mismatched)
 *       payload_size_bytes
 *       crc32     of the payload (Castagnoli-style, simple bytewise)
 *   }
 *
 * Save flow:
 *   1. Caller hands persist a contiguous buffer to write.
 *   2. persist_save erases the region's page(s), then programs header +
 *      payload + zero-pad.
 *   3. Returns false on any flash error.
 *
 * Load flow:
 *   1. Caller calls persist_load(region, payload_out, payload_out_cap,
 *                                version, *out_size).
 *   2. persist_load reads the in-flash header, checks magic + version + size.
 *   3. CRC-validates the payload.
 *   4. On success copies payload bytes to caller's buffer, returns true.
 *   5. On any failure returns false; caller must fall back to defaults.
 *
 * Caller owns the schema (struct layout) for each region. persist treats
 * the payload as opaque bytes; only checks magic/version/size/CRC.
 */

#ifndef APP_PERSIST_H
#define APP_PERSIST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PERSIST_REGION_CONFIG  = 0,    /* axis_manager: joystick cal + motion limits */
    PERSIST_REGION_SHOTS   = 1,    /* 100-slot CAMERAD shot table */
    PERSIST_REGION_NETWORK = 2,    /* IP / netmask / gateway / device_no */
    PERSIST_REGION_BOOT    = 3,    /* boot_meta: "stay in bootloader on next boot" flag */
    PERSIST_REGION_COUNT
} persist_region_t;

/* Maximum payload size each region can hold (region size minus 16-byte
 * header, minus a small safety margin). */
#define PERSIST_CONFIG_MAX_BYTES   (2048u - 16u)
#define PERSIST_SHOTS_MAX_BYTES    (4096u - 16u)
#define PERSIST_NETWORK_MAX_BYTES  (2048u - 16u)
#define PERSIST_BOOT_MAX_BYTES     (2048u - 16u)

/* No-op init for now — bsp/flash has no module-level state and the
 * persist module is stateless. Reserved for future (e.g. preloading
 * a cache). */
void persist_init(void);

/* Load a region's payload into `out`. Returns:
 *   true  — header is valid, CRC matches, payload copied. `*out_size`
 *           is set to the actual payload size (≤ caller's `out_cap`).
 *   false — region is uninitialised, corrupted, wrong version, or
 *           wrong size for the caller. Caller falls back to defaults.
 *
 * `version` is the schema version the caller expects. A mismatch
 * causes load to return false (so callers can bump the schema and
 * gracefully ignore stale on-flash data). */
bool persist_load(persist_region_t region,
                  void *out, size_t out_cap, uint16_t version,
                  size_t *out_size);

/* Save a region's payload. Erases the region, then programs
 * header + payload. `version` is stored alongside. Returns true on
 * success.
 *
 * Blocks for the duration of the erase + program (~30 ms for a config
 * page, ~60 ms for the two shot pages). Do not call from a hot path. */
bool persist_save(persist_region_t region,
                  const void *payload, size_t size, uint16_t version);

#ifdef __cplusplus
}
#endif

#endif /* APP_PERSIST_H */
