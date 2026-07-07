/*
 * app/boot_meta — see boot_meta.h for the contract.
 */

#include "boot_meta.h"

#include "app/log/log.h"
#include "app/persist/persist.h"
#include "bsp/time/time.h"

#include "stm32g4xx_hal.h"    /* NVIC_SystemReset */

#include <string.h>

/* On-flash blob shape. Wrapped in a persist blob so the CRC + version
 * checks come free. Version starts at 1; future fields (last-good-CRC,
 * boot-count, etc.) bump it. */
#define BOOT_META_BLOB_VERSION  1u
typedef struct __attribute__((packed)) {
    uint32_t magic;    /* BOOT_META_STAY_MAGIC or BOOT_META_CLEAR_MAGIC */
} boot_meta_blob_t;

static bool     s_flag_was_set;         /* snapshot of the flag at boot */
static bool     s_cleared_this_boot;    /* have we written CLEAR_MAGIC yet? */
static uint32_t s_boot_ms;              /* time_ms() at boot_meta_init */

/* Write the blob. Returns true on success. */
static bool write_flag(uint32_t magic)
{
    boot_meta_blob_t blob = { .magic = magic };
    return persist_save(PERSIST_REGION_BOOT, &blob, sizeof(blob), BOOT_META_BLOB_VERSION);
}

/* Read the blob. Returns the on-flash magic, or CLEAR_MAGIC on any failure
 * (missing region / CRC mismatch / wrong version — all safe defaults that
 * boot into the app). */
static uint32_t read_flag(void)
{
    boot_meta_blob_t blob = {0};
    size_t got = 0;
    if (!persist_load(PERSIST_REGION_BOOT, &blob, sizeof(blob),
                      BOOT_META_BLOB_VERSION, &got)) {
        return BOOT_META_CLEAR_MAGIC;
    }
    if (got != sizeof(blob)) return BOOT_META_CLEAR_MAGIC;
    return blob.magic;
}

void boot_meta_init(void)
{
    uint32_t magic = read_flag();
    s_flag_was_set      = (magic == BOOT_META_STAY_MAGIC);
    s_cleared_this_boot = false;
    s_boot_ms           = time_ms();
    LOG_INFO("boot_meta: flag at boot = %s (magic=0x%08lX)",
             s_flag_was_set ? "STAY" : "CLEAR",
             (unsigned long)magic);
}

bool boot_meta_flag_was_set_at_boot(void)
{
    return s_flag_was_set;
}

void boot_meta_tick(void)
{
    /* Nothing to do if the flag was already CLEAR or we've cleared it
     * this session — avoids gratuitous flash writes. */
    if (s_cleared_this_boot)   return;
    if (!s_flag_was_set)       return;
    /* Wait for BOOT_META_HEALTHY_MS of runtime before declaring healthy.
     * If we crash before that, the flag stays STAY and the next boot
     * re-enters bootloader — brick-proof property (see boot_meta.h). */
    if (time_elapsed_ms(s_boot_ms) < BOOT_META_HEALTHY_MS) return;

    if (write_flag(BOOT_META_CLEAR_MAGIC)) {
        s_cleared_this_boot = true;
        LOG_INFO("boot_meta: app healthy after %u ms — boot flag cleared",
                 (unsigned)BOOT_META_HEALTHY_MS);
    } else {
        /* Flash write failed — retry next tick (may be transient). Don't
         * mark cleared, so we'll try again. Bounded by flash-erase cost
         * per attempt; if it never succeeds the operator sees the flag
         * still-STAY in the log at every next boot until the flash is
         * repaired. */
        LOG_WARN("boot_meta: flash write failed clearing boot flag — will retry");
    }
}

void boot_meta_enter_bootloader(void)
{
    LOG_INFO("boot_meta: entering bootloader — setting STAY flag then resetting");
    if (!write_flag(BOOT_META_STAY_MAGIC)) {
        LOG_ERROR("boot_meta: flash write failed setting STAY flag — reset aborted, PC tool should retry");
        return;
    }
    /* Small delay to give the log TCP + response UDP a moment to drain
     * from the W6100's TX buffer before we cut the power to the CPU
     * mid-frame. Not strictly necessary — the PC tool retries — but the
     * log message above is helpful for debug. */
    uint32_t deadline = time_ms() + 50u;
    while (time_ms() < deadline) { /* spin */ }
    NVIC_SystemReset();
    /* not reached */
}
