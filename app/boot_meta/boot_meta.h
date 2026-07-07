/*
 * app/boot_meta — persistent "stay in bootloader on next boot" flag.
 *
 * Backs the app side of the dual-bootloader handshake
 * (Documentation/dual_bootloader_design.md §5.3). A single 32-bit magic
 * lives in PERSIST_REGION_BOOT (page 251, 0x0807D800). Both the app and
 * the (future) bootloader read the same field via the persist module.
 *
 * Flag states:
 *   0xB007107D   STAY   — app requested update; bootloader must stay in
 *                         bootloader mode on next boot and wait for a
 *                         segmented firmware push.
 *   0x00000000   CLEAR  — app confirmed healthy running; bootloader
 *                         validates + jumps to app on next boot.
 *   any other    ~CLEAR — treated as CLEAR (persist blob missing / CRC
 *                         mismatch / never written). Safe default: run
 *                         the app.
 *
 * Sequence:
 *   1. PC tool writes 0x1F51:1 = MC_IF_PROG_START to the app.
 *   2. app/od dispatches to boot_meta_enter_bootloader():
 *      - persist_save(BOOT, STAY_MAGIC),
 *      - NVIC_SystemReset().
 *   3. Bootloader boots, reads the flag, sees STAY, waits for downloads.
 *   4. Bootloader flashes the new app + jumps to it.
 *   5. New app boots. After BOOT_HEALTHY_MS of no-fault operation the
 *      main-loop tick clears the flag (writes CLEAR_MAGIC).
 *   6. Any subsequent power-cycle without step 1 → bootloader sees CLEAR
 *      → validates + jumps to app.
 *
 * Recovery: if the new app crashes before step 5, the flag stays STAY.
 * Next boot re-enters bootloader → operator can retry the update. This
 * is the brick-proof guarantee that motivates using flash (not a RAM
 * marker) for the flag.
 */

#ifndef APP_BOOT_META_H
#define APP_BOOT_META_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot flag magic — persistence layer wraps this in a persist blob so
 * the on-flash bytes aren't just the raw magic. The values are exposed
 * for the bootloader binary to reference via the same header when it
 * lands in Phase 2. */
#define BOOT_META_STAY_MAGIC   (0xB007107Du)
#define BOOT_META_CLEAR_MAGIC  (0x00000000u)

/* How long the app must run without fault before it declares itself
 * healthy and clears the boot flag. Set long enough to survive normal
 * startup (network link-up, W6100 init, first cyclic exchange with the
 * motor) but short enough that a genuine crash loop is caught within a
 * few boot cycles. */
#define BOOT_META_HEALTHY_MS   (5000u)

/* Read the flag from flash. Called once at boot. Populates internal
 * state that the tick and getter use. Safe to call even if the persist
 * blob is missing / corrupt — treats those as CLEAR. */
void boot_meta_init(void);

/* Was the flag STAY at boot? Useful for logging + PC-tool observability. */
bool boot_meta_flag_was_set_at_boot(void);

/* Tick called from the main loop. After BOOT_META_HEALTHY_MS of runtime,
 * if the flag was set at boot, clears it (persist_save with CLEAR_MAGIC).
 * No-op once the flag has been cleared this boot, or if the flag was
 * already CLEAR at boot (avoids gratuitous flash wear). */
void boot_meta_tick(void);

/* Set the flag STAY and reboot. Called by app/od when the PC tool writes
 * 0x1F51:1 = MC_IF_PROG_START. Blocks for one page write (~30 ms) then
 * NVIC_SystemReset() — this function does not return.
 *
 * Best-effort: if the flash write fails (very unlikely), logs an error
 * and returns without resetting so the caller can send a NOT_READY back
 * to the PC tool. */
void boot_meta_enter_bootloader(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BOOT_META_H */
