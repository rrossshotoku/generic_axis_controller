/*
 * boot_flash — write the incoming firmware image to flash + verify it.
 *
 * Owns the app flash region 0x08008000..0x0807D7FF (470 KB, pages 4..250
 * in bsp/flash linear numbering). Callers push doubleword-aligned chunks
 * via boot_flash_write; boot_flash_verify computes the CRC32 across all
 * bytes that have been accepted and compares to the operator-supplied
 * expected value (the same CRC the PC tool used to short-circuit
 * "already up to date").
 *
 * State machine (mirror of MC_IF_FLASH_* in Interface/mc_if_od.h):
 *   IDLE       — no session, ready to accept PROG_START.
 *   ERASING    — inside boot_flash_begin.
 *   PROGRAMMING— session live, accepting bytes.
 *   VERIFYING  — inside boot_flash_verify.
 *   FAULT      — one of the operations failed; PROG_ABORT resets to IDLE.
 */

#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    BOOT_FLASH_IDLE        = 0,
    BOOT_FLASH_ERASING     = 1,
    BOOT_FLASH_PROGRAMMING = 2,
    BOOT_FLASH_VERIFYING   = 3,
    BOOT_FLASH_FAULT       = 4,
} boot_flash_state_t;

/* Boundaries of the region we'll erase + program. Matches the app's
 * FLASH region in STM32G474RETX_FLASH.ld. */
#define BOOT_FLASH_APP_BASE       0x08008000u
#define BOOT_FLASH_APP_END        0x0807D800u   /* one past last programmable byte */
#define BOOT_FLASH_APP_MAX_LEN    (BOOT_FLASH_APP_END - BOOT_FLASH_APP_BASE)

void               boot_flash_init(void);
boot_flash_state_t boot_flash_get_state(void);

/* Begin a session: erase every page in the app region, seed the write
 * cursor at APP_BASE, transition to PROGRAMMING. Blocking (~5 s worst
 * case for 245 page erases). */
bool boot_flash_begin(uint32_t total_bytes);

/* Append n bytes to the running write. n need not be dword-aligned;
 * unaligned tail bytes are buffered until 8 accumulate. Returns false
 * and transitions to FAULT on any HAL error / range overflow. */
bool boot_flash_write(const uint8_t *src, size_t n);

/* Compute CRC32 over the bytes written so far and compare to expected.
 * On success returns to IDLE (ready for another session if the operator
 * wants to retry). On mismatch transitions to FAULT — same as any other
 * failure; PROG_ABORT clears it. */
bool boot_flash_verify(uint32_t expected_crc32);

/* Compute CRC32 over the currently-installed app image (BASE .. cursor
 * where cursor = written bytes if a session is open, else the whole
 * region up to APP_END). Used to answer OD reads of 0x1F56
 * program_software_id. */
uint32_t boot_flash_current_image_crc32(void);

/* Abort a session: forget cursor + state, back to IDLE. Does not erase. */
void boot_flash_abort(void);

#endif
