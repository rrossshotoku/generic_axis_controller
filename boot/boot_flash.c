/*
 * boot_flash — see boot_flash.h.
 *
 * Uses bsp/flash primitives (which the bootloader links). The doubleword
 * accumulator handles the case where segments arrive that don't neatly
 * fall on 8-byte boundaries (segment payload is up to ~49 bytes today
 * so most will straddle).
 */

#include "boot_flash.h"

#include "bsp/flash/flash.h"

#include <string.h>

/* Linear page indices for the app region (mirrors bsp/flash's 0..255
 * numbering across both banks). APP_BASE 0x08008000 -> linear page 16;
 * APP_END 0x0807D800 -> linear page 251 (exclusive). */
#define APP_FIRST_PAGE  16u
#define APP_LAST_PAGE   250u   /* inclusive */

static boot_flash_state_t s_state;
static uint32_t           s_cursor;         /* next byte address to write */
static uint32_t           s_total_expected; /* total_bytes from PROG_START */
static uint32_t           s_bytes_written;  /* bytes actually programmed */

/* Doubleword accumulator — segment tail bytes buffer here until we have
 * 8 to program. */
static uint8_t            s_dw_buf[8];
static uint8_t            s_dw_fill;

/* CRC32 (IEEE 802.3, polynomial 0xEDB88320) — bytewise. Same as
 * app/persist/persist.c and boot/boot_flag.c. Not the fastest but no
 * table required and good enough at update-flow speeds. */
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

void boot_flash_init(void)
{
    s_state         = BOOT_FLASH_IDLE;
    s_cursor        = BOOT_FLASH_APP_BASE;
    s_total_expected = 0u;
    s_bytes_written = 0u;
    s_dw_fill       = 0u;
    memset(s_dw_buf, 0xFFu, sizeof(s_dw_buf));
}

boot_flash_state_t boot_flash_get_state(void) { return s_state; }

bool boot_flash_begin(uint32_t total_bytes)
{
    if (s_state != BOOT_FLASH_IDLE && s_state != BOOT_FLASH_FAULT) return false;
    if (total_bytes == 0u || total_bytes > BOOT_FLASH_APP_MAX_LEN) {
        s_state = BOOT_FLASH_FAULT;
        return false;
    }

    s_state = BOOT_FLASH_ERASING;
    if (!flash_unlock()) { s_state = BOOT_FLASH_FAULT; return false; }
    for (uint32_t p = APP_FIRST_PAGE; p <= APP_LAST_PAGE; p++) {
        if (!flash_erase_page(p)) {
            flash_lock();
            s_state = BOOT_FLASH_FAULT;
            return false;
        }
    }
    flash_lock();   /* re-locked between phases; PROGRAMMING unlocks again */

    s_cursor        = BOOT_FLASH_APP_BASE;
    s_bytes_written = 0u;
    s_total_expected = total_bytes;
    s_dw_fill       = 0u;
    memset(s_dw_buf, 0xFFu, sizeof(s_dw_buf));
    s_state         = BOOT_FLASH_PROGRAMMING;
    return true;
}

bool boot_flash_write(const uint8_t *src, size_t n)
{
    if (s_state != BOOT_FLASH_PROGRAMMING) return false;
    if (src == NULL) { s_state = BOOT_FLASH_FAULT; return false; }
    if (s_bytes_written + n > s_total_expected) {
        s_state = BOOT_FLASH_FAULT;
        return false;
    }

    if (!flash_unlock()) { s_state = BOOT_FLASH_FAULT; return false; }

    while (n > 0u) {
        /* Fill the dword accumulator first. */
        size_t room = 8u - s_dw_fill;
        size_t take = (n < room) ? n : room;
        memcpy(&s_dw_buf[s_dw_fill], src, take);
        s_dw_fill += (uint8_t)take;
        src       += take;
        n         -= take;
        s_bytes_written += (uint32_t)take;

        if (s_dw_fill == 8u) {
            uint64_t v;
            memcpy(&v, s_dw_buf, sizeof(v));
            if (!flash_program_dword(s_cursor, v)) {
                flash_lock();
                s_state = BOOT_FLASH_FAULT;
                return false;
            }
            s_cursor  += 8u;
            s_dw_fill  = 0u;
            memset(s_dw_buf, 0xFFu, sizeof(s_dw_buf));
        }
    }

    /* If this write completed the image, flush the tail (pad with 0xFF
     * to a full dword). */
    if (s_bytes_written == s_total_expected && s_dw_fill != 0u) {
        uint64_t v;
        memcpy(&v, s_dw_buf, sizeof(v));
        if (!flash_program_dword(s_cursor, v)) {
            flash_lock();
            s_state = BOOT_FLASH_FAULT;
            return false;
        }
        s_cursor += 8u;
        s_dw_fill = 0u;
    }

    flash_lock();
    return true;
}

bool boot_flash_verify(uint32_t expected_crc32)
{
    if (s_state != BOOT_FLASH_PROGRAMMING) return false;
    if (s_bytes_written != s_total_expected) {
        s_state = BOOT_FLASH_FAULT;
        return false;
    }
    s_state = BOOT_FLASH_VERIFYING;
    uint32_t crc = crc32_calc((const uint8_t *)BOOT_FLASH_APP_BASE, s_bytes_written);
    if (crc != expected_crc32) {
        s_state = BOOT_FLASH_FAULT;
        return false;
    }
    s_state = BOOT_FLASH_IDLE;
    return true;
}

uint32_t boot_flash_current_image_crc32(void)
{
    uint32_t len = (s_bytes_written > 0u)
                 ? s_bytes_written
                 : BOOT_FLASH_APP_MAX_LEN;
    return crc32_calc((const uint8_t *)BOOT_FLASH_APP_BASE, len);
}

void boot_flash_abort(void)
{
    s_state         = BOOT_FLASH_IDLE;
    s_cursor        = BOOT_FLASH_APP_BASE;
    s_bytes_written = 0u;
    s_total_expected = 0u;
    s_dw_fill       = 0u;
}
