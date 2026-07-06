/*
 * bsp/flash — STM32G431 internal flash, raw erase/program/read primitives.
 *
 * The G431RB has 128 KB of flash in 2 KB pages (0x800 bytes). Programming
 * happens 64-bit (double-word) at a time at 8-byte-aligned addresses. Erase
 * is per-page. Endurance ~10k cycles/page (typical for STM32G4).
 *
 * Usage pattern for a save:
 *   flash_unlock();
 *   flash_erase_page(page_index);
 *   for each 8-byte chunk:
 *       flash_program_dword(addr, value);
 *   flash_lock();
 *
 * All operations BLOCK the CPU. Erase is the slow one (~20-30 ms per page).
 * Program is fast (~85 us per dword). On a cooperative super-loop this
 * means an erase causes ~30 missed cyclic ticks (~30 ms of drift in
 * cia402's drift-tolerant cadence). Acceptable for explicit user-triggered
 * saves; do not call from a hot path.
 *
 * Reads use direct memory access — flash is memory-mapped. No primitive is
 * needed for read; cast a const uint8_t* at the address.
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* STM32G474RE dual-bank flash constants (verified against HAL headers).
 * Linear page numbers 0..255 span both banks:
 *   pages 0..127   -> Bank 1, 0x08000000..0x0803FFFF
 *   pages 128..255 -> Bank 2, 0x08040000..0x0807FFFF
 * flash_erase_page translates transparently. */
#define FLASH_BSP_PAGE_BYTES   2048u   /* 2 KB per page */
#define FLASH_BSP_PAGE_COUNT    256u   /* 512 KB / 2 KB */
#define FLASH_BSP_BASE_ADDR    0x08000000u

/* Convert between page index and absolute address. */
static inline uint32_t flash_page_addr(uint32_t page) {
    return FLASH_BSP_BASE_ADDR + page * FLASH_BSP_PAGE_BYTES;
}
static inline uint32_t flash_addr_page(uint32_t addr) {
    return (addr - FLASH_BSP_BASE_ADDR) / FLASH_BSP_PAGE_BYTES;
}

/* Unlock / lock the FLASH peripheral. Required around every erase or
 * program. Idempotent — calling unlock twice is safe (HAL handles it). */
bool flash_unlock(void);
void flash_lock  (void);

/* Erase a single page (2 KB). Returns true on success. Caller MUST
 * unlock first. */
bool flash_erase_page(uint32_t page);

/* Program a 64-bit double-word at an 8-byte-aligned address. Returns true
 * on success. Caller MUST unlock first; address MUST be within the
 * caller's reserved (already-erased) range. */
bool flash_program_dword(uint32_t addr, uint64_t value);

/* Convenience: program an arbitrary-length buffer starting at addr.
 * Length must be a multiple of 8 (G4 only programs in dword chunks).
 * Caller MUST unlock first. Pads the source buffer with zeros if the
 * caller arranges; this function does NOT pad. */
bool flash_program_buf(uint32_t addr, const void *src, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FLASH_H */
