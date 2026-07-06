/*
 * bsp/flash — STM32G474RE internal flash, raw primitives.
 *
 * Thin wrapper over HAL_FLASHEx_Erase + HAL_FLASH_Program. All blocking.
 * Page numbering is linear 0..(FLASH_BSP_PAGE_COUNT-1) across both banks;
 * flash_erase_page translates to HAL's per-bank page index.
 */

#include "flash.h"

#include "stm32g4xx_hal.h"

#include <string.h>

bool flash_unlock(void)
{
    /* HAL_FLASH_Unlock returns OK whether or not the peripheral was
     * already locked. */
    HAL_StatusTypeDef rc = HAL_FLASH_Unlock();
    return rc == HAL_OK;
}

void flash_lock(void)
{
    (void)HAL_FLASH_Lock();
}

bool flash_erase_page(uint32_t page)
{
    if (page >= FLASH_BSP_PAGE_COUNT) return false;

    /* STM32G474RE is dual-bank: 512 KB split into Bank 1 (0..127 →
     * 0x08000000..0x0803FFFF) and Bank 2 (0..127 → 0x08040000..0x0807FFFF).
     * HAL_FLASHEx_Erase takes a page number WITHIN a bank plus the bank
     * selection, so we translate our linear 0..255 numbering. */
    FLASH_EraseInitTypeDef erase;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    if (page < 128u) {
        erase.Banks = FLASH_BANK_1;
        erase.Page  = page;
    } else {
        erase.Banks = FLASH_BANK_2;
        erase.Page  = page - 128u;
    }
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    HAL_StatusTypeDef rc = HAL_FLASHEx_Erase(&erase, &page_err);
    return rc == HAL_OK;
}

bool flash_program_dword(uint32_t addr, uint64_t value)
{
    if ((addr & 0x7u) != 0u) return false;   /* must be 8-byte aligned */
    HAL_StatusTypeDef rc = HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)addr, value);
    return rc == HAL_OK;
}

bool flash_program_buf(uint32_t addr, const void *src, size_t len)
{
    if (src == NULL)         return false;
    if ((addr & 0x7u) != 0u) return false;
    if ((len  & 0x7u) != 0u) return false;   /* dword multiples only */

    const uint8_t *p = (const uint8_t *)src;
    for (size_t off = 0; off < len; off += 8u) {
        /* Copy bytewise into a uint64_t so an unaligned src buffer is OK. */
        uint64_t v = 0;
        memcpy(&v, p + off, 8u);
        if (!flash_program_dword(addr + (uint32_t)off, v)) {
            return false;
        }
    }
    return true;
}
