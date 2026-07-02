# bsp/flash

## Purpose
Internal flash read/erase/program. Implements the dual-bank versioned storage primitive that `config` builds on.

## Owns
- The flash address map: locations and sizes of the config region and the shot-table region.
- Sector erase, word/double-word program, read.
- Verify-after-program.
- The atomic-save primitive: erase target bank, program, verify, bump sequence — exposed as a single call.

## Does NOT do
- Define the settings struct — `config` does.
- Decide which fields persist — `config` does.
- Cache values in RAM — callers cache if they want to.

## Public API
```c
void flash_init(void);

/* Raw access — used sparingly */
bool   flash_read       (uint32_t addr, void *out, size_t len);
bool   flash_erase_sector(uint32_t sector_addr);
bool   flash_program    (uint32_t addr, const void *in, size_t len);

/* Versioned dual-bank primitive — used by config */
typedef struct {
    uint32_t bank_a_addr;
    uint32_t bank_b_addr;
    size_t   bank_size;          /* must be a multiple of the flash sector size */
} flash_region_t;

bool flash_region_load(const flash_region_t *r, void *out, size_t len);
bool flash_region_save(const flash_region_t *r, const void *in, size_t len);
```

`flash_region_load` reads both banks, picks the one with the highest valid sequence number whose CRC matches, and copies it out. Returns false only if neither bank validates.

`flash_region_save` writes to the bank that was not most recently loaded, bumps the sequence, computes the CRC. Atomic across power-fail.

## Dependencies
- `Drivers/STM32G4xx_HAL_Driver` (FLASH HAL).

## Acceptance criteria
- Erase + program + read-back returns programmed values.
- 100,000-cycle save loop does not corrupt the unrelated region (separate sector pairs).
- Power-fail mid-`flash_region_save` (simulated by deliberately resetting) results in either old or new content on next load, never a third state.

## Notes
- STM32G431RB has 128 KB flash in 2 KB sectors. Two config sectors + four shot sectors = 12 KB out of the top of flash. Linker script must reserve this — the `.ld` placement is part of this module's responsibility (documented at the top of `bsp/flash.c`).
- Erase counts per sector are exposed via the OD (entry TBD) for wear monitoring.
- All flash operations disable interrupts for the duration of a single page program (a few hundred µs) — known and tolerated.
