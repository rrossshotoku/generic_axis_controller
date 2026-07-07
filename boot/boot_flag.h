/*
 * boot_flag — read the persistent "stay in bootloader" flag from the
 * BOOT persist region at 0x0807D800.
 *
 * Mirrors the format that app/boot_meta.c writes: a persist_header_t
 * (16 bytes: "PRST" magic + version + payload_size + CRC32) followed by
 * a 4-byte payload (the STAY / CLEAR magic).
 *
 * Fail-safe reads: any header inconsistency, CRC mismatch, size wrong,
 * or unknown payload magic is treated as CLEAR — i.e. the bootloader
 * jumps to the app rather than getting stuck. Recovery is via the
 * "app image invalid" check in main.c which forces bootloader mode.
 */

#ifndef BOOT_FLAG_H
#define BOOT_FLAG_H

#include <stdbool.h>

/* Returns true iff the flag was successfully read AND equals STAY_MAGIC. */
bool boot_flag_is_stay(void);

#endif
