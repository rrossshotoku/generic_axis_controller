/*
 * boot_net_cfg — read the NETWORK persist blob so the bootloader comes
 * up on the same IP the app was configured for. Otherwise the PC tool
 * loses the CMC across the app -> bootloader reboot (app might be at
 * .101; bootloader would sit on hardcoded .100).
 *
 * The persist blob layout duplicated here MUST match app/config/config.c's
 * network_persist_blob_t. If that layout ever changes, bump the version
 * check + refresh the fields here.
 */

#ifndef BOOT_NET_CFG_H
#define BOOT_NET_CFG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
} boot_net_cfg_t;

/* Populate `out` from the NETWORK persist blob at 0x0807F800.
 *
 * Returns true if the blob decodes cleanly (magic + version + payload
 * size + CRC all match). Returns false and leaves `out` untouched on
 * any failure — caller should fall back to defaults. */
bool boot_net_cfg_load(boot_net_cfg_t *out);

#endif
