/*
 * bsp/identity — device identity provider.
 *
 * The CMC needs a MAC address (and, in future, other per-unit identity
 * data such as a serial number) without baking it into the firmware
 * image. This header is the abstraction; the implementation in
 * identity.c chooses where the data actually comes from.
 *
 * Today's implementation: derive a deterministic locally-administered
 * MAC from the STM32 96-bit factory unique ID.
 *
 * Future implementation: read the same fields from an SPI-connected
 * identity device (TBD). The header below does not change — only
 * identity.c is swapped.
 *
 * Callers must not assume *which* mechanism produced the value, only
 * that it is stable for the lifetime of the device.
 */

#ifndef BSP_IDENTITY_H
#define BSP_IDENTITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fill the 6-byte MAC for this unit.
 *
 * Guarantees:
 *   - The same physical device returns the same MAC across reboots.
 *   - Two physically distinct devices return different MACs (best effort
 *     — collision probability depends on the source).
 *   - The MAC is locally-administered and unicast (byte 0 has bit 1 set,
 *     bit 0 clear). This avoids clashing with any IEEE-registered OUI.
 *
 * Costs:
 *   - Today: a few microseconds (three register reads + arithmetic).
 *   - Future (SPI source): tens to hundreds of microseconds. Callers
 *     that need the MAC repeatedly should cache it once at boot rather
 *     than re-reading it.
 */
void identity_get_mac(uint8_t out[6]);

#ifdef __cplusplus
}
#endif

#endif /* BSP_IDENTITY_H */
