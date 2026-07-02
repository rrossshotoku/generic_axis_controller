/*
 * bsp/identity — STM32 UID-derived MAC implementation.
 *
 * Pattern matches the working reference at
 *   uc_camd_interface/Core/Src/app_init.c (derive_mac_from_uid).
 *
 * To swap to an SPI-connected identity source later, replace this
 * file. The header is the contract — do not change it.
 */

#include "identity.h"
#include "stm32g4xx_hal.h"

/* OUI prefix. 0x02 sets the locally-administered bit (and clears the
 * multicast bit), so this MAC cannot collide with an IEEE-registered
 * OUI even if 08:DC happens to be assigned. The 02:08:DC prefix
 * matches the reference project; using the same prefix keeps mixed
 * deployments visually grouped on the network. */
#define IDENTITY_MAC_OUI_0   0x02
#define IDENTITY_MAC_OUI_1   0x08
#define IDENTITY_MAC_OUI_2   0xDC

void identity_get_mac(uint8_t out[6])
{
    /* The 96-bit STM32 factory UID is read via HAL_GetUIDw0..2. XORing
     * the three words down to 32 bits is a deliberate hash: collisions
     * are statistically unlikely across any reasonable production run
     * (2^24 distinct lower-3-byte values), and we want determinism, not
     * cryptographic uniqueness. */
    const uint32_t mixed = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

    out[0] = IDENTITY_MAC_OUI_0;
    out[1] = IDENTITY_MAC_OUI_1;
    out[2] = IDENTITY_MAC_OUI_2;
    out[3] = (uint8_t)(mixed >> 16);
    out[4] = (uint8_t)(mixed >> 8);
    out[5] = (uint8_t)(mixed);
}
