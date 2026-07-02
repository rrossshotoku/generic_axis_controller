/*
 * app/od/cmc_od — OD dispatcher for CMC-owned entries (MC_IF_OWNER_CMC).
 *
 * Called by app/od (the UDP bridge) for any OD request whose index falls
 * in the CMC-owned range. Returns the OD result code; reads also fill
 * out_type, out_data, out_len. No SPI traffic is generated.
 *
 * The dispatch routes to app/axis_manager accessors.
 *
 * Currently handles 0x3000-0x303F (axis 0). Future axes (0x3100-0x31FF
 * etc.) plug in here without touching app/od.
 */

#ifndef APP_OD_CMC_OD_H
#define APP_OD_CMC_OD_H

#include <stdint.h>
#include <stdbool.h>

#include "Interface/mc_if_od.h"
#include "Interface/mc_if_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Quick check: is this index handled here?
 * (Identifies a CMC-owned entry; the actual existence check happens in
 *  cmc_od_read/write and returns MC_IF_OD_ERR_NO_OBJECT if not.) */
bool cmc_od_owns(uint16_t index);

/* Handle a read for a CMC-owned entry. Caller has already verified
 * cmc_od_owns(idx). Returns OD result; on OK, fills out_type, out_data
 * (up to 8 bytes), and out_len. The caller (app/od) bundles these into
 * the OD_READ_RESP datagram. */
MC_IfOdResult_t cmc_od_read(uint16_t idx, uint8_t sub,
                            MC_IfOdType_t expected_type,
                            uint8_t *out_type, uint8_t *out_data, uint8_t *out_len);

/* Handle a write for a CMC-owned entry. Caller has already verified
 * cmc_od_owns(idx). Returns OD result. */
MC_IfOdResult_t cmc_od_write(uint16_t idx, uint8_t sub,
                             MC_IfOdType_t in_type,
                             const uint8_t *in_data, uint8_t in_len);

#ifdef __cplusplus
}
#endif

#endif /* APP_OD_CMC_OD_H */
