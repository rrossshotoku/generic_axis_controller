/*
 * app/od — Object Dictionary network bridge.
 *
 * Implements the network side of Interface/NETWORK_UDP_SPEC.md:
 *   - UDP listener on port 5000 (default): OD_READ/WRITE_REQ -> RESP/ERROR.
 *   - UDP socket on port 5001 (default): TELEMETRY stream to subscribed PC.
 *
 * Bridges to the motor MCU via app/cia402 (Phase 4 will replace its stub).
 *
 * CMC-local OD entries: none. The CMC's own configuration lives in the web
 * UI / app/config and is not exposed via the OD port. See
 * Documentation/architecture.md §11.3 for the rationale.
 *
 * See app/od/README.md for the contract.
 */

#ifndef APP_OD_H
#define APP_OD_H

#include <stdint.h>
#include <stdbool.h>

#include "Interface/mc_if_protocol.h"   /* MC_IfCyclicStatusHeader_t */

#ifdef __cplusplus
extern "C" {
#endif

void od_init(void);
void od_tick(void);                /* services both UDP sockets + telemetry batching */

/*----------------------------------------------------------------------------
 * Read-only state accessors for app/debug. Snapshot semantics — values
 * may change between two consecutive calls.
 *---------------------------------------------------------------------------*/
typedef struct {
    bool     in_flight;
    bool     is_read;
    uint16_t index;
    uint8_t  subindex;
    uint16_t seq;
} od_request_snapshot_t;

typedef struct {
    bool     active;
    uint8_t  peer_ip[4];
    uint16_t peer_port;
    uint16_t rate_divider;
    uint8_t  batch;
} od_subscriber_snapshot_t;

void     od_get_pending     (od_request_snapshot_t *out);
void     od_get_subscriber  (od_subscriber_snapshot_t *out);
uint32_t od_telemetry_datagrams_sent(void);
uint32_t od_telemetry_samples_sent  (void);

#ifdef __cplusplus
}
#endif

#endif /* APP_OD_H */
