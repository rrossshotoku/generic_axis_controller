/*
 * app/cia402 — host-side CiA-402 codec, cyclic exchange, OD pipeline.
 *
 * Wire format authority: Interface/mc_if_protocol.h.
 * OD type / result conventions: Interface/mc_if_od.h.
 *
 * Responsibilities (Phase 4):
 *  - Build/validate 64-byte SPI frames per MC_IfFrameHeader_t/Footer_t
 *    (CRC-16/Modbus on header and payload).
 *  - Run a continuous 1 kHz cyclic exchange (CYCLIC_CMD ↔ CYCLIC_STATUS)
 *    via bsp/motor_spi.
 *  - Pipeline OD reads/writes onto the same SPI transactions
 *    (OD_READ/WRITE_REQ in place of CYCLIC_CMD for one frame; matching
 *    OD_READ/WRITE_RESP picked up on a later frame, correlated by seq).
 *  - Maintain the latest CYCLIC_STATUS for upper layers to consume via
 *    cia402_take_cyclic_status() (polled — keeps dependency direction
 *    one-way: app/od calls down into app/cia402, not the reverse).
 *
 * One outstanding OD request at a time. Phase 6 hardening can widen the
 * pipeline if anyone needs more.
 */

#ifndef APP_CIA402_H
#define APP_CIA402_H

#include <stdint.h>
#include <stdbool.h>

#include "Interface/mc_if_od.h"
#include "Interface/mc_if_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void cia402_init(void);
void cia402_tick(void);                     /* runs the cyclic exchange + services OD pipeline */

/*----------------------------------------------------------------------------
 * Pipelined OD access — single in-flight request.
 *
 * Handle 0 is reserved as "invalid"; valid handles are >= 1.
 *
 * Begin returns CIA402_OD_HANDLE_INVALID when:
 *   - another OD request is already in flight (queue depth 1),
 *   - data length doesn't fit the internal scratch (8 bytes),
 *   - cia402 hasn't been initialised.
 *
 * Poll returns true once the request has completed (response received or
 * timeout). After true is returned for a given handle, that handle is
 * consumed; the slot is free for the next begin().
 *---------------------------------------------------------------------------*/

typedef int16_t cia402_od_handle_t;
#define CIA402_OD_HANDLE_INVALID  ((cia402_od_handle_t)0)

cia402_od_handle_t cia402_od_read_begin (uint16_t idx, uint8_t sub,
                                         MC_IfOdType_t type);

cia402_od_handle_t cia402_od_write_begin(uint16_t idx, uint8_t sub,
                                         MC_IfOdType_t type,
                                         const void *data, uint8_t len);

bool cia402_od_poll(cia402_od_handle_t h,
                    MC_IfOdResult_t *out_result,
                    void *out_data, uint8_t *out_len);

/*----------------------------------------------------------------------------
 * Raw passthrough — used by the CMC-side firmware-update path to forward
 * arbitrary mc_if messages to the motor MCU (specifically the segmented-SDO
 * messages MC_IF_MSG_OD_DOWNLOAD_INIT / _SEGMENT and receive the matching
 * DOWNLOAD_RESP). Shares the same single-slot pipeline as the OD API — only
 * one raw or OD transaction can be in flight at a time.
 *
 * Semantics are identical to OD read/write:
 *   1. cia402_raw_passthrough_begin returns a handle (or INVALID if the slot
 *      is busy / payload too big).
 *   2. cia402_raw_passthrough_poll(h) returns false until the response has
 *      arrived (or timed out), then true — after which the slot is free
 *      for the next begin.
 *
 * out_msg_type on success is whatever mc_if message type the motor replied
 * with (typically MC_IF_MSG_OD_DOWNLOAD_RESP). out_result is set to OK on a
 * clean round-trip; NOT_READY on timeout; other results only if the motor
 * itself emitted an ERROR frame matching our seq.
 *---------------------------------------------------------------------------*/

cia402_od_handle_t cia402_raw_passthrough_begin(uint8_t tx_msg_type,
                                                const void *tx_payload,
                                                uint8_t tx_len);

bool cia402_raw_passthrough_poll(cia402_od_handle_t h,
                                 MC_IfOdResult_t *out_result,
                                 uint8_t *out_msg_type,
                                 void *out_payload, uint8_t *out_len);

/*----------------------------------------------------------------------------
 * Bootloader-mode observability + emergency abort
 *
 * cia402_motor_in_bootloader — returns true once the motor MCU has reported
 * MC_IF_NODE_BOOTLOADER in a cyclic status header. The CMC's app-side
 * axis_manager uses this to pause its per-tick motor-OD polling; those OD
 * entries (fault_flags, encoder cpr, load_factor, ...) don't exist in the
 * motor's bootloader, and firing them while in bootloader mode would leave
 * sub-modules holding stale handles and deadlock the OD slot.
 *
 * cia402_od_abort — force the s_od slot back to OD_IDLE regardless of
 * current state. Any handle held by an upper layer becomes stale
 * (subsequent cia402_od_poll returns false). Called by axis_manager on
 * the rising edge of "motor entered bootloader" so PC-tool OD writes to
 * 0x1F5x (bootloader OD range, forwarded via cia402_raw_passthrough) don't
 * have to wait for an in-flight axis_manager request to complete.
 *---------------------------------------------------------------------------*/
bool cia402_motor_in_bootloader(void);
void cia402_od_abort(void);

/*----------------------------------------------------------------------------
 * Latest cyclic status accessor (polled by app/od).
 *
 * Returns true and copies out the most recent CYCLIC_STATUS header + blob
 * if a new one has arrived since the last call. Returns false otherwise.
 * out_blob must be at least MC_IF_TLM_BLOB_MAX bytes.
 *---------------------------------------------------------------------------*/

bool cia402_take_cyclic_status(MC_IfCyclicStatusHeader_t *out_hdr,
                               uint8_t *out_blob,
                               uint8_t *out_blob_len);

/* Non-consuming peek of the latest CYCLIC_STATUS. Returns false until the
 * first status frame has arrived since boot; from then on always returns
 * true with whatever was last received. Intended for diagnostics
 * (app/debug). Does NOT clear the fresh flag — won't compete with
 * cia402_take_cyclic_status. */
bool cia402_peek_cyclic_status(MC_IfCyclicStatusHeader_t *out_hdr,
                               uint8_t *out_blob,
                               uint8_t *out_blob_len);

/*----------------------------------------------------------------------------
 * Cyclic command setter (called by axis_manager).
 *
 * Replaces the controlword / mode / targets / profile fields of the
 * outgoing CYCLIC_CMD. command_counter is owned by cia402 and is left
 * untouched — it is set automatically on each send.
 *
 * SINGLE-OWNER RULE: only app/axis_manager should call this. Protocol
 * modules drive motion by writing the CMC-owned OD entries (0x3xxx);
 * cmc_od dispatches to axis_manager; axis_manager composes and pushes
 * the cyclic command here. Bypassing this rule desynchronises the
 * axis_manager state from what the motor MCU is actually being told.
 *---------------------------------------------------------------------------*/

void cia402_set_cyclic_cmd(const MC_IfCyclicCommand_t *cmd);

/*----------------------------------------------------------------------------
 * Diagnostic counters (read-only).
 *---------------------------------------------------------------------------*/

typedef struct {
    uint32_t tx_frames;          /* total frames sent */
    uint32_t rx_frames_valid;    /* total CRC-valid frames received */
    uint32_t rx_bad_sync;
    uint32_t rx_bad_version;
    uint32_t rx_bad_header_crc;
    uint32_t rx_bad_payload_crc;
    uint32_t rx_bad_length;
    uint32_t spi_errors;         /* motor_spi_transfer returned negative */
    /* Rate-limit / scheduling diagnostics. The cyclic exchange is meant to
     * run at exactly 1 kHz (CYCLE_PERIOD_MS = 1). When the main loop
     * stalls, the actual gap between consecutive cia402_tick-driven SPI
     * transactions stretches. We use a drift-tolerant rate-limit (no
     * catch-up bursts) and surface the drift here so it's visible. */
    uint32_t max_cycle_gap_ms;   /* worst-case gap between two consecutive frames */
    uint32_t late_cycles;        /* count of cycles where elapsed > CYCLE_PERIOD_MS */
    uint32_t total_drift_ms;     /* sum of (gap - CYCLE_PERIOD_MS) for every frame */
    uint32_t od_timeouts;        /* OD response window expired */
} cia402_stats_t;

void cia402_get_stats(cia402_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_CIA402_H */
