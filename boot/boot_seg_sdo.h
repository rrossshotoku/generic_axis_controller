/*
 * boot_seg_sdo — CiA-301 §7.2.4.3 segmented-SDO receiver for firmware
 * download. Only accepts writes to 0x1F50:1 program_data; anything else
 * targeting a different index is rejected with NO_OBJECT.
 *
 * State transitions driven by the master's messages:
 *   IDLE      -- (INIT ok) --> IN_PROGRESS
 *   IN_PROG   -- (SEGMENT good toggle, more data) --> IN_PROG
 *   IN_PROG   -- (SEGMENT good toggle, last=1)    --> IDLE (writes committed)
 *   any       -- (INIT during session) -> reject BOOTLOADER_BUSY
 *   any       -- (SEGMENT wrong toggle) -> reply with the OTHER toggle
 *                (sender resends the missed segment)
 *
 * Each segment's payload is handed to boot_flash_write immediately —
 * we don't buffer the whole image (the point of segmented SDO is to
 * bound RAM usage regardless of image size).
 *
 * Wire body layouts are in Interface/mc_if_protocol.h.
 */

#ifndef BOOT_SEG_SDO_H
#define BOOT_SEG_SDO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "Interface/mc_if_protocol.h"

void boot_seg_sdo_init(void);

/* Handle a MC_IF_MSG_OD_DOWNLOAD_INIT body. Fills out_resp for the
 * caller to send back. Returns the number of bytes to place in the
 * MC_IfOdDownloadResp_t. */
size_t boot_seg_sdo_on_init(const MC_IfOdDownloadInit_t *req,
                            MC_IfOdDownloadResp_t *out_resp);

/* Handle a MC_IF_MSG_OD_DOWNLOAD_SEGMENT body. body_len is the total
 * bytes AFTER any framing header (i.e. the sizeof-struct portion). */
size_t boot_seg_sdo_on_segment(const MC_IfOdDownloadSegment_t *seg,
                               uint8_t body_len,
                               MC_IfOdDownloadResp_t *out_resp);

/* Called by boot_od when program_control transitions to STOP or ABORT
 * so the receiver forgets any in-flight session. */
void boot_seg_sdo_abort(void);

#endif
