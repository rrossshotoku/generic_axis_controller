/*
 * boot_seg_sdo — see boot_seg_sdo.h.
 */

#include "boot_seg_sdo.h"
#include "boot_flash.h"

#include "Interface/mc_if_od.h"

#include <string.h>

/* Segment flag bits (mirror the doc's convention — INTERFACE_SPEC.md §7c). */
#define SEG_FLAG_TOGGLE  0x01u
#define SEG_FLAG_LAST    0x02u

static bool     s_in_session;
static uint8_t  s_expected_toggle;
static uint32_t s_bytes_accepted;

/* Populate the response body from the running state. bytes_accepted is
 * always echoed; result is the caller's choice; toggle_ack is what the
 * sender should retry against on the next segment. */
static size_t build_resp(MC_IfOdDownloadResp_t *out,
                         uint8_t toggle_ack, MC_IfOdResult_t result)
{
    out->toggle_ack     = toggle_ack;
    out->result         = (uint8_t)result;
    out->reserved       = 0;
    out->bytes_accepted = s_bytes_accepted;
    return sizeof(*out);
}

void boot_seg_sdo_init(void)
{
    s_in_session      = false;
    s_expected_toggle = 0u;
    s_bytes_accepted  = 0u;
}

size_t boot_seg_sdo_on_init(const MC_IfOdDownloadInit_t *req,
                            MC_IfOdDownloadResp_t *out_resp)
{
    /* Only the program_data sink is a valid target. */
    if (req->index != 0x1F50u || req->subindex != 1u) {
        return build_resp(out_resp, 0u, MC_IF_OD_ERR_NO_OBJECT);
    }
    /* Reject overlapping sessions. Operator must ABORT first. */
    if (s_in_session) {
        return build_resp(out_resp, 0u, MC_IF_OD_ERR_BOOTLOADER_BUSY);
    }
    /* boot_flash_begin erases pages 16..250 — this is the ~seconds-long
     * blocking step of the update. Once it returns we're ready to accept
     * segments. */
    if (!boot_flash_begin(req->total_length)) {
        /* Erase failed (rare — WRP hit, or size out of range). Use
         * FLASH_LOCKED as the closest semantic match. */
        return build_resp(out_resp, 0u, MC_IF_OD_ERR_FLASH_LOCKED);
    }
    s_in_session      = true;
    s_expected_toggle = 0u;
    s_bytes_accepted  = 0u;
    return build_resp(out_resp, 0u, MC_IF_OD_OK);
}

size_t boot_seg_sdo_on_segment(const MC_IfOdDownloadSegment_t *seg,
                               uint8_t body_len,
                               MC_IfOdDownloadResp_t *out_resp)
{
    if (!s_in_session) {
        return build_resp(out_resp, 0u, MC_IF_OD_ERR_NOT_READY);
    }

    uint8_t got_toggle = (seg->flags & SEG_FLAG_TOGGLE) ? 1u : 0u;
    if (got_toggle != s_expected_toggle) {
        /* Sender missed our last ack — reply with what they should have
         * used, don't touch the write cursor. Sender resends. */
        return build_resp(out_resp, s_expected_toggle, MC_IF_OD_OK);
    }

    /* Fixed portion of MC_IfOdDownloadSegment_t before data[] = 3 bytes. */
    const uint8_t seg_hdr_bytes = 3u;
    if (body_len < seg_hdr_bytes || (uint32_t)(seg_hdr_bytes + seg->seg_length) > body_len) {
        s_in_session = false;
        boot_flash_abort();
        return build_resp(out_resp, s_expected_toggle, MC_IF_OD_ERR_SIZE);
    }

    if (seg->seg_length > 0u) {
        if (!boot_flash_write(seg->data, seg->seg_length)) {
            s_in_session = false;
            return build_resp(out_resp, s_expected_toggle, MC_IF_OD_ERR_FLASH_LOCKED);
        }
        s_bytes_accepted += seg->seg_length;
    }

    /* Flip the toggle for the next expected segment. */
    s_expected_toggle ^= 1u;

    if (seg->flags & SEG_FLAG_LAST) {
        /* Sender says that was the last segment. Session complete —
         * caller then issues program_control = VERIFY to check CRC. */
        s_in_session = false;
    }
    return build_resp(out_resp, got_toggle, MC_IF_OD_OK);
}

void boot_seg_sdo_abort(void)
{
    s_in_session      = false;
    s_expected_toggle = 0u;
    s_bytes_accepted  = 0u;
    boot_flash_abort();
}
