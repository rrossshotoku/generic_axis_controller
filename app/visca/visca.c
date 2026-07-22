/*
 * app/visca — codec implementation. See visca.h for wire format details.
 */
#include "visca.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * IP header helpers
 *
 * Sony's VISCA-over-IP header is 8 bytes BE. Payload_type and length are
 * u16; sequence_number is u32. Written LSB-last for readability.
 *---------------------------------------------------------------------------*/
static uint16_t rd_be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}
static void wr_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

bool visca_ip_parse_header(const uint8_t *buf, size_t len, visca_ip_header_t *out)
{
    if (!buf || !out || len < VISCA_IP_HEADER_SIZE) return false;
    out->payload_type    = rd_be16(&buf[0]);
    out->payload_length  = rd_be16(&buf[2]);
    out->sequence_number = rd_be32(&buf[4]);
    return true;
}

void visca_ip_build_header(uint8_t *buf, uint16_t payload_type,
                           uint16_t payload_length, uint32_t sequence_number)
{
    wr_be16(&buf[0], payload_type);
    wr_be16(&buf[2], payload_length);
    wr_be32(&buf[4], sequence_number);
}

/*----------------------------------------------------------------------------
 * Address helpers
 *---------------------------------------------------------------------------*/

uint8_t visca_addr_from_first_byte(uint8_t first_byte)
{
    /* Request frames start with 0x8D where D = 1..7 (unicast) or 8 (broadcast).
     * Anything outside that range is malformed. */
    if ((first_byte & 0xF0u) != 0x80u) return 0;
    uint8_t d = first_byte & 0x0Fu;
    if (d < 1u || d > 8u) return 0;
    return d;
}

uint8_t visca_reply_addr_byte(uint8_t src_addr)
{
    /* Reply from device at address `src_addr` (1..7): 0x80 | (0x08 | src) → 0x9S. */
    return (uint8_t)(0x80u | (0x08u | (src_addr & 0x07u)));
}

/*----------------------------------------------------------------------------
 * Frame parser
 *---------------------------------------------------------------------------*/

bool visca_parse(const uint8_t *buf, size_t len, visca_frame_t *out)
{
    if (!buf || !out) return false;
    /* Minimum sane frame: address + class + terminator = 3 bytes. Actual
     * commands are always longer; we validate structure not semantics. */
    if (len < 3u) return false;

    uint8_t dest = visca_addr_from_first_byte(buf[0]);
    if (dest == 0u) return false;

    /* Terminator must be within the buffer. Real-world frames are short
     * (≤16 bytes) so scanning is trivial. */
    size_t term_idx = 0;
    bool   found    = false;
    for (size_t i = 1; i < len; ++i) {
        if (buf[i] == 0xFFu) { term_idx = i; found = true; break; }
    }
    if (!found) return false;
    if (term_idx < 2u) return false;   /* need at least class byte before terminator */

    out->destination = dest;
    out->cls         = buf[1];
    out->category    = (term_idx >= 3u) ? buf[2] : 0u;
    out->command     = (term_idx >= 4u) ? buf[3] : 0u;

    /* Body starts at byte 4 (after address/class/category/command).
     * Some minimal frames (`8x 01 FF` — hypothetical) have no category
     * or command; the parse above uses 0 as a placeholder. Real frames
     * always carry at least category+command. */
    if (term_idx <= 4u) {
        out->body_len = 0;
    } else {
        size_t body_bytes = term_idx - 4u;
        if (body_bytes > VISCA_MAX_BODY_BYTES) return false;
        memcpy(out->body, &buf[4], body_bytes);
        out->body_len = (uint8_t)body_bytes;
    }
    return true;
}

/*----------------------------------------------------------------------------
 * Response builders
 *---------------------------------------------------------------------------*/

size_t visca_build_inquiry_reply(uint8_t *buf, size_t buf_cap,
                                 uint8_t src_addr,
                                 const uint8_t *data, size_t data_len)
{
    /* Frame: 9S 50 [data...] FF — 3 + data_len bytes. */
    size_t need = 3u + data_len;
    if (!buf || buf_cap < need) return 0;
    buf[0] = visca_reply_addr_byte(src_addr);
    buf[1] = 0x50u;                       /* inquiry data reply, socket 0 */
    if (data_len && data) {
        memcpy(&buf[2], data, data_len);
    }
    buf[2u + data_len] = 0xFFu;
    return need;
}

size_t visca_build_ack(uint8_t *buf, size_t buf_cap, uint8_t src_addr, uint8_t socket)
{
    if (!buf || buf_cap < 3u) return 0;
    buf[0] = visca_reply_addr_byte(src_addr);
    buf[1] = (uint8_t)(0x40u | (socket & 0x0Fu));
    buf[2] = 0xFFu;
    return 3u;
}

size_t visca_build_completion(uint8_t *buf, size_t buf_cap, uint8_t src_addr, uint8_t socket)
{
    if (!buf || buf_cap < 3u) return 0;
    buf[0] = visca_reply_addr_byte(src_addr);
    buf[1] = (uint8_t)(0x50u | (socket & 0x0Fu));
    buf[2] = 0xFFu;
    return 3u;
}

size_t visca_build_error(uint8_t *buf, size_t buf_cap,
                         uint8_t src_addr, uint8_t socket, uint8_t err_type)
{
    if (!buf || buf_cap < 4u) return 0;
    buf[0] = visca_reply_addr_byte(src_addr);
    buf[1] = (uint8_t)(0x60u | (socket & 0x0Fu));
    buf[2] = err_type;
    buf[3] = 0xFFu;
    return 4u;
}

/*----------------------------------------------------------------------------
 * Inquiry-body packers
 *---------------------------------------------------------------------------*/

void visca_pack_version_body(uint8_t out[7],
                             uint16_t vendor_id, uint16_t model_id,
                             uint16_t rom_version, uint8_t socket_count)
{
    /* Bytes: vendor_h vendor_l model_h model_l rom_h rom_l socket_count. */
    out[0] = (uint8_t)(vendor_id  >> 8);
    out[1] = (uint8_t)(vendor_id  & 0xFFu);
    out[2] = (uint8_t)(model_id   >> 8);
    out[3] = (uint8_t)(model_id   & 0xFFu);
    out[4] = (uint8_t)(rom_version >> 8);
    out[5] = (uint8_t)(rom_version & 0xFFu);
    out[6] = socket_count;
}

/* Split a signed 16-bit value into 4 low-nibble bytes MSN-first.
 * Example: 0x1234 → 0x01 0x02 0x03 0x04.
 * The receiver reassembles by shifting: (b[0]<<12)|(b[1]<<8)|(b[2]<<4)|b[3]. */
static void split_i16_to_nibbles(int16_t v, uint8_t out[4])
{
    uint16_t u = (uint16_t)v;
    out[0] = (uint8_t)((u >> 12) & 0x0Fu);
    out[1] = (uint8_t)((u >>  8) & 0x0Fu);
    out[2] = (uint8_t)((u >>  4) & 0x0Fu);
    out[3] = (uint8_t)( u        & 0x0Fu);
}

void visca_pack_pantilt_pos_body(uint8_t out[8], int16_t pan, int16_t tilt)
{
    split_i16_to_nibbles(pan,  &out[0]);
    split_i16_to_nibbles(tilt, &out[4]);
}

void visca_pack_zoom_pos_body(uint8_t out[4], int16_t zoom)
{
    split_i16_to_nibbles(zoom, out);
}
