/*
 * app/visca — VISCA protocol codec (bytes ↔ frames), stateless.
 *
 * Mirrors app/camerad: no sockets, no state, no motor calls. Just parse
 * incoming bytes into a frame descriptor and build outbound response
 * bytes. All motion / session logic lives in app/visca_mgr.
 *
 * Layering: L4 — same as app/camerad. Depends only on stdint / stdbool
 * / stddef / string.h. NEVER includes bsp/, Interface/, or any other
 * app/ module.
 *
 * Wire format (Sony VISCA over IP):
 *   Each UDP payload begins with an 8-byte header:
 *     bytes 0-1: payload_type (BE) — see VISCA_IP_TYPE_*
 *     bytes 2-3: payload_length (BE) — VISCA payload byte count
 *     bytes 4-7: sequence number (BE) — echoed in reply
 *   Followed by the raw VISCA payload:
 *     byte 0: address byte 8D where D = destination (1-7) or 8 = broadcast
 *     bytes 1..N-1: command class + category + command + data
 *     byte N: 0xFF terminator
 *
 * Response VISCA frame:
 *     byte 0: address byte 9S where S = source device address (1-7)
 *     byte 1: status byte
 *       0x40+socket : ACK        (0x41 for socket 1)
 *       0x50+socket : Completion (0x51) — or 0x50 for inquiry replies
 *       0x60+socket : Error (followed by 1-byte error type)
 *     bytes 2..N-1: payload
 *     byte N: 0xFF terminator
 *
 * We use socket 1 for all command responses. Real Sony cameras allocate
 * sockets 1-2 for concurrent commands; we ACK+Complete every command
 * immediately (no in-flight tracking), so the socket number is decorative.
 */
#ifndef APP_VISCA_H
#define APP_VISCA_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------
 * VISCA-over-IP UDP header
 *---------------------------------------------------------------------------*/
#define VISCA_IP_HEADER_SIZE   (8u)

/* Payload types (BE-encoded on the wire). Sony's spec assigns these. */
#define VISCA_IP_TYPE_COMMAND      (0x0100u)   /* rx: VISCA command  (81 01 ...) */
#define VISCA_IP_TYPE_INQUIRY      (0x0110u)   /* rx: VISCA inquiry  (81 09 ...) */
#define VISCA_IP_TYPE_REPLY        (0x0111u)   /* tx: VISCA reply    (90 5x/4x/6x ...) */
#define VISCA_IP_TYPE_DEVSET_CMD   (0x0120u)   /* rx: device setting command (rare) */
#define VISCA_IP_TYPE_CTRL_CMD     (0x0200u)   /* rx: control command (sequence reset, etc.) */
#define VISCA_IP_TYPE_CTRL_REPLY   (0x0201u)   /* tx: control reply */

typedef struct {
    uint16_t payload_type;      /* one of VISCA_IP_TYPE_* */
    uint16_t payload_length;    /* VISCA-payload bytes, excluding this header */
    uint32_t sequence_number;   /* echoed unchanged in the reply */
} visca_ip_header_t;

/* Parse the 8-byte IP header out of `buf`. Returns false if `len` is
 * too small. Does NOT validate that payload_length matches the actual
 * bytes beyond the header — caller checks that. */
bool visca_ip_parse_header(const uint8_t *buf, size_t len,
                           visca_ip_header_t *out);

/* Build an 8-byte IP header at the start of `buf`. Caller must ensure
 * `buf` has room for VISCA_IP_HEADER_SIZE bytes. */
void visca_ip_build_header(uint8_t *buf, uint16_t payload_type,
                           uint16_t payload_length, uint32_t sequence_number);

/*----------------------------------------------------------------------------
 * VISCA frame parser
 *---------------------------------------------------------------------------*/

/* Maximum body bytes after the address byte and before the 0xFF terminator.
 * VISCA commands are typically <10 bytes total; give ourselves headroom
 * for future device-setting frames without dynamic allocation. */
#define VISCA_MAX_BODY_BYTES   (14u)

/* Command class values (byte 1 of the raw VISCA payload). */
#define VISCA_CLASS_COMMAND    (0x01u)
#define VISCA_CLASS_CANCEL     (0x02u)   /* rare, not currently handled */
#define VISCA_CLASS_INQUIRY    (0x09u)

typedef struct {
    /* Address decoded from byte 0 (0x8D). Broadcast is 0x88 -> address 8. */
    uint8_t  destination;
    /* Command class from byte 1 (0x01 command, 0x09 inquiry). */
    uint8_t  cls;
    /* Category (byte 2) and command (byte 3). */
    uint8_t  category;
    uint8_t  command;
    /* Data bytes (from byte 4 up to but not including the 0xFF terminator).
     * `body_len` may be 0 for parameter-less commands like Pan_tiltHome. */
    uint8_t  body[VISCA_MAX_BODY_BYTES];
    uint8_t  body_len;
} visca_frame_t;

/* Parse a raw VISCA frame (`8D CLS CAT CMD [data..] FF`) into `out`.
 * Returns false on:
 *   - too-short buffer
 *   - byte 0 not 0x8X
 *   - missing 0xFF terminator within the buffer
 *   - body too long for VISCA_MAX_BODY_BYTES
 *
 * A well-formed frame has minimum length 4 (address + class + category
 * + command + terminator = 5 bytes total actually; but check the spec:
 * some minimal inquiries omit the command byte). Conservative min is 4. */
bool visca_parse(const uint8_t *buf, size_t len, visca_frame_t *out);

/*----------------------------------------------------------------------------
 * Response builders
 *
 * Each builder writes the raw VISCA response frame (without the IP header)
 * into `buf`. Returns the number of bytes written. Caller wraps that in
 * an IP header before sending.
 *
 * `src_addr` is our address (1-7). The response address byte is 0x80 | (0x08 | src) = 0x9S.
 *---------------------------------------------------------------------------*/

/* Inquiry reply — `9S 50 [data...] FF`. Writes 3 + data_len bytes. */
size_t visca_build_inquiry_reply(uint8_t *buf, size_t buf_cap,
                                 uint8_t src_addr,
                                 const uint8_t *data, size_t data_len);

/* ACK on socket — `9S 4Z FF`. Writes 3 bytes. */
size_t visca_build_ack(uint8_t *buf, size_t buf_cap, uint8_t src_addr, uint8_t socket);

/* Completion on socket — `9S 5Z FF`. Writes 3 bytes. */
size_t visca_build_completion(uint8_t *buf, size_t buf_cap, uint8_t src_addr, uint8_t socket);

/* Error reply — `9S 6Z EE FF`. Writes 4 bytes.
 * Common error types:
 *   0x02 SYNTAX  — malformed command
 *   0x03 CMD_BUF_FULL
 *   0x04 CMD_CANCELLED
 *   0x05 NO_SOCKET
 *   0x41 CMD_NOT_EXECUTABLE — right shape but wrong state (e.g. not homed). */
#define VISCA_ERR_SYNTAX             (0x02u)
#define VISCA_ERR_CMD_BUF_FULL       (0x03u)
#define VISCA_ERR_CMD_CANCELLED      (0x04u)
#define VISCA_ERR_NO_SOCKET          (0x05u)
#define VISCA_ERR_NOT_EXECUTABLE     (0x41u)
size_t visca_build_error(uint8_t *buf, size_t buf_cap,
                         uint8_t src_addr, uint8_t socket, uint8_t err_type);

/*----------------------------------------------------------------------------
 * Domain-specific helpers for the three inquiries we answer
 *---------------------------------------------------------------------------*/

/* CAM_VersionInq reply body (7 bytes: vendor_h vendor_l model_h model_l
 * rom_h rom_l socket_count). Caller passes the identifier constants
 * that describe THIS device; visca_mgr picks them. */
void visca_pack_version_body(uint8_t out[7],
                             uint16_t vendor_id, uint16_t model_id,
                             uint16_t rom_version, uint8_t socket_count);

/* Pan_tiltPosInq reply body (8 bytes: 4 nibbles pan + 4 nibbles tilt).
 * Each of the 4 pan / 4 tilt bytes carries one nibble of the signed
 * 16-bit value in its LOW nibble; upper nibble is 0.
 * Byte order is MSN first (MSB nibble of MSB byte, then LSN of MSB byte,
 * then MSN of LSB byte, then LSN of LSB byte). */
void visca_pack_pantilt_pos_body(uint8_t out[8], int16_t pan, int16_t tilt);

/* CAM_ZoomPosInq reply body (4 bytes: 4 nibbles of a signed 16-bit zoom
 * value, same nibble-packing as Pan_tiltPosInq). */
void visca_pack_zoom_pos_body(uint8_t out[4], int16_t zoom);

/*----------------------------------------------------------------------------
 * Address helpers
 *---------------------------------------------------------------------------*/

/* Extract the destination address (1..7) or broadcast (8) from an address byte 0x8D.
 * Returns 0 if the byte isn't in the 0x81..0x88 range. */
uint8_t visca_addr_from_first_byte(uint8_t first_byte);

/* Response address byte for a reply from device `src_addr` (1-7).
 * Convention: 0x8_ with the low nibble = 0x08 | src_addr, so cam 1 → 0x90. */
uint8_t visca_reply_addr_byte(uint8_t src_addr);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISCA_H */
