/*
 * app/camerad — CAMERAD protocol codec implementation.
 *
 * Stateless. See camerad.h for the contract and the scope.
 */

#include "camerad.h"

#include <string.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 * Device-type classifiers
 *---------------------------------------------------------------------------*/

bool camerad_dev_is_s_type(uint32_t dev)
{
    return dev == CAMERAD_DEV_CONTROLLER_S
        || dev == CAMERAD_DEV_LEGACY_CTRL_S;
}

bool camerad_dev_is_t_type(uint32_t dev)
{
    return dev == CAMERAD_DEV_CONTROLLER_T
        || dev == CAMERAD_DEV_LEGACY_CTRL_T
        || dev == CAMERAD_DEV_CONTROLLER_T_PNL
        || dev == CAMERAD_DEV_LEGACY_T_PNL;
}

bool camerad_dev_is_controller(uint32_t dev)
{
    return camerad_dev_is_s_type(dev) || camerad_dev_is_t_type(dev);
}

/*----------------------------------------------------------------------------
 * Header parse / build
 *---------------------------------------------------------------------------*/

static bool version_is_supported(const char ver[4])
{
    /* Only v1.3 is supported by this codec. The wire field is 4 bytes
     * including the NUL terminator. */
    return ver[0] == '1' && ver[1] == '.' && ver[2] == '3' && ver[3] == '\0';
}

bool camerad_parse_header(const uint8_t *buf, size_t len, camerad_header_t *out)
{
    if (buf == NULL || out == NULL) return false;
    if (len < CAMERAD_HEADER_SIZE)  return false;

    /* Magic check — first 8 bytes including the NUL. */
    if (memcmp(buf, CAMERAD_MAGIC, 8) != 0) return false;

    /* Version check — bytes 8..11. */
    char ver[4];
    memcpy(ver, buf + 8, 4);
    if (!version_is_supported(ver)) return false;

    /* Provisionally copy the header so we can read message_length without
     * juggling endianness manually. The struct is __attribute__((packed)). */
    camerad_header_t tmp;
    memcpy(&tmp, buf, CAMERAD_HEADER_SIZE);

    /* Length sanity: must be between header-only and the max frame size. */
    if (tmp.message_length < CAMERAD_HEADER_SIZE) return false;
    if (tmp.message_length > CAMERAD_MAX_FRAME_SIZE) return false;

    /* Defensive — ensure return_address is NUL-terminated. The caller may
     * use it as a C string for IP-format helpers. */
    tmp.return_address[15] = '\0';

    *out = tmp;
    return true;
}

void camerad_build_response_header(uint8_t *out_buf,
                                   const camerad_header_t *request,
                                   uint32_t response_msg_command,
                                   uint16_t body_len,
                                   const char *our_ip,
                                   uint16_t our_port,
                                   uint32_t our_device_no)
{
    camerad_header_t h;
    memset(&h, 0, sizeof(h));

    memcpy(h.magic,   CAMERAD_MAGIC,        8);
    memcpy(h.version, CAMERAD_VERSION_1_3,  4);

    h.msg_command     = response_msg_command;
    /* Echo the requester as the new destination. */
    h.dest_device     = request->return_device;
    h.dest_device_no  = request->return_device_no;

    /* Our address in the return fields. */
    if (our_ip != NULL) {
        strncpy(h.return_address, our_ip, sizeof(h.return_address) - 1);
        h.return_address[sizeof(h.return_address) - 1] = '\0';
    }
    h.return_port       = (uint32_t)our_port;
    /* Advertise as DEV_CMC_BLDC (20) — the same value the Reduced CMC used.
     * Panels are configured to recognise this specific type as a usable
     * CMC. Advertising as DEV_CMC (3, generic CMC) makes the panel see us
     * but refuse to treat us as a controllable target. */
    h.return_device     = CAMERAD_DEV_CMC_BLDC;
    h.return_device_no  = our_device_no;

    h.message_length    = (uint32_t)CAMERAD_HEADER_SIZE + (uint32_t)body_len;
    h.message_id        = request->message_id;   /* echo for correlation */
    h.packet_id         = 0;                     /* CMC always sets 0 */

    memcpy(out_buf, &h, CAMERAD_HEADER_SIZE);
}

/*----------------------------------------------------------------------------
 * IP formatting helpers
 *---------------------------------------------------------------------------*/

size_t camerad_format_ip(const uint8_t ip[4], char out[16])
{
    if (out == NULL) return 0;
    int n = snprintf(out, 16, "%u.%u.%u.%u",
                     (unsigned)ip[0], (unsigned)ip[1],
                     (unsigned)ip[2], (unsigned)ip[3]);
    if (n < 0)  { out[0] = '\0'; return 0; }
    if (n > 15) { n = 15; out[15] = '\0'; }
    return (size_t)n;
}

bool camerad_parse_ip(const char *str, uint8_t out[4])
{
    if (str == NULL || out == NULL) return false;
    unsigned a, b, c, d;
    if (sscanf(str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
    if (a > 255 || b > 255 || c > 255 || d > 255)        return false;
    out[0] = (uint8_t)a;
    out[1] = (uint8_t)b;
    out[2] = (uint8_t)c;
    out[3] = (uint8_t)d;
    return true;
}

/*----------------------------------------------------------------------------
 * Per-opcode expected request body length
 *
 * Returns 0 for "variable / not a request opcode / unknown". The handler
 * uses this for quick sanity-check of inbound messages.
 *---------------------------------------------------------------------------*/

uint16_t camerad_request_body_len(camerad_msg_t cmd)
{
    switch (cmd) {
    case CAMERAD_MSG_POLL:
    case CAMERAD_MSG_SELECT:
    case CAMERAD_MSG_DESELECT:
    case CAMERAD_MSG_GRAB:
    case CAMERAD_MSG_POSITION_REQ:
    case CAMERAD_MSG_LEARN_ID_REQ:
        return 0;       /* header-only */

    case CAMERAD_MSG_KEYPRESS_T1:  return (uint16_t)sizeof(camerad_keypress_t1_t);  /* 5 */
    case CAMERAD_MSG_KEYPRESS_T2:  return (uint16_t)sizeof(camerad_keypress_t2_t);  /* 2 */
    case CAMERAD_MSG_KEYPRESS_T3:  return (uint16_t)sizeof(camerad_keypress_t3_t);  /* 42 */
    case CAMERAD_MSG_MOVEMENT:     return (uint16_t)sizeof(camerad_movement_t);     /* 9 */
    case CAMERAD_MSG_LIMIT:        return (uint16_t)sizeof(camerad_limit_req_t);    /* 2 */

    case CAMERAD_MSG_NONE:
    case CAMERAD_MSG_STORE_LEARN_END_T:
    default:
        return 0;       /* not a recognised request from a controller */
    }
}
