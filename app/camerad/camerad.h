/*
 * app/camerad — CAMERAD protocol codec (stateless).
 *
 * Reverse-engineered from SW050 (see /CAMERAD_Protocol.md in the SW050 tree).
 * On-the-wire format documented in CAMERAD_Protocol.md §2 (header), §4
 * (opcodes), §5 (encodings), §6 (per-message exchanges).
 *
 * SCOPE: this codec implements the subset of CAMERAD that the LCMC speaks
 * to S/T camera-control panels — POLL discovery, SELECT/DESELECT/GRAB,
 * KEYPRESS types 1/2/3, MOVEMENT, LIMIT, POSITION_REQ, LEARN_ID_REQ. It
 * does NOT implement CCU side-channels, on-air broadcasts, location/track
 * broadcasts, RBS legacy bridge, cue-computer, or any of the panel-internal
 * (14-49 / 95) opcodes. Those are explicitly out of scope per the
 * Lightweight_CMC architecture decision (Documentation/architecture.md §3).
 *
 * STATELESSNESS: every function in this module is pure. No globals, no
 * static storage. The caller (app/controller_mgr) owns sockets, per-
 * controller state, response sequencing — this module is just bytes ↔ structs.
 *
 * BYTE ORDER: native little-endian (no htons/ntohl anywhere). All STM32
 * Cortex-M targets are LE so this is fine. A big-endian peer would require
 * field-by-field swapping; not supported here.
 *
 * PACKING: every wire struct in this header is __attribute__((packed)).
 * Direct memcpy between the wire buffer and a struct is safe and intended.
 *
 * S vs T RESPONSE SHAPE: the LCMC chooses the poll-response body shape
 * SOLELY from the `return_device` field in the inbound POLL request — never
 * from any inbound key code, mode flag, or out-of-band hint. This fixes a
 * latent SW050 bug where some response-builder paths could pick the wrong
 * shape under specific request sequences.
 */

#ifndef APP_CAMERAD_H
#define APP_CAMERAD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *  Constants
 *==========================================================================*/

#define CAMERAD_HEADER_SIZE           64u
#define CAMERAD_MAX_BODY_SIZE         164u
#define CAMERAD_MAX_FRAME_SIZE        (CAMERAD_HEADER_SIZE + CAMERAD_MAX_BODY_SIZE)

#define CAMERAD_MAGIC                 "CAMERAD"        /* 8 bytes incl. NUL */
#define CAMERAD_VERSION_1_3           "1.3"            /* 4 bytes incl. NUL */

/* Default UDP ports per SW050 firmware. Overridable via app/config. */
#define CAMERAD_PORT_POLL_DEFAULT     30002u
#define CAMERAD_PORT_GENERAL_DEFAULT  30003u

/*============================================================================
 *  Enums (device types, message opcodes, key codes, status bits)
 *==========================================================================*/

/* Device types — eDevices in SW050 (CameraToolsU.h:139-155), plus the
 * BLDC-CMC variant added by the Reduced-CMC project (uc_camd_interface
 * protocol.h:63). Panels are typically configured to recognise specific
 * device types as "valid CMCs to control"; advertising the wrong one means
 * the panel sees us but doesn't treat us as a usable CMC. */
typedef enum {
    CAMERAD_DEV_UNKNOWN          = 0,
    CAMERAD_DEV_CONTROLLER_S     = 1,   /* S-type controller (no screen) */
    CAMERAD_DEV_CONTROLLER_T     = 2,   /* T-type controller (with screen) */
    CAMERAD_DEV_CMC              = 3,   /* generic CMC */
    CAMERAD_DEV_LEGACY_CTRL_S    = 4,
    CAMERAD_DEV_LEGACY_CTRL_T    = 5,
    CAMERAD_DEV_LEGACY_CMC       = 6,
    CAMERAD_DEV_ALL_CMC          = 7,   /* wildcard for "any CMC" in dest_device */
    CAMERAD_DEV_CONTROLLER_T_PNL = 9,   /* T-panel in emergency (no-screen) mode */
    CAMERAD_DEV_LEGACY_T_PNL     = 10,
    CAMERAD_DEV_CMC_BLDC         = 20,  /* BLDC-motor CMC -- this device.
                                         * Matches Reduced CMC's advertisement;
                                         * panels are configured to recognise
                                         * type 20 as the controllable BLDC CMC. */
} camerad_device_t;

/* Returns true if the given device type is any kind of S-shaped controller
 * (no screen — original or legacy). LCMC handlers use this to pick the
 * S-shaped poll response body. */
bool camerad_dev_is_s_type(uint32_t dev);

/* Returns true if the given device type is any kind of T-shaped controller
 * (with screen — original, legacy, or T-panel emergency mode). */
bool camerad_dev_is_t_type(uint32_t dev);

/* Returns true if the given device type is any kind of controller (S or T,
 * any flavour) — i.e. something that legitimately sends commands to a CMC. */
bool camerad_dev_is_controller(uint32_t dev);

/* Message opcodes — eMessageCommands in SW050 (CameraToolsU.h:237-407).
 * In-scope opcodes have explicit symbols below; out-of-scope ones don't, and
 * are silently dropped by the dispatcher. */
typedef enum {
    CAMERAD_MSG_NONE                = 0,
    CAMERAD_MSG_POLL                = 1,    /* UDP discovery */
    CAMERAD_MSG_SELECT              = 2,    /* take ownership */
    CAMERAD_MSG_DESELECT            = 3,    /* release ownership */
    CAMERAD_MSG_GRAB                = 4,    /* force-take ownership */
    CAMERAD_MSG_KEYPRESS_T1         = 5,    /* shot store/recall, fade/cut/swoop keys */
    CAMERAD_MSG_KEYPRESS_T2         = 6,    /* stop, set/restore limits, toggles */
    CAMERAD_MSG_KEYPRESS_T3         = 7,    /* T-screen full shot frame */
    CAMERAD_MSG_MOVEMENT            = 8,    /* joystick/fader/dial frame */
    CAMERAD_MSG_LIMIT               = 9,    /* set/get soft limits */
    CAMERAD_MSG_POSITION_REQ        = 11,   /* T-screen requests axis positions */
    CAMERAD_MSG_LEARN_ID_REQ        = 12,   /* T-screen requests learn-ID list */
    CAMERAD_MSG_STORE_LEARN_END_T   = 13,   /* notify T-screen learn-store done */
} camerad_msg_t;

/* Key codes — eKeyCodes in SW050 (CameraToolsU.h:542-706). The full table is
 * very large; we list the ones the LCMC actually acts on. Other values are
 * passed through to handlers verbatim and may be ignored or logged.
 *
 * Naming follows the symbolic intent (KC_*). The values are byte-exact with
 * the SW050 enum. */
/* Values are byte-exact with SW050 `enum eKeyCodes` (CameraToolsU.h:542)
 * and the Reduced CMC's `KEY_*` defines (uc_camd_interface protocol.h).
 * Verified against both reference implementations 2026-06-22. */
typedef enum {
    /* --- Type-1 shot keys (0x01..0x12) — body's `value` field carries
     *     the shot number (1..100), except STORE_TIME_TO_SHOT where it's
     *     the fade time in tenths of a second.                            */
    CAMERAD_KC_STORE_SHOT            = 0x01,   /* kcUpdateS  — store current pos as shot N */
    CAMERAD_KC_STORE_NEXT            = 0x02,   /* kcStoreNextS — store as current_shot+1 */
    CAMERAD_KC_SWOOP                 = 0x07,   /* kcSwoopS  — swoop profile to shot N */
    CAMERAD_KC_CUT                   = 0x08,   /* kcCutS    — instant move to shot N */
    CAMERAD_KC_FADE                  = 0x09,   /* kcFadeS   — timed move to shot N */
    CAMERAD_KC_FADE_CUE              = 0x14,   /* kcFadeQ   — fade triggered by cue computer */
    CAMERAD_KC_CUT_CUE               = 0x15,   /* kcCutQ    — cut triggered by cue computer */

    /* --- Number pad (0x2C..0x39) — used for operator entry, not handled here */
    CAMERAD_KC_ENTER                 = 0x2C,
    CAMERAD_KC_CLEAR                 = 0x2D,
    CAMERAD_KC_DECIMAL_POINT         = 0x2E,   /* SW050 kcDecPointS = 0x2E (not 0x2F) */
    CAMERAD_KC_NUM_0                 = 0x30,
    CAMERAD_KC_NUM_9                 = 0x39,

    /* --- T-screen keys (0x60..0x63) — same semantics as the S equivalents
     *     above, but the panel marks the response as T-type (shorter body). */
    CAMERAD_KC_PRELOAD               = 0x60,   /* kcPreloadT */
    CAMERAD_KC_SWOOP_TO              = 0x61,   /* kcSwoopToT */
    CAMERAD_KC_CUT_TO                = 0x62,   /* kcCutToT */
    CAMERAD_KC_FADE_TO               = 0x63,   /* kcFadeToT */

    /* --- Joystick profile (0x80..0x82) */
    CAMERAD_KC_JOY_PROFILE_NORMAL    = 0x80,
    CAMERAD_KC_JOY_PROFILE_MEDIUM    = 0x81,
    CAMERAD_KC_JOY_PROFILE_FINE      = 0x82,

    /* --- Set fade time for the NEXT fade movement (legacy mechanism).
     *     The body's `value` field carries the time in tenths of a second. */
    CAMERAD_KC_STORE_TIME_TO_SHOT    = 0xA1,   /* kcLegacyStoreTimeToShot */

    /* --- Stop (0xF0/0xF1) */
    CAMERAD_KC_STOP                  = 0xF0,
    CAMERAD_KC_STOP_ALL              = 0xF1,   /* deprecated upstream but still arrives */
} camerad_key_code_t;

/* Camera status bits — eCamStatus, v1.3 width (uint16). See SW050
 * CAMERAD_Protocol.md §5.2 for the full bit list. */
#define CAMERAD_CAM_MOVING            (0x0001u)
#define CAMERAD_CAM_ON_SHOT           (0x0002u)
#define CAMERAD_CAM_ZOFF              (0x0004u)
#define CAMERAD_CAM_OP1               (0x0008u)
#define CAMERAD_CAM_OP2               (0x0010u)
#define CAMERAD_CAM_OP3               (0x0020u)
#define CAMERAD_CAM_PREV              (0x0040u)
#define CAMERAD_CAM_TREV              (0x0080u)

/* CMC status bits — eCMCStatus, v1.3 width (uint16). See §5.3. */
#define CAMERAD_CMC_REMOTE            (0x0000u)
#define CAMERAD_CMC_LOCAL             (0x0001u)
#define CAMERAD_CMC_EXT               (0x0004u)
#define CAMERAD_CMC_CONNECTION_OK     (0x0020u)
#define CAMERAD_CMC_REFERENCED        (0x0080u)
#define CAMERAD_CMC_EMERGENCY_STOP    (0x0800u)

/* Movement axis bitmap — eMovementAxes. Used in camerad_movement_t.axis_bitmap. */
#define CAMERAD_AXIS_PAN              (0x01u)
#define CAMERAD_AXIS_TILT             (0x02u)
#define CAMERAD_AXIS_ZOOM             (0x04u)
#define CAMERAD_AXIS_FOCUS            (0x08u)
#define CAMERAD_AXIS_X                (0x10u)
#define CAMERAD_AXIS_Y                (0x20u)
#define CAMERAD_AXIS_HEIGHT           (0x40u)
#define CAMERAD_AXIS_FADER            (0x80u)

/* Move type — eMoveType. */
#define CAMERAD_MOVE_NONE             (0x00u)
#define CAMERAD_MOVE_FADE             (0x01u)
#define CAMERAD_MOVE_CUT              (0x02u)
#define CAMERAD_MOVE_SWOOP            (0x04u)
#define CAMERAD_MOVE_FADE_CRCPS       (0x08u)

/*============================================================================
 *  Wire structs (packed, little-endian)
 *==========================================================================*/

/* 64-byte fixed header. Sent in every CAMERAD frame. */
typedef struct __attribute__((packed)) {
    char     magic[8];              /* "CAMERAD\0" */
    char     version[4];            /* "1.3\0" */
    uint32_t msg_command;           /* camerad_msg_t */
    uint32_t dest_device;           /* camerad_device_t */
    uint32_t dest_device_no;        /* 0 = any */
    char     return_address[16];    /* sender IP, NUL-terminated dotted quad */
    uint32_t return_port;
    uint32_t return_device;         /* sender's camerad_device_t */
    uint32_t return_device_no;
    uint32_t message_length;        /* total bytes incl. header */
    uint32_t message_id;            /* echoed in responses */
    uint32_t packet_id;             /* CMC sets to 0 on responses */
} camerad_header_t;

_Static_assert(sizeof(camerad_header_t) == CAMERAD_HEADER_SIZE,
               "CAMERAD header must be exactly 64 bytes");

/* KEYPRESS Type 1 body (5 bytes): shot store/recall, fade/cut/swoop keys.
 * value field is union{shot_no, time_to_shot} per key_code. */
typedef struct __attribute__((packed)) {
    uint8_t  key_code;              /* camerad_key_code_t */
    int32_t  value;                 /* shot_no or time_to_shot_ms */
} camerad_keypress_t1_t;
_Static_assert(sizeof(camerad_keypress_t1_t) == 5, "KP1 must be 5 bytes");

/* KEYPRESS Type 2 body (2 bytes): stop, set/restore limits, modifiers. */
typedef struct __attribute__((packed)) {
    uint8_t  key_code;
    uint8_t  status;                /* 0/1 for toggle states, etc. */
} camerad_keypress_t2_t;
_Static_assert(sizeof(camerad_keypress_t2_t) == 2, "KP2 must be 2 bytes");

/* KEYPRESS Type 3 body (42 bytes): T-screen full shot frame.
 *
 * Note: shot_type is uint8, not uint16, despite SW050's `usShotType` Hungarian
 * notation. The `us` prefix in SW050 is misleading here — verified against
 * Trunk/CMCapp/CamDMsgStructures.h `struct stKeyPressType3Msg`. The total
 * body width is 1 (key) + 1 (shot_type) + 10×4 (ints) = 42. */
typedef struct __attribute__((packed)) {
    uint8_t  key_code;
    uint8_t  shot_type;
    int32_t  shot_count;
    int32_t  show_no;
    int32_t  time_to_shot;
    int32_t  pan;
    int32_t  tilt;
    int32_t  focus;
    int32_t  zoom;
    int32_t  x;
    int32_t  y;
    int32_t  height;
} camerad_keypress_t3_t;
_Static_assert(sizeof(camerad_keypress_t3_t) == 42, "KP3 must be 42 bytes");

/* MOVEMENT body (9 bytes): joystick/fader/dial frame, sent at controller's
 * joystick rate (~25 ms). No ACK from CMC.
 * axis_bitmap says which of the eight axis values are valid this frame. */
typedef struct __attribute__((packed)) {
    uint8_t  axis_bitmap;           /* CAMERAD_AXIS_* mask */
    int8_t   pan;
    int8_t   tilt;
    int8_t   zoom;
    int8_t   focus;
    int8_t   x;
    int8_t   y;
    int8_t   height;
    int8_t   fader;
} camerad_movement_t;
_Static_assert(sizeof(camerad_movement_t) == 9, "MOVEMENT must be 9 bytes");

/* LIMIT request body (2 bytes). */
typedef struct __attribute__((packed)) {
    uint8_t  key_code;
    uint8_t  mode;                  /* eLimitMsgMode in SW050 */
} camerad_limit_req_t;
_Static_assert(sizeof(camerad_limit_req_t) == 2, "LIMIT req must be 2 bytes");

/* POLL response, S-type, v1.3 (22 bytes). */
typedef struct __attribute__((packed)) {
    uint8_t  camera_selected;       /* bool */
    int32_t  controller_no;         /* which controller currently owns */
    uint16_t camera_status;         /* CAMERAD_CAM_* bits */
    uint16_t cmc_status;            /* CAMERAD_CMC_* bits */
    int32_t  time_to_shot;          /* ms remaining in current move */
    int32_t  shot_no;               /* current shot # */
    int32_t  next_shot_no;          /* preloaded next shot # */
    uint8_t  move_type;             /* CAMERAD_MOVE_* */
} camerad_poll_resp_s_t;
_Static_assert(sizeof(camerad_poll_resp_s_t) == 22, "POLL resp S must be 22 bytes");

/* POLL response, T-type, v1.3 (14 bytes). T-type response drops the shot-
 * number fields (T-screen tracks its own shot table). */
typedef struct __attribute__((packed)) {
    uint8_t  camera_selected;
    int32_t  controller_no;
    uint16_t camera_status;
    uint16_t cmc_status;
    int32_t  time_to_shot;
    uint8_t  move_type;
} camerad_poll_resp_t_t;
_Static_assert(sizeof(camerad_poll_resp_t_t) == 14, "POLL resp T must be 14 bytes");

/* DESELECT response (5 bytes). */
typedef struct __attribute__((packed)) {
    uint8_t  camera_selected;
    int32_t  controller_no;
} camerad_deselect_resp_t;
_Static_assert(sizeof(camerad_deselect_resp_t) == 5, "DESELECT resp must be 5 bytes");

/* POSITION_REQ response (32 bytes, T-screen) — per SW050 stPositionalInfoMsg.
 * Field order is: time_to_shot first, then 7 axes (pan, tilt, focus, zoom,
 * x, y, height). SW050 declares all eight as uint32 (positions are
 * reinterpreted-as-signed by the caller); we follow suit so the codec is
 * a faithful reproduction. Cast to int32 in the caller when treating
 * positions as signed. */
typedef struct __attribute__((packed)) {
    uint32_t time_to_shot;
    uint32_t pan;
    uint32_t tilt;
    uint32_t focus;
    uint32_t zoom;
    uint32_t x;
    uint32_t y;
    uint32_t height;
} camerad_position_info_t;
_Static_assert(sizeof(camerad_position_info_t) == 32, "POSITION info must be 32 bytes");

/*============================================================================
 *  Header parse / build
 *==========================================================================*/

/* Validate and parse a 64-byte CAMERAD header from the network buffer.
 *
 * Returns true if:
 *   - buf is non-NULL and len >= CAMERAD_HEADER_SIZE,
 *   - the first 8 bytes match CAMERAD_MAGIC ("CAMERAD\0"),
 *   - the version string is recognised (currently: "1.3" only),
 *   - message_length is in [CAMERAD_HEADER_SIZE, CAMERAD_MAX_FRAME_SIZE].
 *
 * On success, copies the (validated) header into *out. The body, if any,
 * starts at buf + CAMERAD_HEADER_SIZE and is body_len = out->message_length
 * - CAMERAD_HEADER_SIZE bytes long. The caller is responsible for ensuring
 * the full message_length bytes have been received before parsing body
 * structs.
 *
 * On failure, *out is left untouched. */
bool camerad_parse_header(const uint8_t *buf, size_t len, camerad_header_t *out);

/* Build a response header into the first 64 bytes of out_buf, populating
 * the standard "echo" pattern:
 *   - magic + version copied from the request
 *   - msg_command set to the response opcode (caller decides — usually
 *     same as request)
 *   - dest_device/dest_device_no = request's return_device/return_device_no
 *   - return_address = our_ip (NUL-padded dotted quad)
 *   - return_port = our_port
 *   - return_device = CAMERAD_DEV_CMC
 *   - return_device_no = our_device_no
 *   - message_length = CAMERAD_HEADER_SIZE + body_len
 *   - message_id echoes the request's
 *   - packet_id = 0
 *
 * out_buf must have room for at least 64 bytes. The body (if any) is the
 * caller's responsibility to append starting at out_buf + 64. */
void camerad_build_response_header(uint8_t *out_buf,
                                   const camerad_header_t *request,
                                   uint32_t response_msg_command,
                                   uint16_t body_len,
                                   const char *our_ip,
                                   uint16_t our_port,
                                   uint32_t our_device_no);

/*============================================================================
 *  Helpers
 *==========================================================================*/

/* Format a 4-byte IP as a NUL-terminated dotted quad into out[16].
 * Returns the strlen of the resulting string (0..15). */
size_t camerad_format_ip(const uint8_t ip[4], char out[16]);

/* Parse a NUL-terminated dotted-quad IP from str into out[4].
 * Returns true on success, false on malformed input. */
bool camerad_parse_ip(const char *str, uint8_t out[4]);

/* Returns the expected body length for the given opcode, or 0 if the opcode
 * has a variable-length / response-only body / is unknown. Used by handlers
 * for length validation. */
uint16_t camerad_request_body_len(camerad_msg_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAMERAD_H */
