#ifndef MC_IF_PROTOCOL_H
#define MC_IF_PROTOCOL_H
#include <stdint.h>

/** @file mc_if_protocol.h
 *  @brief SHARED inter-MCU boundary contract: SPI framed protocol (wire format).
 *
 *  This header is the single source of truth for the SPI link between the network MCU
 *  (SPI master, e.g. Lightweight_CMC / STM32G431) and the motor-control MCU (SPI slave,
 *  Generic_motor_controller / STM32G474). BOTH firmwares include this file unchanged. It
 *  defines only the wire format (constants, enums, packed structs) -- no functions, no
 *  platform/HAL dependencies -- so it is portable to firmware and to a host/PC tool.
 *
 *  See INTERFACE_SPEC.md for the full contract (transport, transaction model, semantics).
 *  Any change to the layout MUST bump MC_IF_PROTOCOL_VERSION.
 *
 *  Byte order: little-endian. All multi-byte fields are LE; structs are packed.
 */

#define MC_IF_PROTOCOL_VERSION (4u)  /* v4: cyclic header gained position_actual_scaled + movement_status (REQ-0013/ADR-033) */
#define MC_IF_SYNC_WORD        (0xA55Au)

/* --- SPI transport unit (fixed-size frame per SPI transaction; see spec) --- */
#define MC_IF_FRAME_SIZE   (64u)                 /* bytes clocked per SPI transaction */
#define MC_IF_HEADER_SIZE  (10u)                 /* sizeof(MC_IfFrameHeader_t) */
#define MC_IF_FOOTER_SIZE  (2u)                  /* sizeof(MC_IfFrameFooter_t) */
#define MC_IF_MAX_PAYLOAD  (MC_IF_FRAME_SIZE - MC_IF_HEADER_SIZE - MC_IF_FOOTER_SIZE) /* 52 */

/* --- Operational defaults (agreement-only; not in the wire layout) ---------
 * Both MCUs build against these values so the slave's command-timeout and
 * the master's cyclic cadence agree. Changing them does NOT change the byte
 * pattern on the wire, so no MC_IF_PROTOCOL_VERSION bump is required when
 * tuning these. They may be raised after bring-up characterisation. */
#define MC_IF_CYCLIC_RATE_HZ        (1000u)      /* master cyclic exchange rate */
#define MC_IF_CYCLIC_PERIOD_US      (1000000u / MC_IF_CYCLIC_RATE_HZ)
#define MC_IF_COMMAND_TIMEOUT_MS    (30u)        /* slave safe-state / dead-man if no valid CYCLIC_CMD in this window (~30 cycles at 1 kHz) */
#define MC_IF_SPI_CLOCK_HZ_INITIAL  (6000000u)   /* initial SPI clock during bring-up; raise after stable */
#define MC_IF_SPI_CLOCK_HZ_MAX      (10000000u)  /* not to exceed without re-characterisation */

/** @brief Message types (frame.message_type). */
typedef enum
{
    MC_IF_MSG_CYCLIC_CMD    = 0x01,  /* master -> slave: controlword, mode, targets, limits */
    MC_IF_MSG_CYCLIC_STATUS = 0x02,  /* slave -> master: statusword, actuals, error code */
    MC_IF_MSG_OD_READ_REQ   = 0x10,  /* master -> slave: read index/subindex */
    MC_IF_MSG_OD_READ_RESP  = 0x11,  /* slave -> master: read result + value */
    MC_IF_MSG_OD_WRITE_REQ  = 0x12,  /* master -> slave: write index/subindex */
    MC_IF_MSG_OD_WRITE_RESP = 0x13,  /* slave -> master: write result */
    MC_IF_MSG_HEARTBEAT     = 0x20,  /* both: link supervision / idle clocking */
    MC_IF_MSG_ERROR         = 0x7F   /* both: protocol/object error report */
} MC_IfMessageType_t;

/** @brief OD access result / abort code (OD read/write responses). */
typedef enum
{
    MC_IF_OD_OK            = 0x00,
    MC_IF_OD_ERR_NO_OBJECT = 0x01,  /* index not found */
    MC_IF_OD_ERR_NO_SUB    = 0x02,  /* subindex not found */
    MC_IF_OD_ERR_ACCESS    = 0x03,  /* write to RO / read from WO */
    MC_IF_OD_ERR_TYPE      = 0x04,  /* type mismatch */
    MC_IF_OD_ERR_RANGE     = 0x05,  /* value out of min/max */
    MC_IF_OD_ERR_SIZE      = 0x06,  /* data length mismatch */
    MC_IF_OD_ERR_CALLBACK  = 0x07,  /* read/write callback failed */
    MC_IF_OD_ERR_NOT_READY = 0x08   /* state does not allow this access now */
} MC_IfOdResult_t;

/** @brief Protocol-level error classes (ERROR message). */
typedef enum
{
    MC_IF_ERR_NONE        = 0x00,
    MC_IF_ERR_BAD_SYNC    = 0x01,
    MC_IF_ERR_BAD_VERSION = 0x02,
    MC_IF_ERR_HEADER_CRC  = 0x03,
    MC_IF_ERR_PAYLOAD_CRC = 0x04,
    MC_IF_ERR_BAD_LENGTH  = 0x05,
    MC_IF_ERR_UNKNOWN_MSG = 0x06,
    MC_IF_ERR_SEQUENCE    = 0x07,
    MC_IF_ERR_OD          = 0x08,  /* detail carries an MC_IfOdResult_t */
    MC_IF_ERR_INTERNAL    = 0x09
} MC_IfErrorClass_t;

/** @brief Node (motor-control MCU) state, reported in heartbeat/status. */
typedef enum
{
    MC_IF_NODE_INIT        = 0x00,
    MC_IF_NODE_DISABLED    = 0x01,
    MC_IF_NODE_READY       = 0x02,
    MC_IF_NODE_RUNNING     = 0x03,
    MC_IF_NODE_QUICK_STOP  = 0x04,
    MC_IF_NODE_FAULT       = 0x05,
    MC_IF_NODE_CALIBRATING = 0x06
} MC_IfNodeState_t;

/* ===== Frame ===== */

/** @brief Frame header (10 bytes). header_crc = CRC16/Modbus over the preceding 8 bytes. */
typedef struct __attribute__((packed))
{
    uint16_t sync;           /* MC_IF_SYNC_WORD */
    uint8_t  version;        /* MC_IF_PROTOCOL_VERSION */
    uint8_t  message_type;   /* MC_IfMessageType_t */
    uint16_t payload_length; /* bytes of payload (0..MC_IF_MAX_PAYLOAD) */
    uint16_t sequence;       /* request sequence counter (wraps) */
    uint16_t header_crc;     /* CRC16/Modbus over bytes [0..7] */
} MC_IfFrameHeader_t;

/** @brief Frame footer (2 bytes). payload_crc = CRC16/Modbus over the payload bytes. */
typedef struct __attribute__((packed))
{
    uint16_t payload_crc;
} MC_IfFrameFooter_t;

/* ===== Payloads ===== */

/** @brief CYCLIC_CMD payload (master -> slave) -- streaming-only fields.
 *
 *  Protocol v3: the cyclic command carries ONLY what genuinely needs to stream
 *  every cycle. All setup parameters (mode_of_operation, target_position,
 *  target_torque, target_position_time_ms, profile_velocity,
 *  profile_acceleration, profile_deceleration) travel via SDO over the SPI OD
 *  pipeline and are stored on the motor MCU until a move is triggered.
 *
 *  Streaming fields:
 *   - @c controlword carries the urgent / live bits: ENABLE, QUICK_STOP,
 *     FAULT_RESET, HALT, and NEW_SETPOINT. NEW_SETPOINT (bit 4, CiA-402
 *     standard) rising-edges trigger the motor MCU to execute the most
 *     recently configured move.
 *   - @c velocity_setpoint is the live velocity demand in scaled rad/s
 *     (LSB = MC_IF_VEL_SCALE, same units as OD 0x60FF). Always present;
 *     the motor MCU applies it as the active velocity target whenever in a
 *     velocity-class mode (PROFILE_VELOCITY) and enabled. Steady value =
 *     constant speed; varying value = jog / joystick. The cyclic value is
 *     authoritative — `0x60FF target_velocity` (SDO) is informational only.
 *   - @c command_counter is the LCMC's monotonic heartbeat; the motor MCU's
 *     command-timeout dead-man fires if it stops advancing for more than
 *     MC_IF_COMMAND_TIMEOUT_MS.
 *
 *  Total: 10 bytes of payload per cyclic transaction.
 */
typedef struct __attribute__((packed))
{
    uint16_t controlword;            /* OD 0x6040 -- streaming bits only */
    int32_t  velocity_setpoint;      /* live demand, ×MC_IF_VEL_SCALE rad/s */
    uint32_t command_counter;        /* monotonic; slave dead-man */
} MC_IfCyclicCommand_t;

/** @brief CYCLIC_STATUS / telemetry frame HEADER (slave -> master), 12 bytes.
 *
 *  The configurable telemetry blob follows immediately in the payload: @c map_byte_count bytes,
 *  holding the values of the OD entries in the active telemetry map (0x2A00), packed in order,
 *  little-endian. The master/host knows the layout because it configured the map; @c map_version
 *  lets it detect a remap and re-column. The standard actuals (position/velocity/torque) are no
 *  longer fixed here -- they are PDO-mappable OD entries (0x6064/0x606C/0x6077) included in the
 *  map (a default map typically lists them first). See INTERFACE_SPEC.md "Telemetry mapping".
 */
typedef struct __attribute__((packed))
{
    uint16_t statusword;     /* OD 0x6041 */
    int8_t   mode_display;   /* OD 0x6061 */
    uint8_t  node_state;     /* MC_IfNodeState_t */
    uint16_t error_code;     /* OD 0x603F */
    uint8_t  map_version;    /* increments on every telemetry-map change */
    uint8_t  map_byte_count; /* bytes of mapped blob following this header (0..MC_IF_TLM_BLOB_MAX) */
    uint32_t status_counter; /* echoes the last accepted command_counter */
    int32_t  position_actual_scaled; /* v4: OD 0x6064, MC_IF_POS_SCALE (1e-5 rad/LSB) -- always present */
    uint16_t movement_status;        /* v4: MC_IF_MOVE_* bits (REQ-0013 / ADR-033) */
    /* uint8_t mapped[map_byte_count];  -- telemetry blob, packed LE per the 0x2A00 map */
} MC_IfCyclicStatusHeader_t;

#define MC_IF_STATUS_HEADER_SIZE (18u)   /* v4 (was 12) -- MC_IF_TLM_BLOB_MAX recomputes 40 -> 34 */
#define MC_IF_TLM_BLOB_MAX       (MC_IF_MAX_PAYLOAD - MC_IF_STATUS_HEADER_SIZE) /* 34 bytes */

/* movement_status bits (cyclic header, v4; REQ-0013/ADR-033). 0x0004/0x0008 (setpoint accepted/
 * complete, proposed in REQ-0013) and 0x0040..0x8000 are reserved. */
#define MC_IF_MOVE_MOVING       (0x0001u)  /* axis in motion (drive enabled & |vel demand| or |vel meas| > ~0.01 rad/s) */
#define MC_IF_MOVE_ON_TARGET    (0x0002u)  /* position-loop target reached (mirrors SW_TARGET_REACHED, motor-trajectory terms) */
#define MC_IF_MOVE_AT_LIMIT_LO  (0x0010u)  /* soft min-position limit hit -- reserved, 0 until motor soft limits exist */
#define MC_IF_MOVE_AT_LIMIT_HI  (0x0020u)  /* soft max-position limit hit -- reserved, 0 until motor soft limits exist */

/** @brief OD_READ_REQ payload (master -> slave). */
typedef struct __attribute__((packed))
{
    uint16_t index;
    uint8_t  subindex;
    uint8_t  expected_type;          /* MC_IfOdType_t (mc_if_od.h); 0 = any */
} MC_IfOdReadReq_t;

/** @brief OD_READ_RESP payload (slave -> master). Value is little-endian in data[]. */
typedef struct __attribute__((packed))
{
    uint16_t index;
    uint8_t  subindex;
    uint8_t  type;                   /* actual MC_IfOdType_t */
    uint8_t  result;                 /* MC_IfOdResult_t */
    uint8_t  data_length;            /* valid bytes in data (1,2,4) */
    uint8_t  data[8];                /* value, LE (float32 uses 4 bytes) */
} MC_IfOdReadResp_t;

/** @brief OD_WRITE_REQ payload (master -> slave). Value is little-endian in data[]. */
typedef struct __attribute__((packed))
{
    uint16_t index;
    uint8_t  subindex;
    uint8_t  type;                   /* MC_IfOdType_t */
    uint8_t  data_length;            /* valid bytes in data (1,2,4) */
    uint8_t  reserved;
    uint8_t  data[8];                /* value, LE */
} MC_IfOdWriteReq_t;

/** @brief OD_WRITE_RESP payload (slave -> master). */
typedef struct __attribute__((packed))
{
    uint16_t index;
    uint8_t  subindex;
    uint8_t  result;                 /* MC_IfOdResult_t */
} MC_IfOdWriteResp_t;

/** @brief HEARTBEAT payload (either direction; also used to clock out a pending response). */
typedef struct __attribute__((packed))
{
    uint8_t  node_state;             /* MC_IfNodeState_t (slave); 0 from master */
    uint8_t  flags;                  /* bit0 = response pending on this node's MISO */
    uint16_t seq_echo;               /* last sequence seen */
    uint32_t counter;                /* free-running heartbeat counter */
} MC_IfHeartbeat_t;

/** @brief ERROR payload (either direction). */
typedef struct __attribute__((packed))
{
    uint8_t  error_class;            /* MC_IfErrorClass_t */
    uint8_t  detail;                 /* e.g. MC_IfOdResult_t when error_class == MC_IF_ERR_OD */
    uint16_t ref_sequence;           /* sequence of the offending frame */
    uint16_t ref_index;              /* OD index if relevant, else 0 */
    uint8_t  ref_subindex;
    uint8_t  reserved;
} MC_IfError_t;

#endif /* MC_IF_PROTOCOL_H */
