/*
 * app/cia402 — Phase 4 implementation.
 *
 * Cyclic exchange + OD pipeline + CRC-16/Modbus frame codec over
 * bsp/motor_spi. Wire format per Interface/mc_if_protocol.h.
 *
 * Layering: this module depends only on bsp/motor_spi, bsp/time and the
 * shared Interface headers — nothing in app/. Upper layers (app/od,
 * app/motor_ctrl) call DOWN into here; this module never calls UP into
 * them. The latest CYCLIC_STATUS is exposed via cia402_take_cyclic_status,
 * which app/od polls each tick.
 */

#include "cia402.h"

#include "bsp/motor_spi/motor_spi.h"
#include "bsp/time/time.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Frame layout (all little-endian, packed):
 *   bytes  0..1     sync (0xA55A)
 *   bytes  2        version (1)
 *   bytes  3        message_type
 *   bytes  4..5     payload_length
 *   bytes  6..7     sequence
 *   bytes  8..9     header_crc (CRC-16/Modbus over bytes 0..7)
 *   bytes 10..(10+plen-1)  payload
 *   bytes (10+plen)..(10+plen+1)  payload_crc (CRC-16/Modbus over payload)
 *   bytes (10+plen+2)..63  zero padding
 *---------------------------------------------------------------------------*/

#define HDR_OFFSET_SYNC       0
#define HDR_OFFSET_VERSION    2
#define HDR_OFFSET_MSGTYPE    3
#define HDR_OFFSET_PLEN       4
#define HDR_OFFSET_SEQ        6
#define HDR_OFFSET_HCRC       8
#define PAYLOAD_OFFSET        10

#define CYCLE_PERIOD_MS       1u
/* Bounded by the GUI's 50 ms retransmit window (see
 * Interface/gui/mc_gui/client.py RETRANSMIT_TIMEOUT). If we let the OD
 * response take longer than that, the GUI retransmits while our
 * s_pending.in_flight is still true on the CMC side, which used to
 * trigger spurious queue-full errors. Matches the Interface command-
 * timeout (MC_IF_COMMAND_TIMEOUT_MS = 30 ms) which is the right
 * physical-side budget anyway. */
#define OD_RESPONSE_TIMEOUT_MS  30u
#define SPI_TIMEOUT_MS        5u

/*----------------------------------------------------------------------------
 * Module state
 *---------------------------------------------------------------------------*/

static uint8_t s_tx[MOTOR_SPI_FRAME_BYTES];
static uint8_t s_rx[MOTOR_SPI_FRAME_BYTES];

static bool     s_initialised  = false;
static uint32_t s_last_tick_ms = 0;
static uint16_t s_sequence     = 0;     /* per-frame sequence number */
static uint32_t s_command_counter = 0;  /* per-cyclic command_counter */

/* CYCLIC_CMD payload state (set by motor_ctrl in future; zeros for now
 * meaning "controlword=0, mode=0, all targets 0" — a quiescent command). */
static MC_IfCyclicCommand_t s_cmd = { 0 };

/* Latest cyclic status received. `s_status_fresh` is the consumer-facing
 * one-shot flag used by cia402_take_cyclic_status; the data itself stays
 * around indefinitely so cia402_peek_cyclic_status can serve diagnostics
 * without racing the consumer. `s_status_ever` says whether we've ever
 * seen one, so peek can return a meaningful false at boot. */
static bool                       s_status_fresh = false;
static bool                       s_status_ever  = false;
static MC_IfCyclicStatusHeader_t  s_status_hdr;
static uint8_t                    s_status_blob[MC_IF_TLM_BLOB_MAX];
static uint8_t                    s_status_blob_len = 0;

/* OD request slot (one in flight). */
typedef enum {
    OD_IDLE         = 0,
    OD_REQ_PENDING  = 1,    /* request not yet transmitted */
    OD_AWAITING     = 2,    /* request transmitted; response expected */
    OD_COMPLETE     = 3,    /* response received (or timed out); poll() will collect */
} od_state_t;

static struct {
    od_state_t          state;
    cia402_od_handle_t  handle;
    bool                is_read;
    uint16_t            index;
    uint8_t             subindex;
    MC_IfOdType_t       type;
    uint8_t             tx_data[8];
    uint8_t             tx_data_len;
    uint16_t            req_seq;
    uint32_t            deadline_ms;
    MC_IfOdResult_t     result;
    uint8_t             resp_data[8];
    uint8_t             resp_data_len;
} s_od;

static cia402_od_handle_t s_next_handle = 1;
static cia402_stats_t     s_stats;

/*----------------------------------------------------------------------------
 * CRC-16/Modbus (poly 0xA001 reflected = 0x8005, init 0xFFFF, no final XOR).
 * Bit-by-bit; small and self-contained. Replace with a table-driven version
 * if profile shows it as a hot path.
 *---------------------------------------------------------------------------*/

static uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else          crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/*----------------------------------------------------------------------------
 * Little-endian field helpers
 *---------------------------------------------------------------------------*/

static inline uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void     wr_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }

/*----------------------------------------------------------------------------
 * Frame build / validate
 *---------------------------------------------------------------------------*/

static void build_frame(uint8_t msg_type, uint16_t seq, const void *payload, uint16_t plen)
{
    memset(s_tx, 0, sizeof(s_tx));

    wr_u16(s_tx + HDR_OFFSET_SYNC,    MC_IF_SYNC_WORD);
    s_tx[HDR_OFFSET_VERSION] = MC_IF_PROTOCOL_VERSION;
    s_tx[HDR_OFFSET_MSGTYPE] = msg_type;
    wr_u16(s_tx + HDR_OFFSET_PLEN,    plen);
    wr_u16(s_tx + HDR_OFFSET_SEQ,     seq);

    /* Header CRC over bytes 0..7. */
    uint16_t hcrc = crc16_modbus(s_tx, 8);
    wr_u16(s_tx + HDR_OFFSET_HCRC, hcrc);

    /* Payload + payload CRC. */
    if (plen > 0 && payload != NULL) {
        memcpy(s_tx + PAYLOAD_OFFSET, payload, plen);
    }
    uint16_t pcrc = crc16_modbus(s_tx + PAYLOAD_OFFSET, plen);
    wr_u16(s_tx + PAYLOAD_OFFSET + plen, pcrc);

    /* Remaining bytes left as zero pad. */
}

typedef struct {
    bool     ok;
    uint8_t  msg_type;
    uint16_t seq;
    uint16_t plen;
    const uint8_t *payload;     /* into s_rx */
} rx_view_t;

static rx_view_t validate_rx(void)
{
    rx_view_t v = { .ok = false };

    uint16_t sync = rd_u16(s_rx + HDR_OFFSET_SYNC);
    if (sync != MC_IF_SYNC_WORD) { s_stats.rx_bad_sync++;    return v; }
    if (s_rx[HDR_OFFSET_VERSION] != MC_IF_PROTOCOL_VERSION) {
        s_stats.rx_bad_version++; return v;
    }
    uint16_t got_hcrc = rd_u16(s_rx + HDR_OFFSET_HCRC);
    uint16_t calc_hcrc = crc16_modbus(s_rx, 8);
    if (got_hcrc != calc_hcrc) { s_stats.rx_bad_header_crc++; return v; }

    uint16_t plen = rd_u16(s_rx + HDR_OFFSET_PLEN);
    if ((uint32_t)plen > MC_IF_MAX_PAYLOAD) {
        s_stats.rx_bad_length++; return v;
    }

    uint16_t got_pcrc = rd_u16(s_rx + PAYLOAD_OFFSET + plen);
    uint16_t calc_pcrc = crc16_modbus(s_rx + PAYLOAD_OFFSET, plen);
    if (got_pcrc != calc_pcrc) { s_stats.rx_bad_payload_crc++; return v; }

    v.ok       = true;
    v.msg_type = s_rx[HDR_OFFSET_MSGTYPE];
    v.seq      = rd_u16(s_rx + HDR_OFFSET_SEQ);
    v.plen     = plen;
    v.payload  = s_rx + PAYLOAD_OFFSET;
    s_stats.rx_frames_valid++;
    return v;
}

/*----------------------------------------------------------------------------
 * Cyclic command composition
 *---------------------------------------------------------------------------*/

static void send_cyclic_cmd(uint16_t seq)
{
    s_cmd.command_counter = ++s_command_counter;
    build_frame(MC_IF_MSG_CYCLIC_CMD, seq, &s_cmd, (uint16_t)sizeof(s_cmd));
}

static void send_od_read_req(uint16_t seq)
{
    MC_IfOdReadReq_t r = {
        .index = s_od.index,
        .subindex = s_od.subindex,
        .expected_type = (uint8_t)s_od.type,
    };
    build_frame(MC_IF_MSG_OD_READ_REQ, seq, &r, (uint16_t)sizeof(r));
    s_od.req_seq     = seq;
    s_od.state       = OD_AWAITING;
    s_od.deadline_ms = time_ms() + OD_RESPONSE_TIMEOUT_MS;
}

static void send_od_write_req(uint16_t seq)
{
    MC_IfOdWriteReq_t w = {
        .index = s_od.index,
        .subindex = s_od.subindex,
        .type = (uint8_t)s_od.type,
        .data_length = s_od.tx_data_len,
        .reserved = 0,
    };
    memcpy(w.data, s_od.tx_data, sizeof(w.data));
    build_frame(MC_IF_MSG_OD_WRITE_REQ, seq, &w, (uint16_t)sizeof(w));
    s_od.req_seq     = seq;
    s_od.state       = OD_AWAITING;
    s_od.deadline_ms = time_ms() + OD_RESPONSE_TIMEOUT_MS;
}

/*----------------------------------------------------------------------------
 * Response handlers
 *---------------------------------------------------------------------------*/

static void handle_cyclic_status(const uint8_t *payload, uint16_t plen)
{
    if (plen < MC_IF_STATUS_HEADER_SIZE) return;

    memcpy(&s_status_hdr, payload, MC_IF_STATUS_HEADER_SIZE);

    uint8_t blob_len = s_status_hdr.map_byte_count;
    if (blob_len > MC_IF_TLM_BLOB_MAX) blob_len = MC_IF_TLM_BLOB_MAX;
    if ((uint32_t)MC_IF_STATUS_HEADER_SIZE + blob_len > plen) {
        blob_len = (uint8_t)(plen - MC_IF_STATUS_HEADER_SIZE);
    }

    if (blob_len > 0) {
        memcpy(s_status_blob, payload + MC_IF_STATUS_HEADER_SIZE, blob_len);
    }
    s_status_blob_len = blob_len;
    s_status_fresh    = true;
    s_status_ever     = true;
}

static void complete_od(MC_IfOdResult_t result, const uint8_t *data, uint8_t data_len)
{
    s_od.result = result;
    s_od.resp_data_len = (data_len > sizeof(s_od.resp_data))
                        ? (uint8_t)sizeof(s_od.resp_data) : data_len;
    if (s_od.resp_data_len > 0 && data != NULL) {
        memcpy(s_od.resp_data, data, s_od.resp_data_len);
    }
    s_od.state = OD_COMPLETE;
}

static void handle_od_read_resp(const uint8_t *payload, uint16_t plen, uint16_t rx_seq)
{
    if (s_od.state != OD_AWAITING || rx_seq != s_od.req_seq) return;
    if (plen < 6) { complete_od(MC_IF_OD_ERR_SIZE, NULL, 0); return; }

    MC_IfOdReadResp_t r;
    memset(&r, 0, sizeof(r));
    size_t copy = plen < sizeof(r) ? plen : sizeof(r);
    memcpy(&r, payload, copy);

    uint8_t dl = (r.data_length > sizeof(r.data)) ? (uint8_t)sizeof(r.data) : r.data_length;
    complete_od((MC_IfOdResult_t)r.result, r.data, dl);
}

static void handle_od_write_resp(const uint8_t *payload, uint16_t plen, uint16_t rx_seq)
{
    if (s_od.state != OD_AWAITING || rx_seq != s_od.req_seq) return;
    if (plen < 4) { complete_od(MC_IF_OD_ERR_SIZE, NULL, 0); return; }

    MC_IfOdWriteResp_t r;
    memcpy(&r, payload, sizeof(r));
    complete_od((MC_IfOdResult_t)r.result, NULL, 0);
}

static void handle_error(const uint8_t *payload, uint16_t plen)
{
    if (plen < (uint16_t)sizeof(MC_IfError_t)) return;
    MC_IfError_t e;
    memcpy(&e, payload, sizeof(e));

    /* If the slave is reporting an error for our in-flight OD request,
     * complete it with a meaningful result. The Interface uses
     * MC_IF_ERR_OD as the class and stuffs MC_IfOdResult_t into detail. */
    if (s_od.state == OD_AWAITING && e.ref_sequence == s_od.req_seq) {
        MC_IfOdResult_t res = (e.error_class == MC_IF_ERR_OD)
                            ? (MC_IfOdResult_t)e.detail
                            : MC_IF_OD_ERR_CALLBACK;
        complete_od(res, NULL, 0);
    }
    /* Other ERROR causes (bad sync, version etc. on a previous tx) are
     * logged in the rx_bad_* counters above; nothing further to do here. */
}

/*----------------------------------------------------------------------------
 * Public lifecycle
 *---------------------------------------------------------------------------*/

void cia402_init(void)
{
    memset(&s_cmd,    0, sizeof(s_cmd));
    memset(&s_od,     0, sizeof(s_od));
    memset(&s_stats,  0, sizeof(s_stats));
    memset(s_status_blob, 0, sizeof(s_status_blob));
    s_status_blob_len = 0;
    s_status_fresh    = false;
    s_status_ever     = false;
    s_sequence        = 0;
    s_command_counter = 0;
    s_next_handle     = 1;
    s_last_tick_ms    = time_ms();
    s_initialised     = true;

    motor_spi_init();
}

void cia402_tick(void)
{
    if (!s_initialised) return;

    /* Drift-tolerant rate-limit to MC_IF_CYCLIC_RATE_HZ = 1 kHz: after a
     * main-loop stall we send ONE frame and resume the cadence — no
     * catch-up burst onto SPI3. Long-term cadence drifts forward by the
     * total stall time; the motor MCU's contract only mandates
     * "no silence > MC_IF_COMMAND_TIMEOUT_MS (30 ms)", so a few ms of
     * drift per minute is harmless. Bursts onto the SPI bus are not,
     * because the slave's DMA rearm path can't absorb them (see
     * Interface/REQUESTS.md REQ-0007). */
    uint32_t now_ms = time_ms();
    uint32_t gap_ms = now_ms - s_last_tick_ms;
    if (gap_ms < CYCLE_PERIOD_MS) return;
    s_last_tick_ms = now_ms;

    /* Track scheduling drift for diagnosis. */
    if (gap_ms > s_stats.max_cycle_gap_ms) s_stats.max_cycle_gap_ms = gap_ms;
    if (gap_ms > CYCLE_PERIOD_MS) {
        s_stats.late_cycles++;
        s_stats.total_drift_ms += (gap_ms - CYCLE_PERIOD_MS);
    }

    /* Compose the next frame. OD request takes precedence over cyclic
     * (it preempts one cyclic cycle, per the Interface model). */
    uint16_t seq = ++s_sequence;
    if (s_od.state == OD_REQ_PENDING) {
        if (s_od.is_read) send_od_read_req(seq);
        else              send_od_write_req(seq);
    } else {
        send_cyclic_cmd(seq);
    }

    /* Transfer. */
    int32_t rc = motor_spi_transfer(s_tx, s_rx, SPI_TIMEOUT_MS);
    s_stats.tx_frames++;
    if (rc < 0) {
        s_stats.spi_errors++;
        /* Don't try to parse rx — it's stale/garbage. */
    } else {
        /* Parse rx. */
        rx_view_t v = validate_rx();
        if (v.ok) {
            switch (v.msg_type) {
                case MC_IF_MSG_CYCLIC_STATUS:
                    handle_cyclic_status(v.payload, v.plen);
                    break;
                case MC_IF_MSG_OD_READ_RESP:
                    handle_od_read_resp(v.payload, v.plen, v.seq);
                    break;
                case MC_IF_MSG_OD_WRITE_RESP:
                    handle_od_write_resp(v.payload, v.plen, v.seq);
                    break;
                case MC_IF_MSG_ERROR:
                    handle_error(v.payload, v.plen);
                    break;
                case MC_IF_MSG_HEARTBEAT:
                default:
                    /* Nothing to do. */
                    break;
            }
        }
    }

    /* OD response timeout. */
    if (s_od.state == OD_AWAITING && time_after(time_ms(), s_od.deadline_ms)) {
        complete_od(MC_IF_OD_ERR_NOT_READY, NULL, 0);
        s_stats.od_timeouts++;
    }
}

/*----------------------------------------------------------------------------
 * OD pipeline API
 *---------------------------------------------------------------------------*/

static cia402_od_handle_t alloc_handle(void)
{
    cia402_od_handle_t h = s_next_handle;
    s_next_handle = (cia402_od_handle_t)(s_next_handle + 1);
    if (s_next_handle == CIA402_OD_HANDLE_INVALID) s_next_handle = 1;
    return h;
}

cia402_od_handle_t cia402_od_read_begin(uint16_t idx, uint8_t sub, MC_IfOdType_t type)
{
    if (!s_initialised || s_od.state != OD_IDLE) return CIA402_OD_HANDLE_INVALID;

    s_od.handle   = alloc_handle();
    s_od.is_read  = true;
    s_od.index    = idx;
    s_od.subindex = sub;
    s_od.type     = type;
    s_od.tx_data_len = 0;
    s_od.state    = OD_REQ_PENDING;
    return s_od.handle;
}

cia402_od_handle_t cia402_od_write_begin(uint16_t idx, uint8_t sub, MC_IfOdType_t type,
                                         const void *data, uint8_t len)
{
    if (!s_initialised || s_od.state != OD_IDLE) return CIA402_OD_HANDLE_INVALID;
    if (len > sizeof(s_od.tx_data) || (len > 0 && data == NULL)) return CIA402_OD_HANDLE_INVALID;

    s_od.handle   = alloc_handle();
    s_od.is_read  = false;
    s_od.index    = idx;
    s_od.subindex = sub;
    s_od.type     = type;
    s_od.tx_data_len = len;
    if (len > 0) memcpy(s_od.tx_data, data, len);
    s_od.state    = OD_REQ_PENDING;
    return s_od.handle;
}

bool cia402_od_poll(cia402_od_handle_t h,
                    MC_IfOdResult_t *out_result,
                    void *out_data, uint8_t *out_len)
{
    if (h == CIA402_OD_HANDLE_INVALID || h != s_od.handle) return false;
    if (s_od.state != OD_COMPLETE) return false;

    if (out_result) *out_result = s_od.result;
    if (out_data && out_len) {
        uint8_t cap = *out_len;
        uint8_t n   = (s_od.resp_data_len < cap) ? s_od.resp_data_len : cap;
        if (n > 0) memcpy(out_data, s_od.resp_data, n);
        *out_len = n;
    } else if (out_len) {
        *out_len = 0;
    }

    s_od.state  = OD_IDLE;
    s_od.handle = CIA402_OD_HANDLE_INVALID;
    return true;
}

/*----------------------------------------------------------------------------
 * Cyclic status accessor (polled by app/od)
 *---------------------------------------------------------------------------*/

bool cia402_take_cyclic_status(MC_IfCyclicStatusHeader_t *out_hdr,
                               uint8_t *out_blob,
                               uint8_t *out_blob_len)
{
    if (!s_status_fresh) return false;

    if (out_hdr) *out_hdr = s_status_hdr;
    if (out_blob && out_blob_len) {
        uint8_t n = s_status_blob_len;
        memcpy(out_blob, s_status_blob, n);
        *out_blob_len = n;
    } else if (out_blob_len) {
        *out_blob_len = 0;
    }
    s_status_fresh = false;
    return true;
}

bool cia402_peek_cyclic_status(MC_IfCyclicStatusHeader_t *out_hdr,
                               uint8_t *out_blob,
                               uint8_t *out_blob_len)
{
    if (!s_status_ever) return false;

    if (out_hdr) *out_hdr = s_status_hdr;
    if (out_blob && out_blob_len) {
        uint8_t n = s_status_blob_len;
        memcpy(out_blob, s_status_blob, n);
        *out_blob_len = n;
    } else if (out_blob_len) {
        *out_blob_len = 0;
    }
    /* Does NOT clear s_status_fresh — peek must not race with take. */
    return true;
}

/*----------------------------------------------------------------------------
 * Cyclic command setter
 *
 * Copies the v3 streaming fields (controlword, velocity_setpoint) from `cmd`
 * into our private s_cmd. command_counter is left as cia402's own counter
 * (incremented in send_cyclic_cmd() on every transmission); the caller's
 * command_counter field is ignored.
 *
 * Setup parameters (mode_of_operation, target_position, target_torque,
 * target_position_time_ms, profile_*) are NOT carried in the cyclic command
 * in protocol v3. axis_manager writes them via the SDO pipeline
 * (cia402_od_write_begin) instead, and triggers PROFILE_POSITION execution
 * by setting MC_IF_CW_NEW_SETPOINT in `cmd->controlword` for one cycle.
 * Velocity modes (PROFILE_VELOCITY) follow `velocity_setpoint` live with no
 * trigger.
 *
 * Single-owner: only app/axis_manager calls this. See cia402.h notes.
 *---------------------------------------------------------------------------*/

void cia402_set_cyclic_cmd(const MC_IfCyclicCommand_t *cmd)
{
    if (!s_initialised || cmd == NULL) return;
    s_cmd.controlword       = cmd->controlword;
    s_cmd.velocity_setpoint = cmd->velocity_setpoint;
    /* command_counter is NOT touched — cia402 owns it. */
}

/*----------------------------------------------------------------------------
 * Stats
 *---------------------------------------------------------------------------*/

void cia402_get_stats(cia402_stats_t *out)
{
    if (out) *out = s_stats;
}
