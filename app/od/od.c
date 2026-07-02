/*
 * app/od — Phase 5 implementation.
 *
 * Wire format: Interface/NETWORK_UDP_SPEC.md (authoritative). Summary:
 *   8-byte MC_UdpHeader_t (magic 'MU', version, type, seq, length) + typed
 *   payload. Both ports UDP, no TCP.
 *
 * Bridging: every motor-OD request is handed to app/cia402 via the
 * pipelined OD API. While Phase 4 isn't shipped, cia402 returns
 * MC_IF_OD_ERR_NOT_READY immediately for every request — the bridge is
 * fully exercised, the motor is just always "not ready".
 *
 * Subscribers: one at a time. A second TLM_SUBSCRIBE replaces the first.
 * Phase 6 hardening can widen this if anyone needs N subscribers.
 */

#include "od.h"

#include "app/config/config.h"
#include "app/log/log.h"
#include "app/cia402/cia402.h"
#include "app/od/cmc_od.h"
#include "bsp/net/net.h"
#include "bsp/time/time.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Static W6100 socket allocation (matches Documentation/architecture.md §10.1)
 *---------------------------------------------------------------------------*/

#define OD_ACCESS_SOCKET    ((net_sock_t)4)
#define OD_TELEMETRY_SOCKET ((net_sock_t)5)

/*----------------------------------------------------------------------------
 * Wire format constants — mirror NETWORK_UDP_SPEC.md
 *---------------------------------------------------------------------------*/

#define UDP_MAGIC               0x4D55u    /* 'MU' little-endian */
#define UDP_HDR_BYTES           8u
#define UDP_PAYLOAD_MAX         512u       /* generous; max real msg is far smaller */

/* Telemetry batching limits. One sample = MC_IfCyclicStatusHeader_t (12 B)
 * + blob (≤ 40 B) = ≤ 52 B. Batch up to 4 samples per datagram by default. */
#define TELEMETRY_BATCH_MAX           4u
#define TELEMETRY_RECORD_BYTES_MAX    (MC_IF_STATUS_HEADER_SIZE + MC_IF_TLM_BLOB_MAX)
#define TELEMETRY_KEEPALIVE_MS        1000u

typedef enum {
    MSG_OD_READ_REQ      = 0x01,
    MSG_OD_READ_RESP     = 0x02,
    MSG_OD_WRITE_REQ     = 0x03,
    MSG_OD_WRITE_RESP    = 0x04,
    MSG_TLM_SUBSCRIBE    = 0x10,
    MSG_TLM_UNSUBSCRIBE  = 0x11,
    MSG_TELEMETRY        = 0x20,
    MSG_ERROR            = 0x7F,
} udp_msg_type_t;

/* error_class values per Interface/mc_if_protocol.h MC_IfErrorClass_t. The
 * GUI decodes against the same Interface values, so they MUST match. */
typedef enum {
    ERR_NONE         = 0x00,
    ERR_BAD_MAGIC    = 0x01,   /* MC_IF_ERR_BAD_SYNC */
    ERR_BAD_VERSION  = 0x02,   /* MC_IF_ERR_BAD_VERSION */
    ERR_HEADER_CRC   = 0x03,   /* MC_IF_ERR_HEADER_CRC (unused on UDP — no header CRC) */
    ERR_PAYLOAD_CRC  = 0x04,   /* MC_IF_ERR_PAYLOAD_CRC (unused on UDP — no payload CRC) */
    ERR_BAD_LENGTH   = 0x05,   /* MC_IF_ERR_BAD_LENGTH */
    ERR_UNKNOWN_MSG  = 0x06,   /* MC_IF_ERR_UNKNOWN_MSG */
    ERR_SEQUENCE     = 0x07,   /* MC_IF_ERR_SEQUENCE (unused) */
    ERR_OD           = 0x08,   /* MC_IF_ERR_OD — detail carries an MC_IfOdResult_t */
    ERR_INTERNAL     = 0x09,   /* MC_IF_ERR_INTERNAL — used for queue-full etc. */
} udp_err_class_t;

/*----------------------------------------------------------------------------
 * State
 *---------------------------------------------------------------------------*/

/* One in-flight OD request at a time. Phase 4 may widen via a queue inside
 * cia402; until then a single slot suffices and keeps the logic obvious. */
typedef struct {
    bool                in_flight;
    cia402_od_handle_t  handle;
    net_addr_t          peer;
    uint16_t            seq;
    uint16_t            idx;
    uint8_t             sub;
    MC_IfOdType_t       type;
    bool                is_read;
} pending_od_t;

typedef struct {
    bool        active;
    net_addr_t  peer;            /* PC IP + the rx_port they advertised */
    uint16_t    rate_divider;    /* 1 = full 1 kHz; 10 = 100 Hz; etc. */
    uint8_t     batch;           /* samples per datagram */
    uint16_t    tx_seq;          /* datagram sequence, increments each send */
    uint32_t    last_keepalive_ms;
} subscriber_t;

static bool         s_inited = false;
static pending_od_t s_pending;
static subscriber_t s_sub;

/* Telemetry batching state (used by service_telemetry / od_init). */
static uint8_t  s_sample_buf[TELEMETRY_BATCH_MAX * TELEMETRY_RECORD_BYTES_MAX];

/* Cumulative counters for app/debug. Free-running, not reset across subs. */
static uint32_t s_telemetry_datagrams_sent = 0;
static uint32_t s_telemetry_samples_sent   = 0;
static uint8_t  s_samples_held     = 0;
static uint8_t  s_record_bytes     = 0;
static uint16_t s_decimate_count   = 0;
static uint8_t  s_last_map_version = 0;

/*----------------------------------------------------------------------------
 * LE serialise / parse helpers
 *---------------------------------------------------------------------------*/

static inline uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}
static inline void wr_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void wr_u32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static uint8_t type_size_bytes(MC_IfOdType_t t)
{
    switch (t) {
        case MC_IF_T_U8:  case MC_IF_T_I8:  return 1;
        case MC_IF_T_U16: case MC_IF_T_I16: return 2;
        case MC_IF_T_U32: case MC_IF_T_I32: case MC_IF_T_F32: return 4;
        default: return 0;
    }
}

/*----------------------------------------------------------------------------
 * UDP frame build helpers
 *---------------------------------------------------------------------------*/

/* Build the 8-byte UDP header into `out`. Returns 8. */
static size_t build_hdr(uint8_t *out, uint8_t type, uint16_t seq, uint16_t length)
{
    wr_u16(out + 0, UDP_MAGIC);
    out[2] = MC_IF_PROTOCOL_VERSION;
    out[3] = type;
    wr_u16(out + 4, seq);
    wr_u16(out + 6, length);
    return UDP_HDR_BYTES;
}

static void send_error(const net_addr_t *to, uint16_t ref_seq,
                       udp_err_class_t cls, uint8_t detail)
{
    uint8_t buf[UDP_HDR_BYTES + 4];
    build_hdr(buf, MSG_ERROR, ref_seq, 4);
    buf[8]  = (uint8_t)cls;
    buf[9]  = detail;
    wr_u16(buf + 10, ref_seq);
    (void)net_sendto(OD_ACCESS_SOCKET, to, buf, sizeof(buf));
}

/*----------------------------------------------------------------------------
 * Request dispatch
 *---------------------------------------------------------------------------*/

/* Detail byte for ERR_INTERNAL = "OD request queue is full". The Interface
 * doesn't pre-define detail semantics for MC_IF_ERR_INTERNAL; we use 0x01
 * to mean "in-flight already", documented here. */
#define ERR_DETAIL_QUEUE_FULL  0x01u

/* Build + send an OD_READ_RESP datagram. Used by both:
 *   - service_pending (async response from the cia402 OD pipeline);
 *   - handle_od_read for CMC-owned entries (synchronous, no SPI). */
static void send_od_read_resp(const net_addr_t *peer, uint16_t seq,
                              uint16_t idx, uint8_t sub, MC_IfOdType_t type,
                              MC_IfOdResult_t result,
                              const uint8_t *data, uint8_t data_len)
{
    uint8_t out[UDP_HDR_BYTES + 6 + 8];
    uint8_t  dl   = (result == MC_IF_OD_OK) ? data_len : 0;
    uint16_t plen = (uint16_t)(6u + dl);

    build_hdr(out, MSG_OD_READ_RESP, seq, plen);
    wr_u16(out + 8,  idx);
    out[10] = sub;
    out[11] = (uint8_t)type;
    out[12] = (uint8_t)result;
    out[13] = dl;
    if (dl > 0 && data != NULL) memcpy(out + 14, data, dl);

    (void)net_sendto(OD_ACCESS_SOCKET, peer, out, (size_t)(UDP_HDR_BYTES + plen));
}

static void send_od_write_resp(const net_addr_t *peer, uint16_t seq,
                               uint16_t idx, uint8_t sub,
                               MC_IfOdResult_t result)
{
    uint8_t out[UDP_HDR_BYTES + 4];
    build_hdr(out, MSG_OD_WRITE_RESP, seq, 4);
    wr_u16(out + 8, idx);
    out[10] = sub;
    out[11] = (uint8_t)result;
    (void)net_sendto(OD_ACCESS_SOCKET, peer, out, sizeof(out));
}

/* Returns true if the incoming (peer, seq) matches the request that is
 * already in flight — i.e. it's a retransmit, not a new request. The GUI's
 * _transaction() retransmits up to 3 times with a 50 ms timeout each, so
 * if the motor MCU takes >50 ms to stage its response we will see the same
 * peer+seq again. Silently dropping the retransmit lets the in-flight
 * request's eventual response satisfy the GUI (it matches by seq). */
static bool is_retransmit_of_in_flight(const net_addr_t *peer, uint16_t seq)
{
    if (!s_pending.in_flight) return false;
    if (s_pending.seq != seq) return false;
    if (memcmp(s_pending.peer.addr, peer->addr, sizeof(peer->addr)) != 0) return false;
    if (s_pending.peer.port != peer->port) return false;
    return true;
}

static void handle_od_read(const net_addr_t *peer, uint16_t seq,
                           const uint8_t *payload, uint16_t plen)
{
    if (plen != 4) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }

    uint16_t      idx  = rd_u16(payload + 0);
    uint8_t       sub  = payload[2];
    MC_IfOdType_t type = (MC_IfOdType_t)payload[3];

    /* CMC-owned entries are handled locally and respond synchronously.
     * No cia402 OD pipeline traffic, no in-flight slot consumed. */
    if (cmc_od_owns(idx)) {
        uint8_t       out_data[8] = {0};
        uint8_t       out_type    = 0;
        uint8_t       out_len     = 0;
        MC_IfOdResult_t res = cmc_od_read(idx, sub, type, &out_type, out_data, &out_len);
        send_od_read_resp(peer, seq, idx, sub,
                          (res == MC_IF_OD_OK) ? (MC_IfOdType_t)out_type : type,
                          res, out_data, out_len);
        return;
    }

    /* Motor-owned: queue to cia402, response when SPI round-trip completes. */
    if (is_retransmit_of_in_flight(peer, seq)) return;     /* drop, in-flight will reply */
    if (s_pending.in_flight) {
        send_error(peer, seq, ERR_INTERNAL, ERR_DETAIL_QUEUE_FULL);
        return;
    }

    cia402_od_handle_t h = cia402_od_read_begin(idx, sub, type);
    if (h == CIA402_OD_HANDLE_INVALID) {
        send_error(peer, seq, ERR_INTERNAL, ERR_DETAIL_QUEUE_FULL);
        return;
    }
    s_pending = (pending_od_t){
        .in_flight = true, .handle = h, .peer = *peer, .seq = seq,
        .idx = idx, .sub = sub, .type = type, .is_read = true,
    };
}

static void handle_od_write(const net_addr_t *peer, uint16_t seq,
                            const uint8_t *payload, uint16_t plen)
{
    /* payload: idx(2) + sub(1) + type(1) + len(1) + data[len] = 5 + len */
    if (plen < 5) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }

    uint16_t      idx  = rd_u16(payload + 0);
    uint8_t       sub  = payload[2];
    MC_IfOdType_t type = (MC_IfOdType_t)payload[3];
    uint8_t       len  = payload[4];

    if ((uint16_t)(5 + len) != plen) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }
    if (len != type_size_bytes(type)) { send_error(peer, seq, ERR_OD, MC_IF_OD_ERR_SIZE); return; }

    /* CMC-owned entries handled locally + synchronously. No SPI traffic,
     * no in-flight slot consumed. */
    if (cmc_od_owns(idx)) {
        MC_IfOdResult_t res = cmc_od_write(idx, sub, type, payload + 5, len);
        send_od_write_resp(peer, seq, idx, sub, res);
        return;
    }

    /* Motor-owned: queue to cia402, response when SPI round-trip completes. */
    if (is_retransmit_of_in_flight(peer, seq)) return;
    if (s_pending.in_flight) {
        send_error(peer, seq, ERR_INTERNAL, ERR_DETAIL_QUEUE_FULL);
        return;
    }

    cia402_od_handle_t h = cia402_od_write_begin(idx, sub, type, payload + 5, len);
    if (h == CIA402_OD_HANDLE_INVALID) {
        send_error(peer, seq, ERR_INTERNAL, ERR_DETAIL_QUEUE_FULL);
        return;
    }
    s_pending = (pending_od_t){
        .in_flight = true, .handle = h, .peer = *peer, .seq = seq,
        .idx = idx, .sub = sub, .type = type, .is_read = false,
    };
}

static void handle_subscribe(const net_addr_t *peer, uint16_t seq,
                             const uint8_t *payload, uint16_t plen)
{
    if (plen != 5) { send_error(peer, seq, ERR_BAD_LENGTH, 0); return; }

    uint16_t rx_port      = rd_u16(payload + 0);
    uint16_t rate_divider = rd_u16(payload + 2);
    uint8_t  batch        = payload[4];

    if (rate_divider == 0) rate_divider = 1;
    if (batch == 0)        batch        = 1;
    /* Cap at our buffer size — otherwise emit_telemetry would overflow. */
    if (batch > TELEMETRY_BATCH_MAX) batch = TELEMETRY_BATCH_MAX;

    s_sub.active            = true;
    s_sub.peer.addr[0]      = peer->addr[0];
    s_sub.peer.addr[1]      = peer->addr[1];
    s_sub.peer.addr[2]      = peer->addr[2];
    s_sub.peer.addr[3]      = peer->addr[3];
    s_sub.peer.port         = rx_port;
    s_sub.rate_divider      = rate_divider;
    s_sub.batch             = batch;
    s_sub.tx_seq            = 0;
    s_sub.last_keepalive_ms = time_ms();
    /* Clear any in-flight batch from a previous subscriber. */
    s_samples_held   = 0;
    s_decimate_count = 0;

    LOG_INFO("od: subscriber %u.%u.%u.%u:%u div=%u batch=%u",
             peer->addr[0], peer->addr[1], peer->addr[2], peer->addr[3],
             (unsigned)rx_port, (unsigned)rate_divider, (unsigned)batch);
}

static void handle_unsubscribe(const net_addr_t *peer, uint16_t seq)
{
    (void)peer; (void)seq;
    if (s_sub.active) {
        LOG_INFO("od: subscriber unsubscribed");
        s_sub.active = false;
    }
}

/*----------------------------------------------------------------------------
 * Pending-request completion (Phase 5: completes immediately with NOT_READY)
 *---------------------------------------------------------------------------*/

static void service_pending(void)
{
    if (!s_pending.in_flight) return;

    MC_IfOdResult_t res    = MC_IF_OD_OK;
    uint8_t         buf[8] = {0};
    uint8_t         vlen   = sizeof(buf);

    if (!cia402_od_poll(s_pending.handle, &res, buf, &vlen)) return;

    if (s_pending.is_read) {
        send_od_read_resp(&s_pending.peer, s_pending.seq,
                          s_pending.idx, s_pending.sub, s_pending.type,
                          res, buf, vlen);
    } else {
        send_od_write_resp(&s_pending.peer, s_pending.seq,
                           s_pending.idx, s_pending.sub, res);
    }

    s_pending.in_flight = false;
}

/*----------------------------------------------------------------------------
 * Incoming dispatch on the OD access socket
 *---------------------------------------------------------------------------*/

static void service_access_socket(void)
{
    uint8_t    buf[UDP_HDR_BYTES + UDP_PAYLOAD_MAX];
    net_addr_t from;
    int32_t    n = net_recvfrom(OD_ACCESS_SOCKET, &from, buf, sizeof(buf));
    if (n <= 0) return;

    if (n < (int32_t)UDP_HDR_BYTES) return;  /* too short, silently drop */

    uint16_t magic   = rd_u16(buf + 0);
    uint8_t  version = buf[2];
    uint8_t  type    = buf[3];
    uint16_t seq     = rd_u16(buf + 4);
    uint16_t plen    = rd_u16(buf + 6);

    if (magic   != UDP_MAGIC)                   { send_error(&from, seq, ERR_BAD_MAGIC,   0); return; }
    if (version != MC_IF_PROTOCOL_VERSION)      { send_error(&from, seq, ERR_BAD_VERSION, version); return; }
    if ((uint32_t)plen + UDP_HDR_BYTES != (uint32_t)n) {
        send_error(&from, seq, ERR_BAD_LENGTH, 0); return;
    }

    const uint8_t *payload = buf + UDP_HDR_BYTES;

    switch (type) {
        case MSG_OD_READ_REQ:     handle_od_read    (&from, seq, payload, plen); break;
        case MSG_OD_WRITE_REQ:    handle_od_write   (&from, seq, payload, plen); break;
        case MSG_TLM_SUBSCRIBE:   handle_subscribe  (&from, seq, payload, plen); break;
        case MSG_TLM_UNSUBSCRIBE: handle_unsubscribe(&from, seq);                break;
        default:                  send_error(&from, seq, ERR_UNKNOWN_MSG, type); break;
    }
}

/*----------------------------------------------------------------------------
 * Telemetry stream
 *
 * Each od_tick pulls the latest fresh CYCLIC_STATUS from cia402 (polled,
 * non-blocking). When one is available we decimate by the subscriber's
 * rate_divider, accumulate `batch` samples in a local buffer, and send one
 * MSG_TELEMETRY datagram when full.
 *
 * If no fresh status has been seen for KEEPALIVE_MS, emit an empty
 * TELEMETRY datagram (sample_count = 0) so the PC can confirm the link is
 * still alive. This is what runs when the motor MCU isn't producing
 * cyclic status (Phase 4 cia402 still ticks but motor SPI may be silent).
 *---------------------------------------------------------------------------*/

static void emit_telemetry(uint8_t map_version, uint8_t sample_count,
                           uint8_t record_bytes, const uint8_t *records, uint16_t records_len)
{
    uint8_t out[UDP_HDR_BYTES + 4 + sizeof(s_sample_buf)];
    if ((size_t)(4 + records_len) > sizeof(out) - UDP_HDR_BYTES) return; /* defensive */

    build_hdr(out, MSG_TELEMETRY, s_sub.tx_seq++, (uint16_t)(4 + records_len));
    out[8]  = map_version;
    out[9]  = sample_count;
    out[10] = record_bytes;
    out[11] = 0;
    if (records_len > 0) memcpy(out + UDP_HDR_BYTES + 4, records, records_len);
    (void)net_sendto(OD_TELEMETRY_SOCKET, &s_sub.peer, out,
                     (size_t)(UDP_HDR_BYTES + 4 + records_len));
    s_telemetry_datagrams_sent++;
    s_telemetry_samples_sent += sample_count;
}

static void service_telemetry(void)
{
    if (!s_sub.active) return;

    /* Pull any fresh cyclic status from cia402. */
    MC_IfCyclicStatusHeader_t hdr;
    uint8_t                   blob[MC_IF_TLM_BLOB_MAX];
    uint8_t                   blob_len = 0;

    if (cia402_take_cyclic_status(&hdr, blob, &blob_len)) {
        /* Decimate: only keep one in every rate_divider samples. */
        if (++s_decimate_count >= s_sub.rate_divider) {
            s_decimate_count = 0;

            /* Map-version change → flush whatever we had buffered so the PC
             * doesn't try to unpack mixed-layout records. */
            if (hdr.map_version != s_last_map_version && s_samples_held > 0) {
                uint16_t bytes = (uint16_t)(s_samples_held * s_record_bytes);
                emit_telemetry(s_last_map_version, s_samples_held, s_record_bytes,
                               s_sample_buf, bytes);
                s_samples_held = 0;
            }
            s_last_map_version = hdr.map_version;
            s_record_bytes     = (uint8_t)(MC_IF_STATUS_HEADER_SIZE + blob_len);

            /* Append the current sample to the buffer. */
            if (s_samples_held < TELEMETRY_BATCH_MAX
             && s_samples_held < s_sub.batch
             && s_record_bytes <= TELEMETRY_RECORD_BYTES_MAX) {
                uint8_t *slot = s_sample_buf + (s_samples_held * s_record_bytes);
                memcpy(slot, &hdr, MC_IF_STATUS_HEADER_SIZE);
                if (blob_len > 0) memcpy(slot + MC_IF_STATUS_HEADER_SIZE, blob, blob_len);
                s_samples_held++;
            }

            /* Flush if we've reached the subscriber's batch size. */
            if (s_samples_held >= s_sub.batch) {
                uint16_t bytes = (uint16_t)(s_samples_held * s_record_bytes);
                emit_telemetry(s_last_map_version, s_samples_held, s_record_bytes,
                               s_sample_buf, bytes);
                s_samples_held = 0;
            }
            s_sub.last_keepalive_ms = time_ms();
        }
    } else if (time_elapsed_ms(s_sub.last_keepalive_ms) >= TELEMETRY_KEEPALIVE_MS) {
        /* No fresh data within the keep-alive window. Emit an empty
         * datagram so the PC knows the path is up but quiet. */
        s_sub.last_keepalive_ms += TELEMETRY_KEEPALIVE_MS;
        emit_telemetry(s_last_map_version, 0, 0, NULL, 0);
    }
}

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

void od_init(void)
{
    memset(&s_pending, 0, sizeof(s_pending));
    memset(&s_sub,     0, sizeof(s_sub));
    memset(s_sample_buf, 0, sizeof(s_sample_buf));
    s_samples_held     = 0;
    s_record_bytes     = 0;
    s_decimate_count   = 0;
    s_last_map_version = 0;

    const network_cfg_t *net = config_get_network();

    if (!net_open(OD_ACCESS_SOCKET, NET_PROTO_UDP, net->od_udp_port, false)) {
        LOG_ERROR("od: failed to open access socket on UDP %u", (unsigned)net->od_udp_port);
        return;
    }
    /* Telemetry socket needs a local port too (the PC's TLM_SUBSCRIBE tells
     * us where to send; the local port is what tcpdump will show). Reuse
     * od_udp_port + 1 by convention (Interface defaults: 5000 + 5001). */
    if (!net_open(OD_TELEMETRY_SOCKET, NET_PROTO_UDP,
                  (uint16_t)(net->od_udp_port + 1), false)) {
        LOG_ERROR("od: failed to open telemetry socket on UDP %u",
                  (unsigned)(net->od_udp_port + 1));
        return;
    }

    s_inited = true;
    LOG_INFO("od: ready. OD access UDP %u, telemetry UDP %u",
             (unsigned)net->od_udp_port, (unsigned)(net->od_udp_port + 1));
}

/* Drain any unsolicited inbound on the telemetry socket (e.g. firewall
 * "punch" packets from the PC tool — see comments in
 * Interface/gui/mc_gui/client.py:_punch_telemetry_flow). The W6100's
 * per-socket RX buffer is 2 KB; if we never read, even tiny incoming
 * packets accumulate and eventually stall further RX on that slot. */
static void drain_telemetry_rx(void)
{
    uint8_t    scratch[64];
    net_addr_t from;
    /* Loop in case multiple datagrams have queued up between ticks. */
    while (net_recvfrom(OD_TELEMETRY_SOCKET, &from, scratch, sizeof(scratch)) > 0) {
        /* discard */
    }
}

void od_tick(void)
{
    if (!s_inited) return;
    service_pending();
    service_access_socket();
    drain_telemetry_rx();
    service_telemetry();
}

/*----------------------------------------------------------------------------
 * Debug accessors (consumed by app/debug). Plain snapshots — no locking
 * because the firmware is single-threaded on a cooperative super-loop.
 *---------------------------------------------------------------------------*/

void od_get_pending(od_request_snapshot_t *out)
{
    if (!out) return;
    out->in_flight = s_pending.in_flight;
    out->is_read   = s_pending.is_read;
    out->index     = s_pending.idx;
    out->subindex  = s_pending.sub;
    out->seq       = s_pending.seq;
}

void od_get_subscriber(od_subscriber_snapshot_t *out)
{
    if (!out) return;
    out->active       = s_sub.active;
    out->peer_ip[0]   = s_sub.peer.addr[0];
    out->peer_ip[1]   = s_sub.peer.addr[1];
    out->peer_ip[2]   = s_sub.peer.addr[2];
    out->peer_ip[3]   = s_sub.peer.addr[3];
    out->peer_port    = s_sub.peer.port;
    out->rate_divider = s_sub.rate_divider;
    out->batch        = s_sub.batch;
}

uint32_t od_telemetry_datagrams_sent(void) { return s_telemetry_datagrams_sent; }
uint32_t od_telemetry_samples_sent  (void) { return s_telemetry_samples_sent;   }
