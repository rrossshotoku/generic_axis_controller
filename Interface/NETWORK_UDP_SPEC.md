# Network-Side UDP Protocol (PC ↔ network MCU)

How the **network MCU** (Lightweight_CMC) exposes the motor-control MCU's object dictionary to
the **PC** over **UDP** (the network side is all-UDP; no TCP). The network MCU is a near-
transparent bridge: it mirrors the OD/message model of the SPI link (see `INTERFACE_SPEC.md`),
translating OD-over-UDP ⟷ OD-over-SPI and relaying telemetry. The PC uses the same OD indices,
types, and scaling from `mc_if_od.h`.

```
PC  ──UDP──►  Network MCU (CMC)  ──SPI (mc_if_protocol)──►  Motor-control MCU (OD)
    ◄──UDP──                     ◄──SPI──
```

## Ports (defaults; configurable)
- **OD access** (request/response): UDP `5000`. PC → CMC requests, CMC → PC responses.
- **Telemetry** (stream): UDP `5001`. CMC → PC, pushed to a subscribed endpoint.

## Common datagram header (8 bytes, little-endian)
```c
typedef struct __attribute__((packed)) {
    uint16_t magic;     /* 0x4D55  ('MU') */
    uint8_t  version;   /* == MC_IF_PROTOCOL_VERSION */
    uint8_t  type;      /* MC_UdpMsgType (below) */
    uint16_t seq;       /* request id (req/resp matching); stream sequence for telemetry */
    uint16_t length;    /* payload bytes following this header */
} MC_UdpHeader_t;
```
UDP already carries a checksum, so no extra CRC. Mismatched `version` ⇒ ERROR.

## Message types
| Value | Name | Dir | Payload |
|---:|---|---|---|
| 0x01 | OD_READ_REQ   | PC→CMC | `{u16 index, u8 sub, u8 type}` |
| 0x02 | OD_READ_RESP  | CMC→PC | `{u16 index, u8 sub, u8 type, u8 result, u8 len, u8 data[len]}` |
| 0x03 | OD_WRITE_REQ  | PC→CMC | `{u16 index, u8 sub, u8 type, u8 len, u8 data[len]}` |
| 0x04 | OD_WRITE_RESP | CMC→PC | `{u16 index, u8 sub, u8 result}` |
| 0x10 | TLM_SUBSCRIBE | PC→CMC | `{u16 rx_port, u16 rate_divider, u8 batch}` |
| 0x11 | TLM_UNSUBSCRIBE | PC→CMC | `{}` |
| 0x20 | TELEMETRY     | CMC→PC | see below |
| 0x7F | ERROR         | both   | `{u8 error_class, u8 detail, u16 ref_seq}` |

`result` codes are `MC_IfOdResult_t`; `type` values are `MC_IfOdType_t` (both from `mc_if_od.h`).
OD values in `data[]` are little-endian (float32 for 0x2xxx objects; scaled-int per the OD).

## OD access (reliable over UDP)
- PC assigns `seq`, sends OD_READ_REQ / OD_WRITE_REQ, and **retransmits on timeout**
  (suggest 50 ms, up to 3 tries). The CMC echoes `seq` in the response.
- Reads are idempotent; a write is confirmed by its RESP `result` (PC may read back to verify).
- The CMC performs the corresponding OD access on the motor MCU over SPI (OD_READ/WRITE_REQ) and
  returns the result. CMC-local objects (e.g. its own network config) may be served directly.

## Telemetry stream (fire-and-forget)
1. PC sends **TLM_SUBSCRIBE** with the port it will receive on, a `rate_divider` (decimate from
   the 1 kHz cyclic rate, e.g. 1 = 1 kHz, 10 = 100 Hz), and `batch` (samples per datagram).
2. The CMC receives the motor MCU's cyclic telemetry each SPI cycle, decimates by `rate_divider`,
   accumulates `batch` samples, and sends a **TELEMETRY** datagram to the PC's endpoint.
3. No retransmission — a dropped datagram is a dropped graph segment. The PC detects gaps via the
   per-sample `status_counter` and a map change via `map_version`.

**TELEMETRY payload:**
```c
typedef struct __attribute__((packed)) {
    uint8_t  map_version;     /* echoes the motor MCU telemetry-map version */
    uint8_t  sample_count;    /* number of records in this datagram (== batch) */
    uint8_t  record_bytes;    /* size of each record below */
    uint8_t  reserved;
    /* sample_count records follow; each record = the cyclic status header
       (MC_IfCyclicStatusHeader_t, 12 B) + the mapped blob (map_byte_count B). */
} MC_UdpTelemetryHeader_t;
```
The PC unpacks each record using the **active telemetry map** (which it configured via OD writes
to 0x2A00) — it knows the field order, types, and scaling. `record_bytes` and `map_version`
guard against unpacking stale layouts.

## Choosing what to graph (end-to-end, runtime)
1. PC writes the telemetry map on the motor MCU: OD_WRITE to `0x2A00:*` over UDP → CMC bridges to
   SPI → motor MCU rebuilds its map and bumps `map_version`.
2. PC sends TLM_SUBSCRIBE; telemetry datagrams begin, carrying the new `map_version`.
3. To change signals, rewrite `0x2A00` — live; the PC re-columns when `map_version` changes.

## Bandwidth note
At 1 kHz cyclic with `batch`=10 ⇒ 100 datagrams/s, each ≈ `8 + 4 + 10*(12 + ~40)` ≈ 530 bytes ⇒
~53 KB/s. Negligible for 100 Mbit Ethernet; the SPI link is the real constraint (well within it).

## Open points (confirm)
- Final UDP ports; whether OD access and telemetry share one port.
- Endpoint discovery (fixed PC IP, broadcast announce, or PC-initiated subscribe — default: PC
  initiates, CMC replies to source).
- Whether the CMC exposes its own config objects (network settings) in a reserved OD range.
