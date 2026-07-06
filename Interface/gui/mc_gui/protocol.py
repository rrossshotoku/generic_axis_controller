"""PC <-> network-MCU (CMC) UDP wire codec.

Implements the datagrams defined in ``../NETWORK_UDP_SPEC.md``: an 8-byte common
header, OD read/write request/response, telemetry subscribe/unsubscribe, the pushed
telemetry stream, and the error message. All multi-byte fields are little-endian.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

PROTOCOL_VERSION = 5  # MC_IF_PROTOCOL_VERSION (v5: bootloader contract — 0x1F5x + segmented SDO + node-state; v4: cyclic header + position_actual + movement_status; v3: cyclic-cmd reshape)
UDP_MAGIC = 0x4D55  # 'MU'

# Default ports (NETWORK_UDP_SPEC "Ports").
DEFAULT_OD_PORT = 5000
DEFAULT_TLM_PORT = 5001

# MC_UdpMsgType
MSG_OD_READ_REQ = 0x01
MSG_OD_READ_RESP = 0x02
MSG_OD_WRITE_REQ = 0x03
MSG_OD_WRITE_RESP = 0x04
MSG_TLM_SUBSCRIBE = 0x10
MSG_TLM_UNSUBSCRIBE = 0x11
MSG_TELEMETRY = 0x20
MSG_ERROR = 0x7F

# MC_IfOdResult_t (from mc_if_protocol.h)
OD_RESULT_NAME = {
    0x00: "OK", 0x01: "NO_OBJECT", 0x02: "NO_SUB", 0x03: "ACCESS",
    0x04: "TYPE", 0x05: "RANGE", 0x06: "SIZE", 0x07: "CALLBACK", 0x08: "NOT_READY",
}
OD_OK = 0x00

# MC_IfErrorClass_t
ERROR_CLASS_NAME = {
    0x00: "NONE", 0x01: "BAD_SYNC", 0x02: "BAD_VERSION", 0x03: "HEADER_CRC",
    0x04: "PAYLOAD_CRC", 0x05: "BAD_LENGTH", 0x06: "UNKNOWN_MSG", 0x07: "SEQUENCE",
    0x08: "OD", 0x09: "INTERNAL",
}

# MC_IfNodeState_t (reported in the cyclic status header)
NODE_STATE_NAME = {
    0x00: "INIT", 0x01: "DISABLED", 0x02: "READY", 0x03: "RUNNING",
    0x04: "QUICK_STOP", 0x05: "FAULT", 0x06: "CALIBRATING",
}

_HEADER = struct.Struct("<HBBHH")  # magic, version, type, seq, length
HEADER_SIZE = _HEADER.size  # 8

# MC_IfCyclicStatusHeader_t (v4): statusword, mode_display, node_state, error_code,
# map_version, map_byte_count, status_counter, position_actual_scaled, movement_status  (18 bytes)
_STATUS_HEADER = struct.Struct("<HbBHBBIiH")
STATUS_HEADER_SIZE = _STATUS_HEADER.size  # 18

# MC_UdpTelemetryHeader_t: map_version, sample_count, record_bytes, reserved (4 bytes)
_TLM_HEADER = struct.Struct("<BBBB")
TLM_HEADER_SIZE = _TLM_HEADER.size  # 4


def pack_header(msg_type: int, seq: int, payload: bytes) -> bytes:
    return _HEADER.pack(UDP_MAGIC, PROTOCOL_VERSION, msg_type, seq & 0xFFFF, len(payload)) + payload


@dataclass
class UdpHeader:
    magic: int
    version: int
    type: int
    seq: int
    length: int


def parse_header(data: bytes) -> UdpHeader | None:
    if len(data) < HEADER_SIZE:
        return None
    magic, version, mtype, seq, length = _HEADER.unpack_from(data, 0)
    if magic != UDP_MAGIC:
        return None
    return UdpHeader(magic, version, mtype, seq, length)


# --- request builders ---------------------------------------------------------
def build_read_req(seq: int, index: int, sub: int, expected_type: int = 0) -> bytes:
    return pack_header(MSG_OD_READ_REQ, seq, struct.pack("<HBB", index, sub, expected_type))


def build_write_req(seq: int, index: int, sub: int, type_code: int, data: bytes) -> bytes:
    payload = struct.pack("<HBBB", index, sub, type_code, len(data)) + bytes(data)
    return pack_header(MSG_OD_WRITE_REQ, seq, payload)


def build_subscribe(seq: int, rx_port: int, rate_divider: int, batch: int) -> bytes:
    return pack_header(MSG_TLM_SUBSCRIBE, seq, struct.pack("<HHB", rx_port, rate_divider, batch))


def build_unsubscribe(seq: int) -> bytes:
    return pack_header(MSG_TLM_UNSUBSCRIBE, seq, b"")


# --- response parsers ---------------------------------------------------------
@dataclass
class ReadResp:
    index: int
    sub: int
    type: int
    result: int
    data: bytes


def parse_read_resp(payload: bytes) -> ReadResp | None:
    if len(payload) < 6:
        return None
    index, sub, type_code, result, length = struct.unpack_from("<HBBBB", payload, 0)
    data = payload[6:6 + length]
    return ReadResp(index, sub, type_code, result, data)


@dataclass
class WriteResp:
    index: int
    sub: int
    result: int


def parse_write_resp(payload: bytes) -> WriteResp | None:
    if len(payload) < 4:
        return None
    index, sub, result = struct.unpack_from("<HBB", payload, 0)
    return WriteResp(index, sub, result)


@dataclass
class ErrorMsg:
    error_class: int
    detail: int
    ref_seq: int

    def describe(self) -> str:
        cls = ERROR_CLASS_NAME.get(self.error_class, f"0x{self.error_class:02X}")
        if self.error_class == 0x08:  # OD error -> detail is an OD result
            return f"{cls}/{OD_RESULT_NAME.get(self.detail, self.detail)} (ref seq {self.ref_seq})"
        return f"{cls} detail=0x{self.detail:02X} (ref seq {self.ref_seq})"


def parse_error(payload: bytes) -> ErrorMsg | None:
    if len(payload) < 4:
        return None
    error_class, detail, ref_seq = struct.unpack_from("<BBH", payload, 0)
    return ErrorMsg(error_class, detail, ref_seq)


def is_transient_error(err: "ErrorMsg | None") -> bool:
    """True if an error is worth retrying — server backpressure (INTERNAL / queue-full) or a
    momentarily not-ready OD — as opposed to a hard error (bad index, read-only, type, range)
    that a retry can never fix."""
    if err is None:
        return False
    if err.error_class == 0x09:                         # INTERNAL -> queue-full / backpressure
        return True
    if err.error_class == 0x08 and err.detail == 0x08:  # OD result NOT_READY (server busy)
        return True
    return False


@dataclass
class StatusHeader:
    statusword: int
    mode_display: int
    node_state: int
    error_code: int
    map_version: int
    map_byte_count: int
    status_counter: int
    position_actual_scaled: int = 0   # v4: OD 0x6064, 1e-5 rad/LSB (always present)
    movement_status: int = 0          # v4: MC_IF_MOVE_* bits


@dataclass
class TelemetryDatagram:
    map_version: int
    sample_count: int
    record_bytes: int
    records: list[tuple[StatusHeader, bytes]]  # (status header, mapped blob)


def parse_telemetry(payload: bytes) -> TelemetryDatagram | None:
    """Parse a TELEMETRY datagram payload into per-sample (status header, blob)."""
    if len(payload) < TLM_HEADER_SIZE:
        return None
    map_version, sample_count, record_bytes, _reserved = _TLM_HEADER.unpack_from(payload, 0)
    records: list[tuple[StatusHeader, bytes]] = []
    offset = TLM_HEADER_SIZE
    for _ in range(sample_count):
        if offset + STATUS_HEADER_SIZE > len(payload):
            break
        sh = StatusHeader(*_STATUS_HEADER.unpack_from(payload, offset))
        blob_start = offset + STATUS_HEADER_SIZE
        blob = payload[blob_start:blob_start + sh.map_byte_count]
        records.append((sh, blob))
        # record_bytes is authoritative for stride; fall back to header+blob.
        stride = record_bytes if record_bytes else (STATUS_HEADER_SIZE + sh.map_byte_count)
        offset += stride
    return TelemetryDatagram(map_version, sample_count, record_bytes, records)
