"""Threaded UDP client for the CMC.

Two worker threads keep the real-time telemetry path independent of acyclic OD
access:

* **command thread** owns the OD socket and serialises read/write/subscribe
  transactions, each with seq-matching and retransmit (per NETWORK_UDP_SPEC
  "OD access (reliable over UDP)").
* **telemetry thread** owns the telemetry socket and only ever receives, so a
  pending OD transaction can never stall the graph feed.

Results are delivered to the GUI thread via Qt signals (auto-queued across
threads).
"""
from __future__ import annotations

import queue
import socket
import threading
import time
from dataclasses import dataclass

from PySide6.QtCore import QObject, Signal

from . import protocol as proto
from .od import OdEntry, OdModel

RETRANSMIT_TIMEOUT = 0.05  # 50 ms (NETWORK_UDP_SPEC suggestion)
RETRANSMIT_TRIES = 3


@dataclass
class TelemetrySample:
    """One unpacked telemetry record."""
    counter: int          # status_counter (echoes command_counter; cyclic ticks)
    values: dict[str, float]  # channel name -> SI value
    statusword: int
    mode_display: int
    node_state: int
    error_code: int
    map_version: int
    layout_ok: bool


class NetworkClient(QObject):
    connected_changed = Signal(bool)
    log_message = Signal(str)
    od_read_done = Signal(object)    # {entry, ok, result, raw, si, error}
    od_write_done = Signal(object)   # {entry, ok, result, error}
    error_received = Signal(object)  # proto.ErrorMsg
    map_applied = Signal(object)     # {ok, entries, message}
    telemetry_samples = Signal(object)  # list[TelemetrySample]
    telemetry_stats = Signal(object)    # {rate_hz, dropped, frame_map_version, last}

    def __init__(self, od: OdModel, parent=None):
        super().__init__(parent)
        self.od = od
        self._od_sock: socket.socket | None = None
        self._tlm_sock: socket.socket | None = None
        self._addr: tuple[str, int] | None = None
        self._cmc_tlm_addr: tuple[str, int] | None = None
        self._cmd_q: "queue.Queue" = queue.Queue()
        self._cmd_thread: threading.Thread | None = None
        self._tlm_thread: threading.Thread | None = None
        self._running = False
        self._seq = 0
        # Ordered list of OD entries currently streamed (the active 0x2A00 map).
        self._active_map: list[OdEntry] = []
        # telemetry stats
        self._last_counter: int | None = None
        self._dropped = 0
        self._stat_count = 0
        self._stat_t0 = 0.0
        self._frame_map_version = -1

    # --- lifecycle ------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._running

    @property
    def active_map(self) -> list[OdEntry]:
        return list(self._active_map)

    def connect(self, ip: str, od_port: int, tlm_port: int) -> None:
        if self._running:
            self.disconnect()
        try:
            self._addr = (ip, od_port)
            # CMC convention: telemetry port = od_port + 1 (NETWORK_UDP_SPEC.md).
            # We need to know this so we can "punch" the firewall — see
            # _punch_telemetry_flow() below.
            self._cmc_tlm_addr = (ip, od_port + 1)

            self._od_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._od_sock.bind(("", 0))  # ephemeral source port for OD req/resp
            self._tlm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._tlm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # The requested telemetry port may be in a Windows reserved/excluded range
            # (Hyper-V/WSL/Docker) -> WinError 10013. Fall back to an ephemeral port;
            # the CMC is told the actual rx port in TLM_SUBSCRIBE anyway.
            try:
                self._tlm_sock.bind(("", tlm_port))
            except OSError as exc:
                self.log_message.emit(
                    f"Telemetry port {tlm_port} unavailable ({exc}); "
                    f"using an ephemeral port instead.")
                self._tlm_sock.bind(("", 0))
            self._tlm_port = self._tlm_sock.getsockname()[1]
        except OSError as exc:
            self.log_message.emit(f"Connect FAILED: {exc}")
            self._cleanup_sockets()
            self.connected_changed.emit(False)
            return

        self._running = True
        self._stat_t0 = time.monotonic()
        self._cmd_thread = threading.Thread(target=self._cmd_loop, name="od-cmd", daemon=True)
        self._tlm_thread = threading.Thread(target=self._tlm_loop, name="tlm-rx", daemon=True)
        self._cmd_thread.start()
        self._tlm_thread.start()

        # "Punch" the firewall: many host firewalls (incl. Windows Firewall)
        # are stateful — they only allow inbound UDP that matches a flow the
        # local socket has previously sent outbound from. Our telemetry
        # receive socket would otherwise see no outbound traffic at all and
        # its inbound datagrams would be dropped silently. Sending a 0-byte
        # UDP from the receive socket to the CMC's telemetry port (5001 by
        # default) registers the flow PC:tlm_port <-> CMC:cmc_tlm_port in
        # the firewall's state table; subsequent inbound telemetry then
        # matches and is allowed through. The CMC discards the empty
        # datagram (it doesn't read from its telemetry socket for input).
        self._punch_telemetry_flow()

        self.log_message.emit(
            f"Connected: OD -> {ip}:{od_port}, telemetry RX on local :{self._tlm_port}"
        )
        self.connected_changed.emit(True)

    def _punch_telemetry_flow(self) -> None:
        """Send a 0-byte UDP from the telemetry rx socket to the CMC's
        telemetry port. Establishes a stateful-firewall flow entry so
        subsequent inbound telemetry datagrams are accepted."""
        try:
            self._tlm_sock.sendto(b"", self._cmc_tlm_addr)
        except OSError as exc:
            # Non-fatal: if punching fails the user just needs a real
            # firewall allow rule for the chosen port.
            self.log_message.emit(
                f"Telemetry firewall-punch send failed: {exc}. "
                f"If you don't see telemetry datagrams, add an inbound UDP "
                f"firewall rule for port {self._tlm_port}.")

    def disconnect(self) -> None:
        if not self._running:
            return
        self._running = False
        self._cmd_q.put(None)  # wake the command thread
        for t in (self._cmd_thread, self._tlm_thread):
            if t and t.is_alive():
                t.join(timeout=1.0)
        self._cleanup_sockets()
        self.log_message.emit("Disconnected")
        self.connected_changed.emit(False)

    def _cleanup_sockets(self) -> None:
        for sock in (self._od_sock, self._tlm_sock):
            try:
                if sock:
                    sock.close()
            except OSError:
                pass
        self._od_sock = self._tlm_sock = None

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    # --- public API (thread-safe; enqueued onto the command thread) -----------
    def read_async(self, entry: OdEntry) -> None:
        self._cmd_q.put(("read", entry))

    def write_async(self, entry: OdEntry, raw_value) -> None:
        self._cmd_q.put(("write", entry, raw_value))

    def apply_map_async(self, entries: list[OdEntry]) -> None:
        self._cmd_q.put(("apply_map", entries))

    def subscribe_async(self, rate_divider: int, batch: int) -> None:
        self._cmd_q.put(("subscribe", rate_divider, batch))

    def unsubscribe_async(self) -> None:
        self._cmd_q.put(("unsubscribe",))

    # --- command thread -------------------------------------------------------
    def _cmd_loop(self) -> None:
        while self._running:
            try:
                cmd = self._cmd_q.get(timeout=0.25)
            except queue.Empty:
                continue
            if cmd is None:
                break
            try:
                self._dispatch(cmd)
            except Exception as exc:  # never let a bad command kill the thread
                self.log_message.emit(f"Command error: {exc}")

    def _dispatch(self, cmd: tuple) -> None:
        kind = cmd[0]
        if kind == "read":
            self._do_read(cmd[1])
        elif kind == "write":
            self._do_write(cmd[1], cmd[2])
        elif kind == "apply_map":
            self._do_apply_map(cmd[1])
        elif kind == "subscribe":
            self._do_subscribe(cmd[1], cmd[2])
        elif kind == "unsubscribe":
            self._do_unsubscribe()

    def _transaction(self, req: bytes, seq: int):
        """Send a request and wait for the matching-seq response, with retransmit.

        Returns (header, payload) or None on timeout. Stale/foreign datagrams are
        skipped; ERROR datagrams matching our seq are surfaced and returned.
        """
        sock = self._od_sock
        if sock is None:
            return None
        for _ in range(RETRANSMIT_TRIES):
            try:
                sock.sendto(req, self._addr)
            except OSError as exc:
                self.log_message.emit(f"Send failed: {exc}")
                return None
            deadline = time.monotonic() + RETRANSMIT_TIMEOUT
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    sock.settimeout(remaining)
                    data, _ = sock.recvfrom(2048)
                except socket.timeout:
                    break
                except OSError:
                    return None
                hdr = proto.parse_header(data)
                if hdr is None or hdr.seq != seq:
                    continue
                if hdr.version != proto.PROTOCOL_VERSION:
                    self.log_message.emit(
                        f"Version mismatch from CMC (got {hdr.version})"
                    )
                payload = data[proto.HEADER_SIZE:proto.HEADER_SIZE + hdr.length]
                return hdr, payload
        return None

    def _do_read(self, entry: OdEntry) -> None:
        seq = self._next_seq()
        req = proto.build_read_req(seq, entry.index, entry.sub, entry.type_code)
        resp = self._transaction(req, seq)
        if resp is None:
            self.od_read_done.emit({"entry": entry, "ok": False, "error": "timeout"})
            return
        hdr, payload = resp
        if hdr.type == proto.MSG_ERROR:
            err = proto.parse_error(payload)
            if err:
                self.error_received.emit(err)
            self.od_read_done.emit({"entry": entry, "ok": False,
                                    "error": err.describe() if err else "error"})
            return
        rr = proto.parse_read_resp(payload)
        if rr is None or rr.result != proto.OD_OK:
            name = proto.OD_RESULT_NAME.get(rr.result, rr.result) if rr else "bad-resp"
            self.od_read_done.emit({"entry": entry, "ok": False, "result": rr.result if rr else None,
                                    "error": str(name)})
            return
        raw = entry.decode(rr.data)
        self.od_read_done.emit({"entry": entry, "ok": True, "result": rr.result,
                                "raw": raw, "si": entry.raw_to_si(raw)})

    def _do_write(self, entry: OdEntry, raw_value) -> None:
        seq = self._next_seq()
        data = entry.encode(raw_value)
        req = proto.build_write_req(seq, entry.index, entry.sub, entry.type_code, data)
        resp = self._transaction(req, seq)
        if resp is None:
            self.od_write_done.emit({"entry": entry, "ok": False, "error": "timeout"})
            return
        hdr, payload = resp
        if hdr.type == proto.MSG_ERROR:
            err = proto.parse_error(payload)
            if err:
                self.error_received.emit(err)
            self.od_write_done.emit({"entry": entry, "ok": False,
                                     "error": err.describe() if err else "error"})
            return
        wr = proto.parse_write_resp(payload)
        ok = wr is not None and wr.result == proto.OD_OK
        result_name = (proto.OD_RESULT_NAME.get(wr.result, wr.result) if wr else "bad-resp")
        self.od_write_done.emit({"entry": entry, "ok": ok,
                                 "result": wr.result if wr else None,
                                 "error": None if ok else str(result_name)})

    def _do_apply_map(self, entries: list[OdEntry]) -> None:
        """Write the 0x2A00 telemetry map atomically: deactivate, list, activate.

        Mirrors INTERFACE_SPEC "Telemetry mapping": write count=0 first, then each
        map word, then the new count to activate.
        """
        from .od import map_word, TLM_MAP_INDEX, TLM_MAX_ENTRIES, TLM_MAX_BYTES

        total_bytes = sum(e.size for e in entries)
        if len(entries) > TLM_MAX_ENTRIES:
            self.map_applied.emit({"ok": False, "message": f"too many entries (>{TLM_MAX_ENTRIES})"})
            return
        if total_bytes > TLM_MAX_BYTES:
            self.map_applied.emit({"ok": False,
                                   "message": f"map is {total_bytes} B, exceeds {TLM_MAX_BYTES} B budget"})
            return
        for e in entries:
            if not e.is_pdo:
                self.map_applied.emit({"ok": False, "message": f"{e.label} is not PDO-mappable"})
                return

        count_entry = self.od.get(TLM_MAP_INDEX, 0)
        if count_entry is None:
            self.map_applied.emit({"ok": False, "message": "0x2A00:0 missing from OD"})
            return

        # 1) deactivate
        if not self._blocking_write(count_entry, 0):
            self.map_applied.emit({"ok": False, "message": "failed to deactivate map (0x2A00:0=0)"})
            return
        # 2) write each map word
        for i, e in enumerate(entries, start=1):
            slot = self.od.get(TLM_MAP_INDEX, i)
            if slot is None:
                self.map_applied.emit({"ok": False, "message": f"0x2A00:{i} missing from OD"})
                return
            word = map_word(e.index, e.sub, e.size * 8)
            if not self._blocking_write(slot, word):
                self.map_applied.emit({"ok": False, "message": f"failed writing slot {i} ({e.label})"})
                return
        # 3) activate
        if not self._blocking_write(count_entry, len(entries)):
            self.map_applied.emit({"ok": False, "message": "failed to activate map (0x2A00:0=count)"})
            return

        self._active_map = list(entries)  # atomic ref swap for the telemetry thread
        self._last_counter = None
        self.map_applied.emit({"ok": True, "entries": list(entries),
                               "message": f"map applied: {len(entries)} channels, {total_bytes} B"})

    def _blocking_write(self, entry: OdEntry, raw_value) -> bool:
        seq = self._next_seq()
        req = proto.build_write_req(seq, entry.index, entry.sub, entry.type_code,
                                    entry.encode(raw_value))
        resp = self._transaction(req, seq)
        if resp is None:
            return False
        hdr, payload = resp
        if hdr.type == proto.MSG_ERROR:
            err = proto.parse_error(payload)
            if err:
                self.error_received.emit(err)
            return False
        wr = proto.parse_write_resp(payload)
        return wr is not None and wr.result == proto.OD_OK

    def _do_subscribe(self, rate_divider: int, batch: int) -> None:
        # Re-punch the firewall flow before subscribing — Windows ages out
        # unused UDP "connection" entries after a few minutes, so if a user
        # connected long before subscribing, the connect-time punch may
        # have expired.
        self._punch_telemetry_flow()

        seq = self._next_seq()
        req = proto.build_subscribe(seq, self._tlm_port, rate_divider, batch)
        # fire-and-forget (no response defined); send a few for reliability over UDP
        for _ in range(3):
            try:
                self._od_sock.sendto(req, self._addr)
            except OSError:
                break
            time.sleep(0.01)
        self.log_message.emit(
            f"Subscribed: rx :{self._tlm_port}, divider {rate_divider}, batch {batch}"
        )

    def _do_unsubscribe(self) -> None:
        seq = self._next_seq()
        req = proto.build_unsubscribe(seq)
        for _ in range(3):
            try:
                self._od_sock.sendto(req, self._addr)
            except OSError:
                break
            time.sleep(0.01)
        self.log_message.emit("Unsubscribed")

    # --- telemetry thread -----------------------------------------------------
    def _tlm_loop(self) -> None:
        sock = self._tlm_sock
        while self._running and sock is not None:
            try:
                sock.settimeout(0.5)
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                self._emit_stats(force=False)
                continue
            except OSError:
                break
            hdr = proto.parse_header(data)
            if hdr is None or hdr.type != proto.MSG_TELEMETRY:
                continue
            payload = data[proto.HEADER_SIZE:proto.HEADER_SIZE + hdr.length]
            dg = proto.parse_telemetry(payload)
            if dg is None:
                continue
            samples = self._unpack(dg)
            if samples:
                self.telemetry_samples.emit(samples)
            self._stat_count += len(samples)
            self._frame_map_version = dg.map_version
            self._emit_stats(force=False)

    def _unpack(self, dg: proto.TelemetryDatagram) -> list[TelemetrySample]:
        active = self._active_map  # snapshot the ref
        expected_bytes = sum(e.size for e in active)
        out: list[TelemetrySample] = []
        for sh, blob in dg.records:
            layout_ok = (active and sh.map_byte_count == expected_bytes
                         and len(blob) >= expected_bytes)
            values: dict[str, float] = {
                "statusword": float(sh.statusword),
                "mode_display": float(sh.mode_display),
                "node_state": float(sh.node_state),
                "error_code": float(sh.error_code),
                "status_counter": float(sh.status_counter),
                "movement_status": float(sh.movement_status),   # v4 header (REQ-0013)
            }
            if layout_ok:
                offset = 0
                for e in active:
                    raw = e.decode(blob[offset:offset + e.size])
                    values[e.name] = e.raw_to_si(raw)
                    offset += e.size
            out.append(TelemetrySample(
                counter=sh.status_counter, values=values, statusword=sh.statusword,
                mode_display=sh.mode_display, node_state=sh.node_state,
                error_code=sh.error_code, map_version=sh.map_version, layout_ok=layout_ok,
            ))
            # drop detection from the status counter cadence
            if self._last_counter is not None:
                delta = (sh.status_counter - self._last_counter) & 0xFFFFFFFF
                if 0 < delta < 1_000_000:  # ignore wrap/reset
                    self._dropped += max(0, delta - 1)
            self._last_counter = sh.status_counter
        return out

    def _emit_stats(self, force: bool) -> None:
        now = time.monotonic()
        dt = now - self._stat_t0
        if dt < 0.5 and not force:
            return
        rate = self._stat_count / dt if dt > 0 else 0.0
        self.telemetry_stats.emit({
            "rate_hz": rate, "dropped": self._dropped,
            "frame_map_version": self._frame_map_version,
        })
        self._stat_count = 0
        self._stat_t0 = now
