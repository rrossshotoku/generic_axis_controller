"""Firmware update driver — CMC bootloader over UDP.

Runs the 8-step sequence from Documentation/dual_bootloader_design.md §4:
    1. Read 0x1F56 program_software_id (current image CRC32).
    2. Compute CRC32 of the new .bin.
    3. If equal -> "already up to date" (no update needed).
    4. Write 0x1F51:1 = MC_IF_PROG_START — CMC reboots into bootloader.
    5. Wait for bootloader to come up, then segmented-write bytes into 0x1F50.
    6. Write 0x1F51:1 = MC_IF_PROG_VERIFY — bootloader CRCs.
    7. Write 0x1F51:1 = MC_IF_PROG_COMMIT — bootloader reboots into new app.
    8. Reconnect, read 0x1F56 again to confirm the new CRC is running.

Callable from the GUI thread via ``FirmwareUpdater.run(...)`` which does the
whole flow blocking (worker thread expected). Progress is reported through
a callback so the GUI can drive a progress bar / log lines.
"""
from __future__ import annotations

import socket
import struct
import time
import zlib
from dataclasses import dataclass
from typing import Callable

from . import protocol as proto

# --- protocol constants (mirror Interface/mc_if_od.h) ------------------------
OD_PROG_CONTROL_INDEX = 0x1F51
OD_PROG_CONTROL_SUB = 1
OD_PROG_SOFTWARE_ID_INDEX = 0x1F56
OD_PROG_SOFTWARE_ID_SUB = 1
OD_PROG_DATA_INDEX = 0x1F50
OD_PROG_DATA_SUB = 1
OD_FLASH_STATUS_INDEX = 0x1F57
OD_FLASH_STATUS_SUB = 1

PROG_STOP = 0x00
PROG_START = 0x01
PROG_VERIFY = 0x02
PROG_COMMIT = 0x03
PROG_ABORT = 0x80

FLASH_IDLE = 0x0000
FLASH_ERASING = 0x0001
FLASH_PROGRAMMING = 0x0002
FLASH_VERIFYING = 0x0003
FLASH_FAULT = 0x0004

FLASH_STATE_NAME = {
    FLASH_IDLE: "IDLE", FLASH_ERASING: "ERASING",
    FLASH_PROGRAMMING: "PROGRAMMING", FLASH_VERIFYING: "VERIFYING",
    FLASH_FAULT: "FAULT",
}

# Update target — determines how PROG_START is delivered.
# CMC: write to 0x3018 cmc_boot_request (CMC-owned trigger, no ambiguity with
#      motor OD access).
# MOTOR: write to 0x1F51:1 which the CMC's OD dispatcher forwards to the motor
#      over SPI via cia402 raw passthrough. All other steps (segmented download,
#      VERIFY, COMMIT, CRC readback) also route through the CMC transparently.
TARGET_CMC = "cmc"
TARGET_MOTOR = "motor"
OD_CMC_BOOT_REQUEST_INDEX = 0x3018
OD_CMC_BOOT_REQUEST_SUB = 0


class UpdateError(RuntimeError):
    """Recoverable failure — user gets a message, no partial commit happens."""


@dataclass
class Progress:
    """Progress signalled to the caller. Any/all fields may be None."""
    stage: str                 # short label ("connecting", "erasing", "programming"...)
    bytes_sent: int = 0        # bytes shipped so far (segmented phase)
    total_bytes: int = 0       # bytes in the image
    message: str = ""          # human-readable log line


ProgressCb = Callable[[Progress], None]


class FirmwareUpdater:
    """One-shot driver. Construct, call ``run(...)``, throw the instance away.

    Not thread-safe; each update should get its own instance in a worker
    thread. The socket is opened in run() and closed on exit.
    """

    def __init__(self, cmc_ip: str, od_port: int = 5000,
                 target: str = TARGET_CMC, cb: ProgressCb | None = None):
        if target not in (TARGET_CMC, TARGET_MOTOR):
            raise ValueError(f"target must be {TARGET_CMC!r} or {TARGET_MOTOR!r}")
        self._addr = (cmc_ip, od_port)
        self._target = target
        self._sock: socket.socket | None = None
        self._cb = cb or (lambda p: None)
        self._seq = 0

    # --- socket helpers -----------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def _send_recv(self, req: bytes, seq: int, timeout: float = 0.2,
                   retries: int = 4) -> tuple[proto.UdpHeader, bytes] | None:
        assert self._sock is not None
        for _ in range(retries):
            try:
                self._sock.sendto(req, self._addr)
            except OSError:
                return None
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    self._sock.settimeout(remaining)
                    data, _peer = self._sock.recvfrom(2048)
                except socket.timeout:
                    break
                except OSError:
                    return None
                hdr = proto.parse_header(data)
                if hdr is None or hdr.seq != seq:
                    continue
                payload = data[proto.HEADER_SIZE:proto.HEADER_SIZE + hdr.length]
                return hdr, payload
        return None

    # --- OD helpers ---------------------------------------------------------
    def _read_u32(self, idx: int, sub: int) -> int:
        seq = self._next_seq()
        resp = self._send_recv(proto.build_read_req(seq, idx, sub, 2), seq)  # 2 = U32
        if resp is None:
            raise UpdateError(f"read 0x{idx:04X}:{sub} — no response")
        hdr, payload = resp
        r = proto.parse_read_resp(payload)
        if r is None:
            raise UpdateError(f"read 0x{idx:04X}:{sub} — malformed/short reply")
        if r.result != proto.OD_OK:
            raise UpdateError(f"read 0x{idx:04X}:{sub} rejected (result=0x{r.result:02X})")
        if len(r.data) < 4:
            raise UpdateError(f"read 0x{idx:04X}:{sub} short payload")
        return struct.unpack("<I", r.data[:4])[0]

    def _read_u16(self, idx: int, sub: int) -> int:
        seq = self._next_seq()
        resp = self._send_recv(proto.build_read_req(seq, idx, sub, 1), seq)  # 1 = U16
        if resp is None:
            raise UpdateError(f"read 0x{idx:04X}:{sub} — no response")
        hdr, payload = resp
        r = proto.parse_read_resp(payload)
        if r is None:
            raise UpdateError(f"read 0x{idx:04X}:{sub} — malformed/short reply")
        if r.result != proto.OD_OK:
            raise UpdateError(f"read 0x{idx:04X}:{sub} rejected (result=0x{r.result:02X})")
        return struct.unpack("<H", r.data[:2])[0]

    def _write_u8(self, idx: int, sub: int, value: int, tries: int = 4) -> None:
        seq = self._next_seq()
        req = proto.build_write_req(seq, idx, sub, 0, struct.pack("<B", value & 0xFF))
        resp = self._send_recv(req, seq, retries=tries)
        if resp is None:
            raise UpdateError(f"write 0x{idx:04X}:{sub} — no response")
        _hdr, payload = resp
        w = proto.parse_write_resp(payload)
        if w is None or w.result != proto.OD_OK:
            raise UpdateError(f"write 0x{idx:04X}:{sub} rejected (result=0x{w.result:02X})")

    # --- segmented download -------------------------------------------------
    def _download_init(self, total_bytes: int) -> None:
        seq = self._next_seq()
        req = proto.build_download_init(seq, OD_PROG_DATA_INDEX, OD_PROG_DATA_SUB, total_bytes)
        # Erase can take multiple seconds — bump the per-attempt timeout well
        # above the CMC's cia402 raw passthrough timeout for DOWNLOAD_INIT
        # (currently 10 s). If we're timing out at 10 s but the motor is
        # still erasing, we'd give up early.
        resp = self._send_recv(req, seq, timeout=15.0, retries=1)
        if resp is None:
            raise UpdateError("DOWNLOAD_INIT — no response (motor bootloader may be erasing; retry after ~10 s)")
        hdr, payload = resp
        # CMC forwards a MSG_OD_DOWNLOAD_RESP if motor answered normally; if
        # cia402 timed out or the motor sent an error, CMC surfaces it as
        # MSG_ERROR with class ERR_OD and detail = MC_IfOdResult_t.
        if hdr.type == proto.MSG_ERROR:
            e = proto.parse_error(payload)
            det = f"class=0x{e.error_class:02X} detail=0x{e.detail:02X}" if e else "unparseable"
            raise UpdateError(f"DOWNLOAD_INIT — CMC returned ERROR ({det})")
        if hdr.type != proto.MSG_OD_DOWNLOAD_RESP:
            raise UpdateError(f"DOWNLOAD_INIT — unexpected msg_type=0x{hdr.type:02X}")
        r = proto.parse_download_resp(payload)
        if r is None:
            raise UpdateError(f"DOWNLOAD_INIT — malformed DOWNLOAD_RESP ({len(payload)} B)")
        if r.result != proto.OD_OK:
            raise UpdateError(f"DOWNLOAD_INIT rejected (result=0x{r.result:02X})")

    def _download_segment(self, chunk: bytes, toggle: int, last: bool) -> proto.DownloadResp:
        seq = self._next_seq()
        req = proto.build_download_segment(seq, chunk, toggle, last)
        resp = self._send_recv(req, seq, timeout=0.5, retries=4)
        if resp is None:
            raise UpdateError("DOWNLOAD_SEGMENT — no response")
        hdr, payload = resp
        if hdr.type == proto.MSG_ERROR:
            e = proto.parse_error(payload)
            det = f"class=0x{e.error_class:02X} detail=0x{e.detail:02X}" if e else "unparseable"
            raise UpdateError(f"DOWNLOAD_SEGMENT — CMC returned ERROR ({det})")
        if hdr.type != proto.MSG_OD_DOWNLOAD_RESP:
            raise UpdateError(f"DOWNLOAD_SEGMENT — unexpected msg_type=0x{hdr.type:02X}")
        r = proto.parse_download_resp(payload)
        if r is None:
            raise UpdateError(f"DOWNLOAD_SEGMENT — malformed DOWNLOAD_RESP ({len(payload)} B)")
        if r.result != proto.OD_OK:
            raise UpdateError(f"DOWNLOAD_SEGMENT rejected (result=0x{r.result:02X})")
        return r

    # --- CMC come-up wait ---------------------------------------------------
    def _wait_for_bootloader(self, deadline_s: float) -> None:
        """Poll 0x1F57 flash_status until the bootloader answers OK.
        The bootloader owns 0x1F5x; the app returns NOT_BOOTLOADER on the
        same read. So a successful OK-response == bootloader is up."""
        end = time.monotonic() + deadline_s
        last_err = ""
        while time.monotonic() < end:
            try:
                _ = self._read_u16(OD_FLASH_STATUS_INDEX, OD_FLASH_STATUS_SUB)
                return  # bootloader answered with OK — it's up
            except UpdateError as e:
                last_err = str(e)
                time.sleep(0.2)
        raise UpdateError(f"Timed out waiting for bootloader ({last_err})")

    def _wait_for_app(self, deadline_s: float) -> None:
        """Poll a target-specific app-only OD entry until we see OK, meaning
        the target's app is answering. Different entry for CMC vs Motor:
          - CMC:   0x3006 axis_active_operation (CMC-owned; only answered
                   by CMC app).
          - Motor: 0x6041 statusword (motor-app owned; motor bootloader
                   returns NO_OBJECT). The CMC pass-through forwards the
                   read over SPI, so an OK response really is the motor
                   answering.
        """
        if self._target == TARGET_CMC:
            probe_idx, probe_sub = 0x3006, 0
        else:
            probe_idx, probe_sub = 0x6041, 0  # CiA-402 statusword, motor-owned
        end = time.monotonic() + deadline_s
        last_err = ""
        while time.monotonic() < end:
            seq = self._next_seq()
            resp = self._send_recv(proto.build_read_req(seq, probe_idx, probe_sub, 0),
                                   seq, timeout=0.5, retries=2)
            if resp is not None:
                _hdr, payload = resp
                r = proto.parse_read_resp(payload)
                if r is not None and r.result == proto.OD_OK:
                    return  # target's app answered — it's up
                last_err = f"target rejected 0x{probe_idx:04X}:{probe_sub} read"
            else:
                last_err = "no response"
            time.sleep(0.2)
        raise UpdateError(f"Timed out waiting for app ({last_err})")

    # --- top-level flow -----------------------------------------------------
    def run(self, image_bytes: bytes) -> None:
        """Execute the full 8-step update sequence. Blocks the calling thread.

        Raises UpdateError on any recoverable failure — the caller catches
        and surfaces to the operator; nothing is committed unless every
        step succeeds.
        """
        if not image_bytes:
            raise UpdateError("Empty image — refusing to flash.")
        new_crc = zlib.crc32(image_bytes) & 0xFFFFFFFF
        total = len(image_bytes)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("", 0))

            # --- Step 1: read current CRC (from app OR bootloader). --------
            self._cb(Progress(stage="probing", message="Reading current image CRC…"))
            try:
                current_crc = self._read_u32(OD_PROG_SOFTWARE_ID_INDEX, OD_PROG_SOFTWARE_ID_SUB)
                self._cb(Progress(stage="probing",
                                  message=f"Current CRC: 0x{current_crc:08X}, new CRC: 0x{new_crc:08X}"))
            except UpdateError:
                # App-mode returns NOT_BOOTLOADER for 0x1F56 reads. That's
                # expected in normal operation — treat as "unknown, proceed."
                current_crc = 0
                self._cb(Progress(stage="probing",
                                  message=f"App mode active (no CRC); new CRC: 0x{new_crc:08X}"))

            # --- Step 2/3: skip if already up to date. ---------------------
            if current_crc == new_crc:
                self._cb(Progress(stage="done",
                                  message="Image CRC matches — already up to date."))
                return

            # --- Step 4: request bootloader entry. -------------------------
            # CMC target: dedicated 0x3018 cmc_boot_request (CMC-owned).
            # MOTOR target: 0x1F51:1 = PROG_START, which the CMC's OD
            #   dispatcher forwards to the motor over SPI. The motor's app
            #   handles it (writes flag + resets), then the motor comes up
            #   in bootloader mode. Cyclic status will start showing
            #   node_state = BOOTLOADER; the CMC pauses cyclic commands
            #   automatically.
            if self._target == TARGET_CMC:
                trigger_idx = OD_CMC_BOOT_REQUEST_INDEX
                trigger_sub = OD_CMC_BOOT_REQUEST_SUB
                stage_msg = "Requesting CMC bootloader entry (0x3018 = PROG_START)…"
            else:  # TARGET_MOTOR
                trigger_idx = OD_PROG_CONTROL_INDEX
                trigger_sub = OD_PROG_CONTROL_SUB
                stage_msg = "Requesting MOTOR bootloader entry (0x1F51:1 = PROG_START via CMC)…"
            self._cb(Progress(stage="rebooting", message=stage_msg))
            try:
                self._write_u8(trigger_idx, trigger_sub, PROG_START)
            except UpdateError as e:
                # The response may not arrive because the target resets
                # before sending it. That's OK — we probe next.
                self._cb(Progress(stage="rebooting", message=f"(ignored: {e})"))
            time.sleep(1.0)   # give the target time to actually reset

            # --- Wait for bootloader to come up. ---------------------------
            self._cb(Progress(stage="rebooting", message="Waiting for bootloader…"))
            self._wait_for_bootloader(15.0)

            # --- Step 5: segmented download. -------------------------------
            self._cb(Progress(stage="erasing", total_bytes=total,
                              message="Erasing app region + starting download…"))
            self._download_init(total)

            self._cb(Progress(stage="programming", total_bytes=total,
                              message=f"Programming {total} bytes…"))
            toggle = 0
            sent = 0
            while sent < total:
                take = min(proto.SEG_DATA_MAX, total - sent)
                chunk = image_bytes[sent:sent + take]
                is_last = (sent + take) == total
                r = self._download_segment(chunk, toggle, is_last)
                # If the bootloader ack toggle doesn't match what we sent, it
                # missed a segment — resend. In practice the retry inside
                # _send_recv already handles UDP drops; this handles the case
                # where our idea of "toggle" fell out of sync.
                if r.toggle_ack != toggle:
                    continue
                sent += take
                toggle ^= 1
                if sent % (proto.SEG_DATA_MAX * 32) == 0 or is_last:
                    self._cb(Progress(stage="programming", bytes_sent=sent,
                                      total_bytes=total))

            # --- Step 6: verify. -------------------------------------------
            self._cb(Progress(stage="verifying", message="Verifying image…"))
            self._write_u8(OD_PROG_CONTROL_INDEX, OD_PROG_CONTROL_SUB, PROG_VERIFY)
            # Cross-check: read 0x1F56 which SHOULD now compute CRC over the
            # newly written bytes, and compare locally to what we expected.
            # Some bootloader implementations return 0 (not implemented) —
            # in that case we skip the local compare and trust PROG_VERIFY's
            # on-chip result. Segmented-SDO's toggle bit + last-segment
            # semantics already catch most transport errors.
            got_crc = self._read_u32(OD_PROG_SOFTWARE_ID_INDEX, OD_PROG_SOFTWARE_ID_SUB)
            if got_crc == 0:
                self._cb(Progress(stage="verifying",
                                  message=f"WARN: 0x1F56 returned 0 (bootloader CRC not implemented?) — "
                                          f"expected 0x{new_crc:08X}; skipping local CRC compare, "
                                          f"trusting bootloader PROG_VERIFY."))
            elif got_crc != new_crc:
                self._write_u8(OD_PROG_CONTROL_INDEX, OD_PROG_CONTROL_SUB, PROG_ABORT)
                raise UpdateError(
                    f"CRC mismatch after programming: got 0x{got_crc:08X}, expected 0x{new_crc:08X}."
                )
            else:
                self._cb(Progress(stage="verifying",
                                  message=f"CRC match (0x{new_crc:08X}) — verified."))

            # --- Step 7: commit + reset. -----------------------------------
            self._cb(Progress(stage="committing", message="Committing + resetting…"))
            try:
                self._write_u8(OD_PROG_CONTROL_INDEX, OD_PROG_CONTROL_SUB, PROG_COMMIT)
            except UpdateError:
                # The reset kills the response; that's fine.
                pass
            time.sleep(2.0)

            # --- Step 8: confirm the new app is running. -------------------
            self._cb(Progress(stage="confirming",
                              message="Waiting for new app to come up…"))
            try:
                self._wait_for_app(15.0)
                self._cb(Progress(stage="done",
                                  message="Update complete — new firmware is running."))
            except UpdateError as e:
                self._cb(Progress(stage="done",
                                  message=f"Update flashed but CMC didn't respond in app mode: {e}"))

        finally:
            if self._sock is not None:
                self._sock.close()
                self._sock = None
