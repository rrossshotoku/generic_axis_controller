"""Modal "Update firmware…" dialog + worker thread.

Extracted from main_window.py to keep that file focused on orchestration.
The dialog is entirely self-contained: pass a parent QWidget, a CMC IP, and
a diagnostics-timer to pause/resume; everything else lives here.

Usage:

    from .dialog_firmware_update import open_dialog
    open_dialog(parent=self, cmc_ip=ip, diag_timer=getattr(self, "diag_timer", None))
"""
from __future__ import annotations

from PySide6.QtCore import QThread, QTimer, Signal
from PySide6.QtWidgets import (QButtonGroup, QDialog, QDialogButtonBox,
                                QFileDialog, QFormLayout, QHBoxLayout, QLabel,
                                QMessageBox, QPlainTextEdit, QProgressBar,
                                QRadioButton, QVBoxLayout, QWidget)

from .firmware_update import (FirmwareUpdater, Progress, TARGET_CMC,
                              TARGET_MOTOR, UpdateError)


class _Worker(QThread):
    """QThread wrapping FirmwareUpdater so the GUI stays responsive."""
    progress = Signal(object)
    done = Signal(bool, str)

    def __init__(self, ip: str, img: bytes, target: str) -> None:
        super().__init__()
        self._ip = ip
        self._img = img
        self._target = target

    def run(self) -> None:
        updater = FirmwareUpdater(self._ip, target=self._target,
                                  cb=lambda p: self.progress.emit(p))
        try:
            updater.run(self._img)
            self.done.emit(True, "")
        except UpdateError as e:
            self.done.emit(False, str(e))
        except Exception as e:  # pragma: no cover — surface as failure log
            self.done.emit(False, f"Unexpected: {e!r}")


def open_dialog(parent: QWidget, cmc_ip: str,
                diag_timer: QTimer | None = None) -> None:
    """Show the firmware-update dialog. Blocks until the user closes it.

    diag_timer, if given, is stopped while the dialog is open (to avoid the
    diagnostics poller racing the update flow for the OD-access UDP socket)
    and re-started ~1 s after close.
    """
    if not cmc_ip:
        QMessageBox.warning(parent, "Not connected",
                            "Connect to the CMC first — the update path re-uses the OD-access socket.")
        return

    path, _ = QFileDialog.getOpenFileName(
        parent, "Select firmware .bin", "", "Firmware image (*.bin)")
    if not path:
        return
    try:
        with open(path, "rb") as f:
            image_bytes = f.read()
    except OSError as exc:
        QMessageBox.warning(parent, "Open failed", str(exc)); return

    dlg = QDialog(parent)
    dlg.setWindowTitle(f"Update firmware ({cmc_ip})")
    dlg.setModal(True)
    dlg.resize(560, 400)
    v = QVBoxLayout(dlg)
    form = QFormLayout()
    form.addRow("Image:", QLabel(f"{path}  ({len(image_bytes)} bytes)"))

    # Target selector.
    rb_cmc = QRadioButton("CMC")
    rb_motor = QRadioButton("Motor MCU (via CMC passthrough)")
    rb_cmc.setChecked(True)
    target_group = QButtonGroup(dlg)
    target_group.addButton(rb_cmc)
    target_group.addButton(rb_motor)
    row = QHBoxLayout()
    row.addWidget(rb_cmc); row.addWidget(rb_motor); row.addStretch(1)
    form.addRow("Target:", row)

    lbl_stage = QLabel("(waiting)")
    lbl_stage.setStyleSheet("font-weight: bold;")
    form.addRow("Stage:", lbl_stage)
    v.addLayout(form)

    pbar = QProgressBar()
    pbar.setRange(0, max(1, len(image_bytes)))
    v.addWidget(pbar)

    log = QPlainTextEdit()
    log.setReadOnly(True)
    log.setStyleSheet("font-family: monospace; font-size: 10px;")
    v.addWidget(log)

    buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                              | QDialogButtonBox.StandardButton.Close)
    buttons.button(QDialogButtonBox.StandardButton.Ok).setText("Start")
    buttons.button(QDialogButtonBox.StandardButton.Close).setEnabled(False)
    buttons.rejected.connect(dlg.reject)
    v.addWidget(buttons)

    def _on_progress(p: Progress) -> None:
        lbl_stage.setText(p.stage.upper())
        if p.total_bytes:
            pbar.setRange(0, p.total_bytes)
        if p.bytes_sent:
            pbar.setValue(p.bytes_sent)
        if p.message:
            log.appendPlainText(p.message)

    def _on_done(ok: bool, err: str) -> None:
        buttons.button(QDialogButtonBox.StandardButton.Close).setEnabled(True)
        log.appendPlainText("✓ complete." if ok else f"✗ FAILED: {err}")

    if diag_timer is not None:
        diag_timer.stop()

    worker_ref: dict[str, _Worker | None] = {"w": None}

    def _start() -> None:
        if worker_ref["w"] is not None:
            return
        target = TARGET_MOTOR if rb_motor.isChecked() else TARGET_CMC
        rb_cmc.setEnabled(False); rb_motor.setEnabled(False)
        buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        log.appendPlainText(f"Target: {target.upper()}")
        w = _Worker(cmc_ip, image_bytes, target)
        w.progress.connect(_on_progress)
        w.done.connect(_on_done)
        worker_ref["w"] = w
        w.start()

    buttons.button(QDialogButtonBox.StandardButton.Ok).clicked.connect(_start)
    dlg.exec()

    if worker_ref["w"] is not None:
        worker_ref["w"].wait(5000)
    if diag_timer is not None:
        QTimer.singleShot(1000, diag_timer.start)
