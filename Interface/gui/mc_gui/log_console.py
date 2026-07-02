"""Dockable terminal-style debug console.

Thread-safe: ``log()`` may be called from any thread (it marshals onto the GUI
thread via a signal), so the network worker threads can log directly.
"""
from __future__ import annotations

import html
from datetime import datetime

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QCheckBox, QFileDialog, QHBoxLayout, QPlainTextEdit, QPushButton, QVBoxLayout,
    QWidget,
)

_LEVEL_COLORS = {
    "DEBUG": "#888888",
    "INFO": "#202020",
    "WARN": "#b35900",
    "ERROR": "#c0152f",
    "TX": "#0a6cbf",
    "RX": "#117a37",
}


class LogConsole(QWidget):
    _post = Signal(str, str)  # level, preformatted html line

    def __init__(self, parent=None):
        super().__init__(parent)
        self.verbose = False
        self._build_ui()
        self._post.connect(self._append)

    def _build_ui(self) -> None:
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)

        bar = QHBoxLayout()
        self.chk_autoscroll = QCheckBox("Auto-scroll")
        self.chk_autoscroll.setChecked(True)
        bar.addWidget(self.chk_autoscroll)
        self.chk_verbose = QCheckBox("Verbose (debug)")
        self.chk_verbose.toggled.connect(self._on_verbose)
        bar.addWidget(self.chk_verbose)
        bar.addStretch(1)
        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self.clear)
        bar.addWidget(btn_clear)
        btn_save = QPushButton("Save...")
        btn_save.clicked.connect(self._save)
        bar.addWidget(btn_save)
        lay.addLayout(bar)

        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(5000)
        self.view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        font = QFont("Consolas")
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setPointSize(9)
        self.view.setFont(font)
        lay.addWidget(self.view)

    def _on_verbose(self, on: bool) -> None:
        self.verbose = on
        self.log(f"verbose logging {'enabled' if on else 'disabled'}", "INFO")

    # --- public API -----------------------------------------------------------
    def log(self, msg: str, level: str = "INFO") -> None:
        if level == "DEBUG" and not self.verbose:
            return
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = _LEVEL_COLORS.get(level, "#202020")
        line = (f'<span style="color:#999">{ts}</span> '
                f'<span style="color:{color}; font-weight:bold">{level:<5}</span> '
                f'<span style="color:{color}">{html.escape(str(msg))}</span>')
        self._post.emit(level, line)  # safe from any thread

    def _append(self, _level: str, line_html: str) -> None:
        self.view.appendHtml(line_html)
        if self.chk_autoscroll.isChecked():
            sb = self.view.verticalScrollBar()
            sb.setValue(sb.maximum())

    def clear(self) -> None:
        self.view.clear()

    def _save(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save log", "cmc_log.txt",
                                              "Text files (*.txt);;All files (*)")
        if path:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(self.view.toPlainText())
            self.log(f"log saved to {path}", "INFO")
