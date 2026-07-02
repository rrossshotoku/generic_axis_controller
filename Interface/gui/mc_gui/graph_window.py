"""Pop-out real-time streaming graph window.

Backed by pyqtgraph, so mouse interaction is native: **left-drag pans**,
**scroll-wheel zooms** (both axes; hover an axis to zoom only that one),
**right-drag** box-style zoom, **right-click** for the context menu (export, axis
modes). The toolbar adds **Pause** (freeze the view while you inspect it),
**Auto-scroll** (X follows the newest sample), and channel selection.

**Add marker** drops a draggable vertical time cursor spanning the whole plot; drag
two onto points of interest to read the time between them (and 1/Δt as a frequency)
in the readout below the plot. Double-click a marker to remove it, or **Clear
markers** to remove them all. (Pause first if auto-scroll keeps moving the view.)
"""
from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QPushButton, QVBoxLayout, QWidget,
)

from .buffer import TelemetryBuffer

# distinct, high-contrast curve colours
_PALETTE = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#46f0f0",
    "#f032e6", "#bcf60c", "#fabebe", "#008080", "#9a6324", "#800000",
    "#808000", "#000075", "#e6beff", "#808080",
]

REDRAW_HZ = 60


class GraphWindow(QWidget):
    _counter = 0

    def __init__(self, buffer: TelemetryBuffer, available: list[str],
                 initial: list[str] | None = None, parent=None):
        super().__init__(parent)
        GraphWindow._counter += 1
        self.setWindowTitle(f"Live Graph {GraphWindow._counter}")
        self.resize(900, 560)
        # Independent top-level window with full title-bar controls so the
        # operator can minimize / maximize / close it without affecting the
        # main app (MainWindow passes parent=None for full independence —
        # we still set the flag to keep behaviour correct if the constructor
        # is ever called with a parent during tests). MinimizeButtonHint +
        # MaximizeButtonHint make the controls explicit; on some Qt styles
        # the minimize button is hidden by default for non-QMainWindow tops.
        self.setWindowFlags(
            Qt.WindowType.Window
            | Qt.WindowType.WindowMinimizeButtonHint
            | Qt.WindowType.WindowMaximizeButtonHint
            | Qt.WindowType.WindowCloseButtonHint
        )

        self.buffer = buffer
        self.curves: dict[str, pg.PlotDataItem] = {}
        self.markers: list[pg.InfiniteLine] = []   # vertical time cursors (measure Δt)
        self.paused = False
        self.autoscroll = True
        self._color_idx = 0
        self._dirty = True             # force first paint
        self._last_t = -1.0            # latest_t at last paint (skip redraw if unchanged)
        self._last_range: list | None = None

        self._build_ui(available)
        for name in (initial or []):
            self._set_channel(name, True)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._redraw)
        self.timer.start(int(1000 / REDRAW_HZ))

    # --- UI -------------------------------------------------------------------
    def _build_ui(self, available: list[str]) -> None:
        root = QHBoxLayout(self)

        # left: channel picker
        left = QVBoxLayout()
        left.addWidget(QLabel("Channels"))
        self.channel_list = QListWidget()
        self.channel_list.setMaximumWidth(220)
        self.channel_list.itemChanged.connect(self._on_item_changed)
        left.addWidget(self.channel_list, 1)
        self.set_available_channels(available)
        root.addLayout(left)

        # right: plot + toolbar
        right = QVBoxLayout()
        bar = QHBoxLayout()
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause)
        bar.addWidget(self.btn_pause)

        self.chk_autoscroll = QCheckBox("Auto-scroll")
        self.chk_autoscroll.setChecked(True)
        self.chk_autoscroll.toggled.connect(self._on_autoscroll)
        bar.addWidget(self.chk_autoscroll)

        bar.addWidget(QLabel("Window (s):"))
        self.window_s = QDoubleSpinBox()
        self.window_s.setRange(0.1, 600.0)
        self.window_s.setValue(10.0)
        self.window_s.setSingleStep(1.0)
        self.window_s.valueChanged.connect(lambda _: self._mark_dirty())
        bar.addWidget(self.window_s)

        btn_yauto = QPushButton("Y auto")
        btn_yauto.setToolTip(
            "Refit the Y axis to currently visible data and keep it auto-fitting "
            "as new samples arrive. Mouse-zoom or mouse-pan on Y disables the "
            "auto-fit again until you click this.")
        btn_yauto.clicked.connect(self._y_auto)
        bar.addWidget(btn_yauto)

        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self.buffer.clear)
        bar.addWidget(btn_clear)

        btn_addmarker = QPushButton("Add marker")
        btn_addmarker.setToolTip("Drop a draggable vertical time marker. Drag two onto points of "
                                 "interest to read the time between them below; double-click a "
                                 "marker to remove it.")
        btn_addmarker.clicked.connect(self._add_marker)
        bar.addWidget(btn_addmarker)

        btn_clearmarkers = QPushButton("Clear markers")
        btn_clearmarkers.clicked.connect(self._clear_markers)
        bar.addWidget(btn_clearmarkers)

        bar.addStretch(1)
        right.addLayout(bar)

        # Antialiasing is very expensive with many points; off for real-time plotting.
        pg.setConfigOptions(antialias=False)
        self.plot_widget = pg.PlotWidget(background="w")
        self.plot = self.plot_widget.getPlotItem()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.addLegend(offset=(10, 10))
        self.plot.setDownsampling(mode="peak", auto=True)
        self.plot.setClipToView(True)
        # turning the mouse manually disengages auto-scroll so you can inspect freely
        self.plot.getViewBox().sigRangeChangedManually.connect(self._on_manual_range)
        # double-click (near) a marker removes it
        self.plot_widget.scene().sigMouseClicked.connect(self._on_scene_clicked)
        right.addWidget(self.plot_widget, 1)

        self.status = QLabel("")
        right.addWidget(self.status)
        self.marker_label = QLabel("")    # Δt readout between time markers
        self.marker_label.setStyleSheet("font-weight: bold; color: #b00020;")
        right.addWidget(self.marker_label)
        root.addLayout(right, 1)

    def set_available_channels(self, names: list[str]) -> None:
        """Refresh the channel list, preserving existing checks/curves."""
        existing = {self.channel_list.item(i).text(): self.channel_list.item(i)
                    for i in range(self.channel_list.count())}
        wanted = sorted(set(names) | set(self.curves.keys()))
        self.channel_list.blockSignals(True)
        for name in wanted:
            if name in existing:
                continue
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Checked if name in self.curves
                               else Qt.CheckState.Unchecked)
            self.channel_list.addItem(item)
        self.channel_list.blockSignals(False)

    def show_channel(self, name: str, on: bool = True) -> None:
        """Public: add/remove a channel (used to mirror the OD 'Watch' ticks)."""
        if on:
            self.set_available_channels([name])
        self._set_channel(name, on)

    # --- channel toggling -----------------------------------------------------
    def _on_item_changed(self, item: QListWidgetItem) -> None:
        self._set_channel(item.text(), item.checkState() == Qt.CheckState.Checked,
                          from_list=True)

    def _mark_dirty(self) -> None:
        self._dirty = True

    def _set_channel(self, name: str, on: bool, from_list: bool = False) -> None:
        if on and name not in self.curves:
            color = _PALETTE[self._color_idx % len(_PALETTE)]
            self._color_idx += 1
            # width=1 cosmetic pen uses Qt's fast path (wide pens are much slower)
            self.curves[name] = self.plot.plot(pen=pg.mkPen(color, width=1), name=name)
            self._dirty = True
        elif not on and name in self.curves:
            self.plot.removeItem(self.curves.pop(name))
            self._dirty = True
        if not from_list:
            # reflect state into the list widget
            for i in range(self.channel_list.count()):
                it = self.channel_list.item(i)
                if it.text() == name:
                    it.setCheckState(Qt.CheckState.Checked if on else Qt.CheckState.Unchecked)
                    break

    # --- toolbar handlers -----------------------------------------------------
    def _on_pause(self, checked: bool) -> None:
        self.paused = checked
        self.btn_pause.setText("Resume" if checked else "Pause")
        self._dirty = True  # repaint once on resume

    def _on_autoscroll(self, checked: bool) -> None:
        self.autoscroll = checked
        if checked:
            self.plot.enableAutoRange(axis="y")
        self._dirty = True

    def _on_manual_range(self, *args) -> None:
        """User panned/zoomed with the mouse. Never disables autoscroll —
        mouse interaction is an inspection gesture, not a control input.

        Autoscroll only turns off via the explicit checkbox. When it's on,
        the redraw loop keeps snapping X back to (latest - window) every
        new sample, so any manual pan you do is visible until the next
        data tick. Mouse Y zoom/pan still works freely; X mouse-pan looks
        like a brief flick before autoscroll re-asserts on the next sample.
        For a sustained look-back, un-tick "Auto-scroll" first.

        (Previously this handler unchecked autoscroll on any mouse change,
        which made the live view "stick" after a single accidental scroll.)
        """
        self._dirty = True
        _ = args  # mask not consulted any more

    def _y_auto(self) -> None:
        """Refit Y to currently visible data and keep auto-fitting going forward.

        Does BOTH enableAutoRange (so subsequent setData calls refit) AND
        an explicit autoRange call (force an immediate refit even when the
        data hasn't moved — mouse-pan can leave the ViewBox in a stale
        manual range that enableAutoRange alone doesn't visibly correct
        until the next data tick). Without both, the button was a no-op
        in the steady-state-no-new-data case.
        """
        vb = self.plot.getViewBox()
        try:
            vb.enableAutoRange(axis=vb.YAxis, enable=True)
            vb.updateAutoRange()
        except Exception:
            # Fallback for older pyqtgraph variants
            self.plot.enableAutoRange(axis="y")
            self.plot.autoRange()
        self._dirty = True

    # --- time markers (measure Δt between two points) -------------------------
    def _add_marker(self) -> None:
        """Drop a draggable vertical marker at the centre of the current view."""
        (x_lo, x_hi), _ = self.plot.getViewBox().viewRange()
        x = 0.5 * (x_lo + x_hi)
        line = pg.InfiniteLine(
            pos=x, angle=90, movable=True,
            pen=pg.mkPen("#202020", width=1, style=Qt.PenStyle.DashLine),
            hoverPen=pg.mkPen("#202020", width=2),
            label="{value:.4f}s",
            labelOpts={"position": 0.92, "color": "#202020",
                       "fill": (255, 255, 255, 180), "movable": True},
        )
        line.sigPositionChanged.connect(self._update_marker_readout)
        self.plot.addItem(line)
        self.markers.append(line)
        self._update_marker_readout()

    def _clear_markers(self) -> None:
        for line in self.markers:
            self.plot.removeItem(line)
        self.markers.clear()
        self._update_marker_readout()

    def _remove_marker(self, line: pg.InfiniteLine) -> None:
        if line in self.markers:
            self.plot.removeItem(line)
            self.markers.remove(line)
            self._update_marker_readout()

    def _on_scene_clicked(self, ev) -> None:
        """Double-click on (or next to) a marker removes it."""
        if not self.markers or not ev.double():
            return
        vb = self.plot.getViewBox()
        x = vb.mapSceneToView(ev.scenePos()).x()
        (x_lo, x_hi), _ = vb.viewRange()
        tol = (x_hi - x_lo) * 0.01  # within 1% of the view width counts as a hit
        nearest = min(self.markers, key=lambda m: abs(m.value() - x))
        if abs(nearest.value() - x) <= tol:
            self._remove_marker(nearest)
            ev.accept()

    def _update_marker_readout(self) -> None:
        xs = sorted(m.value() for m in self.markers)
        if len(xs) < 2:
            self.marker_label.setText(f"Marker @ {xs[0]:.4f} s" if xs else "")
            return
        deltas = [b - a for a, b in zip(xs[:-1], xs[1:])]
        if len(deltas) == 1:
            dt = deltas[0]
            freq = f"{1.0 / dt:.3f} Hz" if dt > 1e-12 else "--"
            self.marker_label.setText(f"Δt = {dt:.4f} s   ({freq})")
        else:
            self.marker_label.setText(
                "Δt = " + ", ".join(f"{d:.4f}" for d in deltas) + f" s   [{len(xs)} markers]"
            )

    # --- redraw ---------------------------------------------------------------
    def _redraw(self) -> None:
        if self.paused:
            return  # frozen: keep showing the last frame, full mouse interaction stays live

        latest = self.buffer.latest_t
        # Skip the whole frame when nothing changed (no new data, no user interaction).
        # This caps real work to the data rate instead of the 60 Hz timer rate.
        if not self._dirty and latest == self._last_t:
            return
        self._dirty = False
        self._last_t = latest

        # Decide the visible X-range, then push ONLY that slice to the plotter.
        # This keeps redraw cost tied to what's on screen, not to total history.
        if self.autoscroll and self.curves:
            x_hi = latest
            x_lo = x_hi - self.window_s.value()
            self.plot.setXRange(x_lo, x_hi, padding=0)  # programmatic: no manual-range signal
        else:
            (x_lo, x_hi), _ = self.plot.getViewBox().viewRange()

        margin = max(1e-9, (x_hi - x_lo) * 0.1)  # a little slack so small pans don't reveal gaps
        q_lo, q_hi = x_lo - margin, x_hi + margin
        shown = 0
        for name, curve in self.curves.items():
            seg = self.buffer.window(name, q_lo, q_hi)
            if seg is not None and len(seg[0]):
                # skipFiniteCheck: our data is always finite, so skip the O(n) NaN scan
                curve.setData(seg[0], seg[1], skipFiniteCheck=True)
                shown += len(seg[0])
            else:
                curve.setData([], [])
        self.status.setText(
            f"{len(self.curves)} channel(s) | t={latest:.3f}s | "
            f"{'auto-scroll' if self.autoscroll else 'manual view'} | {shown} pts on screen"
        )

    def closeEvent(self, event) -> None:
        self.timer.stop()
        super().closeEvent(event)
