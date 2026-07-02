"""Rolling telemetry buffer shared by the graph windows.

The network client emits samples on the GUI thread; the main window feeds them
here, and each graph window pulls ordered arrays on its own redraw timer. This
decouples the (up to 1 kHz) sample rate from the (~60 Hz) plot redraw rate.

Storage is a contiguous, amortised ring: appends are O(1), the valid data is
always a *view* (no per-frame copy), and the timestamps are monotonic so a time
window can be located with a binary search. Graph windows query only the visible
window each frame, so redraw cost depends on what's on screen, not on how long
the session has been running or how many points are buffered.
"""
from __future__ import annotations

import time

import numpy as np

from .client import TelemetrySample


class RingBuffer:
    """Keeps the most recent ``capacity`` (time, value) pairs, contiguously.

    Backed by a 2x-capacity physical array: appends fill forwards; when full, the
    last ``capacity`` samples are shifted to the front (amortised O(1) per append).
    The valid data ``[lo:n]`` is therefore always contiguous, ordered, and
    returnable as a view without copying.
    """

    def __init__(self, capacity: int):
        self.capacity = capacity
        self._phys = capacity * 2
        self.t = np.empty(self._phys, dtype=np.float64)
        self.v = np.empty(self._phys, dtype=np.float64)
        self.n = 0

    @property
    def _lo(self) -> int:
        return max(0, self.n - self.capacity)

    def append(self, t: float, v: float) -> None:
        if self.n >= self._phys:
            keep = self.capacity
            self.t[:keep] = self.t[self.n - keep:self.n]
            self.v[:keep] = self.v[self.n - keep:self.n]
            self.n = keep
        self.t[self.n] = t
        self.v[self.n] = v
        self.n += 1

    def data(self) -> tuple[np.ndarray, np.ndarray]:
        lo = self._lo
        return self.t[lo:self.n], self.v[lo:self.n]

    def window(self, t_lo: float, t_hi: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the slice whose timestamps fall in [t_lo, t_hi], plus one
        neighbouring sample each side so lines reach the edges of the view."""
        lo = self._lo
        t = self.t[lo:self.n]
        v = self.v[lo:self.n]
        i0 = int(np.searchsorted(t, t_lo, side="left"))
        i1 = int(np.searchsorted(t, t_hi, side="right"))
        i0 = max(0, i0 - 1)
        i1 = min(len(t), i1 + 1)
        return t[i0:i1], v[i0:i1]

    def clear(self) -> None:
        self.n = 0


class TelemetryBuffer:
    """Per-channel rolling buffers, keyed by channel name."""

    def __init__(self, capacity: int = 120_000, rate_hz: float = 1000.0):
        self.capacity = capacity
        self.rate_hz = rate_hz
        self.channels: dict[str, RingBuffer] = {}
        self.latest_t = 0.0
        self.t0: float | None = None          # wall-clock origin (connect time)
        self._anchor_counter: int | None = None  # status_counter at first stream sample
        self._anchor_t = 0.0

    def set_rate(self, rate_hz: float) -> None:
        self.rate_hz = max(1.0, rate_hz)

    def set_origin(self) -> None:
        """Mark 'now' as t=0 for the X axis and start a fresh session (call on connect)."""
        self.t0 = time.monotonic()
        self._anchor_counter = None
        self.latest_t = 0.0
        # Drop prior-session data: its timestamps used a different origin and would
        # break the monotonic-time assumption the windowed query relies on.
        for rb in self.channels.values():
            rb.clear()

    def _now(self) -> float:
        return 0.0 if self.t0 is None else (time.monotonic() - self.t0)

    def _counter_to_t(self, counter: int) -> float:
        # Anchor the device's status_counter to wall-clock at the first sample, then
        # advance by counter delta. Keeps streamed channels well-spaced (per-tick) and
        # on the SAME time origin as polled channels, so they line up on one X axis.
        if self._anchor_counter is None:
            self._anchor_counter = counter
            self._anchor_t = self._now()
        return self._anchor_t + (counter - self._anchor_counter) / self.rate_hz

    def _ring(self, name: str) -> RingBuffer:
        rb = self.channels.get(name)
        if rb is None:
            rb = RingBuffer(self.capacity)
            self.channels[name] = rb
        return rb

    def add_samples(self, samples: list[TelemetrySample]) -> None:
        """High-rate streamed telemetry (status_counter time base)."""
        for s in samples:
            t = self._counter_to_t(s.counter)
            self.latest_t = t
            for name, value in s.values.items():
                self._ring(name).append(t, float(value))

    def add_poll(self, name: str, value: float) -> None:
        """A single acyclic (polled) value, time-stamped at arrival (wall clock)."""
        t = self._now()
        if t > self.latest_t:
            self.latest_t = t
        self._ring(name).append(t, float(value))

    def names(self) -> list[str]:
        return sorted(self.channels.keys())

    def get(self, name: str) -> tuple[np.ndarray, np.ndarray] | None:
        rb = self.channels.get(name)
        return rb.data() if rb else None

    def window(self, name: str, t_lo: float, t_hi: float) -> tuple[np.ndarray, np.ndarray] | None:
        rb = self.channels.get(name)
        return rb.window(t_lo, t_hi) if rb else None

    def clear(self) -> None:
        for rb in self.channels.values():
            rb.clear()
        self._anchor_counter = None
        self.latest_t = self._now()