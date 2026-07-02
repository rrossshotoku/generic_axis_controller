# CMC Object Dictionary GUI

A PC tool to browse and tune the motor-control MCU's **object dictionary (OD)**
through the network MCU (CMC) over **Ethernet/UDP**. It speaks the contract in
`../NETWORK_UDP_SPEC.md` and parses the OD layout/scaling at runtime from
`../mc_if_od.h`, so it never drifts from the firmware.

```
This GUI ──UDP──► CMC (Lightweight_CMC) ──SPI──► Motor MCU (OD)
```

## What it does

- **Live OD browser** — every object from `MC_IF_OD_OBJECTS`, with type, access,
  flags, units and scaling. Tick **Watch** to poll an entry at 5 Hz; PDO entries
  that are in the active telemetry map update straight from the stream.
- **Acyclic read/write** — read or write any non-cyclic entry. Scaled CiA-402
  ints and float32 manufacturer objects are entered/shown in **SI units**;
  bitfields (controlword/statusword/error) accept and show hex. Writes are
  confirmed and read back.
- **Motor Config tab** — set the motor controller's persistent configuration
  (motor model, position/velocity/FOC gains, estimator, fault & motion-profile
  limits, alignment params) from grouped SI-unit fields with per-field *Apply*,
  and fire the OD-triggered commands — **electrical alignment**, **set mechanical
  zero**, **current offset**, **save to flash**, **factory reset**. A
  **completeness** line (from `cal_done_flags`, 0x2700:5) shows which calibrations
  are still *outstanding*, plus live `cal_status` / `store_status` read-back.
  (CMC-owned config has its own **CMC Setup** tab.)
- **Configurable telemetry map (0x2A00)** — choose up to 16 PDO-mappable signals
  (≤ 40 bytes). *Apply* writes the map atomically (deactivate → list → activate),
  exactly as the spec requires.
- **Real-time pop-out graphs** — one or more streaming windows. Pick channels,
  **left-drag to pan, scroll-wheel to zoom, right-drag to box-zoom**, **Pause** to
  freeze and inspect, **Auto-scroll** to follow the newest sample (auto-disengages
  the moment you pan/zoom). **Add marker** drops draggable vertical time cursors —
  drag two onto points of interest to read the time between them (Δt and frequency);
  double-click a marker to remove it, **Clear markers** removes all. Backed by a
  rolling buffer so plotting stays smooth even at 1 kHz.

## Install & run

```powershell
cd Interface\gui
python -m pip install -r requirements.txt
python run.py            # or:  python -m mc_gui
```

If `mc_if_od.h` isn't auto-found, pass it explicitly:
`python run.py --od-header ..\mc_if_od.h`

## Typical workflow

1. Set the **CMC IP** (OD port 5000, telemetry port 5001 by default) and
   **Connect**.
2. Browse the OD; **Watch** or **Read/Write** the entries you care about
   (e.g. write `vel_kp` at `0x2300:1`).
3. On the **Telemetry / Graphing** tab, *Load default* (or build your own map),
   **Apply map**, then **Subscribe**.
4. **New graph window** → tick channels → tune live and watch the response.
   To graph different signals, change the map and *Apply* again — live.

## Layout

| File | Role |
|---|---|
| `mc_gui/od.py` | Parses `mc_if_od.h` (X-macro + scales) into the OD model; value encode/decode |
| `mc_gui/protocol.py` | UDP wire codec (`NETWORK_UDP_SPEC.md`) |
| `mc_gui/client.py` | Threaded UDP client: OD req/resp + retransmit, telemetry RX/unpack |
| `mc_gui/buffer.py` | Rolling per-channel telemetry ring buffer |
| `mc_gui/graph_window.py` | Pop-out pyqtgraph streaming plot (pause / pan / zoom) |
| `mc_gui/main_window.py` | OD browser, read/write, telemetry-map editor, connection bar |
| `mc_gui/app.py` | Entry point (`python -m mc_gui`) |
| `smoke_test.py` | Headless self-test (parser, protocol, telemetry unpack, GUI build) |

## Notes & assumptions

- **Telemetry time base** comes from each sample's `status_counter` (cyclic
  ticks) divided by the **Cyclic rate (Hz)** field (default 1000), so the X axis
  is real seconds and dropped datagrams show as gaps.
- A telemetry datagram is unpacked only when its `map_byte_count` matches the
  active map's size — a stale layout is shown header-only rather than as garbage.
- OD access uses 50 ms timeout / 3 retransmits, matching the spec's reliability
  guidance; telemetry is fire-and-forget on a separate socket/thread.
- **Firewall punch.** Stateful host firewalls (Windows Firewall in particular)
  only allow inbound UDP that matches a flow the local socket has previously
  sent outbound from. Our telemetry receive socket has nothing to send under
  normal use, so its inbound datagrams would be silently dropped. The client
  works around this by sending a 0-byte UDP from the receive socket to the
  CMC's telemetry port (`od_port + 1`) at *connect* and again just before
  *subscribe*. This registers the PC-tlm ↔ CMC-tlm flow in the firewall's
  state table so subsequent telemetry datagrams are accepted. The CMC ignores
  the empty datagrams (it drains the telemetry socket once per tick). If the
  punch send itself fails (e.g. very restrictive egress policy), the log line
  `Telemetry firewall-punch send failed` is emitted — fall back to adding an
  explicit inbound UDP allow rule for your chosen telemetry port.
