# Inter-MCU Interface Contract (v1)

The frozen boundary between the **network MCU** (SPI master — Lightweight_CMC / STM32G431)
and the **motor-control MCU** (SPI slave — Generic_motor_controller / STM32G474). Both
firmwares build against the headers in this folder; a host/PC tool reaches the same object
dictionary indirectly (PC → Ethernet → network MCU → SPI → motor MCU OD).

Source of truth: `mc_if_protocol.h` (wire format) + `mc_if_od.h` (object map). This document
explains the contract and the design rationale. **Any layout change bumps
`MC_IF_PROTOCOL_VERSION`.**

## 1. Roles & transport

| | Network MCU | Motor-control MCU |
|---|---|---|
| SPI role | **Master** | **Slave** |
| Drives | clock, NSS, cyclic cadence | responds within a transaction |

- **SPI mode 0** (CPOL=0, CPHA=0), **8-bit**, **MSB-first** (matches the motor MCU's SPI2
  slave config). Clock ≤ ~6 MHz initially (raise after bring-up).
- **NSS per transaction**: the master asserts NSS, clocks exactly one frame, deasserts. The
  slave uses the NSS edge to (re)synchronise its DMA to a frame boundary.

## 2. Framing (redesigned for this link)

A **fixed 64-byte frame** is exchanged every SPI transaction, in **both directions**
(full-duplex). Rationale:

- A constant transaction length makes the **slave DMA trivial and self-synchronising** — it
  always arms RX/TX for exactly 64 bytes; no length negotiation, no mid-frame stalls.
- Full-duplex means **command and status move in the same transaction** (master sends
  CYCLIC_CMD while the slave returns CYCLIC_STATUS), which is the natural real-time pattern.
- Unused payload bytes are zero-padded; integrity is from the CRCs and `payload_length`.

```
 byte 0          2    3    4              6        8         10                 10+len      +2
 +---------------+----+----+--------------+--------+---------+------ payload ----+----------+
 | sync (0xA55A) | ver| typ| payload_len  | seq    | hdr_crc | (payload_len B)  | pl_crc   | ...pad to 64
 +---------------+----+----+--------------+--------+---------+------------------+----------+
   MC_IfFrameHeader_t (10 B)                                  body              footer (2 B)
```

- **`header_crc`** = CRC16/Modbus over header bytes [0..7] (everything before `header_crc`).
- **`payload_crc`** = CRC16/Modbus over the `payload_length` payload bytes.
- **CRC16/Modbus**: poly `0xA001` (reflected 0x8005), init `0xFFFF`, no final XOR — the
  algorithm already implemented in the motor MCU (`MC_Spi_Crc16`).
- **Endianness**: little-endian throughout (both MCUs and a PC are LE).
- Max payload per frame = `MC_IF_MAX_PAYLOAD` = 52 bytes (64 − 10 − 2). The largest payload
  (CYCLIC_CMD, 31 B) fits comfortably.

## 3. Transaction model (request → response)

Every transaction carries an independent framed message **each way**, each with its own
`message_type` and CRC. The receiver always dispatches on the incoming frame's `message_type`.

- **Cyclic (default):** master sends `CYCLIC_CMD`, slave returns `CYCLIC_STATUS`. Run at a
  fixed rate (e.g. 1 kHz) as the real-time channel.
- **OD access (pipelined):** to read/write a parameter the master sends `OD_READ_REQ` /
  `OD_WRITE_REQ` in place of a cyclic command on one transaction. The slave processes it
  (off the real-time path) and **stages the response**, which the master receives on a
  **subsequent** transaction's MISO as `OD_READ_RESP` / `OD_WRITE_RESP`. The master correlates
  by `sequence`. While no OD response is staged, the slave's outbound frame is the latest
  `CYCLIC_STATUS` (whose `node_state`/heartbeat `flags` bit0 also signals "response pending").
- **HEARTBEAT** may be sent by either side as an idle frame (e.g. to clock out a pending
  response, or for link supervision when not cycling).
- **ERROR** is returned for malformed frames (bad sync/version/CRC/length/unknown type) or OD
  faults (`error_class = MC_IF_ERR_OD`, `detail` = `MC_IfOdResult_t`).

### Timeout / safe-state
- The slave tracks `command_counter`; if no valid `CYCLIC_CMD` arrives within the command
  timeout, it raises the SPI-timeout fault → quick stop if feasible, else inhibit (per the
  motor MCU fault manager).
- A bad CRC/version frame is dropped (counts toward timeout) and an `ERROR` is staged; it never
  corrupts motion.

## 3aa. Streaming vs SDO split (protocol v3)

The cyclic command carries **only** what genuinely needs to stream every 1 ms:

| Field | Type | Purpose |
|---|---|---|
| `controlword` | u16 | Urgent / live bits (ENABLE, QUICK_STOP, NEW_SETPOINT, FAULT_RESET, HALT) |
| `velocity_setpoint` | i32 | Live velocity demand, scaled by `MC_IF_VEL_SCALE` (same units as 0x60FF). Applied when in a velocity mode and enabled |
| `command_counter` | u32 | LCMC's monotonic heartbeat for the slave's command-timeout dead-man |

Total: 10 bytes per cycle. Everything else — `mode_of_operation`, `target_position`, `target_torque`, `target_position_time_ms`, `profile_velocity` / `_acceleration` / `_deceleration` — is **SDO-only**, stored on the motor MCU between moves.

**Setup-then-trigger** is the standard pattern for **position** moves:

1. LCMC writes the relevant setup OD entries via SDO (`OD_WRITE_REQ` over the SPI OD pipeline).
2. LCMC sets `controlword |= MC_IF_CW_NEW_SETPOINT` in the next cyclic command — rising edge triggers the motor MCU to latch the stored setup and execute.
3. LCMC clears NEW_SETPOINT on subsequent cycles (so the next rising edge can re-arm).

**Velocity** modes don't need a trigger. Once `mode_of_operation == MC_IF_MODE_PROFILE_VELOCITY` is set (via SDO) and the drive is enabled, the motor MCU follows the cyclic `velocity_setpoint` live every cycle. Steady value → constant speed; varying value → jog. The CMC's `axis_op_mode = JOYSTICK` is implemented entirely on the LCMC side — `axis_manager` computes `velocity_setpoint = joystick_value × joystick_max_velocity` locally; the motor MCU has no application-specific joystick concept.

`0x60FF target_velocity` (SDO) is **informational only** in v3 — the cyclic `velocity_setpoint` is authoritative for velocity demand. Reads of 0x60FF return whatever the motor MCU last received (per its choice — cyclic or SDO), but the motor follows the cyclic stream.

`quick_stop`, `fault_reset`, `halt` are always honoured live — they're carried in every cyclic, regardless of mode.

## 3a. Continuous motion commands (jog / joystick)

Continuous, time-critical commands — **jog**, joystick velocity, streamed setpoints — ride the
**cyclic command (RX-PDO)**, never OD/SDO writes. SDO is acyclic and would add latency and
displace telemetry; the cyclic command carries a fresh setpoint **every transaction**, which is
the right channel for real-time motion.

- **Mechanism:** the master streams `CYCLIC_CMD` each cycle with `mode_of_operation` =
  joystick-velocity and the jog rate in `target_velocity` (the motor MCU conditions it —
  deadband, acceleration limit, soft-limit slowdown — then drives the velocity loop). Position or
  torque jog use the corresponding target field + mode.
- **Rate:** the master runs the cyclic exchange **continuously** at a fixed rate ≥ the fastest
  command requirement. **Recommended 1 kHz** (1 ms latency; matches the motor MCU medium loop);
  100 Hz is the minimum to meet a 10 ms jog update.
- **Watchdog (dead-man):** `command_counter` must advance each cycle. If it does not advance for
  longer than the **command-timeout (recommended ~30 ms)**, the motor MCU treats the command
  stream as lost and **quick-stops** (per the fault manager). This makes jog inherently safe — a
  hung master or broken link stops motion.
- OD/SDO remains for occasional parameter access (gains, config, calibration), interleaved at a
  much lower rate.

## 4. Object dictionary

Full map in `mc_if_od.h` (`MC_IF_OD_OBJECTS` X-macro — both sides generate their tables from
it). Conventions:

- **CiA-402 standard objects** (`0x1xxx`, `0x6xxx`): scaled integers. Scale factors:
  position `MC_IF_POS_SCALE` (1e-5 rad), velocity `MC_IF_VEL_SCALE` (1e-3 rad/s),
  acceleration `MC_IF_ACC_SCALE`, torque/current `MC_IF_CUR_SCALE` (1e-3 A). `SI = raw*scale`.
- **Manufacturer objects** (`0x2xxx`): **FLOAT32 in SI units** — exact, no scaling. This is
  what gain tuning and live graphing use (write a gain as a real float; stream telemetry as
  real floats). Ranges per spec 05: 0x2000 axis/motor, 0x2200 position, 0x2300 velocity,
  0x2400 current/FOC, 0x2500 encoder/estimator, 0x2600 faults/limits, 0x2700 calibration,
  0x2800 persistence, 0x2900 commissioning/test-injection.
- **CMC-owned axis_manager objects** (`0x3xxx`, added in protocol v2): FLOAT32 SI for analog
  values, U8 for modes / state / triggers. These back the network MCU's `axis_manager` and
  are the universal command surface for protocol modules (CAMERAD, VISCA, PC tool). They are
  handled locally on the network MCU and **never generate SPI traffic** — the motor MCU
  does not own them and must skip them when building its OD table. Range layout:
  `0x3000-0x300F` state (RO); `0x3010-0x301F` command triggers; `0x3020-0x302F` mode + targets
  (joystick raw + cal at 0x3026-0x302A); `0x3030-0x303F` per-axis limits; `0x3040-0x304F`
  dynamics (payload / future gain schedules); `0x3050-0x305F` persistence triggers (write
  `MC_IF_SAVE_MAGIC` to commit the named region to the CMC's internal flash via
  `app/persist`). `0x3100-0x31FF`, `0x3200-0x32FF` etc. are reserved for future axes 1, 2, …
- Values in OD payloads are little-endian in `data[]`, length 1/2/4 by type.

### Owner column (protocol v2)

Every X(...) line in `MC_IF_OD_OBJECTS` now carries a trailing **owner** column —
`MC_IfOdOwner_t` — which says which firmware is responsible for reads and writes:

| Value | Meaning |
|---|---|
| `MC_IF_OWNER_MOTOR` | Motor MCU's OD table handles it. Reads/writes initiated by the network MCU travel over SPI via the cia402 OD pipeline. |
| `MC_IF_OWNER_CMC`   | Network MCU's `app/od/cmc_od` handles it. No SPI traffic. |

Both firmwares filter `MC_IF_OD_OBJECTS(X)` by owner when generating their local tables:
the motor MCU builds entries where `owner == MC_IF_OWNER_MOTOR`; the network MCU builds
entries where `owner == MC_IF_OWNER_CMC`. Host tools (PC GUI) iterate everything and
display the owner alongside each entry.

### Tuning workflow this enables
- **Read/write gains:** OD access to e.g. `0x2300:1` (vel_kp) as float32 — live, no reflash.
- **Graph variables:** configure the telemetry map (below) with the signals you want streamed.
- **Step changes:** `0x2900` — set `inject_enable`, `inject_target` (iq/vel/pos),
  `inject_step_amplitude`, then pulse `inject_step_trigger`.

## 4a. Telemetry mapping (configurable, runtime)

The cyclic telemetry frame has **no fixed payload layout**. The host chooses which OD entries
are streamed, by writing a mapping list into OD object **`0x2A00`** — at runtime, over the live
link, with no reflash/reboot. This is the CANopen TX-PDO-mapping idea adapted to our OD.

**The map object `0x2A00`:**
- `0x2A00:0` — entry count (U8, `0..MC_IF_TLM_MAX_ENTRIES`).
- `0x2A00:1..16` — each a U32 = `(index<<16) | (subindex<<8) | bitlen`
  (`MC_IF_TLM_MAP_ENTRY(idx,sub,bits)`), naming one OD entry to stream.

**Configure (acyclic OD writes):**
```
0x2A00:0 = 0                                 # deactivate while editing
0x2A00:1 = MC_IF_TLM_MAP_ENTRY(0x2410,2,32)  # iq_meas
0x2A00:2 = MC_IF_TLM_MAP_ENTRY(0x2310,3,32)  # iq_cmd
0x2A00:3 = MC_IF_TLM_MAP_ENTRY(0x2310,1,32)  # vel_demand
0x2A00:4 = MC_IF_TLM_MAP_ENTRY(0x2310,2,32)  # vel_actual
0x2A00:0 = 4                                 # activate
```

**Slave behaviour:** on the count write, validate every entry (exists, is `MC_IF_F_PDO`,
total bytes ≤ `MC_IF_TLM_BLOB_MAX` = 40, count ≤ max). On success, build a gather list
(pointer,size), bump `map_version`, and swap it in **atomically** (double-buffer) so a cyclic
frame is never half-old/half-new. Reject a bad map with an OD error and keep the previous map.

**Each cycle:** copy the mapped values into the telemetry blob after
`MC_IfCyclicStatusHeader_t`, in map order, LE; set `map_byte_count` and `map_version`.

**Host:** knows the layout it configured; reads `map_version` in each frame to confirm the new
map is live (and to re-column across the one-cycle transition); unpacks and plots. To graph
different signals, rewrite the map — live.

**Budget (64-byte frame):** 12-byte status header + up to **40 bytes** of mapped blob = ~10
float32 channels, plus the always-present `statusword`/`error_code`/`status_counter`. The
standard actuals (`0x6064`/`0x606C`/`0x6077`) are mappable entries; a recommended default map
lists them first, then the tuning telemetry.

## 5. Side-effects (writes)
- `controlword` (0x6040) → mode-manager command latch (not controllers directly).
- Gains (0x22xx/0x23xx/0x24xx/0x25xx) → shadow config, applied at safe points.
- `cal_command` (0x2700:1) → starts calibration only if state/safety allow.
- `store_save_command` (0x2800:1 = `MC_IF_SAVE_MAGIC`) → slow-loop persistent save.

## 5a. Network side (Ethernet / UDP)

The network MCU bridges this OD to the PC over **UDP only** (no TCP). The PC speaks the same OD
(indices, types, scaling); the network MCU translates OD-over-UDP to OD-over-SPI and relays the
telemetry stream. The PC↔network-MCU datagram protocol is specified in **`NETWORK_UDP_SPEC.md`**.
Summary: a request/response OD channel (PC retransmits on timeout for reliability over UDP) and a
pushed telemetry stream (fire-and-forget, batched, carrying `map_version` + sample counters so
the PC detects drops and remaps). The telemetry map (0x2A00) is configured by the PC via OD
writes bridged to the motor MCU — so "which signals to graph" is chosen end-to-end at runtime.

For an OD request, the network MCU inspects the entry's owner: motor-owned entries are
translated to SPI OD pipelining (as before); CMC-owned entries (`0x3xxx`, axis_manager) are
handled locally and respond immediately, never reaching the motor MCU.

## 5b. CMC-side: axis_manager and the protocol-module pattern

The CMC's `app/axis_manager` owns motor command state at a high level — op-mode, targets,
limits, axis state. **Its public API is the CMC-owned section of the OD** (`0x3xxx`). Every
protocol module that drives the motor — CAMERAD, VISCA, the PC tool's OD writer, any future
addition — does it the same way: write `axis_op_mode` to choose what's being commanded, write
the per-mode targets (`axis_joystick_value`, `axis_target_velocity`, `axis_target_position` +
`axis_target_time`), trigger `axis_enable`, `axis_quick_stop`, `axis_clear_fault` as needed,
and read back state (`axis_state`, `axis_position_actual`, `axis_velocity_actual`,
`axis_error_code`).

Protocol modules MUST NOT bypass `axis_manager` by writing CiA-402 entries
(`controlword 0x6040`, `target_velocity 0x60FF` etc.) directly. Those entries remain
visible/writable for debug and tuning, but routing motor commands through them would
desynchronise `axis_state` from reality and break arbitration between protocols.

The network MCU's `axis_manager_tick` translates the OD state into the cyclic command sent
to the motor MCU each ms — controlword, mode_of_operation, scaled targets. Single owner of
the cyclic command frame.

## 6. Versioning
`MC_IF_PROTOCOL_VERSION` covers the framing and any **fixed** on-wire byte layout. A frame whose
`version` mismatches is rejected with `ERROR(MC_IF_ERR_BAD_VERSION)`.

**Appending a new OD object to the X-macro does NOT bump the version.** Acyclic `OD_READ`/`OD_WRITE`
requests carry the index, subindex and type explicitly, and the cyclic telemetry frame has no fixed
payload layout (§4 — the host configures `0x2A00`); so a new object adds no fixed wire layout. Older
consumers simply never reference it, newer ones can. Bump `MC_IF_PROTOCOL_VERSION` only for changes to
the wire itself: the frame/header format, the message-type set, or the size / meaning / scale of an
existing field. Every change is still recorded in `CHANGELOG.md`, whose `3.x` minor versions track these
additive, version-stable additions (e.g. `0x2200:4`, `0x2910:8`–`10`, added during bring-up).

## Operational defaults (committed)

These were "open points" in the original draft; they are now committed as v1 defaults. None of them change the byte layout on the wire, so they do not require a `MC_IF_PROTOCOL_VERSION` bump when re-tuned during bring-up.

- **Scale factors** for the CiA-402 objects: defined in `mc_if_od.h` as
  `MC_IF_POS_SCALE = 1e-5 rad/LSB`, `MC_IF_VEL_SCALE = 1e-3 rad/s/LSB`,
  `MC_IF_ACC_SCALE = 1e-3 rad/s²/LSB`, `MC_IF_CUR_SCALE = 1e-3 A/LSB`. `SI = raw × scale`.
- **Cyclic exchange rate**: `MC_IF_CYCLIC_RATE_HZ = 1000` (1 kHz, period 1 ms), continuous.
- **Command timeout / dead-man**: `MC_IF_COMMAND_TIMEOUT_MS = 30` (~30 cycles). Slave enters
  safe state (see §3a) if no valid `CYCLIC_CMD` arrives within this window.
- **SPI clock rate**: `MC_IF_SPI_CLOCK_HZ_INITIAL = 6 MHz` for bring-up;
  `MC_IF_SPI_CLOCK_HZ_MAX = 10 MHz` not to exceed without re-characterisation.

Both firmware projects include `mc_if_protocol.h` and read these constants directly — no value is duplicated per-codebase. Re-tuning means editing this header and rebuilding both sides; no version bump, because the wire layout is unchanged.

## 7. Bootloader control surface (protocol v5)

The bootloader is a separately-linked binary on each MCU that shares the same wire format, framing, and OD dispatch as the app but implements a **reduced OD subset** (`0x1F5x`) for firmware updates. Full architecture in `Documentation/dual_bootloader_design.md`; this section documents only the wire additions.

### 7a. Node state signals which binary is running

The slave's `node_state` field (`CYCLIC_STATUS.node_state` + `HEARTBEAT.node_state`) reports which binary is active:

| Value | Symbol | Meaning |
|---:|---|---|
| 0x00–0x06 | `MC_IF_NODE_INIT`…`_CALIBRATING` | App is running; existing OD table applies. |
| **0x07** | **`MC_IF_NODE_BOOTLOADER`** | Bootloader is running. Only `0x1F5x` entries are accessible; all other OD reads return `ERR_NOT_BOOTLOADER`. Master **must** pause normal cyclic commands. |

Transitions happen only across a reset — the master triggers "enter bootloader" via `OD_WRITE_REQ(0x1F51:1 = MC_IF_PROG_START)`, the slave resets into its bootloader, and the next `CYCLIC_STATUS` (or the response to any OD read) reports the new state.

### 7b. OD entries — owner `MC_IF_OWNER_BOOTLOADER`

Filtered out of the app-side OD table by the X-macro's owner column, so an app-mode target replies `ERR_NO_OBJECT` on read/write. Reads/writes are dispatched only when `node_state == MC_IF_NODE_BOOTLOADER`.

| Index | Sub | Name | Type | Access | Purpose |
|---:|---:|---|---|---|---|
| `0x1F50` | 1 | `program_data` | U8 | WO | **Logical** firmware-bytes sink. NOT written via expedited `OD_WRITE_REQ`; bytes flow via the segmented-download message types below. The entry exists so the dispatch table is enumerable and its owner is explicit. |
| `0x1F51` | 1 | `program_control` | U8 | RW | State command: `0x00 STOP`, `0x01 START`, `0x02 VERIFY`, `0x03 COMMIT`, `0x80 ABORT` — see `MC_IF_PROG_*`. |
| `0x1F56` | 1 | `program_software_id` | U32 | RO | CRC32 of the currently running image. Used to short-circuit "already up-to-date" and to confirm a post-COMMIT reboot moved to the intended image. |
| `0x1F57` | 1 | `flash_status` | U16 | RO | Bootloader state: `MC_IF_FLASH_IDLE / ERASING / PROGRAMMING / VERIFYING / FAULT`. |

### 7c. Segmented download — three new message types

Bulk firmware bytes are carried over three new master→slave / slave→master message types instead of `OD_WRITE_REQ` (whose 4 B expedited limit is unusable at KB scale). Reference: CiA-301 §7.2.4.3.

| Value | Symbol | Direction | Payload |
|---:|---|---|---|
| `0x14` | `MC_IF_MSG_OD_DOWNLOAD_INIT` | master → slave | `MC_IfOdDownloadInit_t` — target index + subindex + total length |
| `0x15` | `MC_IF_MSG_OD_DOWNLOAD_SEGMENT` | master → slave | `MC_IfOdDownloadSegment_t` — flags (bit 0 = toggle, bit 1 = last-segment), `seg_length`, `data[seg_length]` |
| `0x16` | `MC_IF_MSG_OD_DOWNLOAD_RESP` | slave → master | `MC_IfOdDownloadResp_t` — echoed `toggle_ack`, `result`, `bytes_accepted` running total |

Segment payload capacity is `MC_IF_MAX_PAYLOAD − 3` (roughly 49 B on the current 64 B frame). Sender alternates the toggle bit 0/1 across successive segments; receiver echoes the received toggle in its `RESP` and holds the "bytes accepted" watermark. Toggle mismatch → sender resends the same segment with the correct toggle. Last-segment bit signals end of stream — after receipt, the master issues `OD_WRITE_REQ(0x1F51:1 = MC_IF_PROG_VERIFY)` to trigger the CRC check.

Apps that do not implement the bootloader OD range MUST reply `ERROR(MC_IF_ERR_UNKNOWN_MSG)` for these three types.

### 7d. Bootloader-specific `MC_IfOdResult_t` codes

New values on top of the existing generic set:

| Value | Symbol | Meaning |
|---:|---|---|
| `0x09` | `MC_IF_OD_ERR_FLASH_LOCKED` | Sector write-protected (e.g. bootloader attempted to erase its own region). |
| `0x0A` | `MC_IF_OD_ERR_CRC` | Verify failed — CRC32 of received image mismatches the expected. |
| `0x0B` | `MC_IF_OD_ERR_BOOTLOADER_BUSY` | A download session is already in progress; abort first or wait. |
| `0x0C` | `MC_IF_OD_ERR_NOT_BOOTLOADER` | Write to `0x1F5x` arrived while the target is in app mode (not bootloader). |

### 7e. Update sequence (host-side)

Both MCUs are addressed identically — the PC tool's "Update firmware" flow is one implementation that handles either target based only on IP/owner selection.

```
1. Read      0x1F56  (CRC of running image)
2. If matches new image's CRC -> abort (already up-to-date)
3. Write     0x1F51 = MC_IF_PROG_START  -> slave resets into bootloader
4. Segmented-write bytes into 0x1F50 via DOWNLOAD_INIT + N × DOWNLOAD_SEGMENT
5. Write     0x1F51 = MC_IF_PROG_VERIFY -> slave CRCs, transitions IDLE or FAULT
6. Write     0x1F51 = MC_IF_PROG_COMMIT -> slave marks image valid + reboots into it
7. Read      0x1F56  (confirm new CRC now running)
```

## Open point (still confirm)

- Whether to add configurable PDO mapping (host chooses which entries stream) vs the fixed
  `CYCLIC_STATUS` set — v1 ships the fixed set + OD polling for arbitrary variables.
  The telemetry mapping via OD `0x2A00` (§4a) addresses this; this open point closes once
  §4a is verified working end-to-end during Phase 5.
