# Cross-Project Requests Log

This file is the **shared queue of requests that span project boundaries**. The three codebases that share the Interface contract — `Lightweight_CMC` (network MCU), `Generic_motor_controller` (motor MCU), and the PC tool under `Interface/gui/` — record requests for changes here so the affected project's developer / agent sees the queue on next inspection.

This file is **process** documentation, not contract documentation. It does **not** change the wire format and does not require an `MC_IF_PROTOCOL_VERSION` bump. Edits here are routine — they exist so cross-project asks are visible and tracked, rather than living in chat history.

## How to use

- **Opening a request**: add a new `## REQ-NNNN: …` block at the **bottom** of the file (so chronological order is preserved). Use the next free 4-digit number. Fill in every field.
- **Working on a request**: change `Status:` to `in-progress` and note progress under *Discussion*. Don't delete the request; it remains a record.
- **Completing a request**: change `Status:` to `done`, fill in `Closed:`, and (if relevant) link the commit(s) / PR / ADR that resolved it. Leave the entry; future readers want the history.
- **Cancelling a request**: change `Status:` to `cancelled` with a one-line reason. Same as done — leave the entry.

When a request requires a wire-format change, that is **separate** — bump `MC_IF_PROTOCOL_VERSION` per `INTERFACE_SPEC.md §6` and add an entry to `CHANGELOG.md` as well as this file.

## Status legend

| Status | Meaning |
|---|---|
| `open` | Filed; no work started |
| `in-progress` | Target project has picked it up |
| `blocked` | Waiting on a dependency — name it under *Discussion* |
| `done` | Implemented; closed |
| `cancelled` | Decided not to proceed; reason under *Discussion* |

## Entry template

```
## REQ-NNNN: <short title>
- **Source**: <project asking>
- **Target**: <project receiving>
- **Status**: open | in-progress | blocked | done | cancelled
- **Opened**: YYYY-MM-DD
- **Closed**: -
- **Priority**: blocking | functional | stylistic

### Why
<one paragraph: what is broken / missing / sub-optimal today, and what
becomes possible once this is done>

### What's needed
<concrete list of changes the target project needs to make. Reference
files / line numbers / specific Interface items where possible.>

### Acceptance
<how the target / source project can verify the change is done correctly.
End-to-end if possible.>

### Discussion
<progress notes, blockers, decisions made along the way. Append; don't
overwrite. Sign with date.>
```

---

## REQ-0001: Add 18 missing CiA-402 standard OD entries on motor MCU
- **Source**: `Lightweight_CMC` (network MCU)
- **Target**: `Generic_motor_controller` (motor MCU)
- **Status**: done
- **Opened**: 2026-06-21
- **Closed**: 2026-06-21
- **Priority**: blocking

### Why

The motor MCU's OD table (`src/mc_od.c`) currently implements 38 manufacturer entries (range `0x2xxx`) but **none** of the 18 CiA-402 standard objects in the `0x1xxx` / `0x6xxx` ranges. The `MC_IF_OD_OBJECTS(X)` X-macro in `Interface/mc_if_od.h` lists all 44 entries as part of the shared contract.

The wire-level cyclic exchange works correctly — `MC_IfCyclicCommand_t` and `MC_IfCyclicStatusHeader_t` carry control word, status word, targets and actuals as packed fields the motor MCU reads/writes directly. But **any acyclic OD access** (the network MCU's `OD_READ_REQ` / `OD_WRITE_REQ` over SPI on behalf of the PC tool) for one of these indices returns `MC_IF_OD_ERR_NO_OBJECT`.

That means the PC tool's CiA-402 host story (read statusword, write controlword, set mode of operation, configure profile parameters) is non-functional today.

### What's needed

Add OD entries on the motor MCU with read/write callbacks routing to the appropriate internal modules. The 18 standard entries, per `Interface/mc_if_od.h:MC_IF_OD_OBJECTS`:

| Index   | Sub | Name                            | Type    | Access | Source on motor MCU |
|---------|----:|---------------------------------|---------|--------|---------------------|
| `0x1000`| 0   | `device_type`                   | U32     | RO     | constant (per CiA-402 device profile) |
| `0x1001`| 0   | `error_register`                | U8      | RO     | `mc_faults` |
| `0x603F`| 0   | `error_code`                    | U16     | RO     | `mc_faults` (latched fault code, also PDO) |
| `0x6040`| 0   | `controlword`                   | U16     | RW     | `mc_mode_manager` — written triggers mode commands; also already consumed in `CYCLIC_CMD` |
| `0x6041`| 0   | `statusword`                    | U16     | RO     | `mc_mode_manager` / `mc_faults` |
| `0x6060`| 0   | `modes_of_operation`            | I8      | RW     | `mc_mode_manager` |
| `0x6061`| 0   | `modes_of_operation_display`    | I8      | RO     | `mc_mode_manager` |
| `0x607A`| 0   | `target_position`               | I32     | RW     | `mc_position_controller` (scaled by `MC_IF_POS_SCALE`) |
| `0x6064`| 0   | `position_actual`               | I32     | RO     | `mc_state_estimator` |
| `0x6081`| 0   | `profile_velocity`              | U32     | RW     | (trajectory parameter; module TBD) |
| `0x6083`| 0   | `profile_acceleration`          | U32     | RW     | same |
| `0x6084`| 0   | `profile_deceleration`          | U32     | RW     | same |
| `0x6085`| 0   | `quick_stop_deceleration`       | U32     | RW     | same |
| `0x60FF`| 0   | `target_velocity`               | I32     | RW     | `mc_velocity_controller` (scaled by `MC_IF_VEL_SCALE`) |
| `0x606C`| 0   | `velocity_actual`               | I32     | RO     | `mc_state_estimator` |
| `0x6071`| 0   | `target_torque`                 | I32     | RW     | `mc_current_request` (scaled by `MC_IF_CUR_SCALE`) |
| `0x6077`| 0   | `torque_actual`                 | I32     | RO     | `mc_current_sense` / `mc_state_estimator` |

(Confirm types/access against `Interface/mc_if_od.h` at the time of implementation — that file is authoritative.)

Internal pattern (suggested, but the motor MCU project will own the decision per its own conventions): each owning module exposes a small accessor pair (e.g. `mc_mode_manager_get_statusword`), and the OD entry's read/write callback calls them. ADR-worthy decision. The accessors keep the dependency direction clean (OD → controller, never the reverse).

### Acceptance

- A PC tool, via the network MCU's OD-over-UDP bridge (UDP 5000, `OD_READ_REQ`), can read every one of the 18 entries above and gets back the live motor value with `MC_IF_OD_OK`.
- A PC tool can write `0x6040 = MC_IF_CW_ENABLE` and observe the motor transition through the CiA-402 state machine to `MC_IF_NODE_RUNNING`, then `0x6041` reflects `MC_IF_SW_ENABLED`.
- Writing `0x607A` and `0x6060 = 1` (Profile Position) commands a position move that the motor MCU executes (Phase 4 of `Lightweight_CMC` will be the integration test for this).

### Discussion

*2026-06-21 (CMC side audit)*: filed after audit of motor MCU vs `Interface/`. Phase 5 of the CMC network bridge is in place and stubs everything as `MC_IF_OD_ERR_NOT_READY`; once these entries exist on the motor side, swap the stub in `Lightweight_CMC/app/cia402/cia402.c` for a real SPI request (this is Phase 4 work on the CMC side).

*2026-06-21 (motor MCU, ADR-017)*: all 17 standard entries added to `src/mc_od.c` bound to `g_od` backing fields. **OD read/write now works** for every index (acceptance bullet 1 — unblocks integration): RO actuals (`0x6064/0x606C/0x6077`, statusword, error) are mirrored from live state, scaled to wire units (`MC_IF_*_SCALE`), each medium loop; RW objects (controlword/modes/targets/profile params) are stored. **Caveat:** acceptance bullets 2–3 (write `0x6040=ENABLE` → CiA-402 state machine → `RUNNING`/`SW_ENABLED`; profile-position move) require the **mode manager (E1)**, which consumes these stored RW objects — not yet built. Recommend a follow-up request if the state-machine behaviour is needed before E1 lands.

---

## REQ-0002: Extend `MC_OdStatus_t` to match `MC_IfOdResult_t` (9 codes, not 7)
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done
- **Opened**: 2026-06-21
- **Closed**: 2026-06-21
- **Priority**: blocking

> *2026-06-21 (motor MCU, ADR-017)*: `MC_OD_ERR_NO_SUB` + `MC_OD_ERR_NOT_READY` added to `include/mc_od.h`; `MC_Od_Read/Write/ReadRaw` now return `NO_SUB` when the index exists but the subindex doesn't (`od_notfound()`); the wire mapping in `src/mc_comms.c:od_result()` covers all 9 codes 1:1.

### Why

The motor MCU's internal OD result enum (`MC_OdStatus_t` in `include/mc_od.h`) has 7 codes (`0x00`–`0x06`). The Interface's `MC_IfOdResult_t` in `Interface/mc_if_protocol.h` has 9, including the additional `MC_IF_OD_ERR_NO_SUB` (`0x02`) and `MC_IF_OD_ERR_NOT_READY` (`0x08`).

The SPI handler maps motor codes onto wire codes when building `OD_READ_RESP` / `OD_WRITE_RESP`. Without the two extra codes:

- A request for an existing index but a missing subindex returns `NO_OBJECT` instead of `NO_SUB`, hiding the actual failure mode from the network MCU / PC.
- A "module not yet ready" condition (e.g. calibration in progress, persistent store busy) has no clean way to be reported — the motor either reports `CALLBACK` (which is too generic) or appears to succeed.

### What's needed

- Add `MC_OD_ERR_NO_SUB` and `MC_OD_ERR_NOT_READY` (or whatever the motor's naming convention dictates) to `include/mc_od.h:MC_OdStatus_t`. Values can be local; the wire mapping translates them.
- Update the wire-mapping table in `src/mc_comms.c` (around line 96-108) so every motor code has a 1:1 mapping to a `MC_IfOdResult_t` value.
- Update `docs/spec/05_object_dictionary.md` if it enumerates result codes.

### Acceptance

- An OD read of a known index with a non-existent subindex returns `0x02` (NO_SUB) on the wire, not `0x01` (NO_OBJECT).
- An OD access of an entry whose owning module is intentionally not ready (e.g. a calibration entry while not in calibration mode) returns `0x08` (NOT_READY) on the wire.

### Discussion

*2026-06-21*: small dependency of REQ-0001 — easiest if both land in the same change.

---

## REQ-0003: Add 6 missing manufacturer OD entries on motor MCU
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done
- **Opened**: 2026-06-21
- **Closed**: 2026-06-21
- **Priority**: functional

> *2026-06-21 (motor MCU, ADR-017)*: all 6 added to `src/mc_od.c` + `g_od` (`0x2000:3` R, `0x2000:4` L from the motor model; `0x2600:1` fault_flags; `0x2700:2` cal_status; `0x2800:2` store_status mirrored from the persistent store; `0x2800:3` factory-reset, magic `0x7274` → slow-loop factory reset).

### Why

Six entries from the `MC_IF_OD_OBJECTS(X)` X-macro are missing from `src/mc_od.c`:

| Index   | Sub | Name                  | Purpose |
|---------|----:|-----------------------|---------|
| `0x2000`| 3   | `motor_resistance_ohm`| Motor model R |
| `0x2000`| 4   | `motor_inductance_h`  | Motor model L |
| `0x2600`| 1   | `fault_flags`         | Live fault bitmap |
| `0x2700`| 2   | `cal_status`          | Live calibration status |
| `0x2800`| 2   | `store_status`        | Persistent-store result |
| `0x2800`| 3   | `store_factory_reset` | Magic write to factory-reset |

These are all manufacturer (float32 SI or U8/U16 status) entries. Less critical than REQ-0001 but the network MCU and PC tool both reference them.

### What's needed

Add the entries with appropriate callbacks. Most read straightforwardly from existing modules; `0x2800:3` writes the `MC_IF_FACTORY_RESET_MAGIC` (`0x7274`) to trigger the factory-reset path.

### Acceptance

PC tool reads each entry and gets a sensible value. PC tool writes `0x2800:3 = 0x7274` and the motor MCU factory-resets its persistent store on next slow-loop tick.

### Discussion

*2026-06-21*: lower priority than REQ-0001 — the system works without these, the host UI just has gaps. Bundle with REQ-0001 if convenient.

---

## REQ-0004: Move telemetry-map (`0x2A00`) into the OD table
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done — telemetry map is now an OD array at 0x2A00 (per mc_if_od.h MC_IF_TLM_MAP_INDEX); shipped as part of the v2/v3 protocol consolidation
- **Opened**: 2026-06-21
- **Closed**: 2026-06-26 (housekeeping — actual delivery rode v2/v3 protocol bumps; REQ status was just never updated)
- **Priority**: functional

> *2026-06-21 (motor MCU)*: **deferred.** The wire behaviour works today (the map lives in `mc_comms.c`); this is an introspection/cleanliness refactor. It needs OD-engine support for sub-indexed *array* objects (`0x2A00:1..16`) plus index/subindex context in the write callback (the current `MC_OdWriteCallback_t` has neither) — a small engine change. Bundling it with the **unified config registry** cleanup (the comms↔OD decoupling already on the roadmap) so it's done once, properly. Left `open`.

### Why

`Interface/mc_if_od.h:MC_IF_OD_OBJECTS` lists `0x2A00:0..16` as ordinary OD entries (`tlm_map_count` and 16 `tlm_map_entry` slots). The motor MCU implements the runtime telemetry mapping behaviour (atomic-swap on count write, validate `MC_IF_F_PDO` flag etc.) but does it as a **special case** in `src/mc_comms.c` (around lines 14-17 and 110-144), bypassing the OD table.

This works on the SPI wire but means:
- `MC_Od_Find(0x2A00, sub)` returns NULL, so any non-SPI caller can't introspect the map.
- The OD table is no longer a complete description of "what can be read/written" — there's an invisible side door.

### What's needed

Move the telemetry-map state-machine into OD read/write callbacks on `0x2A00:0..16`. The behaviour (atomic swap on count write, validation, `map_version` bump) is unchanged — just routed through the standard OD plumbing.

### Acceptance

A PC tool writes `0x2A00:1 = MC_IF_TLM_MAP_ENTRY(0x6041, 0, 16)` then `0x2A00:0 = 1`, and the next `TELEMETRY` datagram shows `map_version` incremented with statusword as the first record.

### Discussion

*2026-06-21*: low-risk refactor. Existing wire behaviour is preserved; just the implementation routing changes.

---

## REQ-0005: Stage `ERROR` messages on frame-validation failure
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done
- **Opened**: 2026-06-21
- **Closed**: 2026-06-21
- **Priority**: functional

> *2026-06-21 (motor MCU, ADR-017)*: `src/mc_comms.c` now emits `MC_IF_MSG_ERROR` (class + detail + ref_sequence) as the next transaction's frame on bad sync/version/header-CRC/payload-CRC/length and on unknown message type, instead of returning telemetry. (Single-shot, produced as `tx_next` — matches the pipelined model.)

### Why

When the motor MCU detects a bad SPI frame (sync mismatch, version mismatch, header CRC, payload CRC, length, unknown message type), `src/mc_comms.c` increments a local counter and silently returns the latest `CYCLIC_STATUS` instead. The network MCU has no visibility into why a frame was rejected.

`Interface/mc_if_protocol.h:MC_IfErrorClass_t` defines the codes (`BAD_SYNC`, `BAD_VERSION`, `HEADER_CRC`, `PAYLOAD_CRC`, `BAD_LENGTH`, `UNKNOWN_MSG`, `SEQUENCE`, `OD`, `INTERNAL`) and the protocol allows the slave to stage an `ERROR` message that is returned on the next transaction.

### What's needed

- On frame rejection, populate a one-slot `staged_error` struct: `{class, detail, ref_sequence}`.
- On the next transaction, if `staged_error` is occupied, emit `MC_IF_MSG_ERROR` with that payload (and clear the slot) instead of (or alongside) the normal cyclic status.

### Acceptance

CMC's `nc <ip> 30200` log shows `cia402: motor ERROR class=0x04 detail=… ref_seq=…` when a deliberately-corrupted frame is sent (e.g. by writing a wrong byte mid-transfer on a logic-analyser bridge).

### Discussion

*2026-06-21*: safe to defer until a real interop bug needs it, but valuable for diagnostics during Phase 4 CMC bring-up.

---

## REQ-0006: Remove stale duplicate definitions on motor MCU
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done
- **Opened**: 2026-06-21
- **Closed**: 2026-06-21
- **Priority**: stylistic

> *2026-06-21 (motor MCU, ADR-017)*: deleted `include/mc_spi_protocol.h`; removed `MC_SPI_PROTOCOL_VERSION` + `MC_SPI_MAX_PAYLOAD` from `include/mc_config.h` (left a one-line pointer to the shared header). `grep MC_SPI_` now matches only the unrelated `MC_SpiSlave_*` transport module.

### Why

Three stale definitions are dead code but mislead anyone reading them:

| File:line | Issue |
|---|---|
| `include/mc_spi_protocol.h` (whole file) | Redefines `MC_SpiMessageType_t`, `MC_SpiFrameHeader_t`, `MC_SpiCyclicStatus_t` etc. — renamed copies of Interface types. Source code uses the `MC_If*` types from `Interface/mc_if_protocol.h`, never these |
| `include/mc_config.h:16` | `MC_SPI_PROTOCOL_VERSION` — duplicate of `MC_IF_PROTOCOL_VERSION`. Unused |
| `include/mc_config.h:17` | `MC_SPI_MAX_PAYLOAD = 256` — **wrong value** (Interface mandates 52). Unused, but actively misleading |

### What's needed

Delete (or reduce to a deprecation comment pointing at `Interface/mc_if_protocol.h`).

### Acceptance

`grep -r MC_SPI_ src/ include/` returns only references inside the files being removed, or no references at all. Build still succeeds.

### Discussion

*2026-06-21*: lowest priority. Bundle with one of the above as cleanup.

---

## REQ-0007: Harden SPI slave rearm against 0x2A00 telemetry-map writes
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done — resolved in motor MCU as part of the v2/v3 SPI-slave refactor; no further SPI lockups observed in robustness testing
- **Opened**: 2026-06-21
- **Closed**: 2026-06-26 (housekeeping — actual fix shipped with v2/v3 protocol work; REQ status was never updated)
- **Priority**: functional

> *2026-06-22 (motor MCU, ADR-016)*: resolved as a two-side split — motor side ships a **pipelined double-buffer** SPI2 slave (re-arm before the handler; commit `66c9340`). Points 1 & 3 (raise the re-arm ISR above the control loops / sustain a zero re-arm gap) **declined** for control-loop primacy, superseded by the master removing its `cia402_tick` catch-up so frames stay gapped at the cyclic rate. `in-progress` pending on-target 16-PDO acceptance.

### Why

When the PC tool applies a telemetry map containing 6 or more PDOs, the GUI emits 8 back-to-back `OD_WRITE_REQ` frames to `0x2A00:0..6` (deactivate → 6 map words → activate). Each one preempts a cyclic CMD on the SPI bus; the motor MCU must process the map-update callback AND stage `OD_WRITE_RESP` for the next transaction.

Observed effect: the motor MCU reports many SPI **slave rearm failures and resets** during this burst. Symptoms scale with map size (small maps are fine; 6+ PDOs trigger it).

The underlying weakness appears to be that **slow-loop activity is blocking SPI slave DMA rearm**. Any future scenario that bursts OD writes (calibration upload, gain re-tune, configuration restore from PC) will hit the same wall.

### What's needed

1. Audit the SPI slave DMA rearm path. It must run from an ISR with priority above any loop that handles OD-write callbacks (medium/slow). Rearming should never wait on a callback in progress.
2. Specifically for `0x2A00` writes: the callback is allowed to defer the actual map-table rebuild to a slow-loop deadline as long as the **write itself is acknowledged within `MC_IF_COMMAND_TIMEOUT_MS` (30 ms)** with `MC_IF_OD_OK`. Staging fast is what matters; the heavy table work can run after the response is sent.
3. Confirm the SPI slave can sustain back-to-back transactions at 1 kHz with zero rearm gap even when the medium/slow loops are running their full per-tick budget.

### Acceptance

- Apply Map for 16 PDOs (the spec max) from the PC tool: zero rearm-fail log lines on the motor MCU side.
- `MC_IF_OD_OK` returned within 30 ms on every `0x2A00` sub-index write regardless of slow-loop load.
- The CMC's `OD_WRITE_REQ` frames can arrive back-to-back at the cyclic rate without inducing slave-side recovery.

### Discussion

*2026-06-21 (CMC side)*: filed after Phase 5 GUI integration testing turned up the rearm failures.

*2026-06-22 (motor MCU, ADR-016)*: Resolved as a two-side split.

**Motor side:** the single-buffer SPI2 slave was replaced with a **pipelined double-buffer** (`src/mc_spi_slave_stm32g474.c`, commit `66c9340`). On transfer-complete the DMA is re-armed *first* (~1–2 µs: pointer swap + `HAL_SPI_TransmitReceive_DMA`), and the per-transaction handler (frame validate, OD apply, `0x2A00` map rebuild, telemetry gather) runs *after*. Re-arm latency is now independent of handler work — this satisfies **point 2**: the `OD_WRITE_RESP` is staged immediately (pipelined as the next outbound frame) while the heavy map-table work happens after it is composed, well inside the 30 ms ack window.

**Points 1 & 3 declined by design.** Raising the SPI2 re-arm ISR above the medium/slow loops was implemented and then **reverted** (ADR-016, 2026-06-22): a comms ISR preempting the 1 kHz velocity loop inverts control-loop primacy and adds bounded but real jitter (~25 µs, CRC-dominated) to the control cascade. The slave therefore stays at priority 3 (below both control loops) and is *not* designed to absorb a true zero re-arm gap.

**Burst removed at source instead.** The network MCU dropped its `cia402_tick` catch-up (`app/cia402/cia402.c` now `s_last_tick_ms = now_ms` — one frame after a stall, then resume cadence, drift-tolerant). Frames are therefore always gapped at the cyclic rate, which is what **acceptance bullet 3** actually specifies ("back-to-back *at the cyclic rate*"). Note the 8 map-apply writes were already 1 ms-gapped, since `cia402_tick` sends one frame per tick (an OD request preempts one cyclic cycle) — not a true zero-gap burst.

**Agreed division of responsibility:** the **master** guarantees the inter-frame gap (no zero-gap bursts); the **slave** guarantees a prompt pipelined re-arm within that gap. The minimum safe gap ≈ the slave's worst-case medium-loop duration (`g_mc_debug.medium_cycles_max ÷ 170 MHz`) + ~5 µs; measure on-target.

**Pending:** end-to-end acceptance — apply a 16-PDO map from the PC tool and confirm zero rearm-fail lines (`g_spi_slave.rearm_fail` / `resets` stay flat). Flip to `done` once verified on hardware.

---

## REQ-0008: Adopt OD owner column + protocol v2 (skip CMC-owned entries)
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done — MC_IfOdOwner_t (MOTOR/CMC) shipped in v2; motor MCU filters MC_IF_OD_OBJECTS(X) by owner. Contract has since advanced to v4; this REQ has long been delivered.
- **Opened**: 2026-06-22
- **Closed**: 2026-06-26 (housekeeping — shipped with protocol v2; REQ status was never updated)
- **Priority**: blocking

> *2026-06-22 (motor MCU, ADR-019)*: **no functional change needed this side.** The motor OD table is hand-maintained (it never expands `MC_IF_OD_OBJECTS`), so the owner column and the `0x3xxx` block are invisible to it; the version check/stamp already uses `MC_IF_PROTOCOL_VERSION` (rebuild → accept/emit v2, reject v1); an absent `0x3xxx` index already returns `NO_OBJECT`. Host syntax-check clean against v2. `in-progress` pending on-target v2↔v2 verification + coordinated flash.

### Why

The shared OD now hosts **two classes** of entries, distinguished by a new **owner** column on every `X(...)` line in `MC_IF_OD_OBJECTS`:

| owner | who handles reads/writes |
|---|---|
| `MC_IF_OWNER_MOTOR` | Motor MCU's existing OD table — every CiA-402 standard entry plus every `0x2xxx` manufacturer entry |
| `MC_IF_OWNER_CMC`   | Network MCU's `app/od/cmc_od` — the new `0x3xxx` axis_manager entries that back the high-level command surface (op-mode, joystick value, target velocity/position/time, limits, axis state) |

CMC-owned entries **must not** be built into the motor MCU's OD table — they have no backing implementation on the motor side and would either fail to compile (if the X-macro handler dereferences a non-existent backing field) or, worse, create phantom OD entries that respond with garbage.

The two affected files in `Interface/`:

- `mc_if_od.h` — `MC_IF_OD_OBJECTS(X)` X-macro signature grew by one trailing column (`owner`). The new `MC_IfOdOwner_t` enum lives in this header. Every existing entry is tagged `MC_IF_OWNER_MOTOR`. A new block of entries at the bottom (`0x3000-0x303F`) is tagged `MC_IF_OWNER_CMC`.
- `mc_if_protocol.h` — `MC_IF_PROTOCOL_VERSION` bumped **1 → 2**. The wire packet layout is unchanged (`MC_IfFrameHeader_t.version`, `MC_IfFrameFooter_t`, all payload structs are byte-identical). What changed is the OD-extension contract and the version byte that goes in every header.

### What's needed (motor MCU side)

1. **Update every X-handler** that consumes `MC_IF_OD_OBJECTS(X)` to accept the new trailing parameter. The canonical pattern is to make the handler dispatch on the owner so non-motor entries are silently dropped from the build:

   ```c
   /* Old handler — change the signature: */
   #define BUILD_OD_ROW(idx, sub, name, type, access, flags)  ...

   /* New handler — dispatch on owner: */
   #define BUILD_OD_ROW(idx, sub, name, type, access, flags, owner) \
       BUILD_OD_ROW_FOR_##owner(idx, sub, name, type, access, flags)

   /* Motor MCU builds entries owned by it: */
   #define BUILD_OD_ROW_FOR_MC_IF_OWNER_MOTOR(idx, sub, name, type, access, flags) \
       /* original handler body — emit a table row, define a callback, etc. */

   /* Motor MCU SKIPS CMC-owned entries: */
   #define BUILD_OD_ROW_FOR_MC_IF_OWNER_CMC(idx, sub, name, type, access, flags) \
       /* empty — produces no code, no table entry, no callback */

   MC_IF_OD_OBJECTS(BUILD_OD_ROW)
   ```

   Apply this transform wherever the macro is consumed (table builder, callback registration, name lookup, anywhere else). Same shape every time.

2. **Update the wire-version check.** Wherever `MC_IfFrameHeader_t.version` is validated against a hard-coded `1`, change it to accept `2` (or to compare against `MC_IF_PROTOCOL_VERSION` from the shared header, which is the recommended pattern). A v1 frame from a stale peer should now be rejected with `MC_IF_ERR_BAD_VERSION` as before.

3. **No new functional code required.** The motor MCU does not need to implement, store, or even acknowledge the new `0x3xxx` entries — they're handled entirely on the network MCU. If an `OD_READ_REQ` or `OD_WRITE_REQ` ever arrives over SPI for an index in the CMC-owned range, treat it as `MC_IF_OD_ERR_NO_OBJECT` (the network MCU promises not to forward those, but a defensive `NO_OBJECT` response keeps the bus consistent if something goes wrong).

### Acceptance

- Motor MCU builds cleanly against `Interface/mc_if_od.h` v2 (with the new owner column on every `X(...)` line and the new `0x3xxx` block at the end).
- `grep -c MC_IF_OWNER_CMC` in the generated OD table source files = 0 (CMC entries are filtered out at preprocess time).
- Motor MCU's OD table size and behaviour are identical to v1 — same set of motor-owned entries respond the same way to SPI OD_READ/WRITE requests.
- Motor MCU's SPI slave accepts and responds to `MC_IfFrameHeader_t.version == 2` frames without `MC_IF_ERR_BAD_VERSION`.
- A v1 frame (with `version == 1`) is rejected with `MC_IF_ERR_BAD_VERSION` (existing behaviour, just the boundary moved).
- Defensive check: an incoming `OD_READ_REQ` over SPI for index `0x3000` returns `MC_IF_OD_ERR_NO_OBJECT` (or whatever the motor MCU's "index outside my table" path returns today).

### Discussion

*2026-06-22 (CMC side, filed when adopting)*: filed alongside the v2 bump. The network MCU is implementing `app/axis_manager` + `app/od/cmc_od` in parallel — that work doesn't unblock until v2 is in your tree, since both sides need to be on v2 for the SPI bus to be consistent. The version mismatch failure mode is loud (every cyclic frame is rejected with `MC_IF_ERR_BAD_VERSION`), so the rollover is obvious — there's no silent failure window. If you want to coordinate the exact deploy order, ping the CMC side first; we'll wait until you've cut the v2 motor MCU firmware before flashing the v2 CMC.

*2026-06-22 (motor MCU, ADR-019)*: Adopted — **no motor-side code change required**, and the briefing's premise doesn't hold here (worth flagging for the contract record).

**The motor does not generate its OD from the X-macro.** Its table is a hand-maintained static array (`src/mc_od.c`, `MC_Od_Find` over `s_od_table[]`); the only `MC_IF_OD_OBJECTS` reference in the motor tree is a doc comment. Consequently:
- **Point 1 (owner filter):** nothing to do — there is no X-handler to change, and the `0x3xxx` CMC entries are simply absent from the hand table. CMC-owned entries in the motor's OD source = 0 by construction.
- **Point 2 (version):** already satisfied — `mc_comms.c` validates against `MC_IF_PROTOCOL_VERSION` (`:195`) and stamps it on TX (`:43`). Rebuilding against the v2 header makes the motor accept/emit v2 and reject v1 with `MC_IF_ERR_BAD_VERSION`.
- **Point 3 (defensive `0x3xxx`):** already satisfied — an absent index resolves `od_notfound()` → `MC_OD_ERR_NOT_FOUND` (`mc_od.c:190`) → wire `MC_IF_OD_ERR_NO_OBJECT` (`mc_comms.c:103`).

Verified: `gcc -fsyntax-only -std=c11 -I include -I ../Lightweight_CMC/Interface src/mc_comms.c src/mc_od.c` is clean against the v2 headers (acceptance bullet 1). Recorded in motor-side ADR-019.

**Deploy:** the motor accepts *only* `MC_IF_PROTOCOL_VERSION`, so a v2 motor + v1 CMC reject every frame (`BAD_VERSION`) until both are v2 — link down in the gap (loud, not silent). Will cut the v2 motor firmware; coordinate the CMC flash per your note. Remaining: on-target v2↔v2 check (link runs, `0x3000` → `NO_OBJECT`) before flipping to `done`.

**Standing caveat (not blocking):** because the motor table is hand-written and decoupled from `MC_IF_OD_OBJECTS`, the canonical map and the motor table can drift silently — no compile-time owner enforcement on this side. v2 added nothing to motor-owned ranges, so no divergence now; the durable fix is owner-filtered X-macro generation (REQ-0004 territory), still deferred.

*2026-06-22 (motor MCU, ADR-020 — follow-up)*: the standing caveat above is now **closed**. The motor OD table is generated from `MC_IF_OD_OBJECTS(X)` (owner-filtered, bound to `g_od` by field name), so a motor-owned contract entry with no backing field is a compile error and drift can no longer pass silently; type/access/PDO/PERSIST follow the contract automatically. Verified by diffing the pre/post table dumps — identical (62 entries) except a latent bug it surfaced: the `0x2900` `inject_*` entries were locally marked persistent, but the contract tags them `MC_IF_F_NONE`; generation corrected them (behaviourally inert — `persistent` is unused today; the consumed flag `pdo_mappable` is unchanged). `0x2A00` (REQ-0004) is the one entry still hand-bound, in `mc_comms`.

---

## REQ-0009: Adopt v3 cyclic-command redesign (streaming vs SDO split)
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: done — shipped in protocol v3 (cyclic carries controlword + velocity_setpoint + command_counter only; mode + targets + profile-params move to SDO via the setup-sequencer in axis_manager). Contract has since advanced to v4 (status header gained position_actual + movement_status per REQ-0013).
- **Opened**: 2026-06-22
- **Closed**: 2026-06-26 (housekeeping — shipped with protocol v3; REQ status was never updated)
- **Priority**: blocking

> *2026-06-22 (motor MCU)*: **blocked pending REQ-0010.** The streaming-vs-SDO split and the NEW_SETPOINT/PROFILE_POSITION parts are good and stand. But the *joystick* part pushes an application concept into a generic motor controller and round-trips velocity→i16→velocity: the CMC's `axis_manager` already holds `joystick_value` (−1..+1) and `joystick_max_velocity` (rad/s), computes a velocity, normalises it back to i16 to stream, and the motor multiplies by a separately-SDO'd scale to recover the same velocity. Counter-proposal (REQ-0010): the cyclic command streams a **velocity (rad/s)**; joystick→velocity stays in `axis_manager`; drop the motor's JOYSTICK mode + `0x2320:1`. Implementing REQ-0009's joystick scaling is on hold until this settles.

> *2026-06-22 (CMC side)*: **REQ-0010 accepted; this is unblocked.** Cyclic now streams `velocity_setpoint` (i32 scaled) rather than `joystick_value`; the motor MCU's joystick mode + `joystick_scale` entry are out. See REQ-0010 discussion for the full settlement. The acceptance criteria below are amended in line with REQ-0010 — `joystick_value` field references become `velocity_setpoint`, and bullet 3's `0x2320:1` entry is removed.

> *2026-06-22 (motor MCU, ADR-021)*: **implemented** per the re-cut contract. 10-byte cyclic unpack (controlword→0x6040, velocity_setpoint→0x60FF as the authoritative live demand, command_counter→dead-man); mode/targets are now SDO-owned and persist (so host SDO mode/target writes finally stick); `0x607B` auto-generated (table 62→63); version 3 accepted via the shared constant; `MC_IF_CW_NEW_SETPOINT` decoded + rising-edge latched (`ds.new_setpoint_latched`) for the D3 trajectory engine. Build clean + OD dump verified. **Pending:** PROFILE_POSITION move *execution* (acceptance bullet 3) is the motor's deferred D3 work (position loop + trajectory) — the trigger seam is in place. Version + velocity-streaming + OD-entry acceptance are met.

### Why

The v2 `MC_IfCyclicCommand_t` carried 8 OD-mapped fields each cycle, but most of them are conceptually one-shot setup that doesn't need to stream at 1 kHz. v3 splits the protocol into:

- **Cyclic (streaming)**: `controlword`, `joystick_value`, `command_counter`. 8 bytes.
- **SDO (setup)**: `mode_of_operation`, `target_position`, `target_position_time_ms` (new), `target_velocity`, `target_torque`, `profile_velocity`, `profile_acceleration`, `profile_deceleration`, `joystick_scale_rad_s` (new).
- **Trigger**: rising edge of `MC_IF_CW_NEW_SETPOINT` (bit 4 of controlword) — motor MCU latches the most recently SDO-written setup and executes.

This matches how the system is actually used (operator sets up a move, then triggers; joystick streams live), keeps the slave's hot path minimal, and aligns with CiA-402 conventions.

### What's needed (motor MCU side)

1. **Unpack the smaller cyclic command.**
   ```c
   typedef struct __attribute__((packed)) {
       uint16_t controlword;
       int16_t  joystick_value;
       uint32_t command_counter;
   } MC_IfCyclicCommand_t;   /* 8 bytes */
   ```
   Apply `controlword` bits live as today. Use `command_counter` for the dead-man. Apply `joystick_value` only when current `mode_of_operation == MC_IF_MODE_JOYSTICK_VELOCITY`, computing `target_velocity = joystick_value * joystick_scale_rad_s / 32767.0f` (i.e. the i16 is a normalised -1..+1 in 16-bit fixed point).

2. **Implement the NEW_SETPOINT trigger.**
   - Detect rising edge of `controlword & MC_IF_CW_NEW_SETPOINT` (= `0x0010`).
   - On rising edge: latch the currently-stored setup values (mode, target_position, target_position_time_ms, profile_velocity, profile_acceleration, profile_deceleration as applicable) and start executing the move.
   - The motor MCU should continue executing the move on subsequent cycles even after NEW_SETPOINT is cleared. Clearing the bit is the LCMC's way of re-arming for the next move.

3. **Add the new OD entries** (both motor-MCU-owned; both should appear automatically once you regenerate against the v3 header):
   - `0x607B target_position_time_ms` (U32, RW) — duration in milliseconds for a PROFILE_POSITION move. `0` = ASAP within profile limits. Used by the motor MCU's trajectory engine to plan the velocity profile. LCMC stores no math on this value; it's purely your trajectory engine's input.
   - `0x2320:1 joystick_scale_rad_s` (F32, RW PERSIST) — rad/s at full-scale joystick deflection. Persistent; survives reboot. Used in JOYSTICK mode to scale the streamed `joystick_value`.

4. **Update the wire-version check.** Accept `MC_IF_PROTOCOL_VERSION == 3`; reject earlier versions with `MC_IF_ERR_BAD_VERSION`.

5. **Setup-then-trigger semantics.** Between SDO writes (mode/targets/timing) and the NEW_SETPOINT rising edge, the motor MCU should hold its current state (whatever it was doing before — typically idle/disabled, or finishing the previous move). Mid-flight setup writes during an active move are allowed to either:
   - Take effect on the next NEW_SETPOINT (preferred — clean separation).
   - Take effect immediately if the motor MCU's trajectory engine supports live update of the moving target.

   Either is acceptable; document which you implement.

### Acceptance

- Motor MCU builds cleanly against `Interface/mc_if_od.h` v3 and `Interface/mc_if_protocol.h` v3.
- Motor MCU accepts `MC_IfFrameHeader_t.version == 3` and rejects `<= 2` with `MC_IF_ERR_BAD_VERSION`.
- A complete PROFILE_POSITION move from the host via the v3 sequence (SDO setup writes → NEW_SETPOINT rising edge → cyclic status shows in-motion → cyclic status shows TARGET_REACHED) works on the bench.
- JOYSTICK mode: SDO writes set `mode = JOYSTICK_VELOCITY` and `joystick_scale_rad_s`; subsequent cyclic frames with non-zero `joystick_value` drive the motor at the expected velocity. Setting `joystick_value = 0` brings the motor to a stop without needing NEW_SETPOINT.
- `quick_stop`, `fault_reset`, `halt` continue to be honoured live from the cyclic controlword regardless of mode.
- New OD entries (`0x607B`, `0x2320:1`) are readable/writable via SDO with the correct types and persistence flags.

### Discussion

*2026-06-22 (CMC side)*: filed alongside the v3 bump. The LCMC side will:
- shrink its `MC_IfCyclicCommand_t` to match,
- add a setup-sequencer in `axis_manager` that issues the SDO writes (via the existing `cia402_od_write_begin` pipeline) when the protocol modules write new mode/target values to the CMC OD entries (0x3xxx),
- add an `axis_start_move` trigger to the CMC OD (`0x3013` or similar) that protocol modules pulse to fire NEW_SETPOINT,
- continue streaming `joystick_value` directly when `op_mode = JOYSTICK` is active.

Given ADR-020's owner-filtered generation, the motor MCU side adoption should be largely mechanical at the OD table level. The novel piece is the trigger semantics — recommend an ADR documenting how the motor MCU's state machine handles the NEW_SETPOINT rising edge (when to ignore, when to abort current move, etc.).

---

## REQ-0010: Stream a velocity (rad/s) in the cyclic command, not a joystick value
- **Source**: `Generic_motor_controller` (motor MCU)
- **Target**: `Lightweight_CMC` + Interface contract
- **Status**: done — cyclic command now carries `velocity_setpoint` (scaled int32 rad/s via MC_IF_VEL_SCALE) as part of the v3 redesign (see REQ-0009). axis_manager computes velocity from joystick_value × joystick_max_velocity (or target_velocity directly in PROFILE_VELOCITY mode); the motor MCU's application-specific "joystick mode" is gone.
- **Opened**: 2026-06-22
- **Closed**: 2026-06-26 (housekeeping — shipped with protocol v3; REQ status was never updated)
- **Priority**: blocking (blocks the joystick part of REQ-0009)

### Why

REQ-0009 / v3 puts a normalised `joystick_value` (i16, −32767..+32767) on the cyclic wire and has the **motor MCU** recover a velocity via `target_velocity = joystick_value / 32767 * joystick_scale_rad_s` (new OD `0x2320:1`). Two problems:

1. **It leaks an application concept into a generic motor controller.** This firmware is a *generic* motor controller; its command surface should be motion primitives (velocity / torque / position). "Joystick" is a camera/operator-input concept — a different reuse of this firmware has no joystick. `mc_if_od.h` already declares the `0x3020` axis-modes (incl. JOYSTICK) as **CMC `axis_manager`** concepts that "translate into the appropriate motor MCU mode + cyclic targets" — so a JOYSTICK mode + `joystick_scale` on the *motor* is exactly that abstraction leaking one layer too far down.
2. **It's a round trip.** The CMC's `axis_manager` already holds `joystick_value` (−1..+1) and `joystick_max_velocity` (rad/s at full deflection) (`axis_manager.c:74-75`). It already has everything to send a velocity, but instead normalises back to i16, streams it, and the motor multiplies by a re-sent scale to recover the same number: velocity → normalise → i16 → ×scale → velocity.

### What's needed (contract + both sides)

1. **Cyclic command carries a streaming velocity** instead of `joystick_value`:
   ```c
   typedef struct __attribute__((packed)) {
       uint16_t controlword;        /* OD 0x6040 -- streaming bits */
       int32_t  velocity_setpoint;  /* live demand, scaled ×MC_IF_VEL_SCALE (rad/s), same units as 0x60FF */
       uint32_t command_counter;    /* dead-man */
   } MC_IfCyclicCommand_t;          /* 10 bytes */
   ```
   The motor applies `velocity_setpoint * MC_IF_VEL_SCALE` as the velocity demand whenever in a velocity mode and enabled, every cycle. Steady value → constant speed; varying value → jog/joystick.
2. **Drop from the motor side:** `MC_IF_MODE_JOYSTICK_VELOCITY` and OD entry `0x2320:1 joystick_scale_rad_s`.
3. **CMC `axis_manager`:** in JOYSTICK op-mode, compute `velocity = joystick_value * joystick_max_velocity` (it already has both) and place it in `velocity_setpoint`. `joystick_max_velocity` stays CMC-side config; no scale is sent to the motor.
4. **Versioning:** v3 isn't deployed yet (motor hasn't implemented it; CMC's `compose_cyclic_cmd` is "not yet wired"), so the cleanest is to **re-cut [3.0.0]** with this shape rather than add a [4.0.0]. CMC author's call; bump + CHANGELOG either way.

### Acceptance

- Cyclic command is `{controlword, velocity_setpoint, command_counter}`; no joystick fields on the wire or in the motor OD; no `MC_IF_MODE_JOYSTICK_VELOCITY`.
- Motor in velocity mode + enabled follows `velocity_setpoint` live; `velocity_setpoint = 0` stops it without NEW_SETPOINT.
- The CMC JOYSTICK op-mode drives the expected motor velocity via `joystick_value * joystick_max_velocity`, computed CMC-side.

### Discussion

*2026-06-22 (motor MCU)*: raised **before** implementing REQ-0009 so we don't build joystick scaling just to tear it out. This is *only* about the joystick/velocity-streaming part — REQ-0009's streaming-vs-SDO split, the new `0x607B target_position_time_ms`, and the NEW_SETPOINT/PROFILE_POSITION setup-then-trigger all still stand (PROFILE_POSITION *execution* is the motor's deferred D3 work regardless).

Open detail for the CMC author: with a live `velocity_setpoint` in the stream, OD `0x60FF target_velocity` (SDO) is redundant for streaming. Suggest the motor treats the cyclic `velocity_setpoint` as authoritative in velocity mode; `0x60FF` either drops or stays as a set-and-hold default. Let's settle that here before the re-cut.

*2026-06-22 (CMC side)*: **accepted, adopting now.** Your point lands — pushing "joystick" onto a generic motor controller is a layering bug, and the round-trip is wasted compute. Going with your shape exactly:

```c
typedef struct __attribute__((packed)) {
    uint16_t controlword;
    int32_t  velocity_setpoint;  /* scaled ×MC_IF_VEL_SCALE, same units as 0x60FF */
    uint32_t command_counter;
} MC_IfCyclicCommand_t;          /* 10 bytes */
```

**On `0x60FF target_velocity`:** keep it as an OD entry for read access (CiA-402 compatibility, useful for "what's the current velocity demand?" diagnostics), but the motor MCU should treat the **cyclic `velocity_setpoint` as authoritative** in every velocity mode. The LCMC's `axis_manager` will NOT SDO-write `0x60FF` anymore — it just streams the setpoint via cyclic each tick.

Reads of `0x60FF` from a PC tool: motor MCU can either return the last-cyclic value (most informative) or the last SDO-written value (CiA-402-traditional). Either is fine; the entry is informational only — writes via SDO don't drive the motor. Suggest documenting whichever you pick in your README.

**Re-cut [3.0.0] in place** rather than [4.0.0] — your call was right. v3 isn't deployed anywhere yet so the version-history damage is purely paper. The CHANGELOG entry is being amended on this side too.

*2026-06-22 (motor MCU, ADR-021)*: **motor side implemented + verified.** Cyclic unpacks `{controlword, velocity_setpoint, command_counter}`; in `PROFILE_VELOCITY` the live demand is `velocity_setpoint` (written into `0x60FF`, which the velocity loop consumes); `MC_IF_MODE_JOYSTICK_VELOCITY` and the joystick scale are gone. Per your note, a PC read of `0x60FF` returns the **last-cyclic value** (most informative) — documented in ADR-021. `gcc -fsyntax-only` clean against v3; OD table dump = 63 entries, no joystick entry. **Pending:** on-target end-to-end (your `axis_manager` streaming `velocity_setpoint` + a bench run: SDO `0x6060=3` + enable + non-zero setpoint → motor runs; `0` → stops).

**LCMC-side changes about to land:**
- `MC_IfCyclicCommand_t` reshaped to `{controlword, velocity_setpoint, command_counter}`.
- `MC_IF_MODE_JOYSTICK_VELOCITY` define removed from `mc_if_od.h`.
- `0x2320:1 joystick_scale_rad_s` removed from `MC_IF_OD_OBJECTS`.
- `axis_manager`: drop `SEQ_JOYSTICK_SCALE` and `SEQ_TARGET_VELOCITY` from the setup-sequencer (no more SDO writes for either — velocity streams). `compose_cyclic_cmd` computes `velocity_setpoint` from the current op-mode: JOYSTICK → `joystick_value × joystick_max_velocity`; PROFILE_VELOCITY → `target_velocity`; PROFILE_POSITION → 0 (motor uses target_position trajectory in position mode); HOLD/OFF → 0.
- For JOYSTICK / PROFILE_VELOCITY: no NEW_SETPOINT pulse needed. Motor is in `MC_IF_MODE_PROFILE_VELOCITY` and follows the streaming setpoint live. (CMC's `axis_op_mode = JOYSTICK` and `= PROFILE_VELOCITY` both map to motor's `mode_of_operation = MC_IF_MODE_PROFILE_VELOCITY = 3`. The CMC-side distinction is "where the value comes from".)
- For PROFILE_POSITION: SDO setup (mode=1, target_position, target_position_time_ms, profile_*) + NEW_SETPOINT trigger — unchanged from previous v3 plan.

REQ-0009 unblocks once your side is happy with this answer.

---

## REQ-0011: Add `trajectory_time_remaining_ms` (live ETA for in-progress moves)
- **Source**: `Lightweight_CMC`
- **Target**: `Generic_motor_controller`
- **Status**: open
- **Opened**: 2026-06-22
- **Closed**: -
- **Priority**: functional

### Why

Starting the CAMERAD protocol on the LCMC. Camera-control panels (S/T screens) expect every poll response to carry `iTimeToShot` — the live countdown for an in-progress shot recall. Panel UIs render a progress bar from this field. Today the LCMC has no way to know how much time is left in a motor-side trajectory; only `position_actual` + `target_position` are visible.

Could be computed approximately CMC-side as `|target − actual| / current_velocity`, but that's inaccurate near end-of-move (deceleration ramp) and is exactly the kind of LCMC-side motion math we agreed (REQ-0010, axis_manager redesign) belongs on the motor side. The motor MCU's trajectory engine knows the exact remaining time because it's running the planner.

### What's needed

A new motor-MCU OD entry exposing the remaining time on the **currently executing** PROFILE_POSITION trajectory:

| Index    | Sub | Name                              | Type | Access | Notes |
|----------|----:|-----------------------------------|------|--------|-------|
| `0x6082` | 0   | `trajectory_time_remaining_ms`    | U32  | RO     | ms remaining until target reached. `0` when no trajectory is in progress (idle, velocity mode, or already at target). PDO-mappable so it can ride the telemetry blob too. |

(Index choice is a suggestion — `0x6082` is unallocated in the current CiA-402 reserved range. Motor MCU project can pick a different unused index if they prefer.)

Semantics:
- During a PROFILE_POSITION move: counts down from `target_position_time_ms` (or the engine's computed duration if `target_position_time_ms = 0`) to `0`.
- After target reached: stays at `0` until the next NEW_SETPOINT fires.
- In any non-position mode (PROFILE_VELOCITY, JOYSTICK, DISABLED): always `0`.
- Update rate: at the motor MCU's medium-loop rate is fine (1 kHz). LCMC will read at the panel poll rate (~25-100 ms), so accuracy beats ~1 ms doesn't matter.

### Acceptance

- LCMC SDO-reads `0x6082` while a PROFILE_POSITION move is in progress and gets back a value in (0, target_position_time_ms].
- After the move completes, `0x6082` reads as `0`.
- Adding `0x6082` to the telemetry map (`0x2A00`) results in the value being streamed in the cyclic telemetry blob.
- LCMC's CAMERAD poll responses surface this value as `iTimeToShot` (motor MCU has no involvement in that piece; it's purely a LCMC consumer).

### Discussion

*2026-06-22 (CMC side)*: filed while starting CAMERAD protocol implementation. Surfacing live trajectory-time to camera panels is a CAMERAD UX requirement (progress-bar rendering during fades). Could be approximated CMC-side from position/velocity, but per the design principle established in REQ-0010 (motion math lives on the motor MCU), an authoritative OD entry is the right answer. No version bump required — purely additive motor-owned OD entry.

---

## REQ-0012: Add a current/torque command mode to `axis_manager` (PC command page)
- **Source**: PC tool (`Interface/gui`)
- **Target**: `Lightweight_CMC` (network MCU)
- **Status**: **done** — CMC + PC tool implemented, CHANGELOG [4.3.0]
- **Opened**: 2026-06-22
- **Closed**: 2026-06-25
- **Priority**: functional

### Why

The PC tool is getting a **motor command page** (set mode; request velocity / position / current). Per `INTERFACE_SPEC.md §5b` the page commands through `axis_manager` (`0x30xx`), not the motor's CiA-402 objects directly. Velocity and position already have an `axis_manager` path; **current does not** — `compute_desired` hardcodes `target_torque_scaled = 0` and there is no torque op-mode. The **motor MCU already supports it end-to-end**: `mc_scheduler.c` routes `MC_MODE_TORQUE_CURRENT` (`0x6060 = MC_IF_MODE_TORQUE = 4`) to `s_eff_iq_cmd = target_torque (0x6071) × MC_IF_CUR_SCALE` (amps). Only the CMC surface is missing. (Decision recorded in motor **ADR-027**.)

### What's needed

All CMC-side (no motor change). Commanded in **amps** to match the motor's `0x6071` (scaled `MC_IF_CUR_SCALE = 1e-3 A/LSB`).

1. **Contract (`mc_if_od.h`)** — additive, **no `MC_IF_PROTOCOL_VERSION` bump**; add a `CHANGELOG.md` entry when done:
   - New CMC-owned entry: `X(0x302B, 0, axis_target_current, MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE, MC_IF_OWNER_CMC)` — commanded current [A], effective only in the new torque mode. (`0x302B` is the next free slot after the joystick block; pick another if you prefer.)
   - New axis op-mode: `#define MC_IF_AXIS_MODE_TORQUE (5u)` (existing values stop at `HOLD = 4`).
2. **`app/axis_manager/axis_manager.h`**: add `AXIS_OP_MODE_TORQUE = MC_IF_AXIS_MODE_TORQUE`; add `float axis_manager_get_target_current(void)` / `bool axis_manager_set_target_current(float a)`.
3. **`app/axis_manager/axis_manager.c`**:
   - `s_axis.target_current_a` field; getter/setter (reject NaN).
   - `axis_mode_to_cia402(AXIS_OP_MODE_TORQUE) → MC_IF_MODE_TORQUE`.
   - `axis_manager_set_op_mode`: accept `5`.
   - `compute_desired`: `out->target_torque_scaled = to_scaled_i32(s_axis.target_current_a, MC_IF_CUR_SCALE);` (replaces the hardcoded 0). The existing `SEQ_TARGET_TORQUE` sequencer step then SDO-writes `0x6071` — wire it into the `setup_sequencer_tick` diff chain (it's defined but currently never compared).
   - `current_velocity_demand_rad_s` already returns 0 for non-velocity modes, so no `velocity_setpoint` is streamed in torque mode — good. Enable/controlword path is unchanged (`ENABLE` set for any non-OFF mode).
4. **`app/od/cmc_od.c`**: dispatch `0x302B` read→`axis_manager_get_target_current`, write→`axis_manager_set_target_current`.

**Set-and-hold via SDO** (the v3 cyclic frame streams only `velocity_setpoint`); high-rate current streaming would need a cyclic-frame revision (separate, wire-breaking) — out of scope here.

### Acceptance

- PC writes `axis_op_mode (0x3020) = 5`, `axis_target_current (0x302B) = I`, `axis_enable (0x3010) = 1` → motor enters torque mode; FOC commands `iq ≈ I`; `torque_actual (0x6077)` and motion reflect it.
- Setting `axis_target_current = 0` or `axis_enable = 0` / quick-stop stops the current.
- `axis_state (0x3000)` tracks RUNNING/READY as for the other modes.

### Discussion

*2026-06-22 (PC-tool/motor side)*: filed while building the PC motor command page. Motor side already wired (`mc_scheduler.c:551`, no change needed); decision + rationale in motor **ADR-027**. The PC command page will expose a **Torque** mode + a current field once this lands; until then the page shows it as pending this REQ. Contract additions above are deliberately **not** pre-applied to `mc_if_od.h` — the CMC owns these `0x30xx` entries, so they should be added by the CMC change that implements them.

*2026-06-25 (CMC + PC tool)*: **implemented** exactly per the spec above. Added `MC_IF_AXIS_MODE_TORQUE = 5` and CMC-owned OD entry `0x302B axis_target_current` (F32 RW, no PERSIST — current setpoints are operator-live, not saved). `axis_manager` now stores `target_current_a`, maps `AXIS_OP_MODE_TORQUE → MC_IF_MODE_TORQUE` in `axis_mode_to_cia402`, populates `target_torque_scaled` from the new field in `compute_desired`, and wires the long-defined-but-unused `SEQ_TARGET_TORQUE` step into `setup_sequencer_tick`'s diff chain (also re-added to `setup_sequencer_busy` now that it's actually written). `cmc_od` dispatches `0x302B` reads and writes. PC tool's Motor Command page enables the Torque mode in the combo and the "Apply current" button writes `0x302B`. Recorded as CHANGELOG [4.3.0] — no `MC_IF_PROTOCOL_VERSION` bump (CMC-owned additive entry).

---

## REQ-0013: Expand `MC_IfCyclicStatusHeader_t` with position_actual + movement_status
- **Source**: `Lightweight_CMC` (network MCU)
- **Target**: `Generic_motor_controller` (motor MCU)
- **Status**: done — motor + GUI shipped v4 (ADR-033, CHANGELOG [4.0.0]); CMC consumer implemented: `axis_manager` populates `position_actual_rad` (from `hdr.position_actual_scaled * MC_IF_POS_SCALE`) and a new `movement_status` field from `hdr.movement_status`; `cmc_state_update_from_motor` now uses `MC_IF_MOVE_ON_TARGET` + `MC_IF_MOVE_MOVING` for POLL response `on_shot`/`moving` (replaces the position-tolerance hack); `cia402.c`'s `rx_bad_version` check auto-rejects stale v3 motor frames now that we build against `MC_IF_PROTOCOL_VERSION=4`.
- **Opened**: 2026-06-24
- **Closed**: 2026-06-24
- **Priority**: functional (blocks CMC shot-recall accuracy + panel "on target" indicator)

### Why

The CMC's CAMERAD shot-store feature captures `axis_manager_get_position_actual()` at the moment the operator presses STORE. Today that always returns `0` because `s_axis.position_actual_rad` is never populated — the value lives in `0x6064 position_actual` which is only available via the variable telemetry blob (mapped per `0x2A00`). Parsing the blob would require the CMC to track the host-configurable map, and any GUI reconfiguration of the map breaks the shot system.

Same problem affects panel UX: the CAMERAD POLL response carries an `on_shot` bit and a `moving` bit. Both should reflect motor reality. Today the CMC fakes them by comparing target to a position it doesn't actually have, so they're meaningless. Operators see no live "on shot" / "moving" indicator.

The fix is to put live position and a movement-status bitfield in the **fixed** header so the CMC always has them, independent of the telemetry-map configuration. They're not application-specific — they're the most basic feedback any controller needs.

### What's needed (motor MCU side)

**Wire-breaking**: header layout changes → `MC_IF_PROTOCOL_VERSION` bumps from `3` to `4`.

1. **Contract (`mc_if_protocol.h`)** — extend the header:
   ```c
   typedef struct __attribute__((packed)) {
       uint16_t statusword;             /* OD 0x6041 */
       int8_t   mode_display;           /* OD 0x6061 */
       uint8_t  node_state;             /* MC_IfNodeState_t */
       uint16_t error_code;             /* OD 0x603F */
       uint8_t  map_version;
       uint8_t  map_byte_count;
       uint32_t status_counter;
       /* NEW in v4: */
       int32_t  position_actual_scaled; /* OD 0x6064, MC_IF_POS_SCALE (1e-5 rad/LSB) */
       uint16_t movement_status;        /* MC_IF_MOVE_* bits below */
   } MC_IfCyclicStatusHeader_t;
   ```
   Plus:
   - `#define MC_IF_STATUS_HEADER_SIZE (18u)` (was 12).
   - `MC_IF_TLM_BLOB_MAX` recomputed automatically (40 → 34 bytes — slightly less telemetry budget; acceptable).
   - `#define MC_IF_PROTOCOL_VERSION (4u)`.

2. **New constants** for `movement_status` bits:
   ```c
   #define MC_IF_MOVE_MOVING              (0x0001u)  /* axis is actively moving (non-zero velocity demand or trajectory running) */
   #define MC_IF_MOVE_ON_TARGET           (0x0002u)  /* within tolerance of last commanded target (mirrors SW TARGET_REACHED but in motor-trajectory terms, not bus-mode terms) */
   #define MC_IF_MOVE_SETPOINT_ACCEPTED   (0x0004u)  /* last NEW_SETPOINT pulse was latched (rises on accept, clears on next NEW_SETPOINT) */
   #define MC_IF_MOVE_SETPOINT_COMPLETE   (0x0008u)  /* current PROFILE_POSITION/PROFILE_VELOCITY trajectory has finished */
   #define MC_IF_MOVE_AT_LIMIT_LO         (0x0010u)  /* soft-limit hit (motor-side limit, if any) */
   #define MC_IF_MOVE_AT_LIMIT_HI         (0x0020u)
   ```
   Bits 6-15 reserved.

3. **Cyclic status frame builder**: populate both new fields from existing internal state — `position_actual_scaled` from the same value the OD bridge mirrors into `0x6064`, `movement_status` derived from the trajectory engine state + current vs target position.

4. **CHANGELOG.md** entry: `[4.0.0] — wire-breaking, MC_IF_PROTOCOL_VERSION 3→4, header layout extended`.

### Why fixed header (not mapped blob)

We considered two alternatives:
- *Telemetry-map-based*: CMC ensures `0x6064` is mapped at a known slot. Cleaner against the protocol but fragile — GUI's map editor would have to coexist with reserved slots, and the operator can break the shot system by re-mapping.
- *Acyclic SDO read on store*: ~1 ms per store, but no live `on_target` indicator and no live status for non-shot operations.

Fixed header is the right answer: position is a first-class motor-feedback signal that every controller in the system needs, independent of host-configurable telemetry.

### What needs to update on each side once shipped

- **motor-control MCU**: implement steps 1-3 above; bump version; add CHANGELOG entry.
- **network MCU** (this project):
  - Update `cia402_peek_cyclic_status` consumers in `axis_manager.c` — pull `position_actual_scaled` and `movement_status` from the new header fields instead of leaving `position_actual_rad` at 0.
  - `cmc_state_update_from_motor` uses `MC_IF_MOVE_ON_TARGET` directly instead of computing a tolerance check against position.
  - Reject motor MCU frames where `version != 4` (the existing version-check path), so a stale motor firmware can't silently send the old layout that we'd mis-parse.
- **PC tool**: telemetry-blob budget drops from 40 → 34 bytes (max telemetry signals per cyclic frame). May need a one-line tweak to the map editor's slot count display. New `movement_status` becomes available as a graphable signal (could also be exposed in the GUI's status bar).

### Acceptance

- LCMC's `axis_manager_get_position_actual()` returns a live value matching the motor's actual position.
- CAMERAD shot-store captures the correct position; recall drives to that position (currently always drives to 0).
- POLL response's `cmc_status` bit 2 (on_shot) flips true within ≤ one cyclic cycle of arriving at the target shot's stored position.
- Existing telemetry-map-based graphs still work (max simultaneous signals just slightly reduced).

### Discussion

*2026-06-24 (CMC side)*: filed after the shot-recall-to-0 bug surfaced during panel testing. Option D (acyclic SDO on store) was offered as a quick fix; CMC user chose the proper header-expansion path so the data is always available for other uses (UI on-target indicator, future joystick-based fine-positioning, etc.). Wire-breaking but the right architectural answer: motor MCU is the authoritative source for position and trajectory state, and a fixed-format channel for these basic signals is more robust than threading them through the host-configurable telemetry map.

*2026-06-24 (motor side)*: **implemented with a reduced `movement_status` set** (agreed with the user). Of the six proposed bits, only **`MOVING` (0x0001)** and **`ON_TARGET` (0x0002)** are populated live; **`AT_LIMIT_LO`/`HI` (0x0010/0x0020)** are kept in the contract but **reserved (0)** until the motor has soft position limits (a fast-follow ADR when wanted); **`SETPOINT_ACCEPTED`/`COMPLETE` (0x0004/0x0008)** are **dropped → reserved** since the CMC consumers above only use `MOVING`/`ON_TARGET`. **Bit positions are unchanged** from this request, so nothing renumbers and the dropped/reserved bits can be populated later with no further wire change. Shipped per **ADR-033** + **CHANGELOG [4.0.0]**: `mc_if_protocol.h` header v4, `MC_IF_PROTOCOL_VERSION 3→4`, `position_actual_scaled` read from `0x6064`, `movement_status` pushed from the scheduler (`MC_Comms_SetMovementStatus`). PC tool (GUI) header parser updated too. **CMC side still to do**: the `axis_manager` consumer changes listed above + the `version != 4` reject. If you do want `SETPOINT_ACCEPTED`/`COMPLETE` after all, say so — the positions are reserved.

---

## REQ-0014: Add `vel_load_factor` (0x2300:5) — operator multiplier on velocity-loop kp/ki
- **Source**: `Lightweight_CMC` (network MCU)
- **Target**: `Generic_motor_controller` (motor MCU)
- **Status**: done — motor side implemented (ADR-034, CHANGELOG [4.1.0])
- **Opened**: 2026-06-25
- **Closed**: 2026-06-25
- **Priority**: functional (CMC web slider already implemented and waiting; SDO writes return NO_OBJECT until this lands)

### Why

The CMC's web config page is gaining a **load-factor slider** — operator drags it from "Light load" (0.3) to "Heavy load" (2.0) to scale the velocity-loop response for the current payload. Heavier camera bodies + lens combos need higher loop gain to track demand without lag; lighter setups need less aggressive gain to avoid noise/oscillation. Same axis_payload_weight_kg use-case I filed earlier (REQ no number — was a CMC-only entry at 0x3040), now as a motor-owned dimensionless multiplier so the velocity loop math actually uses it.

The earlier CMC-only `0x3040 axis_payload_weight_kg` (CHANGELOG [3.6.0]) is being removed in the same change-set on the CMC side, since the operator-tunable value belongs where the velocity loop runs.

### What's needed (motor MCU side)

**Additive**, no `MC_IF_PROTOCOL_VERSION` bump — same shape as the other gains in the `0x2300` velocity-loop block.

1. **Contract (`mc_if_od.h`)** — add the entry:
   ```c
   X(0x2300, 5, vel_load_factor, MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR)
   ```
   Default `1.0` (no scaling, matches current behaviour). Rejected on write if outside `[0.3, 2.0]` (or the motor's preferred valid range — the CMC's slider clamps to 0.3..2.0 but feel free to widen).

2. **Velocity loop** — apply the multiplier at runtime:
   ```c
   effective_kp = vel_kp * vel_load_factor;
   effective_ki = vel_ki * vel_load_factor;
   ```
   (kd unchanged unless you have a reason — typically just kp+ki scale with load.) The base `vel_kp` (0x2300:1) and `vel_ki` (0x2300:2) stay as the operator-tuned baseline; `vel_load_factor` is the runtime adjustment.

3. **Persistence**: PERSIST flag — saved alongside the other 0x2300 gains. Survives reboot.

4. **CHANGELOG** entry: `[4.x.0]` additive, motor-owned, no version bump.

### What needs to update on each side once shipped

- **motor-control MCU**: implement above; CHANGELOG entry.
- **network MCU** (this project): **already done** — `axis_manager` has get/set wired (set fires an ad-hoc SDO write to 0x2300:5 via `cia402_od_write_begin`); web page has the slider. Once motor MCU adds the OD entry, writes will succeed.
- **PC tool**: appears automatically in the OD browser as another tunable in the velocity-gains group. The CMC's web page UX is the primary surface, but having it in the PC tool is nice for fine-tuning.

### Acceptance

- Slider in CMC web at value 1.0 → motor's effective_kp = vel_kp × 1.0 (current behaviour, unchanged).
- Slider at 2.0 → motor's effective_kp = vel_kp × 2.0; visibly stiffer response, less lag under load.
- Slider at 0.3 → motor's effective_kp = vel_kp × 0.3; gentler / less aggressive correction.
- Value persists across motor MCU reboot (per PERSIST flag).
- CMC's existing `vel_kp` / `vel_ki` tuning workflow (PC tool Read/Write tab on `0x2300:1/2`) is unaffected — those remain the operator-tuned baseline.

### Discussion

*2026-06-25 (CMC side)*: filed during web-UI work. The original CMC-side `0x3040 axis_payload_weight_kg` was a placeholder pending exactly this kind of motor-side support. Now that the velocity loop will consume the value directly, the CMC-side entry is redundant and being removed in the same change. Range 0.3..2.0 is the operator's preferred clamp for the slider — motor can accept a wider range if helpful for tuning at extremes. Default 1.0 ensures no behavioural change for already-deployed boards on a fresh contract update.

*2026-06-25 (motor side)*: **implemented.** Added `0x2300:5 vel_load_factor` (F32 RW PERSIST, default 1.0); the velocity loop applies `effective_kp = vel_kp × factor`, `effective_ki = vel_ki × factor` (kd unchanged) in `od_apply_gains`, with `factor = clamp(vel_load_factor, 0.3, 2.0)`. Seeded 1.0 in firmware defaults (behaviour-neutral on a fresh contract update). ADR-034 / CHANGELOG [4.1.0]. Your pending SDO writes to `0x2300:5` will now succeed. **Note:** I used a **clamp** to [0.3, 2.0] at apply time rather than rejecting out-of-range writes (the OD write path is a generic typed write, and a clamp can't zero/blow the loop gain) — your slider already clamps to 0.3..2.0 so this is invisible in practice; say if you'd prefer a hard write-rejection instead.

## REQ-0015: Bootloader OD dispatch (0x1F5x) and segmented-SDO receiver

- **Source**: Generic_axis_controller (CMC)
- **Target**: Generic_motor_controller (motor MCU)
- **Status**: in-progress
- **Opened**: 2026-07-03
- **Closed**: -
- **Priority**: blocking (short-term: the contract additions in v5 currently `BAD_VERSION` your v4 SPI parser; longer-term: this REQ unlocks over-the-wire firmware update of the motor)

### Why

We want field-updatable firmware on both MCUs without JTAG. Phase 1 of the dual-bootloader design (`Documentation/dual_bootloader_design.md`) landed in the shared Interface as v5 — 4 new OD entries at `0x1F5x`, 3 new SPI message types for segmented download, new result codes, new node state, new owner value. This is contract only. The wire format is now v5, which is why your firmware currently sees `BAD_VERSION` on every SPI frame from a v5 CMC.

Once you implement the receiver side of the bootloader OD, the CMC can drive both its own and your firmware updates via a single "Update firmware" UI in the PC tool.

Full architecture: `Documentation/dual_bootloader_design.md`. Wire additions: `INTERFACE_SPEC.md §7`.

### What's needed

**1. Rebuild against v5 headers.** Minimum action to unblock existing operation. Your current SPI parser presumably strict-matches the version byte; a v5 CMC will `BAD_VERSION` against a v4 motor and vice versa. This is a rebuild-and-flash, no logic change beyond picking up the new enum values.

**2. Handle `MC_IF_NODE_BOOTLOADER = 0x07` in your node-state reporter.** While the app is running, keep reporting the existing states (INIT / DISABLED / READY / RUNNING / QUICK_STOP / FAULT / CALIBRATING). Only report BOOTLOADER from a *separate bootloader binary* — see §5 of the design doc for the split.

**3. Return `ERR_UNKNOWN_MSG` from the app for the three new message types.** `MC_IF_MSG_OD_DOWNLOAD_INIT / _SEGMENT / _RESP` (`0x14 / 0x15 / 0x16`). Your existing "unknown msg type → ERR" default dispatch should already do this — just confirm.

**4. (Phase 2 — separate binary)** implement the motor-side bootloader:
- Minimal bootloader at `0x0800_0000` that speaks the same `mc_if_protocol` framing over the same SPI link.
- OD dispatch for `0x1F50 / 0x1F51 / 0x1F56 / 0x1F57` only. All other reads return `ERR_NO_OBJECT`.
- Segmented-SDO receiver — reassemble `INIT + N×SEGMENT` into a flash-programming stream. Toggle bit handling per `INTERFACE_SPEC.md §7c`. Write bytes to the app-image slot as they arrive; do NOT buffer the entire image in RAM.
- Flash programming — sector erase on `PROG_START`, word-program during segments, CRC32 on `PROG_VERIFY`, jump on `PROG_COMMIT`.
- Trigger detection — app sets a RAM marker preserved across `NVIC_SystemReset` on receipt of `0x1F51 = MC_IF_PROG_START`; bootloader checks the marker on boot and stays in bootloader mode if set. Hard fallback if the app fails to set an alive flag within N seconds after last boot (protects against a boot-loop-crashing app).
- Shared BSP — reuse your `motor_spi` + `flash` modules across the app and bootloader (a static library or shared-source pattern). Total flash budget is comfortable on the CMC's G474RE; verify yours has headroom for bootloader + app + optional A/B slots.

### Acceptance

Phase 1 (this REQ minimum) done when:
- v5-labelled CMC frames parse cleanly on the motor and vice versa.
- `MC_IfNodeState_t` on your side compiles with `MC_IF_NODE_BOOTLOADER = 0x07` present.
- App returns `ERR_UNKNOWN_MSG` when it sees the three new download message types.

Phase 2 (bootloader binary) done when:
- PC tool can issue `0x1F51 = MC_IF_PROG_START` → motor reboots into bootloader → segmented-write a fresh image → `PROG_VERIFY` returns `IDLE` (not `FAULT`) → `PROG_COMMIT` reboots into the new image → `0x1F56` reports the new CRC.
- Interrupted uploads leave the device recoverable via a re-attempt (no JTAG needed to unbrick).

### Discussion

*2026-07-03 (CMC side)*: filed as Phase 1 of the bootloader work lands. Only the wire additions are in this commit — no bootloader code exists yet on the CMC either. Phase 2 (both bootloader binaries) is a separate multi-week effort. The immediate practical impact for you is item **1** above; without it your firmware refuses any v5 SPI frame, which blocks all downstream OD traffic from the PC tool. Once you rebuild against v5, item 3's default dispatch should already handle the three new message types safely.

*2026-07-03 (motor MCU, ADR-061)*: **Phase 1 (items 1–3) done.** Adopted contract v5 — added `OD_ROW_MC_IF_OWNER_BOOTLOADER` skip in `mc_od.c` (the OD X-macro otherwise fails to compile against v5, undefined owner-row for the `0x1F5x` entries), repointed the build + source include path to the moved canonical Interface, host-compiles clean against v5 (`fw_build 83`). Default message dispatch already returns `ERR_UNKNOWN_MSG` for `0x14/0x15/0x16`; node-state + result-code enums are additive. **Phase 2 (the motor bootloader binary — `0x1F5x` dispatch, segmented-SDO receiver, flash programming, reset-marker trigger, shared BSP) remains open.**

*2026-07-07 (CMC side, Phase 2 status)*: **CMC bootloader binary is now written, flashed, and functional end-to-end** — the PC tool's "Update CMC firmware…" button drives the full 8-step sequence (Documentation/dual_bootloader_design.md §4). Bring-up flagged several design details that had drifted from the original plan; capturing them here so your motor-side Phase 2 doesn't repeat the same iterations. **Skim before you start coding.**

### CMC-side Phase 2 — implementation decisions that differ from the original design

**1. Trigger marker: FLASH not RAM.** The design doc (§5.3) originally suggested a RAM word preserved across `NVIC_SystemReset` + an alive-flag fallback for boot-loop detection. We changed to a **persistent flash marker** in its own persist page for two reasons:
- Survives power-cycle mid-update (RAM would lose the marker → chip boots into a partially-flashed app → possibly bricked).
- Collapses the "we asked for update" + "is the app healthy?" checks into one signal, obviating the alive-flag counter.

Motor-side equivalent: pick a dedicated flash page (or reuse an existing persist region with a new field). Layout on the CMC:
- Page 251 (`0x0807D800`) — 2 KB, holds a `persist_header_t` (16 B) + 4-byte payload magic.
- STAY magic = `0xB007107D`.  CLEAR magic = `0x00000000`.  Anything else (including `0xFFFFFFFF` from a fresh erase) is treated as CLEAR.

**2. Flag lifecycle — app clears, bootloader never touches.**
- App receives `0x1F51:1 = MC_IF_PROG_START` → app writes STAY + `NVIC_SystemReset()` (`app/boot_meta.c:boot_meta_enter_bootloader`).
- Bootloader reads STAY → serves firmware update. **Bootloader never writes the flag.**
- After successful update, bootloader jumps to app (see #3 below).
- New app runs. After `BOOT_META_HEALTHY_MS = 5000` ms of no-fault runtime, app writes CLEAR (`app/boot_meta.c:boot_meta_tick`).
- **Brick-proof property**: if the new app crashes before 5 s, the flag stays STAY. Next reboot re-enters bootloader → operator can retry the update. No JTAG needed to unbrick.

**3. PROG_COMMIT semantics — JUMP, don't RESET.** This one bit me during bring-up. Original design + REQ-0015 both say "reboot into the new image." Don't do that. Sequence:
- If bootloader does `NVIC_SystemReset()` on COMMIT → chip reboots → bootloader reads flag (still STAY) → stays in bootloader → app never runs → flag never clears → **deadlock forever**.
- Instead, bootloader jumps directly to the new app's Reset_Handler (same code path as the CLEAR-at-boot happy path). Flag stays STAY. App clears it after 5 s.
- The jump sequence (`boot/main.c:boot_jump_to_app`): `__disable_irq()` → `HAL_DeInit()` → clear all NVIC interrupts → set `SCB->VTOR` to app's vector table → **`__enable_irq()`** (I hit a separate bug from missing this — HAL_Delay hangs otherwise) → `__set_MSP()` → branch.

**4. PROG_VERIFY — hardware-side is a no-op; PC tool does the check locally.** The bootloader accepts VERIFY and returns OK unconditionally. The PC tool separately reads `0x1F56 program_software_id` (which the bootloader computes as a live CRC32 over the just-programmed bytes) and compares to the CRC32 of the source `.bin`. Simplifies the bootloader; the on-chip → wire round-trip is what makes the verification trustworthy. Feel free to implement VERIFY on-chip if you'd rather.

**5. Segmented-SDO receiver — write-through, no buffering.** Each `MC_IF_MSG_OD_DOWNLOAD_SEGMENT` payload gets handed straight to `boot_flash_write` (which buffers only the tail dword until 8 bytes accumulate — flash program granularity). RAM usage is O(one segment) regardless of image size. This matters more for you than for the CMC since the STM32G474 has 128 KB RAM to spare, but the design assumption is worth documenting.

**6. Toggle bit + last-segment handling.** Follow `INTERFACE_SPEC.md §7c`. Sender alternates toggle 0/1 across successive segments; receiver echoes the received toggle in its RESP. Toggle mismatch = the sender missed our last ack — echo their expected toggle back and don't advance the write cursor; they'll resend. `SEG_FLAG_LAST` bit closes the session cleanly.

**7. Boot flag reader lives in the bootloader, duplicated header definitions.** The bootloader deliberately does NOT link the app's persist module (would drag in too many app-owned modules like axis_manager and log). Instead, `boot/boot_flag.c` duplicates the persist header format and CRC32 impl. Keep these in sync if the header ever changes — worth calling out in a comment on both sides.

**8. Network config in bootloader (CMC-only concern).** The CMC bootloader reads the NETWORK persist blob so it comes up on the same IP the app was on — otherwise a PC tool at `192.1.0.101` (say) loses the CMC when the bootloader takes over at the hardcoded default `192.1.0.100`. Not relevant to you (motor is SPI, not IP-based) — flagging so you don't wonder what `boot/boot_net_cfg.c` is doing when you look at the CMC source for reference.

**9. Response-then-jump ordering + drain delay.** On COMMIT, the bootloader must:
- Send the `MC_IF_MSG_OD_WRITE_RESP` with OK first (from the same UDP dispatch that received the write).
- Delay ~100 ms so the response drains from the W6100 TX buffer + out onto the wire.
- Then perform the jump.

We do this via a `s_commit_pending` flag polled at the tail of `boot_od_tick`. Equivalent on the SPI side would give the master a chance to see the OK before the slave vanishes into the new app.

### What this means for your Phase 2 acceptance criteria

Same functional outcome as before (`PROG_START` → reboot into bootloader → segmented download → verify → commit → new app runs). Just:

- **Don't reset on COMMIT — jump.**
- **Use a flash marker, not RAM** (survives power-cycle mid-update).
- **Let the new app clear the marker** (brick-proof).

Everything else is a mirror of the CMC-side implementation, adjusted for your chip / SPI transport / motor-specific flash layout. Feel free to lift the boot_flag / boot_flash / boot_seg_sdo modules almost verbatim.

*2026-07-07 (motor side, Phase 2 status)*: **Motor bootloader is written — code-complete, host-compiles, NOT yet flashed/verified on target.** Implemented per the nine decisions above (ADR-064 in Generic_motor_controller). **Action for the CMC/network side: wire up the PC-tool → CMC → motor pass-through** so the existing "Update firmware…" flow can target the motor node. Motor-side specifics the pass-through should know:

- **Transport:** the motor bootloader is an SPI **slave** using the *same* frame format as the app (`MC_IfFrameHeader_t` + CRC16-Modbus, `MC_IF_FRAME_SIZE` = 64). No wire/OD change — the CMC forwards the existing `OD_READ/WRITE` + `OD_DOWNLOAD_INIT/SEGMENT` frames unchanged; only the OD range answered differs (0x1F5x while `CYCLIC_STATUS.node_state == MC_IF_NODE_BOOTLOADER` = 0x07).
- **Pipelining:** the bootloader stages its response on the **next** transaction (same pipelined-by-one model as the app's `mc_comms`). The CMC master already correlates by `sequence`, so no change — just don't expect the response in the same transaction that carried the request.
- **Enter bootloader:** `OD_WRITE(0x1F51:1 = PROG_START)` to the motor → the app writes STAY + resets; the motor reappears in bootloader mode within a boot cycle. Other 0x1F5x writes to the *app* now return `MC_IF_OD_ERR_NOT_BOOTLOADER` (0x0C).
- **Flash map:** app relocated to 0x08008800; bootloader 0x08000000 (32K); boot-flag page 0x08008000; config A/B (bank2 pg126/127) untouched by updates. Provisioning flashes `boot.bin`@0x08000000 + `<app>.bin`@0x08008800 once via SWD (see `Generic_motor_controller/boot/README.md`).
- **VERIFY:** on-chip is a no-op; the PC tool reads `0x1F56` (live CRC32 over the programmed bytes) and compares to the `.bin` CRC, exactly like the CMC path.
- **Segment sizing:** motor accepts up to `MC_IF_MAX_PAYLOAD − 3` = 49 data bytes/segment (64 B frame). Same as the CMC.

Open: on-target bring-up on the motor board (`make -C boot`, flash, run the end-to-end checklist in `boot/README.md`).

---

## REQ-0016: Deprecate `0x2300:9 holding_enable` — CMC now owns idle-behaviour decision
- **Source**: `Lightweight_CMC` (network MCU)
- **Target**: `Generic_motor_controller` (motor MCU)
- **Status**: in-progress — motor side implemented (ADR-072); pending on-target end-to-end acceptance
- **Opened**: 2026-07-22
- **Closed**: -
- **Priority**: functional

### Why

The motor's autonomous "release holding current after ~1 s dwell at zero velocity" logic (behind `0x2300:9 holding_enable`, ADR-054) is now redundant with the CMC's op-arbitration layer. Splitting the "when to release the drive" decision across two owners (motor's dwell timer + CMC's arbiter) is a form of implicit state — hard to reason about and hard to test.

The CMC has taken over as the single source of truth via a new CMC-owned entry **`0x3044 axis_holding_enable`** (Interface v5.5.0). On every op-family release (JOYSTICK / SHOT_RECALL / HOMING → NONE), the CMC now explicitly transitions `op_mode`:

- `0x3044 = 1` (default) → `op_mode = HOLD` (motor decelerates via HALT bit and holds at zero velocity)
- `0x3044 = 0`           → `op_mode = OFF` (drive disabled — needed for high-stiction actuators where sub-sensor-threshold voltage causes drift)

Once this request is done, the motor stops making autonomous decisions about holding — it just does what the CMC's op_mode + controlword say, which is what a servo should do.

### What's needed

1. **Remove the autonomous 1-second-dwell release logic** currently triggered by `0x2300:9 = 0` at zero measured velocity. The motor should no longer independently drop its holding current — it should hold whatever the CMC's current op_mode implies until the CMC changes op_mode.

2. **Preserve reads of `0x2300:9`** for one release cycle so the CMC's bootsync (if it ever proxies this entry) doesn't error. Value can be anything (0 or 1); it's advisory-only from this point on.

3. **Optional: delete `0x2300:9`** entirely in the next protocol version bump (v6.0.0 or later — this is a wire-visible OD entry removal, so it needs a version bump and coordination with the CMC + PC tool). Not required for functional completion of this request; can be deferred.

4. **Motor firmware CHANGELOG entry** noting the autonomous logic removal.

### Acceptance

End-to-end on real hardware with both firmwares updated:

- Set CMC `0x3044 = 1`, jog the axis via joystick, release. Motor stays at zero velocity WITH holding current for the whole quiescent period. Repeat with a shot recall — motor holds at target indefinitely.
- Set CMC `0x3044 = 0`, jog the axis, release. Motor's ENABLE bit drops within ~200 ms of the stick centring (CMC's arbiter quiescent window). Statusword goes RUNNING → READY / DISABLED. No holding current. Repeat with shot recall — motor arrives at target, then drive disables.
- Verify motor's `0x2300:9` value is IRRELEVANT to observed behaviour: try setting it to 0 with CMC `0x3044 = 1` — motor should still hold (per CMC's command), not release autonomously.
- Motor `command_counter` dead-man still triggers on CMC crash (safety net unchanged — not related to this request but worth confirming in the test).

### Discussion

- CMC-side implementation shipped in commit `<TBD after this commit lands>`. See `app/axis_manager/axis_manager.c` `apply_op_release_hold()` and `tick_active_op()`.
- Interface CHANGELOG entry: `[5.5.0]` — additive only, so no wire-breaking change until the eventual `0x2300:9` deletion.
- **Interim state is safe**: with the CMC now commanding op_mode → OFF explicitly on release (`0x3044 = 0` case), the motor's autonomous release becomes a no-op — the drive is already disabled by the time the 1-second dwell would fire. For the `0x3044 = 1` case, if the operator forgot to also flip motor's `0x2300:9` to 1, the motor's release logic would still fire and defeat the CMC's HOLD intent. Recommend motor team set factory default of `0x2300:9 = 1` (always hold) once the autonomous logic is removed, so leftover PERSIST values from earlier motor firmwares don't cause surprises.
- 2026-07-22 (Lightweight_CMC): request opened alongside the CMC-side implementation. Ready for motor team to pick up.

- 2026-07-22 (Generic_motor_controller, ADR-072): **motor side implemented.** Removed the autonomous 1 s dwell-release logic entirely (`s_hold_released`/`s_hold_settle_ticks` state, `MC_HOLD_RELEASE_TICKS`/`MC_HOLD_SETTLED_EPS`, the velocity-cascade release block + its entry reset). The velocity cascade now always runs the controller when active → the motor **always holds while enabled**; the drive disables only when the CMC drops the enable (op_mode = OFF → controlword → `vel_active` false → fast-loop safe-off — a clean off, no current-loop hunting). **Pt 1 done, pt 2 done** (`0x2300:9` kept, advisory-only, PERSIST, default 1; never consulted). **Pt 3 (OD deletion) deferred** to a wire-breaking version. Improvement on the interim-safety note: since the logic is *gone*, a leftover PERSIST `0x2300:9 = 0` from old firmware is now a **no-op** — no need to force the default (already 1 anyway). Contract comment on `0x2300:9` marked DEPRECATED/advisory; CHANGELOG deprecation note flipped to done. Build clean, `app.bin` 91924 B (build+review only — no hardware here). **Pending:** the on-target acceptance (both firmwares) below — flip to `done` once verified.

- 2026-07-22 (Generic_motor_controller): **housekeeping flag** — the CHANGELOG has a **duplicate `[5.5.0]`**: motor `tlm_v_arm_v` (2026-07-15) and this CMC `0x3044` entry (2026-07-22). The two teams' version lines collided during sync; the motor-side additive entries have since run 5.6.0–5.9.0. Needs a one-time reconciliation of the CHANGELOG version numbering between the copies (owner's call — not touched here to avoid clobbering the CMC's copy).
