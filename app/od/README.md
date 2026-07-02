# app/od

> **The OD layout, UDP framing, and telemetry-mapping conventions are owned by `Interface/`.** This module *implements* the network side of `NETWORK_UDP_SPEC.md` and bridges to the motor MCU via `app/cia402`. It does not define new OD entries.

## Purpose
Bridge the motor MCU's Object Dictionary to the network. PC tooling reads and writes OD entries on the CMC over UDP; the CMC forwards those operations as `OD_READ_REQ` / `OD_WRITE_REQ` on the SPI link via `app/cia402`, and returns the motor MCU's response. Separately, the CMC pushes a configurable telemetry stream sourced from the motor MCU's `CYCLIC_STATUS` blob to subscribed PC endpoints.

This module is a **pure bridge**. There are **no CMC-local OD entries** — the CMC's own configuration (network settings, motor soft limits, auth) lives entirely in the web UI and `app/config`. This keeps the `Interface/` OD purely a motor-MCU contract.

## Owns
- The UDP listener on the OD access port (default 5000, see `Interface/NETWORK_UDP_SPEC.md`).
- The UDP socket for the telemetry stream (default 5001), and the list of currently subscribed PC endpoints (1 subscriber to start; expand to N if needed).
- The codec for `MC_UdpHeader_t` and the typed message payloads (`OD_READ_REQ/RESP`, `OD_WRITE_REQ/RESP`, `TLM_SUBSCRIBE`, `TLM_UNSUBSCRIBE`, `TELEMETRY`, `ERROR`).
- The bridging logic: turn an incoming UDP `OD_READ_REQ` into a `cia402_od_read_begin` call, await the future, package the result into an outgoing `OD_READ_RESP`.
- The telemetry batching and rate-dividing state. Each `CYCLIC_STATUS` delivered by `cia402` is buffered; once `batch` samples have accumulated (after applying the active subscriber's `rate_divider`), a `TELEMETRY` datagram is sent.
- The `map_version` echo — pulled from `MC_IfCyclicStatusHeader_t` and copied into the outgoing `TELEMETRY` payload header so the PC can detect map changes.

## Does NOT do
- Hold any OD state. The motor MCU is the OD owner; this module is a transport.
- Re-implement the OD index map. The PC writes whatever indices it wants; the bridge passes them through.
- Authenticate. Both UDP ports are unauthenticated — exposure must be on a trusted subnet. (Documented constraint; matches `INTERFACE_SPEC.md §5a`.)
- Mutate the telemetry-map (`0x2A00`). That happens via ordinary OD writes from the PC — the bridge just forwards them.

## Public API
```c
void od_init(void);
void od_tick(void);    /* services both UDP sockets */

/* Called by cia402 on each new CYCLIC_STATUS so od can feed the telemetry
 * batcher. The header + blob are valid only for the duration of the call;
 * od copies what it needs. */
void od_on_cyclic_status(const MC_IfCyclicStatusHeader_t *hdr,
                         const uint8_t *blob, uint8_t blob_len);
```

No external API for OD reads/writes from app code: the CMC has no in-application need to read the motor's OD outside of `motor_ctrl`, which calls `cia402` directly. The web UI's "motor diagnostics" page (Phase 3+) would go through `cia402` likewise.

## Dependencies
- `Interface/mc_if_protocol.h`, `Interface/mc_if_od.h` (frozen contract).
- `app/cia402` (bridges all OD reads/writes; provides the `CYCLIC_STATUS` feed).
- `bsp/net` (two UDP sockets: 5000 OD access, 5001 telemetry).
- `bsp/time` (cyclic timestamping, telemetry rate-dividing).
- `app/config` (settable port numbers, via the web).

## Acceptance criteria
- A PC tool sends `OD_READ_REQ` for an OD entry (e.g. `0x6041` statusword) on UDP 5000 and receives `OD_READ_RESP` with the motor MCU's current value within a bounded time (one cyclic round-trip + UDP round-trip).
- A PC tool writes `0x2300:1` (vel_kp) via `OD_WRITE_REQ` and gets `OD_WRITE_RESP{result=MC_IF_OD_OK}`. A subsequent read returns the new value.
- A PC tool sends `TLM_SUBSCRIBE` on UDP 5000 (specifying its receive port, rate-divider, batch); telemetry datagrams begin arriving on its receive port within one cycle.
- When the PC writes a new telemetry map to `0x2A00` and the motor MCU bumps its `map_version`, the next `TELEMETRY` datagram carries the new version, and the PC can re-column its parser.
- Bad UDP frames (wrong magic, wrong version, length mismatch) are dropped and an `ERROR` datagram is returned with the appropriate class/detail.
- A subscriber that disconnects (no UDP back — they can't, it's connectionless) is eventually pruned if it has not refreshed its subscription within a timeout (TBD; default 30 s, settable via the web).

## Notes
- Per `INTERFACE_SPEC.md §4a`, the slave validates a telemetry-map write atomically and either commits + bumps `map_version`, or rejects with an OD error. This module need not validate anything about the map's content — it just forwards bytes.
- The OD-access UDP and the telemetry UDP can share a port if needed (the spec allows it), but allocating separate ports is cheaper to debug (`tcpdump -i any port 5001` is unambiguously telemetry).
- Subscriber lifecycle: today's plan is **one subscriber at a time** — simpler. A second `TLM_SUBSCRIBE` overrides the first. Multi-subscriber support is a Phase 6 hardening item if anyone asks.
- The W6100 OD UDP sockets are static slots in `bsp/net` (4 and 5). Phase 5 brings them online; they're allocated but unused before that.
