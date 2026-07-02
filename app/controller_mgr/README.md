# app/controller_mgr

CAMERAD controller socket + dispatch. Owns the network-facing side of CAMERAD: poll listener, per-controller TCP sessions, opcode dispatch, response sender.

## v1 status (Path A — minimal POLL-discoverable)

- ✅ **UDP POLL listener** on port 30002 (configurable via `config_get_network()->udp_poll_port`).
- ✅ **Outbound TCP back to the panel for POLL response** — CMC opens TCP to the panel's `return_port` and sends the response over it. Matches both SW050 (`NetComms.c::SendMsgToCtrler`) and the Reduced CMC (`uc_camd_interface::ensure_controller_connection`). The CAMERAD protocol doc previously said "unicast UDP back"; that was wrong, production firmware uses TCP.
- ✅ **POLL response shape** — S or T, picked by `return_device` in the request only (the bug-fix vs SW050/Reduced-CMC).
- ✅ Diagnostic counters via `controller_mgr_get_stats()`.
- ⚠ **Body values are hardcoded** for Path A — `camera_selected=false`, `cmc_status=REMOTE|CONNECTION_OK`, all shot/time fields = 0. A panel sees the LCMC on the network and can read the response shape; nothing actionable. Phase B replaces with `cmc_state` + `axis_manager` reads.
- ⚠ **One controller slot** — Path A. Phase B adds the second slot + GRAB.
- ❌ **Inbound TCP listener** (CAMERAD TCP listener, REDUCE_TCP_SOCK preference) — Phase B.
- ❌ **SELECT / DESELECT / GRAB** — Phase B.
- ❌ **KEYPRESS / MOVEMENT / LIMIT / POSITION_REQ / LEARN_ID** dispatch — Phase B.
- ❌ **5-second inactivity timeout** — Phase B.

## Discovery flow (current)

```
panel                            CMC
  |---- UDP POLL (port 30002) ----->|
  |                                 | parse header
  |                                 | find_or_assign_slot
  |                                 | net_open(TCP, sock 2)
  |                                 | net_tcp_connect(panel:return_port)
  |<-- TCP SYN to return_port ------|
  |---- TCP SYN/ACK -------------->|
  |                                 | conn = ESTABLISHED
  |<-- TCP: POLL response ----------|  (S or T shape by request's return_device)
  |                                 |
  (panel processes response; sends SELECT etc. over the same TCP — Phase B)
```

If the TCP connect doesn't complete within `CONNECT_TIMEOUT_MS` (2 s), the slot is reset and the next POLL will retry. POLLs that arrive while a connect is in progress reset the `pending_poll_response` flag with the latest POLL's header — when the connection eventually opens we respond with the freshest data.

## Owns (when fully built)

- UDP POLL listen socket (default 30002).
- TCP listener socket (Phase B — for incoming controller connections).
- Up to 2 per-controller TCP sockets (per `Documentation/architecture.md §10.1` socket allocation, after the OD UDPs + log + HTTP).
- Per-controller record: device number, device type, return address/port, last-activity timestamp, inbound/outbound TCP socket preference.
- Per-controller receive buffers.
- 5-second inactivity timeout per controller.
- GRAB arbitration: writing controller takes ownership from current, kicks current to "connected but not active".

## Does NOT do

- **Parse or build CAMERAD bytes** — `app/camerad` codec does that.
- **Hold CMC selection / status / shot state** — `app/cmc_state` (Phase B).
- **Touch motors directly** — `axis_manager` does that, called from `cmc_state` or directly from handlers depending on the message.

## Layering

L2 (alongside `app/od`, `app/log`, `app/axis_manager`). Includes:
- `app/camerad` (codec)
- `app/config` (our IP / device number / poll port)
- `app/log` (diagnostics)
- `bsp/net` (sockets)
- `bsp/time` (timestamps — Phase B)

Phase B will add `app/cmc_state` and `app/axis_manager` includes (both downward).

Does NOT include any other `app/*` module that would create a cycle.

## Socket allocation (per architecture.md §10.1)

| Slot | Purpose | Owner |
|---:|---|---|
| 0 | CAMERAD POLL UDP (30002) | controller_mgr |
| 1 | CAMERAD TCP listener (configurable) | controller_mgr (Phase B) |
| 2 | Controller A TCP session | controller_mgr (Phase B) |
| 3 | Controller B TCP session | controller_mgr (Phase B) |
| 4 | OD access UDP (5000) | app/od |
| 5 | OD telemetry UDP (5001) | app/od |
| 6 | TCP log (30200) | app/log |
| 7 | HTTP (80) | app/web (future) |

## Public API

```c
void controller_mgr_init(void);
void controller_mgr_tick(void);
void controller_mgr_get_stats(controller_mgr_stats_t *out);
```

`tick` is called every main-loop iteration; it drains the POLL socket and dispatches. No internal rate-limit — `net_recvfrom` returns 0 immediately when there's nothing waiting.

## Acceptance (v1 / Path A)

- After `controller_mgr_init`, port 30002 is bound; the W6100 socket 0 status is `SOCK_UDP`.
- A POLL request from an S-type controller produces a 86-byte unicast response (64 header + 22 body) sent to the controller's `return_address:return_port`.
- A POLL request from a T-type controller produces a 78-byte unicast response (64 header + 14 body).
- The response's `message_id` echoes the request's.
- The response's `return_address` is the LCMC's own IP from `config_get_network()->ip`, formatted as a NUL-terminated dotted quad.
- `controller_mgr_get_stats()` increments `poll_received` on every valid POLL.
- An unknown-device-type POLL is silently dropped; `poll_rejected_dev` increments.
- A short or magic-mismatched datagram is silently dropped; `poll_rejected_hdr` increments.

## Acceptance (Phase B — when added)

- TCP listener accepts up to 2 simultaneous controller connections; rejects further.
- SELECT moves the requesting controller to "active operator"; subsequent SELECT from a different controller is denied (response carries current operator's controller_no).
- GRAB transfers active-operator status to the requesting controller regardless of current.
- KEYPRESS T1 with shot recall → `axis_manager` SDO sequence + NEW_SETPOINT trigger.
- MOVEMENT body → `axis_manager_set_joystick_raw` for the active operator only.
- 5 s of no POLLs from a controller → its TCP session is closed and its slot freed.
- Inactivity-timer reset happens on every received message from the controller (POLL, key, movement — anything).
