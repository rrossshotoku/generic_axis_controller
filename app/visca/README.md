# app/visca

VISCA protocol codec. Stateless. Bytes ↔ struct + response builders, nothing else.

## What this is

VISCA is Sony's PTZ-camera protocol. This module implements the **VISCA-over-IP** framing (Sony's UDP transport used by IP-connected PTZ cameras and many broadcast PTZ controllers). Serial VISCA uses the same command payloads without the 8-byte IP header — the codec is written so a serial transport can call the same builders and skip `visca_ip_*`.

## Owns

- IP header parse + build (`visca_ip_parse_header`, `visca_ip_build_header`, `visca_ip_header_t`).
- Raw VISCA frame parser (`visca_parse` → `visca_frame_t`).
- Response builders: inquiry reply, ACK, Completion, Error.
- Body packers for the three inquiries we answer: version, pan-tilt position, zoom position.
- Address helpers (0x8D → destination, source_addr → reply-address-byte).
- Enum constants for payload types, command classes, error codes.

## Does NOT do

- **Sockets or transports.** That's `app/visca_mgr`.
- **State.** Sequence number tracking, session state, per-command "in flight" state — all in `visca_mgr` (or not implemented at all yet).
- **Motor calls.** Dispatch in `visca_mgr` translates parsed frames into `cmc_state` / `axis_manager` calls.
- **Validation beyond structural.** The parser rejects short buffers, wrong address byte range, missing 0xFF terminator, or bodies exceeding `VISCA_MAX_BODY_BYTES`. It does NOT check whether a given `(class, category, command)` triple makes sense — that's `visca_mgr`'s dispatch.

## Layering

L4 — same tier as `app/camerad` and `app/cia402`. Depends only on `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`, `<string.h>`. **Does NOT include** anything from `bsp/`, `Interface/`, `app/od`, or any other `app/` module.

## Wire format quick reference

**VISCA-over-IP UDP payload** (Sony spec):

```
[8-byte IP header] [VISCA payload]

IP header:
  bytes 0-1  payload_type    (BE)  0x0100 = command, 0x0110 = inquiry, 0x0111 = reply
  bytes 2-3  payload_length  (BE)  length of the VISCA payload
  bytes 4-7  sequence_number (BE)  echoed unchanged in reply

VISCA payload (request):
  byte 0  0x8D            D = destination 1-7 or 8 = broadcast
  byte 1  command class   0x01 = command, 0x09 = inquiry, 0x02 = cancel
  byte 2  category        0x00 interface, 0x04 camera1, 0x05 camera2 (rare), 0x06 pan/tilt
  byte 3  command code    category-specific
  bytes 4..N-1  data
  byte N  0xFF terminator

VISCA payload (reply):
  byte 0  0x9S            S = 0x08 | source-address (so cam 1 → 0x90)
  byte 1  status
           0x40 | sock  = ACK
           0x50 | sock  = Completion (or 0x50 for inquiry data reply)
           0x60 | sock  = Error (next byte is VISCA_ERR_* code)
  bytes 2..N-1  payload (inquiry data, or error type byte, or empty)
  byte N  0xFF terminator
```

## Supported commands (implemented in visca_mgr as of first pass)

| Bytes | Meaning |
|---|---|
| `81 09 00 02 FF` | `CAM_VersionInq` — vendor/model/rom version + socket count |
| `81 09 06 12 FF` | `Pan_tiltPosInq` — current pan (from axis_manager) + tilt=0 |
| `81 09 04 47 FF` | `CAM_ZoomPosInq` — always zero (no zoom on this CMC) |
| `81 01 06 01 VV WW YY ZZ FF` | `Pan_tiltDrive` — pan speed VV (1-24), pan direction YY (01=L, 02=R, 03=stop). Tilt bytes ignored (single-axis CMC). |
| `81 01 06 04 FF` | `Pan_tiltHome` — runs the homing procedure (`axis_manager_request_home`). Interpreted as "establish mechanical zero" rather than "move to (0,0)" for a rig with an endstop. |
| `81 01 04 3F 01 pp FF` | `Memory_Set` — stores current position at preset `pp` (mapped to shot `pp+1`, valid pp=0..99). |
| `81 01 04 3F 02 pp FF` | `Memory_Recall` — recalls preset `pp` (fade timing from stored `time_to_shot_s`). |

## Not supported (rejected with error)

Any request whose `(class, category, command)` triple isn't in the dispatch table gets a `y0 60 02 FF` Syntax Error reply. Add mappings in `visca_mgr` as needed.

## Layering diagram

```
              PC tool / camera controller
                         │  UDP :52381
                         ▼
                    bsp/net                     (L5)
                         ▲
                         │
                    app/visca_mgr               (L2 — sockets, session, dispatch)
                         │
      ┌──────────────────┼──────────────────┐
      ▼                  ▼                  ▼
  app/visca          app/cmc_state      app/axis_manager   (L3/L4)
  (this module)      (shot table,      (motor surface,
                      handle_movement,  request_home,
                      selection)        joystick)
```

`app/visca` never calls `app/cmc_state` or `app/axis_manager`. It doesn't even know they exist.
