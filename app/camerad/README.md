# app/camerad

CAMERAD protocol codec. Stateless. Bytes ↔ structs and nothing else.

## What this is

CAMERAD is the protocol Shotoku camera-control panels (S-type without screen, T-type with screen) use to talk to a CMC. SW050 implemented the full protocol; the LCMC implements a deliberate **subset**: just what's needed for S/T panels to discover, select, send key-presses, stream movement, and recall shots. CCU side-channels, on-air broadcasts, location/track broadcasts, RBS legacy bridge, cue-computer, panel-internal opcodes — all out of scope.

Authoritative on-the-wire spec: `/CAMERAD_Protocol.md` in the SW050 source tree (reverse-engineered from the SW050 firmware).

## Owns

- Header struct (`camerad_header_t`, 64 bytes, packed).
- Per-opcode body structs (`camerad_keypress_t1_t`, `camerad_movement_t`, `camerad_poll_resp_s_t`, etc.) for all in-scope messages.
- Enum constants: device types, opcodes, key codes, status bits, axis-bitmap, move types.
- Header parse + build functions.
- Small helpers: dotted-quad IP format/parse, expected-body-length lookup.

## Does NOT do

- **Sockets.** That's `app/controller_mgr`.
- **State.** No per-controller registry, no selection memory, no GRAB arbitration. The codec is reusable across multiple controllers because it owns no globals.
- **Motor calls.** Handlers in `controller_mgr` translate parsed messages into `axis_manager` OD writes (and read motor status from `axis_manager` accessors).
- **Validation beyond the wire frame.** Only header magic / version / length and per-opcode body length are checked here. Semantic validation (does this key make sense in the current state?) lives in `controller_mgr`.

## Layering

L4 — same layer as `cia402`. Depends only on `<stdint.h>`, `<stdbool.h>`, `<string.h>`, `<stdio.h>`. **Does NOT include** anything from `bsp/`, `Interface/`, `app/od`, or any other `app/` module.

## In-scope message catalogue

| Code | Symbol | Body bytes | Description |
|---:|---|---:|---|
| 1 | `CAMERAD_MSG_POLL` | 0 | UDP discovery; CMC replies unicast |
| 2 | `CAMERAD_MSG_SELECT` | 0 | Controller asks to take ownership |
| 3 | `CAMERAD_MSG_DESELECT` | 0 | Release ownership |
| 4 | `CAMERAD_MSG_GRAB` | 0 | Force-take ownership |
| 5 | `CAMERAD_MSG_KEYPRESS_T1` | 5 | Shot store/recall, fade/cut/swoop |
| 6 | `CAMERAD_MSG_KEYPRESS_T2` | 2 | Stop, limits set/restore, toggles |
| 7 | `CAMERAD_MSG_KEYPRESS_T3` | 42 | T-screen full shot frame |
| 8 | `CAMERAD_MSG_MOVEMENT` | 9 | Joystick/fader/dial frame, ~25 ms cadence |
| 9 | `CAMERAD_MSG_LIMIT` | 2 | Set / get soft limits |
| 11 | `CAMERAD_MSG_POSITION_REQ` | 0 | T-screen asks for axis positions |
| 12 | `CAMERAD_MSG_LEARN_ID_REQ` | 0 | T-screen asks for learn-ID list |
| 13 | `CAMERAD_MSG_STORE_LEARN_END_T` | 8 | (response only — notify learn-store done) |

## Specifics worth knowing

- **S vs T response shape is decided ONLY by the request's `return_device` field.** Never by inbound key code or any other side-channel. This fixes a latent SW050/Reduced-CMC bug where some response paths could pick the wrong shape.
- **Protocol version 1.3 only.** v1.0/1.2/1.4 codepaths from SW050 are not implemented. A request advertising any other version is rejected at `camerad_parse_header`.
- **Magic check is strict on all 8 bytes** (`"CAMERAD\0"`). Don't repeat the Reduced-CMC's "memcmp 7 bytes only" mistake.
- **Native little-endian.** No `htons`/`ntohl`. Both endpoints are LE; this is intentional.
- **Packed structs.** Direct `memcpy` between the wire buffer and a struct is safe.
- **No CRC, no checksum, no auth.** Wire integrity relies on TCP/UDP checksums only. Authentication relies on trusted-LAN deployment.
- **`message_length` is authoritative.** The header field carries total size including header; the dispatcher uses it to know how much body to read. The opcode → body-length lookup in `camerad_request_body_len` is a sanity check, not the source of truth.

## Acceptance

- `camerad_parse_header` rejects: short buffer, wrong magic, unsupported version, oversize `message_length`.
- `camerad_parse_header` accepts a well-formed v1.3 POLL request and fills the struct correctly.
- `camerad_build_response_header` produces a 64-byte header with: same magic + version, the response opcode, dest = request's return device, our return address + port + device fields, correct `message_length`, echoed `message_id`, `packet_id = 0`.
- `_Static_assert`s in the header confirm every wire struct has the documented size on this target.
- All public functions are pure: no static state, no globals. The same instance of this codec can safely be used for traffic from multiple controllers simultaneously.

## Future expansion (not v1)

- `CAMERAD_VERSION_1_4` support — wider status fields, extended poll response. Easy to add: accept additional version strings in `version_is_supported`, add v1.4 body structs.
- Optional broadcast emitters (camera-select, on-air, etc.) — would live in a separate `camerad_broadcasts.c` if needed.
- Body parse/build helpers — currently the caller does direct memcpy. If body parsing grows more complex (variable-length fields, optional sub-records) add per-opcode parse/build functions here.
