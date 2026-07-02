# Inter-MCU Interface (shared boundary contract)

This folder is the **single source of truth** for the SPI link between the two MCUs of the
motor-drive system. It is consumed by **both** firmware projects and by host tooling, so the
contract is defined once and neither side drifts.

```
Host / PC tool
   └─ Ethernet ─► Network MCU (SPI master, Lightweight_CMC / STM32G431)
                     └─ SPI ─► Motor-control MCU (SPI slave, Generic_motor_controller / STM32G474)
                                  └─ Object Dictionary ─► motor-axis framework
```

## Contents
- `mc_if_protocol.h` — SPI wire format: frame header/footer, message types, all payloads
  (incl. the configurable telemetry frame), result/error codes. Portable C, no HAL, LE, packed.
- `mc_if_od.h` — object dictionary: data-type/access enums, scaling constants, control/status
  bits, the telemetry-map (0x2A00) macros, and the canonical `MC_IF_OD_OBJECTS(X)` list.
- `INTERFACE_SPEC.md` — the SPI contract (CMC ⟷ motor MCU): roles, framing, transaction model,
  OD conventions, **configurable runtime telemetry mapping**, side-effects, versioning, rationale.
- `NETWORK_UDP_SPEC.md` — the network contract (PC ⟷ CMC): all-UDP OD access + telemetry stream
  that the network MCU exposes on Ethernet, mirroring the OD model.
- `CHANGELOG.md` — wire-format / OD changes, with `MC_IF_PROTOCOL_VERSION` impact.
- `REQUESTS.md` — cross-project request log. Items raised by one project for another to
  implement. Process documentation, not wire contract — does **not** bump
  `MC_IF_PROTOCOL_VERSION`. See the file for the entry template.

## How each side uses it
- **Motor-control MCU (slave):** implements the OD from `MC_IF_OD_OBJECTS`, maps each entry to
  a live variable / shadow-config / callback, and runs the SPI-slave framing (decode requests,
  stage responses, serve CYCLIC_STATUS). Reconcile the existing `mc_spi_protocol.{h,c}` to
  include these shared types rather than redefining them.
- **Network MCU (master):** drives the cyclic exchange and OD read/write transactions, and
  bridges the OD to its external (Ethernet) interface.
- **Host tool:** speaks to the network MCU's external interface; uses the same OD indices,
  types, and scaling.

## Rules
- Treat this as a **frozen contract**: changes are deliberate and bump
  `MC_IF_PROTOCOL_VERSION`. Both sides rebuild against the same version.
- Keep these headers dependency-free (no platform/HAL types) so all consumers can include them.
