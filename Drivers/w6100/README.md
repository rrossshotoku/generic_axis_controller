# Drivers/w6100

WIZnet ioLibrary, vendor source — **do not modify**.

Source: WIZnet `ioLibrary_Driver` (Apache 2.0). The files here are the minimal subset needed to drive a W6100 over SPI:

- `socket.c`, `socket.h` — public socket API (`socket`, `listen`, `connect`, `send`, `recv`, `sendto`, `recvfrom`, `close`, `getSn_SR`, …).
- `wizchip_conf.c`, `wizchip_conf.h` — chip selection (`_WIZCHIP_ = W6100`), CRIS / CS / SPI callback registration, network-info accessors.
- `W6100/w6100.c`, `W6100/w6100.h` — W6100-specific register access via the registered SPI callbacks.

`Internet/` from the upstream tree is not included — we don't use DHCP, DNS, HTTPClient, MQTT or the demo apps. If we ever need DHCP, add `Internet/DHCP/` here.

### A note on `Drivers/Application/Application.h`

`wizchip_conf.h` uses a hard-coded relative include `#include "../Application/Application.h"`. That header (19 lines of `SOCK_*` and `AS_*` macros) lives in a sibling folder per the upstream layout. To honour the relative path without editing vendor source, the file sits at **`Drivers/Application/Application.h`** — a sibling of this folder. It's vendor too; don't modify it. The sibling-`Application/` name looks odd next to `STM32G4xx_HAL_Driver/`, but moving it would require patching the vendor include — explicitly against the rule.

## Wiring this into the project

The library uses `_WIZCHIP_ = W6100` selected in `wizchip_conf.h:78`. SPI interface mode is `_WIZCHIP_IO_MODE_SPI_VDM_` (variable-length data, the W6100 default).

Callbacks are registered at runtime by `bsp/net` via:

- `reg_wizchip_cris_cbfunc(enter, exit)` — empty for our polled-SPI case (no shared bus contention).
- `reg_wizchip_cs_cbfunc(select, deselect)` — drives the CS GPIO low/high.
- `reg_wizchip_spi_cbfunc(readbyte, writebyte, readburst, writeburst)` — points at our HAL SPI2 wrappers.

See `bsp/net/wizchip_glue.c` for the actual callbacks.

## CubeIDE build setup

Add `Drivers/w6100` and `Drivers/w6100/W6100` to the project's source folders (or set "Exclude from build" → false on each `.c`). Add `Drivers/w6100` and `Drivers/w6100/W6100` to the include paths so `#include "socket.h"`, `#include "wizchip_conf.h"`, and `#include "w6100.h"` resolve.

## Upgrades

To pull a new ioLibrary release, replace the six files in place and re-test. Do not edit them. If a behaviour change is needed, do it in `bsp/net` and route through the callback boundary — never patch the vendor source.
