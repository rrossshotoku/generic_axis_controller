# boot/ — CMC bootloader binary

> **Design authority:** `Documentation/dual_bootloader_design.md`. **Wire authority:** `Interface/mc_if_od.h` (`0x1F5x` OD range, `MC_IF_PROG_*` / `MC_IF_FLASH_*` constants) and `Interface/mc_if_protocol.h` (`MC_IF_MSG_OD_DOWNLOAD_INIT / _SEGMENT / _RESP`, `MC_IF_NODE_BOOTLOADER`).

## Purpose

Separately-linked ~26 KB binary at `0x08000000`. On reset, decides whether to jump to the app (`0x08008000`) or stay resident and serve a firmware update over UDP port 5000. Layered on the same `bsp/` used by the app.

## Owns

- Its own linker script (`boot_link.ld`): FLASH at `0x08000000` LENGTH 32 KB, RAM shared with the app.
- Its own build (`Makefile`, standalone GNU make — `make -C boot`). Produces `build/boot.{elf,bin,hex,map}`.
- The `boot_flag` reader: parses the BOOT persist blob at `0x0807D800` (page 251) using a **duplicated** copy of `app/persist`'s header + CRC32 so the bootloader doesn't have to link the whole persist module. Any change to `persist_header_t` in `app/persist/persist.c` MUST be mirrored in `boot/boot_flag.c`.
- The `boot_net_cfg` reader: parses `app/config`'s NETWORK persist blob (page 255) so the bootloader answers on the operator-configured IP. Falls back to `192.1.0.100` if the blob is invalid.
- The bootloader's own reduced OD dispatch (`boot_od.c`): handles reads of `0x1F56 program_software_id` (live CRC32 over the programmed bytes) and `0x1F57 flash_status`, writes to `0x1F51 program_control` (`STOP / START / VERIFY / COMMIT / ABORT`). Everything else returns `NO_OBJECT`.
- The segmented-SDO receiver (`boot_seg_sdo.c`): toggle-bit + last-segment state machine per `INTERFACE_SPEC.md §7c`. Rejects overlapping `INIT` with `BOOTLOADER_BUSY`.
- Flash program path (`boot_flash.c`): erases pages 16-250 (app region), streams doubleword programs from segments via `bsp/flash`, runs CRC32 verify on demand.

## Does NOT do

- Link `app/*`. The bootloader is a peer to the app, not a subset — no cross-includes.
- Own SPI or motor-side logic. The motor MCU has its own separate bootloader per REQ-0015 Phase 2.
- Preserve or migrate app state. On `PROG_COMMIT` it just jumps.
- Time-limit the download. `PROG_ABORT` (or a power cycle after the operator gives up) is how downloads end early.

## Boot flow

```
Reset → SystemInit → boot_flag_is_stay() ?
  ├── STAY   → boot_od_init (network + OD dispatch) → poll UDP until PROG_COMMIT
  └── CLEAR  → app_image_looks_valid ? jump_to_app : fall through to STAY path
```

`boot_jump_to_app` disables IRQ → deinits peripherals → clears NVIC → sets VTOR to `0x08008000` → **re-enables IRQ** (missing this hangs the app in `HAL_Delay`) → sets MSP → branches.

`PROG_COMMIT` uses the same `boot_jump_to_app` — no reset. Resetting would loop back into the bootloader (flag still STAY) → app never runs → flag never clears → deadlock. The flag clears only when the app runs successfully for 5 s (`app/boot_meta`), preserving brick-proof.

## Build

```
make -C boot         # produces boot/build/boot.elf + .bin + .hex + .map
```

Toolchain paths default to the ones the CubeIDE app build uses; override `TOOLCHAIN_BIN` on the command line if your install layout differs.
