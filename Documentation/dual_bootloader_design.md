# Dual Bootloader Design — CMC + Motor MCU

> **Status:** design proposal, contract Phase 1 landed. The chip-upgrade gate is
> cleared — this project runs on the **STM32G474RETX** (512 KB flash, 128 KB SRAM),
> which comfortably accommodates bootloader + app + A/B slot layout as described
> below. The motor-MCU side has no chip constraint and can be built independently
> against the same contract in §3.
>
> **Owner of this doc:** updates land in the same change as any code that implements
> any part of it. If you find the doc disagreeing with shipped code, the doc is wrong
> — fix the doc.

---

## 1. Goals

1. **Re-program either MCU over the field-deployed link** — no JTAG cable in the rack.
   - The motor MCU gets new firmware via the CMC over SPI.
   - The CMC itself gets new firmware via Ethernet from the PC tool.
2. **One mental model, one wire protocol, one toolchain.** A single "Update firmware"
   button in the PC tool picks the target; the operator doesn't have to learn two
   different update procedures.
3. **Bootloader cannot be bricked by an interrupted upload.** A failed mid-program
   leaves the device in a state where another update attempt fixes it (does NOT
   require a JTAG/SWD recovery).

## 2. Non-goals (deliberately deferred)

- **OTA over the operator panel.** The PC tool is the only authorised firmware source.
- **Cryptographic image signing.** Worth adding later but not in v1; CRC is sufficient
  for "is this image intact?" detection.
- **Background update with hot-swap.** The device reboots into the new image; no
  attempt to swap code while running.
- **Bootloader-managed configuration migration.** Persist-region format migration
  stays in the app (where it already lives via `AXIS_PERSIST_VERSION` / equivalents).

---

## 3. Shared protocol — same CiA-302 OD entries for both bootloaders

Both bootloaders speak the **same** OD-write protocol over their respective transports
(CMC: UDP / mc_if framing; motor MCU: SPI / mc_if framing). The OD entries used are
the standard CiA-302 device-profile range (`0x1F50`..`0x1F57`), making the two
bootloaders interchangeable from the PC tool's perspective.

### OD entries to add to `Interface/mc_if_od.h`

| Index    | Sub | Name                     | Type  | Access | Notes                                                                            |
|----------|-----|--------------------------|-------|--------|----------------------------------------------------------------------------------|
| `0x1F50` |  1  | `program_data`           | (n/a) | WO     | **Logical target only** — bytes flow via the segmented-download msg types below, not via a regular `MC_IF_MSG_OD_WRITE_REQ`. Listed in the OD so the entry is enumerable + ownership / access are explicit. |
| `0x1F51` |  1  | `program_control`        | U8    | RW     | State command, written via normal expedited OD write: `0x00` stop, `0x01` start download, `0x02` verify, `0x03` commit + reset, `0x80` abort. |
| `0x1F56` |  1  | `program_software_id`    | U32   | RO     | CRC32 of the **currently running** image. Echoes back so the PC can confirm what's live. |
| `0x1F57` |  1  | `flash_status`           | U16   | RO     | Bootloader state — `IDLE / ERASING / PROGRAMMING / VERIFYING / FAULT`. Polled. |

Note: the current `MC_IfOdType_t` (`U8 / U16 / U32 / I8 / I16 / I32 / F32`) deliberately
has **no DOMAIN/raw-bytes type**. Rather than extend the type enum just for
`program_data`, the segmented-download messages below carry the payload
out-of-band and reference `0x1F50:1` as the logical target. The X-macro entry
for `0x1F50:1` can keep `MC_IF_T_U8` for compatibility with the macro; no host
ever reads it as a typed value.

Owner column: deliberately **neither MOTOR nor CMC**. Add a new
`MC_IF_OWNER_BOOTLOADER = 0x02` value to `MC_IfOdOwner_t` so each MCU knows to
route `0x1Fxx` access to its bootloader-OD subset rather than its app-OD
table. The PC tool addresses the target by IP/device — the same indexes work
for either.

### Segmented SDO transfer

CiA-301 §7.2.4.3 segmented protocol. Required because:
- Expedited SDO carries ≤ 4 bytes per write — useless for KB-scale firmware images.
- Block transfer (§7.2.4.4) is faster but more complex; defer to v2 if needed.

Wire-level addition to `mc_if_protocol.h` (also extends UDP frames). New
message types **slot into the existing `MC_IfMsgType_t` enum** at the next
free values after the current OD ops (`0x10`–`0x13`):

| Value  | Name                              | Direction         | Payload                                                                                  |
|--------|-----------------------------------|-------------------|------------------------------------------------------------------------------------------|
| `0x14` | `MC_IF_MSG_OD_DOWNLOAD_INIT`      | master → slave    | target index (U16), subindex (U8), total length (U32), reserved (U8)                     |
| `0x15` | `MC_IF_MSG_OD_DOWNLOAD_SEGMENT`   | master → slave    | flags (U8: bit0 = toggle, bit1 = last-segment), reserved (U8), seg length (U8), bytes…   |
| `0x16` | `MC_IF_MSG_OD_DOWNLOAD_RESP`      | slave  → master   | toggle-ack (U8), result (U8 — `MC_IfOdResult_t`), bytes-accepted-so-far (U32)            |

Each `SEGMENT` carries up to `MC_IF_MAX_PAYLOAD - 3` bytes (≈ 49 bytes today).
Receiver acks each segment with the toggle echoed back; on toggle mismatch the
sender resends the segment. `INIT` collision (a second `INIT` while a download
is in progress) returns `MC_IF_OD_ERR_BOOTLOADER_BUSY` (see error codes
below). After the final segment is acked, the sender must issue
`MC_IF_MSG_OD_WRITE_REQ` to `0x1F51:1` (`= 0x02 verify`) to trigger CRC.

`MC_IF_PROTOCOL_VERSION` bumps to **5** with this change (wire-breaking — new
message types and a new owner value require all consumers to be rebuilt).

### Error codes

Extend `MC_IfOdResult_t` (currently `0x01`–`0x08`) with:

| Value  | Name                              | Meaning                                                              |
|--------|-----------------------------------|----------------------------------------------------------------------|
| `0x09` | `MC_IF_OD_ERR_FLASH_LOCKED`       | sector write-protected (e.g. bootloader trying to erase itself).     |
| `0x0A` | `MC_IF_OD_ERR_CRC`                | verify failed — image CRC32 mismatch.                                |
| `0x0B` | `MC_IF_OD_ERR_BOOTLOADER_BUSY`    | already in a download session; abort first or wait.                  |
| `0x0C` | `MC_IF_OD_ERR_NOT_BOOTLOADER`     | write to `0x1F5x` received by the app (not in bootloader mode).      |

### Node state addition

Extend `MC_IfNodeState_t` (currently `INIT = 0x00` .. `CALIBRATING = 0x06`):

| Value  | Name                       | Meaning                                                                                   |
|--------|----------------------------|-------------------------------------------------------------------------------------------|
| `0x07` | `MC_IF_NODE_BOOTLOADER`    | Slave is running its bootloader, not its app. Cyclic frames now address the bootloader OD subset only — the master must pause normal cyclic commands. |

---

## 4. Update sequence (identical for either MCU)

```
1. Operator picks firmware.bin in the PC tool, selects target (CMC or motor).
2. PC tool reads target's 0x1F56 program_software_id (CRC of running image).
3. PC tool computes CRC32 of the new image; if equal, abort early ("already up to date").
4. PC tool writes 0x1F51 program_control = 0x01 (start download).
   - Target acks; switches its bootloader OD to ERASING then IDLE; ready for data.
   - For the MOTOR target this is forwarded by the CMC over SPI; the motor MCU
     reboots into its bootloader; subsequent CMC<->motor SPI exchanges talk to
     the bootloader's reduced cia402-equivalent rather than the app.
   - For the CMC target this triggers a sys_reboot() into the CMC's bootloader.
5. PC tool segmented-writes 0x1F50 program_data = <image bytes>.
6. PC tool writes 0x1F51 = 0x02 (verify).
   - Bootloader CRCs the written image. Status -> VERIFYING. On success -> IDLE.
   - On CRC mismatch -> FAULT; PC tool retries or aborts.
7. PC tool writes 0x1F51 = 0x03 (commit + reset).
   - Bootloader sets the "valid app" flag, jumps to the new app (or resets, depending
     on architecture). Connection drops briefly.
8. PC tool reconnects, reads 0x1F56 again to confirm the new CRC is now running.
```

Identical 8-step procedure for both targets. The PC tool's "Update firmware" code
is one implementation that handles either.

---

## 5. CMC bootloader (network MCU)

### 5.0 W6100 socket budget — must share, not steal

The W6100 has 8 hardware sockets. The current map (see `bsp/net/README.md`):

| Slot | Owner | Reserved? |
|---:|---|---|
| 0–1 | controller_mgr (UDP POLL + TCP listen) | in use |
| 2   | controller_mgr (controller A outbound TCP) | in use |
| **3** | **controller_mgr (controller B outbound TCP)** | **RESERVED — must remain free for Phase B** |
| 4   | od.c (OD-access UDP, port 5000) | in use |
| 5   | od.c (telemetry UDP, port 5001) | in use |
| 6   | log (TCP, port 30200) | in use |
| 7   | web (HTTP, port 80) | in use |

The CMC bootloader **must not claim a dedicated socket**. Slot 3 belongs to the
second-controller deployment (1×S + 1×T panels); taking it for bootloader use
would lock the network MCU to a single-controller deployment forever.

Two viable approaches that don't burn a socket:

1. **Share `OD_ACCESS_SOCKET` (slot 4)**. The bootloader's OD entries
   (`0x1F50/51/56/57`) are dispatched by the same od.c logic the app uses
   today — bootloader binary just exposes a different OD table (only the
   `0x1F5x` range) but answers on the same UDP port (5000). PC tool sees
   "same address, same protocol, different OD" — exactly the commonality
   we want anyway (§3). **Recommended.**
2. **Use a different transport entirely**. The CMC bootloader could listen
   on a UART instead, leaving the W6100 alone. Cheaper in flash but
   contradicts the over-the-network update goal.

### 5.1 Required flash regions

```
0x0800_0000  ┌──────────────────────────  CMC bootloader        (~20 KB)
             │  - W6100 driver
             │  - bsp/net + framing
             │  - bootloader OD subset (0x1F50/1/6/7)
             │  - segmented-SDO state machine
             │  - flash programming (bsp/flash)
             │
0x0800_5000  ├──────────────────────────  App image slot(s)
             │  Layout option A: single slot, full size
             │  Layout option B: two A/B slots, each half-size
             │
       …      
       …     ├──────────────────────────  Persist regions (unchanged)
0x080?_E000  │  shots region   (4 KB)
0x080?_F000  │  config region  (4 KB)
0x080?_FFFF  └──────────────────────────  FLASH end
```

The bootloader region MUST be:
- Outside the persist regions.
- Sized to a multiple of the MCU's flash-sector granularity.
- Write-protected by `OPTR.WRP1A_STRT/END` once committed (so the app can't accidentally erase the bootloader).

### 5.2 Module structure

```
boot/                     ← new top-level folder (parallel to app/)
  main.c                  - bootloader entry; trigger detection; jumps to app or stays
  boot_od.c               - reduced OD dispatch (only 0x1F5x)
  boot_seg_sdo.c          - segmented-SDO state machine
  boot_flash.c            - sector erase + word program (uses bsp/flash)
  boot_link.ld            - linker script: this binary lives at 0x0800_0000
```

`boot/` LINKS against shared modules — same `bsp/net`, same `bsp/flash`, same
`bsp/time`, same `Drivers/w6100/`. Bootloader's OD dispatch is minimal (just
`0x1F50/1/6/7`); axis_manager, cmc_state, web, controller_mgr are all absent.

### 5.3 Trigger detection (app → boot) — IMPLEMENTED as flash marker

**Design changed during implementation.** The RAM-marker + alive-flag-fallback
sketch above was rejected in favour of a single persistent flash marker.
Reasons:
- A power cycle mid-update loses a RAM marker → chip boots into a
  partially-flashed app → possibly bricked. A flash marker survives.
- Collapses "we asked for an update" + "is the app healthy?" into one signal;
  no separate alive-flag counter needed.

Layout:
- Dedicated 2 KB persist page (`PERSIST_REGION_BOOT`) at `0x0807D800` on the CMC.
  Holds a `persist_header_t` (16 B: PRST magic + version + payload_size +
  CRC32) followed by a 4-byte payload.
- Payload magic: `0xB007107D` = STAY, `0x00000000` = CLEAR. Anything else
  (including `0xFFFFFFFF` from a fresh erase, or a CRC mismatch) is treated
  as CLEAR — the bootloader falls through to jump-to-app.

Lifecycle:
1. App receives `0x1F51:1 = MC_IF_PROG_START` → app writes STAY + `NVIC_SystemReset()`
   (`app/boot_meta.c:boot_meta_enter_bootloader`).
2. Bootloader reads STAY on boot → serves firmware update. **Never writes the flag.**
3. After PROG_COMMIT, bootloader jumps to app (see §5.4).
4. App runs healthy for `BOOT_META_HEALTHY_MS = 5000` ms → app writes CLEAR
   (`app/boot_meta.c:boot_meta_tick`).

Brick-proof property: if the new app crashes before the 5 s window elapses,
the flag stays STAY. Next reboot re-enters bootloader → operator retries.

### 5.4 Jump to app — IMPLEMENTED, notes below

On PROG_COMMIT (or on boot with no boot-trigger):
- Validate app image header — pragmatic check: the initial stack pointer at
  `APP_FLASH_BASE + 0` must be a plausible RAM address (in `0x2000_0000` range).
  `boot/main.c:app_image_looks_valid`. No CRC check on jump-to-app itself; the
  bootloader's PROG_VERIFY step covers image integrity for the just-uploaded
  case, and a bad post-COMMIT jump falls into the app-fails-to-clear-the-flag
  path which drops us back into bootloader on next reset (brick-proof).
- Sequence in `boot/main.c:boot_jump_to_app`:
  1. `__disable_irq()` + `HAL_DeInit()` + clear all `NVIC->ICER/ICPR`.
  2. Set `SCB->VTOR` to `APP_FLASH_BASE` **before** re-enabling interrupts
     so any that fire during app early-init route to the app's handlers.
  3. `__enable_irq()` — critical. Missing this hangs the app in `HAL_Delay`
     (SysTick never fires with PRIMASK=1). Verified empirically during
     first-boot bring-up.
  4. `__set_MSP(app_sp)` + branch to `app_entry`.

**PROG_COMMIT is NOT a reset.** Do not `NVIC_SystemReset()` on commit — the
flag is still STAY, so a reset loops back into the bootloader and the app
never gets a chance to clear the flag. Jump directly. Details in §5.3.

### 5.5 Sharing code with the app

Hard requirement to keep both within total flash budget — `bsp/net` + `Drivers/w6100/`
together are ~10 KB. Don't ship two copies.

**Recommended approach**: build the BSP modules into a small static library
`libcmc_bsp.a` that both `boot/` and `app/` link. The toolchain handles
deduplication; only the symbols each binary actually references are pulled in.

Slightly trickier alternative (smaller still): position-independent BSP code at a
known address, jump table at the start of the bootloader region. App calls BSP
through the jump table. Saves ~5 KB but adds calling-convention complexity. Skip
unless flash is desperate.

---

## 6. Motor-MCU bootloader (Generic_motor_controller)

Owned by the motor-MCU codebase, NOT this project — design here is the CMC's view
of the contract.

### 6.1 What the motor MCU needs to add

Mirror of §5 but over SPI rather than UDP:
- Minimal bootloader at `0x0800_0000` that speaks the **same** mc_if_protocol framing.
- OD dispatch for `0x1F50/1/6/7` only.
- Segmented-SDO receive side.
- Flash programming for the motor MCU's chip.

### 6.2 What the CMC has to do for motor updates

**Pass-through.** The CMC's segmented-SDO transfer logic already handles writes to
arbitrary OD entries (via `cia402_od_write_begin`). The bootloader OD entries route
the same way:
- App-side cia402: when motor MCU's `node_state` indicates BOOTLOADER, the CMC
  knows it's talking to the bootloader rather than the app. The cyclic-frame
  scheduler should pause normal commands and feed only the segmented-SDO writes
  through.
- A new motor-MCU `node_state` value `MC_IF_NODE_BOOTLOADER` makes this explicit;
  add it to `mc_if_protocol.h` (additive, no version bump on its own — bundled
  with the segmented-SDO change anyway).

### 6.3 The CMC's "I'm updating the motor" UI

The web page already has the slider for load factor; the motor-update flow needs:
- A file upload (`POST /api/motor_firmware`) — the new image lands in a CMC RAM
  buffer (limited by RAM; an 80 KB image won't fit in the G431RB's 32 KB RAM).
- **Stream-through** rather than buffer-then-flash: as the HTTP body arrives, the
  CMC immediately segmented-SDO-writes it down to the motor. No RAM buffering
  beyond a few segments.
- Progress bar polls the motor's `0x1F57 flash_status` to show ERASING / PROGRAMMING
  percentages.

---

## 7. A/B layout decision — DECIDED: single slot for v1

**Status: closed 2026-07-07. Single slot.** The brick-proof property we needed
turned out to be delivered by the flag mechanism itself (§5.3): a failed
update leaves the flag STAY, so next boot re-enters bootloader → retry. The
A/B rollback machinery isn't needed for that recovery path.

A/B remains an option for a future v2 if we ever want *update-while-running*
semantics (currently out of scope — see §2 non-goals). Until then, the single
slot buys us a bigger app budget with no operational downside.

CMC flash layout as committed:
```
0x08000000..0x08007FFF  (32 K)  bootloader
0x08008000..0x0807D7FF  (470 K) app
0x0807D800..0x0807DFFF  ( 2 K)  BOOT persist (the flag — §5.3)
0x0807E000..0x0807EFFF  ( 4 K)  SHOTS persist
0x0807F000..0x0807F7FF  ( 2 K)  CONFIG persist
0x0807F800..0x0807FFFF  ( 2 K)  NETWORK persist
```

---

## 8. Implementation roadmap (when prioritised)

Phase 1 — contract:
- Add `0x1F50/1/6/7` to `mc_if_od.h`.
- Add segmented SDO message types + version bump to `mc_if_protocol.h`.
- Bump `MC_IF_PROTOCOL_VERSION` to 5.
- File the change as a REQ; both MCU teams + the PC tool consume in lockstep.

Phase 2 — CMC bootloader (after chip upgrade):
- Bootloader binary in `boot/`.
- `bsp/flash` extended with sector-erase + word-program APIs (mostly there).
- Linker script with bootloader region.
- App integration: respond to `0x1F51:1 = 0x01` by setting marker + rebooting.
- PC tool: "Update CMC firmware" button + HTTP upload + progress polling.

Phase 3 — Motor bootloader:
- Done in the Generic_motor_controller codebase per the same contract.
- CMC pass-through code: handle the new node_state, route segmented-SDOs straight
  through during update, suppress normal cyclic commands.
- PC tool: "Update motor firmware" button — same flow, different IP/owner.

Phase 4 — Safety hardening (after v1 ships):
- Image signing (Ed25519 or similar).
- Persist-region migration support for app updates that change the persist layout.
- A/B with rollback if v1 was single-slot.

---

## 9. Open questions — all resolved by 2026-07-07 (Phase 2 bring-up)

1. **Chip upgrade target.** RESOLVED — STM32G474RETX (512 KB, same package). Confirmed by port + subsequent bring-up.
2. **A/B vs single-slot.** RESOLVED — single slot (§7). Brick-proof property comes from the flag mechanism instead.
3. **PC tool implementation language for the segmented-SDO sender.** RESOLVED — Python. Landed as `Interface/gui/mc_gui/firmware_update.py`.
4. **In-app trigger mechanism.** RESOLVED — dedicated flash marker in `PERSIST_REGION_BOOT` (§5.3). Rejected the RAM marker + alive-flag combo because a flash marker gives us power-cycle safety and collapses the "asked for update" and "app is healthy" checks into one signal.

---

## 10. References

- CiA-301 §7.2.4 — SDO download protocols (expedited / segmented / block).
- CiA-302-3 — bootloader profile (device, controller).
- AN3155 (ST) — STM32 bootloader reference for UART/SPI/I2C (their official
  bootloader uses similar building blocks; useful as a reference even though
  we're rolling our own).
- This project's `Interface/INTERFACE_SPEC.md` — the framing layer the bootloader
  reuses.
- This project's `Documentation/architecture.md` — the broader system layout.
