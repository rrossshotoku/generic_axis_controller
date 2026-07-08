# app/boot_meta

> **Wire authority:** none — this module is CMC-side only. **Design authority:** `Documentation/dual_bootloader_design.md` §5.3 (flag lifecycle) and §5.4 (jump semantics).

## Purpose

App-side half of the dual-bootloader flag handshake. Writes the persistent `STAY` magic to the BOOT persist region when the PC tool requests an update, and clears it back to `CLEAR` after the app has been running healthy for a while. Preserves the "brick-proof" property: if the newly-flashed app crashes before the healthy window elapses, the flag stays `STAY` and the next reboot re-enters the bootloader.

## Owns

- The 32-bit boot flag magic (`0xB007107D` = STAY, `0x00000000` = CLEAR). Anything else — including all-`0xFF` from a fresh page erase or a `persist_header_t` CRC mismatch — is treated as CLEAR by the bootloader on the read side.
- The "healthy" timer: `BOOT_META_HEALTHY_MS` (5000 ms). App must survive this window without a fault to earn a CLEAR write.
- The one-shot boot-flag write path on `PROG_START` (called from `app/od/cmc_od.c` when the operator writes `0x3018 cmc_boot_request = PROG_START`).

## Does NOT do

- Talk to the network or SPI directly. Uses only `app/persist` + `bsp/time` + HAL `NVIC_SystemReset`.
- Care about the motor. This handshake is exclusively for CMC-side updates. Motor updates use its own (mirrored) equivalent per REQ-0015 Phase 2.
- Modify anything on the bootloader-owned page beyond the flag payload. `boot_meta.c` and `boot/boot_flag.c` MUST keep the persist blob layout in sync.

## Public API

```c
void boot_meta_init(void);              /* read flag at boot, log its state */
bool boot_meta_flag_was_set_at_boot(void);
void boot_meta_tick(void);              /* clear STAY after 5 s of runtime */
void boot_meta_enter_bootloader(void);  /* write STAY + reset — does not return */
```

Called from `app/main_loop/main_loop.c` (init + tick) and `app/od/cmc_od.c` (enter, from the `0x3018` handler).
