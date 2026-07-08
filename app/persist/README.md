# app/persist

> **Wire authority:** none — CMC-local. **Storage authority:** `STM32G474RETX_FLASH.ld` reserves the top 10 KB of flash (bank 2 pages 251-255) for the four persist regions.

## Purpose

Versioned, CRC-verified persistent-storage layer for the CMC's own state. Sits on top of `bsp/flash`. Each caller (`axis_manager`, `cmc_state` shots, `config` network settings, `boot_meta` boot flag) picks a region, hands over an opaque byte blob, and gets it back later — or a `false` return if the flash blob is missing, wrong version, wrong size, or fails CRC.

## Owns

- The four persist regions and their fixed flash addresses:
  - `PERSIST_REGION_BOOT`   — page 251 (`0x0807D800`, 2 KB) — `boot_meta` STAY/CLEAR flag
  - `PERSIST_REGION_SHOTS`  — pages 252-253 (`0x0807E000`, 4 KB) — `cmc_state` shot table
  - `PERSIST_REGION_CONFIG` — page 254 (`0x0807F000`, 2 KB) — `axis_manager` calibration + limits
  - `PERSIST_REGION_NETWORK` — page 255 (`0x0807F800`, 2 KB) — `config` IP / netmask / gateway / device_no
- The on-flash header format: 16-byte `persist_header_t` (magic `PRST`, `version`, `payload_size_bytes`, CRC32) followed by the payload zero-padded up to the region's page count.
- The bytewise CRC32 (IEEE 802.3, poly `0xEDB88320`) — deliberately small and self-contained; no lookup table. `boot/boot_flag.c` and `boot/boot_net_cfg.c` duplicate the same implementation to stay independent of app/persist.

## Does NOT do

- Own the payload schema. Callers are responsible for the struct layout inside their payload. Persist just checks header integrity and hands bytes back.
- Migrate old blobs. A `version` mismatch causes `persist_load` to return false; caller falls through to hardcoded defaults. That's the migration policy — a version bump is a hard reset of the region.
- Speculatively cache. Each `persist_load` / `_save` reads/writes flash directly.

## Public API

```c
void persist_init(void);
bool persist_load(persist_region_t region,
                  void *out, size_t out_cap, uint16_t version,
                  size_t *out_size);
bool persist_save(persist_region_t region,
                  const void *payload, size_t size, uint16_t version);
```

`persist_save` blocks for ~30 ms per page erased + programmed. Not for hot paths.
