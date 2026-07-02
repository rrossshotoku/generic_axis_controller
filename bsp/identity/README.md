# bsp/identity

## Purpose
Provide stable per-unit identity (today: MAC address; future: serial number etc.) without baking it into the firmware image. Abstracts the *source* of that identity so the source can be swapped without affecting any caller.

## Owns
- The contract: "for this physical device, return value X — same on every boot."
- One concrete implementation at a time. Today's reads the STM32 96-bit factory UID and hashes it. The intended future implementation reads from an SPI-connected identity device.

## Does NOT do
- Hold any state. Each call computes (or re-reads) afresh. Callers that need the value repeatedly cache it themselves.
- Decide *what* the identity is used for. `bsp/net` uses it for MAC; future callers may use it for other things.

## Public API
```c
void identity_get_mac(uint8_t out[6]);
```

Guarantees and costs are documented in `identity.h`.

## Dependencies
- Today: `Drivers/STM32G4xx_HAL_Driver` (`HAL_GetUIDw0..2`).
- Future: whatever SPI bus and chip the identity device sits on (likely `bsp/motor_spi` or a separate `bsp/identity_spi`).

## Acceptance criteria
- Two builds on the same physical device produce the same MAC.
- Two physically distinct STM32G431RB devices produce different MACs (verified by running on two Nucleos and comparing).
- The MAC is locally-administered (byte 0 bit 1 = 1) and unicast (byte 0 bit 0 = 0).
- Swapping the implementation file (UID → SPI source) does not require changes anywhere outside `bsp/identity/`.

## Notes
- The MAC prefix is `02:08:DC`, matching the reference uc_camd_interface project. Keeping the same prefix lets a network operator visually group Shotoku-family devices on a sniffer.
- Hash quality: XOR-folding 96 bits down to 24 has a birthday collision near ~4000 devices on one segment. If your deployment ever exceeds that scale, change the hash here — `bsp/identity` is the only place that needs to change.
- When the source moves to SPI, this module gains a one-shot caching step internally: `bsp/identity_init()` is called once at boot, reads the SPI device, and stores the result so `identity_get_mac` stays cheap.
