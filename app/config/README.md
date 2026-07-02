# app/config

## Purpose
Persistent settings. Loads a versioned settings struct from flash on boot and saves it on change. Holds two independent regions: one for general config (network + limits + auth + node ID), one for the shot table.

## Owns
- The settings struct schema and version byte.
- The dual-bank layout in flash for each region: two copies, each prefaced with a sequence number and CRC. Loader picks the highest valid sequence.
- Atomic save semantics: erase target bank, program, verify, then bump sequence. A power-fail mid-save leaves the other bank intact.

## Does NOT do
- Apply settings to running peripherals — modules read from `config` when they need a value. Settings that require a reboot (e.g. IP address) are flagged at the API.
- Encrypt anything. The auth password is hashed (SHA-256 + salt); other fields are plaintext.

## Public API
```c
void config_init(void);          /* loads from flash; falls back to defaults if both banks corrupt */

/* General config */
const network_cfg_t * config_get_network(void);
bool                  config_set_network(const network_cfg_t *cfg);   /* persists, returns false on flash error */

const motor_limits_t * config_get_limits(void);
bool                   config_set_limits(const motor_limits_t *lim);

const auth_cfg_t * config_get_auth(void);
bool               config_set_auth_password(const char *new_password);

uint8_t config_get_node_id(void);
bool    config_set_node_id(uint8_t node_id);

/* Shot table — separate region */
bool config_shot_load (uint16_t shot_no, cmc_position_t *out);
bool config_shot_save (uint16_t shot_no, const cmc_position_t *pos);
void config_shot_clear(uint16_t shot_no);
```

## Dependencies
- `bsp/flash` (erase, program, read).

## Acceptance criteria
- Defaults applied cleanly on a virgin device (both banks blank).
- A `config_set_*` call followed by reboot returns the saved value.
- Power-cut mid-save (simulated) results in either old or new value on the next boot — never corruption.
- Shot writes do not affect the general config region (separate sectors).
- Sector erase count visible at runtime (lifetime wear estimate).

## Notes
- General config region: 2 sectors at the top of flash.
- Shot table region: 4 sectors above general config (100 shots × N axes × 4 bytes ≈ a few KB, well under one sector — but dual-bank doubles it).
- Version byte allows future schema migration: loader reads version first, applies field-by-field defaults for any newer fields.
- The CRC is CRC-32 over the serialised struct, excluding the CRC and sequence number bytes.
