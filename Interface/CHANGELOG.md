# Interface Contract — Change Log

This is the **authoritative change history** for the shared inter-MCU boundary contract
(`mc_if_protocol.h`, `mc_if_od.h`, `INTERFACE_SPEC.md`, `NETWORK_UDP_SPEC.md`).

> **Governance:** the files in this folder are a FROZEN CONTRACT shared by the motor-control
> MCU, the network MCU, and the PC tool. **Any change here must be (1) announced explicitly to
> the user and (2) recorded as an entry below**, so every consumer can update in lockstep. A
> change to the **wire format or OD layout** also bumps `MC_IF_PROTOCOL_VERSION` in
> `mc_if_protocol.h`. Each entry states what changed, the version impact, and **which
> consumers must update**.

## Format
```
## [version] - YYYY-MM-DD  (wire-breaking? yes/no)
### Changed / Added / Removed
- ...
### Consumers to update
- motor-control MCU: ...
- network MCU: ...
- PC tool: ...
```

---

## [5.2.0] - 2026-07-08  (wire-breaking? no — additive CMC-owned OD entries)

### Added
- **`0x3006 axis_active_operation`** (U8 RO, CMC-owned): the currently-active operation family per the CMC's operation-level arbiter — `MC_IF_OP_NONE / _HOMING / _SHOT_RECALL / _JOYSTICK`. Same-family requests retarget through, cross-family requests are rejected until the current op completes (or STOP is issued). Exposed for observability so the PC tool + panel can show *why* a command was rejected. See `app/axis_manager` for the state machine.
- **`0x3018 cmc_boot_request`** (U8 WO, CMC-owned): dedicated trigger for CMC-side bootloader entry. Writing `MC_IF_PROG_START (0x01)` sets the CMC's persistent BOOT_STAY flag and calls `NVIC_SystemReset()`. Frees `0x1F51:1` (`program_control`) to route unambiguously to the **motor's** bootloader via CMC-side SPI pass-through — the old app-side `0x1F51:1 = PROG_START` intercept has been removed. See `dual_bootloader_design.md` §5.3 for the flag lifecycle.

### Changed (CMC-side behaviour — no wire change)
- **`0x1F5x` reads/writes in CMC app mode now route to the motor** via `cia402_od_*_begin`. Previously the CMC's app-side OD dispatcher returned `NOT_BOOTLOADER` locally. This enables PC-tool → CMC → motor pass-through for the motor MCU's own bootloader OD (per REQ-0015 Phase 2).
- **`MC_IF_MSG_OD_DOWNLOAD_INIT / _SEGMENT` frames received by the CMC on port 5000 are now forwarded to the motor over SPI** via a new `cia402_raw_passthrough` API — the motor's `DOWNLOAD_RESP` is forwarded back to the PC verbatim. Unchanged wire from either endpoint's perspective; the CMC becomes a transparent SPI hop.
- **CMC pauses cyclic commands (emits HEARTBEATs instead)** when it sees `node_state == MC_IF_NODE_BOOTLOADER` in the motor's cyclic status. Bootloader doesn't handle CiA-402 semantics so we suppress the commands rather than confuse it.

### Consumers to update
- **motor-control MCU**: none — both OD entries are CMC-owned; the pass-through changes are internal to the CMC's dispatcher. Motor bootloader continues to receive the same `0x1F5x` and `DOWNLOAD_*` frames it already implements per REQ-0015.
- **network MCU** (this project): implemented in commits `91406e4` (contract + dispatch + arbiter) and its parents.
- **PC tool**: firmware-update dialog now takes a target selector (CMC vs Motor MCU); wire messages are unchanged.

---

## [5.1.0] - 2026-07-03  (wire-breaking? no — additive motor-owned OD entries)

### Added
- **`0x2600:11/12/13 fault_count_no_config / fault_count_not_homed / fault_count_overcurrent`** (U16 RO, motor-owned): per-fault **since-boot trigger counts** — how many times each `fault_flags` bit has *risen* (saturating, RAM only). Feeds the PC-tool Diagnostics panel (fault-counts row + a manual **Refresh** button). ADR-058.
- **`0x2700:10 mech_zero_set_rad`** (F32 RW transient, motor-owned) + cal command **`MC_IF_CAL_SET_MECH_ZERO_AT`** (`0x2700:1 = 4`): set the mechanical zero to a *commanded* value (not just the current position), so the PC tool can capture both travel extremes (`0x2510:1`) and centre the zero on their midpoint without driving the axis there. ADR-060.
- **`0x2300:10 jog_position_mode`** (U8 RW PERSIST, motor-owned): `0` = joystick / PROFILE_VELOCITY runs the velocity loop directly (default, unchanged); `1` = the motor integrates the velocity setpoint into a position reference and runs the position cascade, so following-error / soft-limit / stiff-hold protection apply while jogging. **Motor-side only** — the CMC still streams `velocity_setpoint`, wire unchanged. ADR-062.

### Consumers to update
- **motor-control MCU**: implemented (motor-owned).
- **network MCU**: none — motor-owned, forwarded as transport; no rebuild for wire compatibility.
- **PC tool**: auto-appears via the X-macro parser; Diagnostics fault-counts + "Centre mech zero" UI added.

---

## [5.0.0] - 2026-07-03  (wire-breaking? YES — dual-bootloader contract adds messages + result codes + node state + owner)

**Phase 1 (contract only)** of the CMC + motor-MCU dual-bootloader design. See `Documentation/dual_bootloader_design.md` for the full architecture; this changelog entry describes only the wire additions. No CMC or motor firmware yet implements bootloader mode — this commit is `.h`-only and produces the shared vocabulary the two firmwares + PC tool will consume.

`MC_IF_PROTOCOL_VERSION` bumps **4 → 5**. Every consumer must recompile.

### Added (OD entries — all owner `MC_IF_OWNER_BOOTLOADER`)
- **`0x1F50:1 program_data`** (U8 WO) — logical firmware-bytes sink. Not written via expedited SDO; bytes arrive via new segmented-SDO messages.
- **`0x1F51:1 program_control`** (U8 RW) — state command: `0x00 stop`, `0x01 start`, `0x02 verify`, `0x03 commit+reset`, `0x80 abort` (`MC_IF_PROG_*`).
- **`0x1F56:1 program_software_id`** (U32 RO) — CRC32 of currently running image; used by the PC tool for "skip if already up-to-date" + post-reboot confirmation.
- **`0x1F57:1 flash_status`** (U16 RO) — bootloader state enum: `IDLE / ERASING / PROGRAMMING / VERIFYING / FAULT` (`MC_IF_FLASH_*`).

### Added (message types — CiA-301 §7.2.4.3 segmented download)
- `MC_IF_MSG_OD_DOWNLOAD_INIT` (0x14) master→slave: opens session with target index/sub + total length.
- `MC_IF_MSG_OD_DOWNLOAD_SEGMENT` (0x15) master→slave: one chunk, flags (toggle bit + last-segment bit) + payload. Segments carry up to `MC_IF_MAX_PAYLOAD − 3` bytes (~49 today).
- `MC_IF_MSG_OD_DOWNLOAD_RESP` (0x16) slave→master: toggle-ack + result + bytes-accepted-so-far.

### Added (owner + node-state)
- `MC_IF_OWNER_BOOTLOADER = 0x02` — third owner value in `MC_IfOdOwner_t`. Apps filter these out of their local OD table; bootloaders serve them.
- `MC_IF_NODE_BOOTLOADER = 0x07` — appears in cyclic status when the slave is in its bootloader. Master must pause normal cyclic commands and only issue bootloader traffic.

### Added (result codes — extend `MC_IfOdResult_t`)
- `0x09 MC_IF_OD_ERR_FLASH_LOCKED` — sector write-protected (e.g. bootloader tried to erase itself).
- `0x0A MC_IF_OD_ERR_CRC` — verify failed, image CRC32 mismatch.
- `0x0B MC_IF_OD_ERR_BOOTLOADER_BUSY` — already in a download session; abort first.
- `0x0C MC_IF_OD_ERR_NOT_BOOTLOADER` — write to `0x1F5x` received while running the app.

### Consumers to update
- **motor-control MCU**: no bootloader implementation yet, but must rebuild against the new header (new enum values change enum size on some compilers; app-side must return `MC_IF_ERR_UNKNOWN_MSG` for the new download message types).
- **network MCU** (this project): no bootloader implementation yet — Phase 2. App must return `MC_IF_ERR_UNKNOWN_MSG` on the new msg types (already does via default dispatch). Rebuild required.
- **PC tool**: rebuild required. The three new download message types + four new OD entries auto-appear in the OD browser via the X-macro parser. Firmware-updater UI is future work (Phase 4).

---

## [4.10.0] - 2026-07-02  (wire-breaking? no — additive: CMC-owned CAMERAD axis role selector)

CMC-owned addition. Every CAMERAD MOVEMENT frame carries all 8 axis values (pan/tilt/zoom/focus/x/y/height/fader). Previously the CMC hardcoded consumption of `mv.pan`. `axis_role` (0x3070, U8, RW, PERSIST, MC_IF_OWNER_CMC) now selects which field the CMC pulls out on every frame. Values mirror the existing `CAMERAD_AXIS_*` bitmap: `PAN 0x01, TILT 0x02, ZOOM 0x04, FOCUS 0x08, X 0x10, Y 0x20, HEIGHT 0x40, FADER 0x80`. Default `PAN` for backwards compatibility. Setter rejects 0 and multi-bit values.

`axis_bitmap` in the MOVEMENT frame is DELIBERATELY still ignored — a future change will gate frame consumption on it. Zero-value axis fields still flow through so releasing the stick drives the axis to a stop.

### Persist
- Adds one byte to `axis_persist_blob_t` — `AXIS_PERSIST_VERSION` bumped **4 → 5**. Boards with v4 blob fall through to coded defaults on first boot (joystick cal, motion limits, LED colour, and now axis_role all reset to defaults). Operator saves once to write the new v5 layout.

### Consumers to update
- **motor-control MCU**: nothing — CMC-owned entry.
- **network MCU** (this project): implemented. `axis_manager` owns `axis_role` state + persist field; `controller_mgr` `handle_movement` picks the field per role.
- **PC tool**: CMC Setup page gets a new "CAMERAD axis role" group with the 0x3070 row. Web UI gets a dedicated "Axis role" section with a dropdown (Pan/Tilt/Zoom/Focus/X/Y/Height/Fader) and an Apply button.

---

## [4.9.0] - 2026-07-01  (wire-breaking? no — additive: CMC-side home-to-endstop orchestration + shot-recall gate)

CMC-owned surface added on top of the motor's existing homing entries (0x2700:8/9 + `NOT_HOMED` bit in `0x2600:1` fault_flags — motor MCU unchanged). `axis_manager` runs the sequence: SDO-writes motor 0x2700:8=1, polls motor 0x2700:9 every 200 ms for `MC_IF_HOME_DONE` / `FAILED`, then re-reads motor `fault_flags` to refresh the local `is_homed` cache. Shot recalls in `cmc_state_move_to_shot` are now GATED on `is_homed`: a stored position on an un-homed incremental encoder is meaningless, so recalls are rejected with a warning until the operator has run the procedure once. Fresh `fault_flags` is also read once at first tick after boot so the gate has a real answer before the first move attempt.

### Added (OD entries)
- **`0x3040 axis_home_command`** (U8 WO, `MC_IF_OWNER_CMC`): write 1 to start the home sequence via axis_manager. Not persisted.
- **`0x3041 axis_home_status`** (U8 RO, `MC_IF_OWNER_CMC`): mirrors the motor's `MC_IF_HOME_*` (IDLE/RUNNING/DONE/FAILED). Sticky between runs.
- **`0x3042 axis_is_homed`** (U8 RO, `MC_IF_OWNER_CMC`): 1 if motor `fault_flags & NOT_HOMED == 0` AND we've successfully read fault_flags at least once. `cmc_state_move_to_shot` reads this before every recall.

### Consumers to update
- **motor-control MCU**: nothing — the motor's own home entries (0x2700:6/7/8/9) and the `NOT_HOMED` fault bit already exist and are unchanged.
- **network MCU** (this project): implemented — `axis_manager` runs the sequencer (`tick_home_sequencer`) via the cia402 OD pipeline, exposes `axis_manager_request_home` / `axis_manager_get_home_status` / `axis_manager_is_homed`; `cmc_od` dispatches 0x3040/1/2; `cmc_state_move_to_shot` rejects when `!is_homed`.
- **PC tool**: no code change required — the new entries auto-appear in the OD browser via the X-macro parser. A future PC-tool UX pass may want a dedicated "Home axis" button that writes 0x3040:0 = 1 and watches 0x3041:0.

---

## [4.8.0] - 2026-07-01  (wire-breaking? no — access-tightening on existing CMC entry + CAMERAD JOY_PROFILE wiring)

CMC-side re-plumbing of joystick full-scale velocity. `axis_joystick_max_velocity` (0x3022) is no longer operator-settable and no longer independently persisted — it is DERIVED from `axis_velocity_limit` (0x3030) × the currently-selected joystick speed profile. CAMERAD `JOY_PROFILE_NORMAL/MEDIUM/FINE` keypresses (previously ack-only stubs) now scale the profile:

- `NORMAL` → 1.00 × velocity_limit (full stick reaches the motion ceiling)
- `MEDIUM` → 0.50 × velocity_limit
- `FINE`   → 0.15 × velocity_limit

### Changed (access)
- `0x3022 axis_joystick_max_velocity`: **MC_IF_A_RW → MC_IF_A_RO** and PERSIST flag dropped. Writes return `MC_IF_OD_ERR_ACCESS`. Reads still work (report the current derived rad/s). Boot default: velocity_limit × 1.0 (profile = NORMAL, not persisted).

### Migration
- The persist blob layout is UNCHANGED (v4). The `joystick_max_velocity` slot in the blob is now IGNORED on load and populated with the current derived value on save. So operators keep their calibration, motion limits, and LED colour across the upgrade — only the standalone max-velocity slider goes away. Rolling back to 4.7.x reads the stale derived value from the slot; no data loss either direction.

### Consumers to update
- **motor-control MCU**: nothing — CMC-owned entry.
- **network MCU** (this project): implemented. `axis_manager` derives via `recompute_joystick_max_velocity()`; `controller_mgr` routes `JOY_PROFILE_*` keypresses; `cmc_od` rejects writes to 0x3022; web page + `/api/config` drop the `max_velocity` field; new public API `axis_manager_set_joy_profile(uint8_t)`.
- **PC tool**: the 0x3022 row is removed from the CMC Setup page. OD browser still lists it (now as RO).

---

## [4.7.0] - 2026-06-30  (wire-breaking? no — additive: CMC-owned auto-fault-clear diagnostic counter)

CMC-only addition. `axis_auto_fault_clears` (0x3014, U16, RO, MC_IF_OWNER_CMC) — since-boot count of motor faults the CMC auto-cleared. The CMC now watches `AXIS_STATE_FAULT`; if it persists for ≥ 5 s the CMC fires `clear_fault_pulse` itself and increments the counter. Re-arms every 5 s while the fault remains, so a non-clearable fault shows up as a steadily increasing counter. Not persisted (since-boot diagnostic). No motor MCU change, no `MC_IF_PROTOCOL_VERSION` bump. Read via SDO 0x3014:0 or in the PC tool / web UI like any other RO entry.

---

## [4.6.0] - 2026-06-26  (wire-breaking? no — additive: CMC RGB status LED)

CMC-only addition. The on-board RGB LED (PC0/PC1/PC2 → TIM1_CH1/2/3 PWM) is now driven by an `app/led_indicator` state machine with operator-tunable colour. Patterns: solid-on at boot for 3 s; 3-flash on network-link-up; breathing (1 s up + 1 s down, triangular ramp) while motor reports moving; solid otherwise. No motor MCU change, no `MC_IF_PROTOCOL_VERSION` bump.

### Added (OD entries)
- `0x3060 led_color_r`, `0x3061 led_color_g`, `0x3062 led_color_b` (U8 RW PERSIST, `MC_IF_OWNER_CMC`). Magnitude 0–255 per channel. Default magenta (128, 0, 128) so an unconfigured unit is visibly alive on first boot.

### Persist
- LED colour rides the axis_persist blob — `AXIS_PERSIST_VERSION` bumped **3 → 4**. The CONFIG persist region is the only one with layout headroom (other regions are at fixed-size structs); rather than spin up a 4th persist region for 3 bytes, the axis blob co-locates it. Boards with v3 blob in flash fall through to coded defaults on first boot after this change.

### Consumers to update
- **motor-control MCU**: nothing — CMC-owned entry.
- **network MCU** (this project): implemented — new `bsp/leds/` (TIM1 PWM wrapper), new `app/led_indicator/` (state machine), `cmc_od` dispatches `0x3060/61/62`, persist co-located in axis blob.
- **PC tool**: implemented — "Indicator LED" section in the Motor Config tab with R/G/B sliders, live colour swatch, "Read LED" + "Save to flash" buttons. New entries auto-appear in the OD browser via the X-macro parser.

---

## [4.5.0] - 2026-06-26  (wire-breaking? no — additive + access change: backend select, current tuning, R/L→derived gains)

Motor-owned additions for the brushed-DC backend (ADR-039) and current-loop tuning (ADR-030).

### Added (OD entry)
- **`0x2000:6 motor_backend_sel`** (U8, RW, PERSIST, motor-owned): drive backend — `0` = BLDC/PMSM FOC (3-shunt), `1` = brushed-DC H-bridge. Per-board; the motor reads it **once at boot** to pick the dispatch path and the current-sense ADC channel, so a change takes effect on **save + reboot**.
- **`0x2400:8 hb_cur_bandwidth`** (F32, RW, PERSIST, motor-owned): brushed current-loop bandwidth ωc [rad/s]. With **`0x2000:3/4 motor_resistance / motor_inductance`** (now *applied config* — no longer mirrored from the model, so writes stick), the motor **derives** the PI gains `kp = ωc·L, ki = ωc·R`.
- **`0x2410:6 tlm_i_arm_a`** (F32, RO, PDO, motor-owned): measured armature current — live Motor-Config readout + graphable.
- **`0x2600:4/5 max_velocity_rad_s / max_accel_rad_s2`** (F32, RW, PERSIST, motor-owned): the motor's **enforced** motion envelope (ADR-040). The motor clamps every move + the velocity demand to these regardless of the CMC's requested profile (`0x6081/3/4`). `0` = disabled (default; set per-board).
- **`0x2600:6/7 pos_limit_lo_rad / pos_limit_hi_rad`** (F32, RW, PERSIST, motor-owned): **manually-set** soft position limits (home-relative rad, like `0x6064`). The motor clamps move targets and **decelerates the velocity demand to a stop *at* them** (ADR-043 — via the `max_accel` envelope `0x2600:5`; hard-stop fallback if `max_accel = 0`), always allows motion back out of the zone, and sets the `AT_LIMIT_LO/HI` movement-status bits. **Enforced only once the mechanical zero is set** (the band is home-relative); `lo >= hi` = disabled (default). Manually set — not auto-populated by homing.
- **`0x2600:8/9 max_jerk_rad_s3 / traj_use_scurve`** (F32 + U8, RW, PERSIST, motor-owned): jerk-limited **S-curve trajectory planner** (ADR-045). `max_jerk` [rad/s³] is the fixed system jerk; `traj_use_scurve = 1` runs PROFILE_POSITION moves through the S-curve planner (peak velocity/accel solved per move within `max_velocity`/`max_acceleration`, time extended if too short to stay within limits), `0` = the trapezoidal planner (default). Motor-internal planner selection — the CMC forwards these like any motor entry (no CMC action; its OD bridge only hand-handles its own 0x30xx entries).
- **`0x2900:5 dac_source`** (U8, RW, not persisted, motor-owned): selects the signal on the debug DAC (PA4 / DAC1_OUT1) for scoping — `0=|iq| 1=iq 2=|id| 3=id 4=ia 5=ib 6=ic 7=i_max 8=i_arm`, scaled 1 V/A. Commissioning/scope aid; the CMC forwards it like any motor entry (no CMC action).
- **`0x2900:6/7/8/9 dq_test_voltage_v / dq_test_angle_rad / dq_test_enable / dq_test_dwell_ms`** (F32/F32/U8/U16, RW, not persisted, motor-owned): GUI-fireable **open-loop d-axis voltage pulse** for plant identification (ADR-046). `dq_test_enable=1` applies `dq_test_voltage_v` (clamped ±12 V ≈ Vbus/2) at `dq_test_angle_rad` open-loop — no current loop — for `dq_test_dwell_ms`, then returns to 0 and disarms. For R/L ID (`τ=L/R`; R via a V–I sweep — dead time biases a single point). Motor-level commissioning; the CMC forwards it like any motor entry (no CMC/axis_manager involvement). **`0x2900:10 dq_test_axis`** selects the drive node: `0` = d-axis (FOC SVPWM, holds — R/L ID), `1` = q-axis (FOC SVPWM, torque axis), `2` = brushed_phase (open-loop H-bridge armature voltage) — so the open-loop tester works on both the FOC and brushed-DC backends.
- **`0x2500:7 est_obs_filter_alpha`** (F32, RW, PERSIST, motor-owned): the position-tracking **observer's output low-pass coefficient** (0–1, first-order at the 1 kHz estimator rate; default 0.3 ≈ 57 Hz). Previously hardcoded — now tunable. **NOTE:** this, not `velocity_filter_hz` (`0x2500:2`), is the filter on the velocity the loops use when `use_observer = 1`; `velocity_filter_hz` shapes only the finite-difference fallback. (ADR-003)
- **`0x2920:1-9 freq_sweep_*`** (F32/U8, RW + RO, not persisted, motor-owned): **stepped-sine current-injection sweep** for resonance / frequency-response ID (ADR-047). `enable = 1` injects `bias + amplitude·sin(2πf t)` as iq (torque/current mode), stepping `f` from `start_hz` to `end_hz` by `step_hz`, holding `dwell_s` at each, then auto-stops. Generated in the **20 kHz fast loop** (clean to ~200 Hz). RO: `current_hz` (`:8`, PDO-mappable — graph it vs `0x2310:2` velocity to find resonances), `active` (`:9`). Motor-internal; the CMC forwards it like any motor entry (no CMC action).
- **`0x2930:1-3 notch_enable / notch_freq_hz / notch_bandwidth_hz`** (U8/F32, RW, PERSIST, motor-owned): **band-reject (notch) on the velocity-loop current command** for resonance suppression (ADR-048). `notch_enable = 1` applies a 2nd-order notch centred on `notch_freq_hz` with `notch_bandwidth_hz` −3 dB width (default **off**, 55 Hz / 30 Hz ≈ 40–70 Hz). It's on the *command* (not the voltage output) so the current loop tracks the notched reference. Motor-internal; the CMC forwards it like any motor entry (no CMC action).
- **`0x2400:6/7 hb_cur_kp / hb_cur_ki` → RW + PERSIST**, and **`0x2400:8 hb_cur_bandwidth` removed** (ADR-049): the brushed armature-current PI gains are now **set directly** (hand-tuned), not derived from a bandwidth + R/L. Non-PDO, no `MC_IF_PROTOCOL_VERSION` bump; CMC unaffected; old saved `0x2400:8` records restore harmlessly (ignored). **Consumers (PC tool): re-sync — the bandwidth field is gone, kp/ki are now editable.**
- **`0x2510:4 quad_encoder_count`** (I32, RO, PDO, motor-owned): raw **TIM2 quadrature count** (4×/TI12, signed around the power-on zero) — diagnostic read for the new quad encoders on PA15/PB3 (ADR-050). The encoder is now started at boot. Additive; the CMC forwards it like any motor entry.
- **`0x2600:1 fault_flags` bit 0 = `MC_IF_FAULT_NO_CONFIG`** (new bit semantic, ADR-051): set when no valid persistent config is loaded at boot; the motor's **operational drive is inhibited** until a valid config loads or is saved (commissioning/align still work). Bit definition on the existing entry (not a new entry) → no `MC_IF_PROTOCOL_VERSION` bump; CMC unaffected. **Consumers (PC tool): may decode the bit for display.**
- **`0x2500:8 quad_counts_per_rev`** (F32, RW, PERSIST, motor-owned): incremental quad-encoder scale (signed = 4× lines; the sign sets count direction) for the brushed axis. Feeds the quad count through the shared state estimator → **closes the brushed velocity loop** (ADR-052). Additive; CMC unaffected. Velocity-loop gains (`0x2300`) reused unchanged.
- **`0x2300:9 holding_enable`** (U8, RW, PERSIST, motor-owned): **boolean — not a current** (the PI provides whatever current is needed; this just toggles holding on/off). `0` = release the held current ~1 s after the axis settles at zero velocity (velocity mode; parks the integrator, bumpless resume; stays released until a non-zero command); `1` (default) = always hold. For self-locking actuators that don't need holding current (ADR-054; **revised from F32 `holding_current_a`** — re-sync the PC tool, type+name changed). Additive; CMC unaffected.
- **`0x2700:6/7/8/9 home_velocity_rad_s / home_current_a / home_command / home_status`** (F32/F32/U8/U8; :6/:7 RW PERSIST, :8 RW, :9 RO, motor-owned): **PC-triggered hard-stop homing** for incremental encoders (ADR-057). `home_command = 1` drives velocity mode at `home_velocity` (signed) until `|armature current| > home_current` for ~10 ms **or the OC trip fires** (the stall against the end stop — a hard stop trips the OC before a long dwell would confirm; an OC hit is consumed/cleared), then sets the encoder zero there and stops (`home_status` → `MC_IF_HOME_DONE`); 30 s safety timeout → `MC_IF_HOME_FAILED`; `home_command = 0` aborts / clears. Absolute position + soft limits become usable on an incremental axis once homed. **Until homed, position recalls are blocked** and `0x2600:1 fault_flags` bit **`MC_IF_FAULT_NOT_HOMED`** is set — the CMC must home the axis before commanding a recall. The homed state is **not persisted** (re-home every power-up); an absolute encoder (SSI) never sets the bit. Motor-level commissioning (bypasses the CMC while running, like the dq-test); the CMC forwards the entries like any motor entry.
- **`0x2600:10 fault_flags_latched`** (U32, RO, motor-owned) + new **`MC_IF_FAULT_OVERCURRENT`** `fault_flags` bit: motor **fault register + since-boot history** for the new PC-tool Diagnostics panel (ADR-058). The OC trip is now a `fault_flags` bit (so `fault_flags` is the complete motor fault register), and `fault_flags_latched` is the sticky OR of every fault bit set since boot — faults that fired and were then cleared still show. Not persisted (resets each power-up). Additive; the CMC forwards like any motor entry. **PC tool: new "Diagnostics (faults & state)" group (motor + CMC operating state, active faults, fault history, CMC auto-cleared-fault count).**
- **`0x2300:6/7/8 vel_accel_up / vel_accel_dn / vel_accel_jerk`** (F32, RW, PERSIST, motor-owned): velocity-demand **acceleration ramp** (ADR-042) — slews the PROFILE_VELOCITY (joystick) demand toward the setpoint under an acceleration cap (`accel_up` while speeding up, `accel_dn` while slowing down [rad/s²]; `0` = that phase off). `accel_jerk` [rad/s³] eases the acceleration *up* to the cap; it falls *freely* on the way down, so it lands on the setpoint with no overshoot. `accel_jerk = 0` (default) = step to the cap = plain accel ramp. The position cascade + tuning generator bypass it. **Consumer (CMC):** these are operator-facing joystick-feel params — surface them in the CMC's joystick config UI as proxied writes to the motor, alongside the `axis_manager` calibration (`0x3022`, `0x3027–0x302A`). They persist on the **motor** (motor save-to-flash) while that calibration persists on the **CMC**, so an operator "save joystick setup" spans both save domains.

### Changed (access)
- **`0x2400:6/7 hb_cur_kp / hb_cur_ki`** RW PERSIST → **RO** (no longer persisted): they are now the **derived** gains (from R/L + bandwidth), read-only readback only. Set R/L/ωc instead of the gains directly. Not wire-breaking (same index/type; access metadata).

### Fixed
- **Observer gains now live from the OD.** `est_obs_kp/ki/kv` + `est_use_observer` (`0x2500:3-6`) were watch-window-only (mirror-trapped: GUI writes were overwritten each tick and never applied). The motor now applies them from the OD (GUI Motor Config, PERSIST). `est_electrical_offset` (`0x2500:1`) is a calibration result — dropped from the editable Motor-Config page (shown via cal status). No wire change.

### Added (define)
- **`MC_IF_TEST_MODE_CURRENT (3)`**: loop-tuning `test_mode` (0x2910:1) value — the on-motor signal generator drives the **current/torque command** (`test_amplitude` in amps; rate 0 → a step pulse; needs TORQUE mode). Joins OFF/VELOCITY/POSITION.

No `MC_IF_PROTOCOL_VERSION` bump (additive, non-PDO).

### Consumers to update
- motor-control MCU: implemented — reads `0x2000:6` at boot → backend + ADC channel; `test_mode=3` → current-loop reference. Build fw 44.
- network MCU: none — forwards both (motor-owned).
- PC tool: BLDC/Brushed-DC selector (writes `0x2000:6`, prompts save+reboot) + "Current tuning" in the tuning panel.

---

## [4.4.0] - 2026-06-25  (wire-breaking? no — additive: on-board UP/DOWN button current jog)

CMC-only addition. While `AXIS_OP_MODE_TORQUE` is active, the network MCU's on-board UP_BUTTON / DOWN_BUTTON (PB2 / PB1) drive `target_current` (0x302B): UP held → `+axis_button_current`, DOWN held → `−axis_button_current`, released → 0. Lets the firmware author bench-test the torque loop without PC-tool interaction. No motor MCU change, no `MC_IF_PROTOCOL_VERSION` bump.

### Added (OD entry)
- `0x302C axis_button_current` (F32, RW, `MC_IF_OWNER_CMC`) — magnitude in amps the on-board buttons apply. Direction comes from which button. Default 0 (buttons command 0 A until the operator configures a magnitude).

### Consumers to update
- **motor-control MCU**: nothing — CMC-owned entry.
- **network MCU** (this project): implemented — added `button_current_a` field + get/set on axis_manager; new `poll_torque_buttons()` runs each `axis_manager_tick` with a 3-sample debounce both directions; takes ownership of `target_current_a` while a button is held and releases it on full release; `cmc_od` dispatches `0x302C` reads + writes.
- **PC tool**: no change required — `0x302C` auto-appears in the OD browser via the X-macro parser. Could add a dedicated field on the Motor Command page later if useful.

---

## [4.3.0] - 2026-06-25  (wire-breaking? no — additive: torque command surface on CMC)

**Closes REQ-0012.** Adds a current/torque command surface on the CMC's `axis_manager` so the PC tool's Motor Command page can drive the motor in torque mode without bypassing axis_manager. Additive CMC-owned entries only — `MC_IF_PROTOCOL_VERSION` stays at **4**, no wire-format change.

### Added (defines / OD entries)
- `MC_IF_AXIS_MODE_TORQUE (5u)` — new axis op-mode, maps to motor's `MC_IF_MODE_TORQUE` (0x6060 = 4) in the cyclic frame.
- `0x302B axis_target_current` (F32, RW, MC_IF_OWNER_CMC) — commanded current in amps. axis_manager rounds to `int32 raw = round(amps / MC_IF_CUR_SCALE)` and SDO-writes motor `0x6071 target_torque`. Effective only while op_mode = TORQUE.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): **nothing** — fully implemented since ADR-027 (`mc_scheduler.c` already routes `MC_MODE_TORQUE_CURRENT` to `s_eff_iq_cmd = 0x6071 × MC_IF_CUR_SCALE`).
- **network MCU** (this project): **implemented** — added `target_current_a` field + getter/setter on axis_manager; wired the long-defined-but-unused `SEQ_TARGET_TORQUE` step into `setup_sequencer_tick`'s diff chain; mode mapper now translates `AXIS_OP_MODE_TORQUE → MC_IF_MODE_TORQUE`; `cmc_od` dispatches `0x302B`.
- **PC tool**: **implemented** — Torque mode + "Apply current" button enabled on the Motor Command page (was disabled with "pending REQ-0012" tooltip); motor-config catalog already auto-includes the new entry via the X-macro parser.

### Workflow
```
PC tool: write axis_op_mode (0x3020) = 5         → CMC mode goes to TORQUE
PC tool: write axis_target_current (0x302B) = I  → CMC SDO-writes motor 0x6071 = round(I / 1e-3)
PC tool: write axis_enable (0x3010) = 1          → motor enables, FOC commands iq ≈ I
PC tool reads torque_actual (0x6077)              → confirms FOC is tracking
```
Set-and-hold via SDO (the v3 cyclic frame carries only `velocity_setpoint`, not current); high-rate current streaming would need a separate wire-breaking change.

---

## [4.2.0] - 2026-06-25  (wire-breaking? no — store_status refined to a bitfield)

**`store_status` (0x2800:2, RO) is now a bitfield** instead of a 0/1 "has valid record" flag, so a host can see a save that is **latched but not yet committed**. The motor commits a save to flash only while the power stage is OFF (flash erase/program can't run during active control — ADR-010), so a save issued while the drive is enabled sits pending until the drive is disabled; previously nothing reported that, and the value silently reverted on reboot. (motor ADR-036)

### Added (defines)
- `MC_IF_STORE_VALID (0x0001)` — a valid saved record exists in flash (this is the old `== 1` meaning, now `bit 0`).
- `MC_IF_STORE_PENDING (0x0002)` — a save is latched, awaiting power-stage-off to commit.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — `store_status = VALID·HasValid | PENDING·SavePending`. ADR-036.
- **network MCU** (this project): nothing required. If anything reads `store_status`, note it's now a bitfield — test `& MC_IF_STORE_VALID` rather than `== 1`.
- **PC tool**: the Motor Config "Save to flash" now shows **"SAVE PENDING — disable the drive to commit"** while pending and **"Saved to flash"** once it commits, by polling `store_status`.

---

## [4.1.0] - 2026-06-25  (wire-breaking? no — load-factor migration: −CMC 0x3040, +motor 0x2300:5)

**Removed CMC-owned `0x3040 axis_payload_weight_kg`** (`mc_if_od.h`). The operator-tunable "load" concept is moving to the motor side as a dimensionless multiplier on velocity-loop gains — see REQ-0014 (`vel_load_factor` at `0x2300:5`). The earlier CMC-stored "payload weight in kg" (CHANGELOG [3.6.0]) was a placeholder pending exactly this motor-side support; it never actually scaled anything. No `MC_IF_PROTOCOL_VERSION` bump — CMC-owned removal, no wire/PDO impact.

### Removed (was `MC_IF_OWNER_CMC`, RW PERSIST)
- `0x3040 axis_payload_weight_kg` (F32, kg) — never consumed by anything. Replaced by motor's `vel_load_factor` (REQ-0014, now implemented).

### Added (`MC_IF_OWNER_MOTOR`, RW PERSIST)
- `0x2300:5 vel_load_factor` (F32) — operator load multiplier on the velocity loop: `effective_kp = vel_kp × factor`, `effective_ki = vel_ki × factor` (kd unchanged). Default 1.0 (no change); clamped to [0.3, 2.0] at apply; seeded in firmware defaults. (motor REQ-0014 / ADR-034)

### Consumers to update
- **motor-control MCU**: nothing for the removal. REQ-0014 **implemented** — added `0x2300:5 vel_load_factor` (F32 RW PERSIST, default 1.0), applied as `effective_kp = vel_kp × clamp(factor, 0.3, 2.0)` (same for ki) in `od_apply_gains`. ADR-034.
- **network MCU** (this project): implemented — removed `axis_manager_get/set_payload_weight_kg`, dropped from the persist blob (bumped `AXIS_PERSIST_VERSION` 2→3 — boards with a v2 blob fall through to defaults at next boot), removed the `0x3040` dispatch from `cmc_od.c`. Added `axis_manager_get/set_load_factor` which caches locally and fires an ad-hoc SDO write to motor `0x2300:5` (will return `NO_OBJECT` until motor MCU ships REQ-0014). Web page's Dynamics section now shows a slider (0.3 "Light load" → 2.0 "Heavy load", apply-on-release).
- **PC tool**: `0x3040` disappears from the OD browser automatically when re-parsing `mc_if_od.h`. No other action required.

---

## [4.0.0] - 2026-06-24  (wire-breaking? YES — MC_IF_PROTOCOL_VERSION 3 → 4; cyclic header extended)

**First wire-breaking change.** Extended the fixed cyclic status header `MC_IfCyclicStatusHeader_t` with two always-present fields so the CMC has live position + motion status independent of the host-configurable telemetry map (REQ-0013):
- `int32_t position_actual_scaled` — OD `0x6064`, `MC_IF_POS_SCALE` (1e-5 rad/LSB).
- `uint16_t movement_status` — `MC_IF_MOVE_*` bits.

`MC_IF_STATUS_HEADER_SIZE` 12 → 18; `MC_IF_TLM_BLOB_MAX` 40 → 34 (slightly less telemetry-blob budget); **`MC_IF_PROTOCOL_VERSION` 3 → 4**. A frame whose `version != 4` is rejected (`MC_IF_ERR_BAD_VERSION`) — motor + CMC + GUI must update together.

### `movement_status` bits (reduced from REQ-0013's proposal — see the REQ-0013 note in REQUESTS.md)
- `0x0001 MC_IF_MOVE_MOVING` — axis in motion (live).
- `0x0002 MC_IF_MOVE_ON_TARGET` — position-loop target reached (live).
- `0x0010 MC_IF_MOVE_AT_LIMIT_LO`, `0x0020 MC_IF_MOVE_AT_LIMIT_HI` — **reserved, populated 0** (no motor soft limits yet).
- `0x0004` / `0x0008` (REQ-0013 `SETPOINT_ACCEPTED` / `COMPLETE`) — **dropped → reserved** (CMC consumers don't use them).

### Consumers to update (all three — wire-breaking)
- **motor-control MCU** (Generic_motor_controller): implemented — header populated in `build_telemetry` (position via `0x6064`, `movement_status` pushed from the scheduler). ADR-033.
- **network MCU** (this project): parse the two new header fields; feed `position_actual_scaled` into `axis_manager_get_position_actual()` and use `MC_IF_MOVE_ON_TARGET` / `MOVING` instead of the faked values; reject motor frames where `version != 4`.
- **PC tool**: cyclic-header parser 12 → 18 bytes + 2 new fields; telemetry-blob budget 34; `movement_status` available for the status display.

---

## [3.11.1] - 2026-06-24  (wire-breaking? no — docs only: versioning policy clarified)

**Clarified `INTERFACE_SPEC.md` §6 (Versioning).** The old text — *"add objects by appending to the X-macro and bumping the version"* — contradicted established practice (every `3.x` additive entry kept `MC_IF_PROTOCOL_VERSION` at **3**). Corrected: **appending a new OD object does NOT bump the version** — acyclic OD requests carry index/subindex/type explicitly and the cyclic map has no fixed layout, so a new object changes no fixed wire layout. The version bumps only for framing / message-type / existing-field (size/meaning/scale) changes. Matches the additive entries `0x2200:4` and `0x2910:8`–`10`.

### Consumers to update
- **None** — documentation clarification only; no code or wire change. `MC_IF_PROTOCOL_VERSION` stays **3**.

---

## [3.11.0] - 2026-06-24  (wire-breaking? no — additive: signal-generator acceleration limit)

**Added motor-owned OD entry `0x2910:10 test_max_accel`** (F32, RW, rad/s²). Acceleration limit for the **position-tuning** profile: when `>0`, the generator drives a **trapezoidal velocity profile** (accelerate at `max_accel` up to cruise `rate`, cruise, decelerate to stop at `amplitude`) instead of a constant-velocity ramp — so the position move's acceleration stays within the bound and the corner current-saturation goes away. `0` = off (linear ramp, previous behaviour). Applies to **position tuning only** (velocity tuning's `rate` is already its acceleration). Additive `0x2910` test-block entry, non-PDO → `MC_IF_PROTOCOL_VERSION` stays **3**. (motor ADR-032)

### Added (`MC_IF_OWNER_MOTOR`, RW)
- `0x2910:10 test_max_accel` (F32) — position-tuning acceleration limit [rad/s²]; 0 = off.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — `mc_signal_gen` trapezoidal profile; scheduler passes it for position tuning, 0 for velocity tuning. ADR-032.
- **network MCU** (this project): **nothing** — motor-owned, forwarded over SPI by the generic OD bridge.
- **PC tool**: a "Max accel" field on the Motor Command tuning section writes `0x2910:10`.

---

## [3.10.0] - 2026-06-24  (wire-breaking? no — additive: signal-generator inter-pulse pause)

**Added motor-owned OD entry `0x2910:9 test_pause_s`** (F32, RW). In continuous (ping-pong) tuning mode the signal generator now **pauses at 0 between pulses** for `test_pause_s` seconds before flipping to the next pulse — independent of the peak hold `test_dwell_s` (0x2910:4). `0` = no pause (the previous immediate-flip behaviour). Additive `0x2910` test-block entry, non-PDO → `MC_IF_PROTOCOL_VERSION` stays **3**. (motor ADR-030)

### Added (`MC_IF_OWNER_MOTOR`, RW)
- `0x2910:9 test_pause_s` (F32) — inter-pulse pause [s] for continuous-mode pulse trains.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — `mc_signal_gen` dwells at 0 for `test_pause_s` between pulses. ADR-030.
- **network MCU** (this project): **nothing** — motor-owned, forwarded over SPI by the generic OD bridge.
- **PC tool**: a "Pause (s)" field on the Motor Command tuning section writes `0x2910:9`.

---

## [3.9.0] - 2026-06-24  (wire-breaking? no — additive: motor velocity-feedforward gain)

**Added motor-owned OD entry `0x2200:4 velocity_ff_gain`** (F32, RW, PERSIST, default 1.0). Trims the velocity feedforward ratio in the position cascade: `velocity_demand = velocity_ff_gain · velocity_ff + position_correction`. 1.0 = full FF (current behaviour); 0 = pure feedback. Also applies to the position-tuning signal generator (which now emits a velocity). Additive `0x22xx` gain entry → `MC_IF_PROTOCOL_VERSION` stays **3**. (motor ADR-031)

### Added (`MC_IF_OWNER_MOTOR`, RW PERSIST)
- `0x2200:4 velocity_ff_gain` (F32) — position-cascade velocity-feedforward ratio. Default 1.0; seeded in firmware defaults so a stray 0 / old flash can't silently disable FF.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — applied in `od_apply_gains`; position-tuning FF uses the generator's velocity. ADR-031.
- **network MCU** (this project): **nothing** — motor-owned, forwarded over SPI by the generic OD bridge.
- **PC tool**: appears on the Motor Config tab's position-gains group.

---

## [3.8.0] - 2026-06-24  (wire-breaking? no — additive: motor position-demand telemetry)

**Added motor-owned PDO `0x2510:3 tlm_pos_demand_rad`** (F32, RO) — the absolute (home-relative) position demand the position loop is chasing (from the trajectory or the position-tuning generator). PDO-mappable into `0x2A00` so the PC tool can graph it vs `position_actual` (0x6064) — the position-loop equivalent of `tlm_vel_demand` (0x2310:1) vs `tlm_vel_actual` (0x2310:2). Not part of the fixed frame → `MC_IF_PROTOCOL_VERSION` stays **3**. (motor ADR-030 / ADR-028)

### Added (`MC_IF_OWNER_MOTOR`, RO, PDO)
- `0x2510:3 tlm_pos_demand_rad` (F32) — absolute home-relative position demand [rad]. 0/stale outside `PROFILE_POSITION`; live during position moves + position tuning.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — mirrored from `g_mc_debug.pos_demand_rad` in `od_mirror_live`.
- **network MCU** (this project): **nothing** — motor-owned, forwarded over SPI by the generic OD bridge.
- **PC tool**: appears automatically in the telemetry-map editor's PDO list; graph it vs `position_actual` (0x6064) for position-loop tuning.

---

## [3.7.0] - 2026-06-24  (wire-breaking? no — additive: motor loop-tuning test block)

**Added motor-owned OD block `0x2910` (loop-tuning test-signal overlay)** + the `MC_IF_TEST_MODE_*` constants (`mc_if_od.h`). A motor-side commissioning overlay for PID tuning: while the matching operational mode is enabled, an on-motor signal generator drives that loop's reference (velocity-tuning → velocity-loop demand; position-tuning → position demand, bypassing the trajectory). Additive acyclic entries, not PDO-mapped → no wire/PDO-layout change → `MC_IF_PROTOCOL_VERSION` stays **3**. (motor ADR-030)

### Added (all `MC_IF_OWNER_MOTOR`)
- `0x2910:1 test_mode` (U8) — `MC_IF_TEST_MODE_OFF (0)` / `VELOCITY (1)` / `POSITION (2)`.
- `0x2910:2 test_amplitude` (F32) — pulse peak; units follow `test_mode` (rad/s for velocity, rad for position).
- `0x2910:3 test_rate` (F32) — ramp rate (rad/s² / rad/s); `0` = step edge.
- `0x2910:4 test_dwell_s` (F32) — hold time at the peak [s].
- `0x2910:5 test_continuous` (U8) — `0` one-shot pulse / `1` alternating ping-pong train.
- `0x2910:6 test_trigger` (U16) — write `1` to fire the generator.
- `0x2910:7 test_active` (U8, RO) — `1` while the generator is running.
- `0x2910:8 test_signal` (F32, RO, **PDO**) — the raw signal-generator output, for graphing the test signal in the PC tool (map it into `0x2A00`). Units follow `test_mode`.

### Rationale
Tuning the velocity/position loops needs clean, repeatable, motor-generated test signals injected at the loop under test — the GUI-side ramp (UDP-paced) is too coarse, and overriding an operational mode conflated test with normal operation. This is a motor-owned overlay (not a CiA-402 mode, not self-enabling), so it reuses the normal enable/safety/CMC drive-state and needs no CMC change.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — `mc_signal_gen` module + scheduler test branch. ADR-030.
- **network MCU** (this project): **nothing** — motor-owned, forwarded over SPI by the generic OD bridge.
- **PC tool**: Motor Command tab gains a Tuning section (test-mode selector + signal params + Fire/Stop), and sets the matching op-mode for the operator.

---

## [3.6.0] - 2026-06-22  (wire-breaking? no — additive: CMC payload-weight entry)

**Added CMC-owned OD entry `0x3040 axis_payload_weight_kg`** (F32 RW PERSIST). Operator-recorded payload mass for the axis, used for the operator's reference today and as the input to future gain-scheduling code (kp/ki/kd as a function of load). No automatic kp adjustment yet — the operator hand-tunes the motor MCU's `pos_kp` (0x2200:1) against the recorded weight. No wire/OD-layout change → `MC_IF_PROTOCOL_VERSION` stays **3**. New CMC OD range `0x3040-0x304F` reserved for dynamics-related entries (future: inertia model, gain schedules).

### Added (`MC_IF_OWNER_CMC`, RW PERSIST)
- `0x3040 axis_payload_weight_kg` (F32, kg) — operator-set payload mass. Default 0 (no payload). Rejected if negative.

### Consumers to update
- **motor-control MCU**: nothing — CMC-owned, not on the SPI bus. If/when an automatic load-aware gain scheduler is added, the motor MCU could expose a "kp_per_kg" parameter or accept gain writes from the CMC.
- **network MCU** (this project): implemented in `app/axis_manager`. Stored in the existing persist CONFIG region — blob version bumped 1→2. Pre-existing flash images with v1 blobs are rejected by persist_load (version mismatch); the operator needs to re-Save to migrate. Web config page exposes the field.
- **PC tool**: appears automatically on the GUI's CMC Setup tab once it re-parses `mc_if_od.h`.

---

## [3.5.0] - 2026-06-22  (wire-breaking? no — additive: motor calibration-completeness entry)

**Added motor-owned RO OD entry `0x2700:5 cal_done_flags`** (U16) and wired the previously-dead `MC_IF_CAL_CURRENT_OFFSET (2)` calibration command (`mc_if_od.h`). `cal_done_flags` is a bitfield reporting which calibrations currently have valid data so a tool can show what is still *outstanding*; a set bit = done, a clear bit = not yet done. Additive acyclic RO entry, not PDO-mapped → no wire/PDO-layout change → `MC_IF_PROTOCOL_VERSION` stays **3**. (motor ADR-026)

### Added (`MC_IF_OWNER_MOTOR`, RO)
- `0x2700:5 cal_done_flags` (U16) — calibration-completeness bitfield. Bit masks (also in `mc_if_od.h`):
  - `MC_IF_CAL_DONE_ELECTRICAL` (0x0001) — electrical-angle offset captured (alignment, `0x2700:1=1`).
  - `MC_IF_CAL_DONE_MECH_ZERO` (0x0002) — mechanical home set (`0x2700:1=3`).
  - `MC_IF_CAL_DONE_CURRENT_OFFSET` (0x0004) — phase-current ADC offsets measured (`0x2700:1=2`).

### Changed
- `MC_IF_CAL_CURRENT_OFFSET (2)` on `0x2700:1` is now **dispatched** by the motor MCU (it triggers the existing phase-current offset routine; rejected unless the power stage is off). It was defined but inert before.

### Rationale
The PC tool's Motor Config tab fires the calibration commands but had no way to show which calibrations were still outstanding — `cal_status` (0x2700:2) only echoes the last command / running / fault. The motor reports per-calibration validity here; the firmware derives it from existing state (electrical/home offsets, current-sense calibrated flag), so there is **no flash store-version bump** and saved gains/calibration survive.

### Consumers to update
- **motor-control MCU** (Generic_motor_controller): implemented — `cal_done_flags` mirrored each medium tick from derived completeness; `0x2700:1=2` dispatched to the current-offset routine. ADR-026.
- **network MCU** (this project): **nothing** — motor-owned, forwarded over SPI by the generic OD bridge.
- **PC tool**: Motor Config tab reads `0x2700:5`, lists the outstanding calibrations, and gains a "Current offset" action button.

---

## [3.4.0] - 2026-06-22  (wire-breaking? no — additive: CMC-side persistence trigger entries)

**Added CMC-owned write-trigger OD entries `0x3050 cmc_save_config` and `0x3051 cmc_save_shots`** (`mc_if_od.h`). Mirrors the motor MCU's `0x2800:1` save pattern on the CMC side. Writing `MC_IF_SAVE_MAGIC` (= 0x7376) commits the corresponding RAM state to the CMC's internal flash via `bsp/flash` + `app/persist`. No wire/OD-layout change → `MC_IF_PROTOCOL_VERSION` stays **3**.

### Added (both `MC_IF_OWNER_CMC`, WO)
- `0x3050 cmc_save_config` (U16) — write `MC_IF_SAVE_MAGIC` to persist axis_manager's PERSIST-flagged entries (joystick cal + motion limits) to flash.
- `0x3051 cmc_save_shots` (U16) — write `MC_IF_SAVE_MAGIC` to persist the shot table (100 slots, when populated).

### Rationale
PERSIST-flagged CMC-owned entries (joystick cal, motion limits) previously had no persistence path on the CMC — RAM-only, lost on reboot. Adding the explicit save trigger (matches motor MCU pattern at `0x2800:1`) gives the PC tool a one-click "commit my config" action without auto-saving on every write (which would burn through flash endurance on a 10k-cycle/page device).

### Consumers to update
- **motor-control MCU**: nothing — these entries are CMC-owned.
- **network MCU** (this project): implemented in `app/od/cmc_od` (dispatches to `axis_manager_save_to_flash()` / `cmc_state_save_shots()`), backed by `bsp/flash` + `app/persist`. Flash layout reserves the top 8 KB of internal flash (linker script `STM32G431RBTX_FLASH.ld` shortens FLASH region to 120 KB).
- **PC tool**: "Save to flash" button on the new CMC Setup tab is enabled and writes `0x3050 = 0x7376`.

---

## [3.3.0] - 2026-06-22  (wire-breaking? no — additive: CMC-owned joystick calibration block)

**Added CMC-owned OD block `0x3026-0x302A`** to `mc_if_od.h` for joystick raw input + linear calibration. CMC-owned (`MC_IF_OWNER_CMC`) — handled entirely by the network MCU's `axis_manager`; **no motor MCU code change**. No wire/OD-layout change → `MC_IF_PROTOCOL_VERSION` stays **3**.

### Added (all `MC_IF_OWNER_CMC`)
- `0x3026 axis_joystick_raw` (I32, RW) — raw stick value from a protocol module (CAMERAD panel, VISCA, PC tool). Writing this triggers re-normalisation into `0x3021 axis_joystick_value`.
- `0x3027 axis_joystick_raw_center` (I32, RW PERSIST) — raw value at rest. Default 0.
- `0x3028 axis_joystick_raw_full_pos` (I32, RW PERSIST) — raw at full positive deflection. Default +32767.
- `0x3029 axis_joystick_raw_full_neg` (I32, RW PERSIST) — raw at full negative deflection. Default −32767.
- `0x302A axis_joystick_raw_deadband` (U32, RW PERSIST) — ± around center → output 0. Default 0.

### Rationale
Protocol modules (CAMERAD, VISCA, ...) deliver raw stick values with different conventions (S/T panels send signed centered; VISCA sends 0x01..0x18 with direction bit; web UI sends pre-normalised). Centralising the raw-to-normalised math in `axis_manager` keeps each protocol module thin and lets the per-axis calibration be tuned from the PC tool + persisted to flash. Symmetric output: `axis_joystick_max_velocity` (0x3022) applies in both directions; no separate min-velocity entry.

### Consumers to update
- **motor-control MCU**: nothing — these entries are CMC-owned and never reach the SPI bus.
- **network MCU** (this project): implemented in `app/axis_manager` + `app/od/cmc_od`. axis_manager's `recompute_joystick_value_from_raw()` runs on every cal/raw write.
- **PC tool**: the entries appear automatically once the GUI re-parses `mc_if_od.h`; PC can read/write them via the OD-over-UDP tool. Useful for live joystick-cal during bringup.

---

## [3.2.0] - 2026-06-22  (wire-breaking? no — additive: new calibration OD entries)

**Added `0x2700:3 cal_align_current_a` (F32, RW PERSIST) and `0x2700:4 cal_align_hold_ms` (U16, RW PERSIST)** to `mc_if_od.h` — parameters for the motor's OD-triggered electrical-alignment routine. Motor-owned OD additions only, not in the cyclic frame, so **no `MC_IF_PROTOCOL_VERSION` bump** and no link break (consistent with [3.1.0]). Motor-originated (motor-side ADR-024).

### Added
- `0x2700:3 cal_align_current_a` — d-axis alignment current [A].
- `0x2700:4 cal_align_hold_ms` — alignment drive/hold duration [ms].

### Consumers to update
- **motor-control MCU**: implemented (ADR-024) — writing `0x2700:1 = MC_IF_CAL_ALIGN_CAPTURE (1)` runs a current-regulated open-loop alignment (drive `cal_align_current_a` at electrical angle 0 for `cal_align_hold_ms`, capture the electrical offset, auto-save). `0x2700:2 cal_status` reports progress.
- **network MCU**: no code change — recompile against the updated header (it forwards motor-owned `0x2700` SDO writes).
- **PC tool**: expose `0x2700:3/4` + a "run electrical alignment" action that sets them and writes `0x2700:1 = 1`.

---

## [3.1.0] - 2026-06-22  (wire-breaking? no — additive: new calibration command code)

**Added `MC_IF_CAL_SET_MECH_ZERO` (3)** to the `0x2700:1 cal_command` codes (`mc_if_od.h`). No wire/OD layout change, so `MC_IF_PROTOCOL_VERSION` stays **3**. Writing `0x2700:1 = 3` (SDO) captures the motor's current absolute (multi-turn) position as the **mechanical home**: `position_actual` (0x6064) — and, once the motor's position loop lands (D3), PROFILE_POSITION targets — become relative to it. Persisted to flash. Motor-originated (motor-side ADR-022).

### Added
- `MC_IF_CAL_SET_MECH_ZERO (3u)` calibration-command code for `0x2700:1`. (`mc_if_od.h`)

### Consumers to update
- **motor-control MCU**: implemented (ADR-022) — `0x2700:1 = 3` captures + persists the home; `position_actual` is home-relative; `0x2700:2 cal_status` echoes the accepted command.
- **network MCU**: no code change — it forwards motor-owned `0x2700` SDO writes over SPI. (Optionally surface a "set home" action in `axis_manager` later.)
- **PC tool**: add a "Set mechanical zero" action that SDO-writes `0x2700:1 = 3`. Position entries (`0x6064` / `0x607A`) are then relative to the captured home.

### Note — single-turn-encoder caveat
The board's encoder is single-turn absolute (21-bit, 1 rev). A captured multi-turn home is exact within a power session, but across a power cycle only the within-one-rev component is recoverable (the turn count resets at boot). Axes whose travel stays within ~1 rev persist correctly; multi-turn axes should re-home after power-up (a homing routine is future work).

---

## [3.0.0] - 2026-06-22  (wire-breaking? yes — cyclic-command payload reshaped)

**`MC_IF_PROTOCOL_VERSION` bumped 2 → 3.** Wire-breaking change: `MC_IfCyclicCommand_t` shrinks from 31 bytes to 10 bytes, removing every field that doesn't genuinely need to stream every millisecond. The streaming/SDO split aligns with how the system is actually used.

*Re-cut 2026-06-22 per REQ-0010 (motor MCU)*: the original [3.0.0] draft carried `joystick_value` (i16) + a motor-side `joystick_scale_rad_s` OD entry. That pushed a CMC application concept (joystick) onto a generic motor controller and round-tripped velocity → i16 → velocity. Re-cut to stream a `velocity_setpoint` (i32 scaled, same units as OD 0x60FF) directly — the LCMC's `axis_manager` does any joystick scaling locally. v3 wasn't deployed when this was caught, so amended in place rather than bumping to v4.

### Rationale

The v2 cyclic command carried 8 OD-mapped fields each cycle (controlword, mode_of_operation, target_position, target_velocity, target_torque, profile_velocity, profile_acceleration, profile_deceleration). In practice most of these are **setup parameters** that change rarely (per move, per mode change, or never) — sending them at 1 kHz wastes bus bandwidth and dilutes the slave's hot-path unpack.

The v3 cyclic command carries only:
- `controlword` — urgent / live bits: ENABLE, QUICK_STOP, NEW_SETPOINT, FAULT_RESET, HALT.
- `velocity_setpoint` (i32) — live velocity demand in scaled rad/s (LSB = `MC_IF_VEL_SCALE`, same units as 0x60FF). Applied as the active velocity target whenever the motor is in a velocity-class mode (`MC_IF_MODE_PROFILE_VELOCITY`) and enabled.
- `command_counter` — the dead-man heartbeat.

Everything else (target_position, target_torque, profile_velocity, profile_acceleration, profile_deceleration, mode_of_operation, target_position_time_ms) is **SDO-only**. The host writes setup values via the OD pipeline, then triggers execution by rising-edging the **`MC_IF_CW_NEW_SETPOINT`** controlword bit (CiA-402 bit 4).

### Added
- New controlword bit: `MC_IF_CW_NEW_SETPOINT` (`0x0010`) — rising edge executes the configured move. (`mc_if_od.h`)
- New motor-MCU OD entry (`mc_if_od.h`):
  - `0x607B target_position_time_ms` (U32, RW) — target duration in milliseconds for a PROFILE_POSITION move. `0` = ASAP within profile limits. Used by the motor MCU's trajectory engine; LCMC does no math on this value.
- New section in `INTERFACE_SPEC.md` documenting the streaming-vs-SDO split and the setup-then-trigger sequence.

### Changed
- `MC_IfCyclicCommand_t` shape (`mc_if_protocol.h`):
  ```c
  // v3:
  typedef struct __attribute__((packed)) {
      uint16_t controlword;
      int32_t  velocity_setpoint;   // ×MC_IF_VEL_SCALE rad/s
      uint32_t command_counter;
  } MC_IfCyclicCommand_t;   // 10 bytes
  ```
- All wire-version checks must accept `MC_IF_PROTOCOL_VERSION == 3`. v2 and earlier are rejected with `MC_IF_ERR_BAD_VERSION`.
- `0x60FF target_velocity` (SDO) becomes informational only. The cyclic `velocity_setpoint` is the authoritative live demand in velocity modes; the motor MCU does NOT use the SDO value to drive motion.

### Removed
- `MC_IF_MODE_JOYSTICK_VELOCITY` (`-1`) — joystick is a CMC application concept, not a motor primitive. The CMC's `axis_op_mode = JOYSTICK` maps to motor `MC_IF_MODE_PROFILE_VELOCITY = 3` with `velocity_setpoint = joystick_value × joystick_max_velocity` computed CMC-side.
- (Setup parameters removed *from the cyclic command*, but the OD entries themselves stay): `mode_of_operation` (0x6060), `target_position` (0x607A), `target_velocity` (0x60FF — now informational), `target_torque_current` (0x6071), `profile_velocity` (0x6081), `profile_acceleration` (0x6083), `profile_deceleration` (0x6084).

### Consumers to update
- **motor-control MCU**: see REQ-0009 + REQ-0010 in `REQUESTS.md` for the full briefing. Summary: unpack the 10-byte cyclic command, store SDO-written setup parameters, execute the configured move on the rising edge of `MC_IF_CW_NEW_SETPOINT`. In any velocity mode with the drive enabled, continuously apply `velocity_setpoint × MC_IF_VEL_SCALE` as the velocity demand. Add OD entry `0x607B`. Accept `MC_IF_PROTOCOL_VERSION = 3`.
- **network MCU** (this project): `axis_manager` issues SDO setup writes for PROFILE_POSITION moves (mode, target_position, target_position_time_ms, profile_*) then triggers via NEW_SETPOINT. For velocity modes (PROFILE_VELOCITY, the CMC's JOYSTICK), `axis_manager` streams `velocity_setpoint` in every cyclic frame; no NEW_SETPOINT needed. (In progress.)
- **PC tool** (`Interface/gui/`): OD-over-UDP path is unchanged; the new entry `0x607B` appears automatically once the GUI re-parses `mc_if_od.h`. The "Apply map" workflow continues to work the same way.

### Setup-then-trigger sequence (motor MCU's view of a PROFILE_POSITION move)

```
LCMC                                             Motor MCU
  | -- SDO: write 0x6060 mode_of_operation = 1 -->
  | <-- OD_WRITE_RESP (OK)                  ---
  | -- SDO: write 0x607A target_position    -->
  | <-- OD_WRITE_RESP (OK)                  ---
  | -- SDO: write 0x607B target_position_time_ms -->
  | <-- OD_WRITE_RESP (OK)                  ---
  | -- SDO: write 0x6081 profile_velocity   -->  (if changed)
  | <-- OD_WRITE_RESP (OK)                  ---
  | -- CYCLIC_CMD: ctrlword |= NEW_SETPOINT -->   <-- rising edge: latch + start move
  | -- CYCLIC_CMD: ctrlword &= ~NEW_SETPOINT-->   (held low to re-arm)
  | <-- CYCLIC_STATUS: statusword shows in-motion --
  ...
  | <-- CYCLIC_STATUS: statusword shows TARGET_REACHED --
```

Velocity mode (CMC's JOYSTICK or PROFILE_VELOCITY) is simpler — no SDO setup at all if the motor is already in PROFILE_VELOCITY:
```
LCMC                                             Motor MCU
  | -- SDO: write 0x6060 mode = 3 (PROFILE_VELOCITY) --> (if not already in this mode)
  | <-- OD_WRITE_RESP (OK)                  ---
  | -- CYCLIC_CMD: velocity_setpoint = V    -->   <-- motor follows V live
  | -- CYCLIC_CMD: velocity_setpoint = W    -->
  ...
```

---

## [2.0.0] - 2026-06-22  (wire-breaking? no — but OD-extension breaking; see below)

**`MC_IF_PROTOCOL_VERSION` bumped 1 → 2.** Wire packet layout (header, footer, payloads) is unchanged. The OD layout is extended: every entry now carries an **owner** column (`MC_IfOdOwner_t`), and a new range `0x3xxx` is added for **CMC-owned** entries that back the network MCU's `axis_manager`.

The version bump is needed because:
1. The `MC_IF_OD_OBJECTS(X)` X-macro signature grows by one column. Every X-handler in both firmwares must be updated.
2. The shared header now contains entries the motor MCU **must skip** when building its own OD table — silently consuming them would create phantom OD entries with no backing implementation on the motor MCU side.
3. Anything that interprets `MC_IF_PROTOCOL_VERSION` for compatibility check (e.g. UDP `MC_UdpHeader_t.version`, SPI `MC_IfFrameHeader_t.version`) must accept the new value. Slaves and PC tools that hard-coded `== 1` will reject v2 traffic.

### Added
- `MC_IfOdOwner_t` enum (`MC_IF_OWNER_MOTOR = 0`, `MC_IF_OWNER_CMC = 1`) in `mc_if_od.h`.
- New trailing **owner** column on every X(...) line of `MC_IF_OD_OBJECTS`. All existing motor-MCU entries are tagged `MC_IF_OWNER_MOTOR`.
- CMC-owned axis_manager OD entries in range `0x3000-0x303F` (axis 0): state, command triggers, op-mode + per-mode targets, limits. Ranges `0x3100-0x31FF`, `0x3200-0x32FF` reserved for future axes 1, 2. (`mc_if_od.h`)
- Axis-mode enum constants `MC_IF_AXIS_MODE_OFF / JOYSTICK / PROFILE_VELOCITY / PROFILE_POSITION / HOLD` for OD `0x3020`.
- Axis-state enum constants `MC_IF_AXIS_STATE_DISABLED / READY / RUNNING / FAULT` for OD `0x3000`.

### Rationale
The CMC's network OD bridge needs a clean way to expose its own command surface (the `axis_manager`'s public API) without writing into motor-MCU OD entries directly. The same single OD now serves both halves; both firmwares filter by owner when building their local tables. Protocol modules on the CMC (CAMERAD, VISCA, future) all speak the **same** `0x3xxx` entries to drive the axis — no per-protocol API.

### Consumers to update
- **motor-control MCU**: update every X-handler that consumes `MC_IF_OD_OBJECTS(X)` to accept the new trailing `owner` parameter. Filter — only build OD entries where `owner == MC_IF_OWNER_MOTOR`. Update `MC_SpiSlave` / `mc_comms` to accept `MC_IF_PROTOCOL_VERSION = 2` on incoming SPI frames. See `REQUESTS.md` REQ-0008 for the full briefing.
- **network MCU** (this project): filter by `owner == MC_IF_OWNER_CMC` when building `cmc_od`'s dispatch table. Accept v2 on UDP `MC_UdpHeader_t.version` and SPI `MC_IfFrameHeader_t.version`. (In progress.)
- **PC tool** (`Interface/gui/`): update `od.py` X-macro parser to accept the new column. Display owner column in the OD tree.

### Migration shape (motor MCU)
The handler form changes from:
```c
#define DEFINE_OD_ENTRY(idx, sub, name, type, access, flags) ...
MC_IF_OD_OBJECTS(DEFINE_OD_ENTRY)
```
to:
```c
#define DEFINE_OD_ENTRY(idx, sub, name, type, access, flags, owner) \
    DO_IF_MOTOR_##owner(idx, sub, name, type, access, flags)
#define DO_IF_MOTOR_MC_IF_OWNER_MOTOR(...)  /* original handler body */
#define DO_IF_MOTOR_MC_IF_OWNER_CMC(...)    /* nothing */
MC_IF_OD_OBJECTS(DEFINE_OD_ENTRY)
```
Or any equivalent macro filter. The exact pattern is up to the motor MCU project — what matters is that CMC-owned entries don't make it into the motor MCU's OD table.

---

## [1.0.1] - 2026-06-21  (wire-breaking? no)
Operational defaults fixed (agreement-only constants; not part of the byte layout, so no
`MC_IF_PROTOCOL_VERSION` bump).

### Added
- `MC_IF_CYCLIC_RATE_HZ` = 1000, `MC_IF_CYCLIC_PERIOD_US`, `MC_IF_COMMAND_TIMEOUT_MS` = 30,
  `MC_IF_SPI_CLOCK_HZ_INITIAL` = 6 MHz, `MC_IF_SPI_CLOCK_HZ_MAX` = 10 MHz. (`mc_if_protocol.h`)

### Consumers to update
- motor-control MCU: use `MC_IF_COMMAND_TIMEOUT_MS` for the command dead-man; SPI clock per above.
- network MCU: drive the cyclic exchange at `MC_IF_CYCLIC_RATE_HZ`; SPI master clock per above.
- PC tool: no change.

### Resolved (were "still open")
- Cyclic rate, command-timeout, SPI clock now fixed (above). Remaining open: CiA-402 scale
  factors finalisation, UDP ports / endpoint discovery, default telemetry map contents.

---

## [1.0.0] - 2026-06-21  (baseline)
Initial contract.

### Added
- SPI framing: fixed 64-byte full-duplex frame, CRC16/Modbus (header + payload), sync 0xA55A,
  message types (cyclic cmd/status, OD read/write req+resp, heartbeat, error). (`mc_if_protocol.h`)
- OD model: type/access enums, CiA-402 scale factors, control/status/mode constants, and the
  canonical `MC_IF_OD_OBJECTS(X)` map (CiA-402 scaled-int + 0x2xxx manufacturer float32 SI).
  (`mc_if_od.h`)
- Configurable runtime telemetry mapping (TX-PDO) at OD `0x2A00`; cyclic telemetry frame =
  12-byte header + mapped blob (~10 float32). (`mc_if_od.h`, `mc_if_protocol.h`, ADR-014)
- Continuous motion commands (jog/joystick) ride the cyclic command with a `command_counter`
  dead-man watchdog; recommended 1 kHz cyclic / ~30 ms timeout. (`INTERFACE_SPEC.md` §3a)
- Network side: all-UDP PC↔network-MCU protocol — OD access (req/resp, retransmit) + pushed
  telemetry stream (batched, fire-and-forget). (`NETWORK_UDP_SPEC.md`, ADR-014)

### Consumers to update
- motor-control MCU (`Generic_motor_controller`): implement OD + SPI-slave per this contract.
- network MCU (`Lightweight_CMC`): implement SPI master + UDP bridge per this contract.
- PC tool: speak OD over UDP per `NETWORK_UDP_SPEC.md`.

### Still open (not yet frozen; will appear as entries when decided)
- CiA-402 scale factor values, cyclic rate / command-timeout finals, SPI clock ceiling.
- UDP ports / endpoint discovery; default telemetry map contents.
