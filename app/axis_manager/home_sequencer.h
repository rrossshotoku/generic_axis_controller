/*
 * app/axis_manager/home_sequencer — home-to-endstop state machine.
 *
 * Extracted from axis_manager.c on 2026-07-22 (see axis_manager/README.md
 * "Refactor 2026-07-22" note). Wraps the motor's own homing at 0x2700:8/9
 * and the NOT_HOMED bit in 0x2600:1 fault_flags, backing the CMC-owned
 * surface at 0x3040/1/2/3.
 *
 * Owns three concerns that share the state machine + cia402 handle:
 *   1. Home sequence itself (CLEARING_CMD → WRITING_CMD → POLLING_STATUS
 *      → READING_FAULT → TERMINAL_DONE|FAILED → IDLE).
 *   2. is_homed cache — refreshed at end of home sequence AND on a slow
 *      background cadence so PC-tool-initiated home commands are noticed.
 *   3. Encoder-type probe (motor 0x2500:8 quad_counts_per_rev) — one-shot
 *      read after boot. Piggybacks on the state machine's cia402 handle
 *      because it too is a rare SDO operation and the sharing is safe
 *      (probe only runs during IDLE background time).
 *   4. home_on_boot flag (0x3043) — persisted, auto-fires request_home
 *      once per boot when encoder is incremental and axis is idle.
 *
 * cia402 slot cooperation: this module runs one cia402 OD operation at a
 * time and defers if cia402 is busy. Cooperates with motor_od_proxy
 * (bootsync + proxy + motor_save + load_factor) — all defer on the same
 * cia402_od_*_begin returning INVALID.
 *
 * Not thread-safe (single-caller from axis_manager_tick).
 *
 * Layering: depends on cia402, bsp/time, log, Interface, axis_manager
 * (for try_begin_op/stop_op arbitration). Does NOT include any other
 * axis_manager sub-module.
 */
#ifndef APP_AXIS_MANAGER_HOME_SEQUENCER_H
#define APP_AXIS_MANAGER_HOME_SEQUENCER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called once from axis_manager_init. Puts the state machine at IDLE and
 * clears every cache — MUST be called before the first tick. */
void home_sequencer_init(void);

/* Called from axis_manager_tick when the motor is NOT in bootloader mode
 * (the guard lives in axis_manager, not here). Advances the state machine,
 * services the background is_homed refresh + encoder-type probe, and
 * fires home_on_boot auto-home if all conditions are met. */
void home_sequencer_tick(void);

/* Called from reset_motor_od_submodules on the rising edge of motor
 * entering the bootloader. Forces state → IDLE, invalidates the cia402
 * handle, and preserves is_homed_known so a bootloader→app cycle doesn't
 * force cmc_state to re-block shot recalls while the fault-read races. */
void home_sequencer_reset(void);

/* Backs 0x3040 (write 1 to start home). Arbitrated through
 * axis_manager_try_begin_op(HOMING). Returns false if arbitration rejected
 * OR if the sequencer is already mid-run. On success transitions from
 * IDLE → CLEARING_CMD immediately; the write goes out on the next tick. */
bool home_sequencer_request_home(void);

/* Backs 0x3041 (RO). Returns the motor's own status enum (MC_IF_HOME_*)
 * from the last poll. Sticky between runs — if a run finished with DONE
 * we still report DONE while sitting IDLE. */
uint8_t home_sequencer_get_home_status(void);

/* Backs 0x3042 (RO). true iff we've successfully read fault_flags AND
 * the NOT_HOMED bit is clear. Conservative — returns false until we have
 * a confirmed reading so shot recalls don't race at boot. */
bool home_sequencer_is_homed(void);

/* Absolute encoders don't need homing. Returns true iff we successfully
 * read a non-zero quad_counts_per_rev from motor 0x2500:8 (incremental).
 * Fallback: while we haven't read yet, returns true — better to show the
 * Home button and have the operator click it needlessly than to hide it
 * when it's actually required. */
bool home_sequencer_encoder_is_incremental(void);

/* home_on_boot persisted flag (0x3043). axis_manager forwards its public
 * getter/setter through these and includes home_on_boot in the persist
 * blob capture/apply — the storage lives here so the auto-fire tick can
 * read it locally without an extra hop. */
uint8_t home_sequencer_get_home_on_boot(void);
bool    home_sequencer_set_home_on_boot(uint8_t v);

/* True while the state machine is actively driving a home operation
 * (i.e. between request_home success and the terminal → IDLE transition
 * that follows READING_FAULT). Used by op_arbiter tick_active_op to
 * detect HOMING op completion. Background encoder-type probes and
 * background is_homed refreshes are NOT considered "running" — they
 * only fire from IDLE and can't overlap a real home sequence. */
bool home_sequencer_is_terminal_or_idle(void);

/* Abort primitive called from axis_manager_stop_op's HOMING case.
 * Best-effort: fires 0x2700:8 = 0 to the motor to abort its side, and
 * forces the CMC-side state machine back to IDLE. If cia402 is busy
 * the SDO write silently drops and the motor's own home-timeout will
 * eventually unwind it. */
void home_sequencer_abort_best_effort(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_AXIS_MANAGER_HOME_SEQUENCER_H */
