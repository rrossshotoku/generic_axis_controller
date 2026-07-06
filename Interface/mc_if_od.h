#ifndef MC_IF_OD_H
#define MC_IF_OD_H
#include <stdint.h>

/** @file mc_if_od.h
 *  @brief SHARED inter-MCU boundary contract: object dictionary (index map + scaling).
 *
 *  Single source of truth for the OD layout exposed by the motor-control MCU over SPI and,
 *  in turn, on the network MCU's external (Ethernet) interface. Both firmwares and host tools
 *  include this file. The canonical object list is the X-macro MC_IF_OD_OBJECTS(X) at the end:
 *  each side expands it to build its own table / generate code, so the two never drift.
 *
 *  Each entry carries an OWNER column (MC_IfOdOwner_t). The owner determines which firmware
 *  actually handles reads and writes for that entry:
 *    - MC_IF_OWNER_MOTOR : motor MCU's OD table handles it. Reads/writes from the network
 *                          MCU travel over SPI via the cia402 OD pipeline. Encompasses every
 *                          CiA-402 standard (0x1xxx/0x6xxx) entry and every motor-MCU
 *                          manufacturer entry (0x2xxx).
 *    - MC_IF_OWNER_CMC   : the network MCU (Lightweight_CMC) handles it locally. These entries
 *                          back the CMC's axis_manager and are the universal command surface
 *                          that all protocol modules (camerad, visca, PC tool, ...) speak to.
 *                          No SPI traffic is generated when these are read/written.
 *
 *  Type convention:
 *   - CiA-402 standard objects (0x1xxx, 0x6xxx): scaled integers, factors below.
 *   - Manufacturer objects (0x2xxx): FLOAT32 in SI units (rad, rad/s, A, Nm, V) -- exact,
 *     no scaling -- which is ideal for gain tuning and live graphing.
 *   - CMC-owned axis_manager objects (0x3xxx): FLOAT32 SI for analog values, U8 for modes /
 *     state / triggers. Same SI convention as 0x2xxx so all of axis_manager's public surface
 *     can be entered and read in physical units.
 */

/** @brief OD data types (values match the motor MCU's MC_OdType_t). */
typedef enum
{
    MC_IF_T_U8 = 0,
    MC_IF_T_U16,
    MC_IF_T_U32,
    MC_IF_T_I8,
    MC_IF_T_I16,
    MC_IF_T_I32,
    MC_IF_T_F32
} MC_IfOdType_t;

/** @brief OD access rights. */
typedef enum
{
    MC_IF_A_RO = 1,
    MC_IF_A_WO = 2,
    MC_IF_A_RW = 3
} MC_IfOdAccess_t;

/** @brief OD entry owner -- which firmware handles reads/writes for the entry.
 *
 *  Added in MC_IF_PROTOCOL_VERSION 2. Both firmwares filter MC_IF_OD_OBJECTS(X) by owner
 *  when building their local OD table; host tools (e.g. PC GUI) iterate everything and
 *  display the owner alongside the entry.
 */
typedef enum
{
    MC_IF_OWNER_MOTOR      = 0,   /* handled by Generic_motor_controller's OD table */
    MC_IF_OWNER_CMC        = 1,   /* handled by Generic_axis_controller's app/od/cmc_od */
    MC_IF_OWNER_BOOTLOADER = 2    /* handled by the bootloader on either MCU. When
                                   * the target is in bootloader mode (node_state ==
                                   * MC_IF_NODE_BOOTLOADER) the 0x1F5x range dispatches
                                   * to the bootloader's OD subset rather than the
                                   * app's. See Documentation/dual_bootloader_design.md */
} MC_IfOdOwner_t;

/** @brief OD entry flags (bitmask). */
#define MC_IF_F_NONE     (0x00u)
#define MC_IF_F_PDO      (0x01u)   /* mappable into the cyclic process data */
#define MC_IF_F_PERSIST  (0x02u)   /* saved by the persistent store */

/* ===== Scaling for CiA-402 standard (scaled-int) objects =====
 * SI value = raw * scale ;  raw = round(SI / scale). */
#define MC_IF_POS_SCALE   (1.0e-5f)  /* rad per LSB   (I32: +-21474 rad ~ +-3418 rev, 10 urad) */
#define MC_IF_VEL_SCALE   (1.0e-3f)  /* rad/s per LSB (1 mrad/s) */
#define MC_IF_ACC_SCALE   (1.0e-3f)  /* rad/s^2 per LSB */
#define MC_IF_CUR_SCALE   (1.0e-3f)  /* A per LSB (1 mA) -- target/actual "torque/current" */
#define MC_IF_TRQ_SCALE   (1.0e-3f)  /* Nm per LSB (1 mNm) if used as torque */

/* ===== Controlword (0x6040) bits ===== */
#define MC_IF_CW_ENABLE        (0x0001u)  /* enable operation */
#define MC_IF_CW_QUICK_STOP    (0x0002u)  /* 0 = quick stop active */
#define MC_IF_CW_NEW_SETPOINT  (0x0010u)  /* rising edge: execute currently-configured move
                                           * (PROFILE_POSITION / PROFILE_VELOCITY / TORQUE etc.
                                           *  All setup parameters MUST have been written via
                                           *  SDO before this rising edge.) */
#define MC_IF_CW_FAULT_RESET   (0x0080u)  /* rising edge clears latched faults */
#define MC_IF_CW_HALT          (0x0100u)  /* controlled stop, hold position */

/* ===== Statusword (0x6041) bits ===== */
#define MC_IF_SW_READY          (0x0001u)
#define MC_IF_SW_ENABLED        (0x0004u)
#define MC_IF_SW_FAULT          (0x0008u)
#define MC_IF_SW_TARGET_REACHED (0x0400u)
#define MC_IF_SW_LIMIT_ACTIVE   (0x0800u)

/* ===== Modes of operation (0x6060) ===== */
#define MC_IF_MODE_DISABLED          (0)
#define MC_IF_MODE_PROFILE_POSITION  (1)
#define MC_IF_MODE_PROFILE_VELOCITY  (3)   /* live velocity from cyclic velocity_setpoint */
#define MC_IF_MODE_TORQUE            (4)
#define MC_IF_MODE_HOMING            (6)
/* Note: a separate "joystick" mode used to live here (-1); v3 removed it.
 * The motor MCU has no application-specific joystick concept. The CMC's
 * axis_manager translates joystick_value × joystick_max_velocity locally
 * and streams the result as velocity_setpoint in PROFILE_VELOCITY mode. */

/* ===== Calibration commands (0x2700/1) ===== */
#define MC_IF_CAL_NONE             (0u)
#define MC_IF_CAL_ALIGN_CAPTURE    (1u)
#define MC_IF_CAL_CURRENT_OFFSET   (2u)   /* measure phase-current ADC offsets; power stage must be off (ADR-026) */
#define MC_IF_CAL_SET_MECH_ZERO    (3u)   /* capture current position as the mechanical home (ADR-022) */
#define MC_IF_CAL_SET_MECH_ZERO_AT (4u)   /* set the mechanical home to mech_zero_set_rad (0x2700:10) -- midpoint-of-travel centering (ADR-022) */

/* ===== Calibration completeness (0x2700/5 cal_done_flags, RO bitfield) =====
 * A set bit means that calibration currently has valid data; a CLEAR bit means it is
 * outstanding (not yet done). Lets a tool show what still needs calibrating. (ADR-026) */
#define MC_IF_CAL_DONE_ELECTRICAL      (0x0001u)  /* electrical-angle offset captured (alignment, 0x2700/1=1) */
#define MC_IF_CAL_DONE_MECH_ZERO       (0x0002u)  /* mechanical home set (0x2700/1=3)                          */
#define MC_IF_CAL_DONE_CURRENT_OFFSET  (0x0004u)  /* phase-current ADC offsets measured (0x2700/1=2)           */

/* ===== Homing / zeroing (0x2700/9 home_status, RO) ===== (ADR-057)
 * PC-triggered drive-to-hard-stop for incremental encoders: drive at home_velocity until the armature
 * current exceeds home_current for ~300 ms (the stall), then set the encoder zero at that position. */
#define MC_IF_HOME_IDLE     (0u)
#define MC_IF_HOME_RUNNING  (1u)
#define MC_IF_HOME_DONE     (2u)
#define MC_IF_HOME_FAILED   (3u)

/* ===== Persistence (0x2800) ===== */
#define MC_IF_SAVE_MAGIC           (0x7376u)  /* write to 0x2800/1 to request a save */
#define MC_IF_FACTORY_RESET_MAGIC  (0x7274u)  /* write to 0x2800/3 to request factory reset */

/* store_status (0x2800/2, RO) bitfield */
#define MC_IF_STORE_VALID    (0x0001u)  /* a valid saved record exists in flash */
#define MC_IF_STORE_PENDING  (0x0002u)  /* a save is latched, awaiting power-stage-off to commit -- flash
                                         * can't be written while the drive is live; disable it to commit */

/* fault_flags (0x2600/1, RO) bitfield -- manufacturer faults */
#define MC_IF_FAULT_NO_CONFIG  (0x00000001u)  /* no valid persistent config loaded -> operational drive inhibited (ADR-051) */
#define MC_IF_FAULT_NOT_HOMED  (0x00000002u)  /* incremental encoder not yet zeroed -> position recalls blocked; re-home each power-up (ADR-057) */
#define MC_IF_FAULT_OVERCURRENT (0x00000004u)  /* over-current trip active (the OC trip); latched into fault_flags_latched too (ADR-058) */

/* ===== Bootloader control (0x1F51:1 program_control) ============================
 * PC tool writes these expedited to walk the target through an update. See
 * Documentation/dual_bootloader_design.md §4 for the full sequence. */
#define MC_IF_PROG_STOP            (0x00u)  /* Idle / cancel-in-progress. */
#define MC_IF_PROG_START           (0x01u)  /* Begin a download session — bootloader erases slot then waits for segments. */
#define MC_IF_PROG_VERIFY          (0x02u)  /* Compute CRC over the written image; flash_status reports VERIFYING then IDLE / FAULT. */
#define MC_IF_PROG_COMMIT          (0x03u)  /* Mark the new image valid and reboot into it. */
#define MC_IF_PROG_ABORT           (0x80u)  /* Force-cancel; discard any in-flight download. */

/* ===== Bootloader flash status (0x1F57:1 flash_status) ==========================
 * PC tool polls to show progress. IDLE = ready for the next command;
 * FAULT = the last operation failed (CRC mismatch, flash write error,
 * segment toggle mismatch, etc.). */
#define MC_IF_FLASH_IDLE           (0x0000u)
#define MC_IF_FLASH_ERASING        (0x0001u)
#define MC_IF_FLASH_PROGRAMMING    (0x0002u)
#define MC_IF_FLASH_VERIFYING      (0x0003u)
#define MC_IF_FLASH_FAULT          (0x0004u)

/* ===== Test-injection targets (0x2900/2) ===== */
#define MC_IF_INJECT_NONE          (0u)
#define MC_IF_INJECT_IQ            (1u)
#define MC_IF_INJECT_VELOCITY      (2u)
#define MC_IF_INJECT_POSITION      (3u)

/* ===== Loop-tuning test modes (0x2910/1 test_mode) ===== (ADR-030)
 * Motor-owned commissioning overlay: while the matching operational mode is enabled, an on-motor
 * signal generator drives that loop's reference for PID tuning. NOT a CiA-402 mode. */
#define MC_IF_TEST_MODE_OFF        (0u)   /* normal operation                                              */
#define MC_IF_TEST_MODE_VELOCITY   (1u)   /* generator -> velocity-loop demand (needs PROFILE_VELOCITY)    */
#define MC_IF_TEST_MODE_POSITION   (2u)   /* generator -> position demand, bypass trajectory (PROFILE_POSITION) */
#define MC_IF_TEST_MODE_CURRENT    (3u)   /* generator -> current/torque command (needs TORQUE mode)        */

/* ===== Telemetry (TX-PDO) map: 0x2A00 =====
 * Host-configurable list of OD entries streamed in the cyclic telemetry frame. The map is an
 * OD array: sub0 = count, sub1..MC_IF_TLM_MAX_ENTRIES = U32 map entries (RW). It is
 * RUNTIME-reconfigurable over the live link; see INTERFACE_SPEC.md "Telemetry mapping". */
#define MC_IF_TLM_MAP_INDEX     (0x2A00u)
#define MC_IF_TLM_MAX_ENTRIES   (16u)     /* 0x2A00:1 .. 0x2A00:16 */
#define MC_IF_TLM_MAX_BYTES     (40u)     /* mapped-blob budget in a 64-byte frame */

/* Map entry (U32) = (index<<16) | (subindex<<8) | bitlen, bitlen in bits (8/16/32). */
#define MC_IF_TLM_MAP_ENTRY(index, sub, bits) \
    (((uint32_t)(index) << 16) | ((uint32_t)(sub) << 8) | (uint32_t)(bits))
#define MC_IF_TLM_MAP_INDEX_OF(e) ((uint16_t)((e) >> 16))
#define MC_IF_TLM_MAP_SUB_OF(e)   ((uint8_t)((e) >> 8))
#define MC_IF_TLM_MAP_BITS_OF(e)  ((uint8_t)(e))

/* ===== axis_manager (CMC-owned) modes for op_mode (0x3020) =====
 * High-level operational modes the CMC's axis_manager exposes to controllers.
 * Distinct from CiA-402 modes_of_operation (0x6060) — axis_manager translates
 * these into the appropriate motor MCU mode + cyclic targets. */
#define MC_IF_AXIS_MODE_OFF                (0u)   /* no commands sent; motor stays disabled */
#define MC_IF_AXIS_MODE_JOYSTICK           (1u)   /* axis follows joystick_value * joystick_max_velocity */
#define MC_IF_AXIS_MODE_PROFILE_VELOCITY   (2u)   /* axis follows target_velocity (rad/s) */
#define MC_IF_AXIS_MODE_PROFILE_POSITION   (3u)   /* axis moves to target_position over target_time */
#define MC_IF_AXIS_MODE_HOLD               (4u)   /* axis holds its current position */
#define MC_IF_AXIS_MODE_TORQUE             (5u)   /* axis follows axis_target_current [A]; motor 0x6060 = MC_IF_MODE_TORQUE (REQ-0012) */

/* ===== axis_manager (CMC-owned) state values for axis_state (0x3000) ===== */
#define MC_IF_AXIS_STATE_DISABLED  (0u)
#define MC_IF_AXIS_STATE_READY     (1u)
#define MC_IF_AXIS_STATE_RUNNING   (2u)
#define MC_IF_AXIS_STATE_FAULT     (3u)

/**
 * @brief Canonical OD object list.  X(index, subindex, name, type, access, flags, owner)
 *
 * Manufacturer 0x2xxx values are FLOAT32 SI. Standard 0x1xxx/0x6xxx are scaled ints.
 * 0x3xxx are CMC-owned axis_manager entries (FLOAT32 SI / U8 modes & state).
 * Expand on each side to build the table; keep both ends generated from this list.
 *
 * Each side filters by owner:
 *   - motor MCU generates its OD table from entries with owner == MC_IF_OWNER_MOTOR
 *   - CMC      generates its OD table from entries with owner == MC_IF_OWNER_CMC
 *   - host tools iterate everything; the owner is shown alongside the entry
 */
#define MC_IF_OD_OBJECTS(X) \
    /* --- CiA-402 core --- */ \
    X(0x1000, 0, device_type,                 MC_IF_T_U32, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x1001, 0, error_register,              MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x603F, 0, error_code,                  MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6040, 0, controlword,                 MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6041, 0, statusword,                  MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6060, 0, modes_of_operation,          MC_IF_T_I8,  MC_IF_A_RW, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6061, 0, modes_of_operation_display,  MC_IF_T_I8,  MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* --- Motion targets/actuals (scaled int).
     * NOTE (v3): target_position / target_velocity / target_torque are no
     * longer carried in MC_IfCyclicCommand_t. They are now SDO-only writes
     * that the motor MCU stores until a move is triggered via the
     * controlword NEW_SETPOINT bit. The MC_IF_F_PDO flag here means
     * "PDO-mappable into the configurable telemetry blob (0x2A00)" -- you
     * can still graph commanded targets vs actuals from the host side. */ \
    X(0x607A, 0, target_position,             MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x607B, 0, target_position_time_ms,     MC_IF_T_U32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x6064, 0, position_actual,             MC_IF_T_I32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6081, 0, profile_velocity,            MC_IF_T_U32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x6083, 0, profile_acceleration,        MC_IF_T_U32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x6084, 0, profile_deceleration,        MC_IF_T_U32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x6085, 0, quick_stop_deceleration,     MC_IF_T_U32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x60FF, 0, target_velocity,             MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x606C, 0, velocity_actual,             MC_IF_T_I32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6071, 0, target_torque,               MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x6077, 0, torque_actual,               MC_IF_T_I32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* --- 0x2000 axis / motor model (float SI) --- */ \
    X(0x2000, 1, motor_kt_nm_per_a,           MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2000, 2, motor_inertia_kg_m2,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2000, 3, motor_resistance_ohm,        MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2000, 4, motor_inductance_h,          MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2000, 5, motor_pole_pairs,            MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Drive backend (per-board, applied at boot, ADR-039): 0 = BLDC/PMSM FOC 3-shunt, 1 = brushed-DC H-bridge. */ \
    X(0x2000, 6, motor_backend_sel,           MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* --- 0x2200 position controller --- */ \
    X(0x2200, 1, pos_kp,                      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2200, 2, pos_ki,                      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2200, 3, pos_kd,                      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2200, 4, velocity_ff_gain,            MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* --- 0x2300 velocity controller + telemetry --- */ \
    X(0x2300, 1, vel_kp,                      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2300, 2, vel_ki,                      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2300, 3, vel_kd,                      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2300, 4, vel_current_limit_a,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2300, 5, vel_load_factor,             MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Velocity-demand acceleration ramp (ADR-042): slew the PROFILE_VELOCITY (joystick) demand toward the \
       setpoint under an acceleration cap -- accel_up [rad/s^2] while speeding up, accel_dn while slowing down \
       (0 = that phase disabled). accel_jerk [rad/s^3] eases the acceleration UP to the cap; the acceleration \
       falls FREELY on the way down (no knob), which is what keeps it from overshooting. accel_jerk 0 = step. \
       Operator-facing (joystick feel): the CMC should expose these in its joystick config UI as proxied motor writes. */ \
    X(0x2300, 6, vel_accel_up,                MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2300, 7, vel_accel_dn,                MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2300, 8, vel_accel_jerk,              MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Holding enable (ADR-054): 1 = hold when stopped (the PI provides whatever current is needed); 0 = */ \
    /* release the held current ~1 s after the axis settles at zero speed. Boolean -- not a current value. */ \
    X(0x2300, 9, holding_enable,              MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2310, 1, tlm_vel_demand_rad_s,        MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2310, 2, tlm_vel_actual_rad_s,        MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2310, 3, tlm_vel_iq_cmd_a,            MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* --- 0x2400 current/FOC gains + telemetry --- */ \
    X(0x2400, 1, foc_id_kp,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2400, 2, foc_id_ki,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2400, 3, foc_iq_kp,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2400, 4, foc_iq_ki,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2400, 5, foc_voltage_limit_v,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Brushed-DC armature-current PI (ADR-039): set motor R (0x2000:3), L (0x2000:4) + bandwidth (:8); \
       the motor DERIVES the gains kp = wc*L, ki = wc*R and reports them read-only at :6/:7. */ \
    X(0x2400, 6, hb_cur_kp,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2400, 7, hb_cur_ki,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2410, 1, tlm_id_meas_a,               MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2410, 2, tlm_iq_meas_a,               MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2410, 3, tlm_vd_v,                    MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2410, 4, tlm_vq_v,                    MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2410, 5, tlm_electrical_angle_rad,    MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2410, 6, tlm_i_arm_a,                 MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* --- 0x2500 encoder / state estimator + telemetry --- */ \
    X(0x2500, 1, est_electrical_offset_rad,   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2500, 2, est_velocity_filter_hz,      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2500, 3, est_obs_kp,                  MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2500, 4, est_obs_ki,                  MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2500, 5, est_obs_kv,                  MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2500, 6, est_use_observer,            MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Observer output low-pass coefficient (0..1, first-order at the 1 kHz estimator rate; ~57 Hz at 0.3). \
       This filters the velocity the loops use when use_observer=1 -- NOT velocity_filter_hz (ADR-003). */ \
    X(0x2500, 7, est_obs_filter_alpha,        MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Incremental quad encoder scale (ADR-052): signed counts/rev (= 4x lines); the sign sets count direction. */ \
    X(0x2500, 8, quad_counts_per_rev,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2510, 1, tlm_mech_position_rad,       MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2510, 2, tlm_mech_velocity_rad_s,     MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2510, 3, tlm_pos_demand_rad,          MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* Raw TIM2 quadrature count (4x / TI12), signed around the power-on zero. Diagnostic; ADR-050. */ \
    X(0x2510, 4, quad_encoder_count,          MC_IF_T_I32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* --- 0x2600 faults / limits / diagnostics --- */ \
    X(0x2600, 1, fault_flags,                 MC_IF_T_U32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2600, 2, current_trip_a,              MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2600, 3, tlm_bus_voltage_v,           MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    /* Motor-owned, enforced motion envelope (ADR-040): the motor clamps every move + the velocity \
       demand to these, regardless of the CMC's requested profile (0x6081/3/4). 0 = disabled. */ \
    X(0x2600, 4, max_velocity_rad_s,          MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2600, 5, max_accel_rad_s2,            MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Soft position limits (ADR-040): manually set (home-relative rad, like 0x6064); the motor clamps move \
       targets + stops the velocity demand at them + sets AT_LIMIT_LO/HI. lo >= hi = disabled. */ \
    X(0x2600, 6, pos_limit_lo_rad,            MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2600, 7, pos_limit_hi_rad,            MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Jerk-limited S-curve trajectory planner (ADR-045): max_jerk_rad_s3 = the fixed system jerk [rad/s^3]; \
       traj_use_scurve = 1 selects the S-curve planner for PROFILE_POSITION moves, 0 = trapezoidal (default). */ \
    X(0x2600, 8, max_jerk_rad_s3,             MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2600, 9, traj_use_scurve,             MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* Sticky OR of every fault_flags bit set since boot -- fault history (faults triggered previously + cleared). Not persisted. (ADR-058) */ \
    X(0x2600, 10, fault_flags_latched,        MC_IF_T_U32, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* Per-fault trigger counts (U16, since boot, saturating): how many times each fault_flags bit has RISEN; RAM only. :11 NO_CONFIG, :12 NOT_HOMED, :13 OVERCURRENT. (ADR-058) */ \
    X(0x2600, 11, fault_count_no_config,      MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2600, 12, fault_count_not_homed,      MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2600, 13, fault_count_overcurrent,    MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* --- 0x2700 calibration --- */ \
    X(0x2700, 1, cal_command,                 MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2700, 2, cal_status,                  MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2700, 3, cal_align_current_a,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2700, 4, cal_align_hold_ms,           MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2700, 5, cal_done_flags,              MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* Homing to a hard end stop for incremental encoders (ADR-057): drive at home_velocity (signed) until */ \
    /* movement goes negligible for 1 s (or an OC trip) -- the stall -- then set the encoder zero there.    */ \
    /* home_command: 1 = run, 0 = idle/abort (also clears done/failed). home_status: MC_IF_HOME_*.          */ \
    X(0x2700, 6, home_velocity_rad_s,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* home_current_a: DEPRECATED -- homing now finds the stop by no-movement + OC trip, not this current.  */ \
    /* Kept (unused) to avoid an OD-layout change / MC_IF_PROTOCOL_VERSION bump.                            */ \
    X(0x2700, 7, home_current_a,              MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2700, 8, home_command,                MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2700, 9, home_status,                 MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* mech_zero_set_rad: target for MC_IF_CAL_SET_MECH_ZERO_AT (cal_command 4) -- the tool writes the       */ \
    /* desired mech-home position (absolute frame, e.g. midpoint of two captured travel extremes) + fires    */ \
    /* the command. Transient (not persisted). (ADR-022)                                                    */ \
    X(0x2700, 10, mech_zero_set_rad,          MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* --- 0x2800 persistent store --- */ \
    X(0x2800, 1, store_save_command,          MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2800, 2, store_status,                MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2800, 3, store_factory_reset,         MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* --- 0x2900 commissioning / test injection (step changes for tuning) --- */ \
    X(0x2900, 1, inject_enable,               MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2900, 2, inject_target,               MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2900, 3, inject_step_amplitude,       MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2900, 4, inject_step_trigger,         MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* Debug DAC (PA4 / DAC1_OUT1) source select: 0=|iq| 1=iq 2=|id| 3=id 4=ia 5=ib 6=ic 7=i_max 8=i_arm. */ \
    X(0x2900, 5, dac_source,                  MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* d-axis voltage-step plant ID (ADR-046): open-loop Vd at a fixed electrical angle, fired from the PC \
       tool. dq_test_enable=1 applies dq_test_voltage_v (clamped +/-3 V) at dq_test_angle_rad; auto-disarms. */ \
    X(0x2900, 6, dq_test_voltage_v,           MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2900, 7, dq_test_angle_rad,           MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2900, 8, dq_test_enable,              MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2900, 9, dq_test_dwell_ms,            MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* d-axis pulse axis select (ADR-046 ext): 0 = d-axis, 1 = q-axis (FOC SVPWM); 2 = brushed phase (H-bridge). */ \
    X(0x2900, 10, dq_test_axis,               MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* --- 0x2910 loop-tuning test-signal overlay (ADR-030; amplitude/rate units follow test_mode) --- */ \
    X(0x2910, 1, test_mode,                   MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 2, test_amplitude,              MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 3, test_rate,                   MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 4, test_dwell_s,                MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 5, test_continuous,             MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 6, test_trigger,                MC_IF_T_U16, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 7, test_active,                 MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 8, test_signal,                 MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2910, 9, test_pause_s,                MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2910, 10, test_max_accel,             MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* --- 0x2920 stepped-sine current sweep for resonance / frequency-response ID (ADR-047) --- */ \
    X(0x2920, 1, freq_sweep_start_hz,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 2, freq_sweep_end_hz,           MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 3, freq_sweep_step_hz,          MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 4, freq_sweep_dwell_s,          MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 5, freq_sweep_bias_a,           MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 6, freq_sweep_amplitude_a,      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 7, freq_sweep_enable,           MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    X(0x2920, 8, freq_sweep_current_hz,       MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_PDO,     MC_IF_OWNER_MOTOR) \
    X(0x2920, 9, freq_sweep_active,           MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* --- 0x2930 current-command notch filter (resonance suppression, ADR-048) --- */ \
    X(0x2930, 1, notch_enable,                MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2930, 2, notch_freq_hz,               MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    X(0x2930, 3, notch_bandwidth_hz,          MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_MOTOR) \
    /* --- 0x2A00 telemetry map: sub0 = count; sub1..16 are U32 map entries (array) --- */ \
    X(0x2A00, 0, tlm_map_count,               MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_MOTOR) \
    /* === CMC-owned axis_manager entries (axis 0). Reserve 0x3100-0x31FF for axis 1, etc. === */ \
    /* --- 0x3000-0x300F state (RO) --- */ \
    X(0x3000, 0, axis_state,                  MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3001, 0, axis_op_mode_actual,         MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3002, 0, axis_position_actual,        MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3003, 0, axis_velocity_actual,        MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3004, 0, axis_error_code,             MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3005, 0, axis_error_register,         MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* --- 0x3010-0x301F commands (write-triggered) --- */ \
    X(0x3010, 0, axis_enable,                 MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3011, 0, axis_quick_stop,             MC_IF_T_U8,  MC_IF_A_WO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3012, 0, axis_clear_fault,            MC_IF_T_U8,  MC_IF_A_WO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3013, 0, axis_start_move,             MC_IF_T_U8,  MC_IF_A_WO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* CMC auto-clears motor faults after they have been active 5 s (defence \
     * against a transient fault the operator hasn't noticed). Counter is \
     * RO, U16, since-boot only (not persisted) — increments each time the \
     * 5 s timer fires whether or not the clear actually succeeds. */ \
    X(0x3014, 0, axis_auto_fault_clears,      MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* --- 0x3020-0x302F mode + per-mode targets --- */ \
    X(0x3020, 0, axis_op_mode,                MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3021, 0, axis_joystick_value,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* 0x3022 was RW-PERSIST through 4.7.x. As of 4.8.0 it is DERIVED from \
     * velocity_limit (0x3030) x joy_profile_scale (CAMERAD JOY_PROFILE_*): \
     * no longer independently settable, no longer separately persisted. \
     * Writes return ACCESS-denied; reads report the current derived value. */ \
    X(0x3022, 0, axis_joystick_max_velocity,  MC_IF_T_F32, MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3023, 0, axis_target_velocity,        MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3024, 0, axis_target_position,        MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3025, 0, axis_target_time,            MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* --- 0x3026-0x302A joystick calibration (raw -> normalised, symmetric output) --- */ \
    /* Protocol modules write the raw stick value to axis_joystick_raw; axis_manager     */ \
    /* normalises using the four cal entries below and updates axis_joystick_value.      */ \
    /* For sources that have an already-normalised value, write axis_joystick_value      */ \
    /* directly; that bypasses the raw-side cal.                                         */ \
    X(0x3026, 0, axis_joystick_raw,           MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3027, 0, axis_joystick_raw_center,    MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3028, 0, axis_joystick_raw_full_pos,  MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3029, 0, axis_joystick_raw_full_neg,  MC_IF_T_I32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x302A, 0, axis_joystick_raw_deadband,  MC_IF_T_U32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    /* Operator current command (REQ-0012). Effective only in AXIS_MODE_TORQUE.        */ \
    /* axis_manager SDO-writes target_torque (0x6071) = round(current_a / CUR_SCALE).  */ \
    X(0x302B, 0, axis_target_current,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* On-board UP/DOWN button current magnitude [A]. While AXIS_MODE_TORQUE is active */ \
    /* and a button is held, axis_manager overrides axis_target_current to             */ \
    /* +button_current (UP) or -button_current (DOWN). Released -> 0. Default 0 A.     */ \
    X(0x302C, 0, axis_button_current,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* --- 0x3030-0x303F limits --- */ \
    X(0x3030, 0, axis_velocity_limit,         MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3031, 0, axis_position_limit_lo,      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3032, 0, axis_position_limit_hi,      MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3033, 0, axis_accel_limit,            MC_IF_T_F32, MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    /* --- 0x3040-0x304F home-to-endstop control surface ---                */ \
    /* CMC-owned mirror of the motor's home sequence (motor's own entries   */ \
    /* live at 0x2700:6/7/8/9). axis_manager runs the sequencer: SDO writes */ \
    /* motor 0x2700:8=1, polls motor 0x2700:9 for DONE / FAILED, then       */ \
    /* re-reads motor 0x2600:1 fault_flags to update is_homed. cmc_state    */ \
    /* gates shot recalls on is_homed. Adds nothing to the wire — just      */ \
    /* re-exposes the motor state through the CMC axis so the PC tool /     */ \
    /* web UI can drive the whole procedure via a single command surface.   */ \
    /* home_command: WO, write 1 to start (0 = abort/idle also forwarded).  */ \
    /* home_status:  RO, reflects the motor's MC_IF_HOME_* enum.            */ \
    /* is_homed:     RO, 1 if motor fault_flags does NOT have NOT_HOMED.    */ \
    X(0x3040, 0, axis_home_command,           MC_IF_T_U8,  MC_IF_A_WO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3041, 0, axis_home_status,            MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3042, 0, axis_is_homed,               MC_IF_T_U8,  MC_IF_A_RO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* --- 0x3070 axis_role — which CAMERAD movement axis this physical CMC \
     * consumes from every MOVEMENT frame. Values mirror CAMERAD_AXIS_* \
     * bitmap: 0x01=PAN, 0x02=TILT, 0x04=ZOOM, 0x08=FOCUS, 0x10=X, 0x20=Y, \
     * 0x40=HEIGHT, 0x80=FADER. controller_mgr uses this to pick the right \
     * field out of camerad_movement_t. Default 0x01 (PAN) so existing \
     * pan-axis units keep behaving as before. PERSIST in axis_persist_blob. */ \
    X(0x3070, 0, axis_role,                   MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    /* --- 0x3050-0x305F CMC persistence triggers --- */ \
    /* Write MC_IF_SAVE_MAGIC (0x7376) to commit the corresponding region   */ \
    /* to the CMC's internal flash. Same magic constant used by the motor   */ \
    /* MCU's 0x2800:1 save_command; this is the CMC-side equivalent.        */ \
    X(0x3050, 0, cmc_save_config,             MC_IF_T_U16, MC_IF_A_WO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    X(0x3051, 0, cmc_save_shots,              MC_IF_T_U16, MC_IF_A_WO, MC_IF_F_NONE,    MC_IF_OWNER_CMC) \
    /* --- 0x3060-0x306F CMC on-board RGB status LED (PC0/PC1/PC2 = TIM1_CH1/2/3) --- */ \
    /* Operator-tunable indicator colour. led_indicator drives the pattern (boot      */ \
    /* solid, network-link flash 3x, breathing while motor moving, idle solid); the   */ \
    /* configured colour below is what the pattern modulates. Saved with cmc_save_    */ \
    /* config (rides the axis_persist blob — AXIS_PERSIST_VERSION bumped 3 -> 4).    */ \
    X(0x3060, 0, led_color_r,                 MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3061, 0, led_color_g,                 MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    X(0x3062, 0, led_color_b,                 MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_PERSIST, MC_IF_OWNER_CMC) \
    /* --- 0x1F5x CiA-302 bootloader control surface — same OD entries on   */ \
    /* both MCUs. Dispatched by the bootloader when the target is in         */ \
    /* MC_IF_NODE_BOOTLOADER; refused (ERR_NOT_BOOTLOADER) when the app is   */ \
    /* running. Owner is BOOTLOADER so the app's X-macro filter drops them   */ \
    /* from the app-side OD table. See Documentation/dual_bootloader_design. */ \
    /*                                                                      */ \
    /* 0x1F50:1 program_data — LOGICAL sink for firmware bytes. Not written */ \
    /* via expedited MC_IF_MSG_OD_WRITE_REQ (max payload 4 B, useless);     */ \
    /* bytes flow via MC_IF_MSG_OD_DOWNLOAD_INIT + _SEGMENT. Kept in the OD */ \
    /* so the entry is enumerable + ownership is explicit. Type U8 is       */ \
    /* nominal — no host reads it as a typed scalar.                        */ \
    X(0x1F50, 1, program_data,                MC_IF_T_U8,  MC_IF_A_WO, MC_IF_F_NONE, MC_IF_OWNER_BOOTLOADER) \
    /* 0x1F51:1 program_control — expedited state command:                 */ \
    /*   0x00 stop, 0x01 start-download, 0x02 verify (CRC),                */ \
    /*   0x03 commit + reset, 0x80 abort.                                  */ \
    X(0x1F51, 1, program_control,             MC_IF_T_U8,  MC_IF_A_RW, MC_IF_F_NONE, MC_IF_OWNER_BOOTLOADER) \
    /* 0x1F56:1 program_software_id — CRC32 of currently running image;    */ \
    /* PC tool reads this to skip re-flashing an image that's already up   */ \
    /* to date and to confirm a post-update reboot.                        */ \
    X(0x1F56, 1, program_software_id,         MC_IF_T_U32, MC_IF_A_RO, MC_IF_F_NONE, MC_IF_OWNER_BOOTLOADER) \
    /* 0x1F57:1 flash_status — MC_IF_FLASH_* enum: IDLE/ERASING/PROGRAMMING/*/ \
    /* VERIFYING/FAULT. Polled by the PC tool for progress + result.        */ \
    X(0x1F57, 1, flash_status,                MC_IF_T_U16, MC_IF_A_RO, MC_IF_F_NONE, MC_IF_OWNER_BOOTLOADER)

#endif /* MC_IF_OD_H */
