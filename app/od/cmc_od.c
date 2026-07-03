/*
 * app/od/cmc_od — OD dispatcher for CMC-owned entries.
 *
 * Routes reads/writes for 0x3000-0x303F to the axis_manager accessors.
 *
 * The dispatch is a switch on (idx, sub) — verbose but obvious. If/when
 * the CMC OD grows beyond a handful of entries, consider switching to an
 * X-macro-generated dispatch table so the OD definition and the dispatch
 * cannot drift.
 */

#include "cmc_od.h"

#include "app/axis_manager/axis_manager.h"
#include "app/cmc_state/cmc_state.h"
#include "app/led_indicator/led_indicator.h"
#include "app/log/log.h"

#include <string.h>

/*----------------------------------------------------------------------------
 * Type / endianness helpers
 *---------------------------------------------------------------------------*/

static uint8_t type_size_bytes(MC_IfOdType_t t)
{
    switch (t) {
    case MC_IF_T_U8:  case MC_IF_T_I8:                       return 1;
    case MC_IF_T_U16: case MC_IF_T_I16:                      return 2;
    case MC_IF_T_U32: case MC_IF_T_I32: case MC_IF_T_F32:    return 4;
    default:                                                  return 0;
    }
}

/* Write a value of the given type to out_data (little-endian, length per type). */
static void put_u8 (uint8_t *p, uint8_t  v) { p[0] = v; }
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static void put_f32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    put_u32(p, u);
}

static uint8_t  get_u8 (const uint8_t *p) { return p[0]; }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t  get_i32(const uint8_t *p) { return (int32_t)get_u32(p); }
static float    get_f32(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

/*----------------------------------------------------------------------------
 * Range check
 *---------------------------------------------------------------------------*/

bool cmc_od_owns(uint16_t index)
{
    return index >= 0x3000u && index <= 0x30FFu;
}

/*----------------------------------------------------------------------------
 * Read dispatch
 *---------------------------------------------------------------------------*/

#define READ_U8(val)  do { put_u8 (out_data, (uint8_t)(val));  *out_type = MC_IF_T_U8;  *out_len = 1; return MC_IF_OD_OK; } while (0)
#define READ_U16(val) do { put_u16(out_data, (uint16_t)(val)); *out_type = MC_IF_T_U16; *out_len = 2; return MC_IF_OD_OK; } while (0)
#define READ_F32(val) do { put_f32(out_data, (float)(val));    *out_type = MC_IF_T_F32; *out_len = 4; return MC_IF_OD_OK; } while (0)

MC_IfOdResult_t cmc_od_read(uint16_t idx, uint8_t sub,
                            MC_IfOdType_t expected_type,
                            uint8_t *out_type, uint8_t *out_data, uint8_t *out_len)
{
    (void)expected_type;   /* informational only; caller validates against returned type */

    if (sub != 0) return MC_IF_OD_ERR_NO_SUB;

    switch (idx) {
    /* --- 0x3000-0x300F state --- */
    case 0x3000: READ_U8 (axis_manager_get_state());
    case 0x3001: READ_U8 (axis_manager_get_op_mode_actual());
    case 0x3002: READ_F32(axis_manager_get_position_actual());
    case 0x3003: READ_F32(axis_manager_get_velocity_actual());
    case 0x3004: READ_U16(axis_manager_get_error_code());
    case 0x3005: READ_U8 (axis_manager_get_error_register());

    /* --- 0x3010-0x301F commands (WO) — reading these is access-denied --- */
    case 0x3010: READ_U8(0);    /* axis_enable is RW; return current latch */
    case 0x3011:                /* quick_stop is WO */
    case 0x3012:                /* clear_fault is WO */
    case 0x3013:                /* start_move  is WO */
        return MC_IF_OD_ERR_ACCESS;
    case 0x3014: READ_U16(axis_manager_get_auto_fault_clears());

    /* --- 0x3020-0x302F mode + targets --- */
    case 0x3020: READ_U8 (axis_manager_get_op_mode());
    case 0x3021: READ_F32(axis_manager_get_joystick_value());
    case 0x3022: READ_F32(axis_manager_get_joystick_max_velocity());
    case 0x3023: READ_F32(axis_manager_get_target_velocity());
    case 0x3024: READ_F32(axis_manager_get_target_position());
    case 0x3025: READ_F32(axis_manager_get_target_time());

    /* --- 0x3026-0x302A joystick raw + calibration --- */
    case 0x3026: {
        int32_t v = axis_manager_get_joystick_raw();
        put_u32(out_data, (uint32_t)v); *out_type = MC_IF_T_I32; *out_len = 4;
        return MC_IF_OD_OK;
    }
    case 0x3027: {
        int32_t v = axis_manager_get_joystick_raw_center();
        put_u32(out_data, (uint32_t)v); *out_type = MC_IF_T_I32; *out_len = 4;
        return MC_IF_OD_OK;
    }
    case 0x3028: {
        int32_t v = axis_manager_get_joystick_raw_full_pos();
        put_u32(out_data, (uint32_t)v); *out_type = MC_IF_T_I32; *out_len = 4;
        return MC_IF_OD_OK;
    }
    case 0x3029: {
        int32_t v = axis_manager_get_joystick_raw_full_neg();
        put_u32(out_data, (uint32_t)v); *out_type = MC_IF_T_I32; *out_len = 4;
        return MC_IF_OD_OK;
    }
    case 0x302A: {
        uint32_t v = axis_manager_get_joystick_raw_deadband();
        put_u32(out_data, v); *out_type = MC_IF_T_U32; *out_len = 4;
        return MC_IF_OD_OK;
    }

    /* --- 0x302B operator current command (REQ-0012 TORQUE mode) --- */
    case 0x302B: READ_F32(axis_manager_get_target_current());
    /* --- 0x302C on-board UP/DOWN button current magnitude --- */
    case 0x302C: READ_F32(axis_manager_get_button_current());

    /* --- 0x3030-0x303F limits --- */
    case 0x3030: READ_F32(axis_manager_get_velocity_limit());
    case 0x3031: READ_F32(axis_manager_get_position_limit_lo());
    case 0x3032: READ_F32(axis_manager_get_position_limit_hi());
    case 0x3033: READ_F32(axis_manager_get_accel_limit());

    /* --- 0x3040-0x304F home-to-endstop control surface --- */
    case 0x3040:                                                        /* WO */
        return MC_IF_OD_ERR_ACCESS;
    case 0x3041: READ_U8(axis_manager_get_home_status());
    case 0x3042: READ_U8(axis_manager_is_homed() ? 1u : 0u);
    case 0x3070: READ_U8(axis_manager_get_axis_role());

    /* --- 0x3050-0x305F persistence triggers (WO) --- */
    case 0x3050:                /* cmc_save_config */
    case 0x3051:                /* cmc_save_shots */
        return MC_IF_OD_ERR_ACCESS;

    /* --- 0x3060-0x3062 RGB status LED colour --- */
    case 0x3060: {
        out_data[0] = led_indicator_get_r();
        *out_type = MC_IF_T_U8; *out_len = 1; return MC_IF_OD_OK;
    }
    case 0x3061: {
        out_data[0] = led_indicator_get_g();
        *out_type = MC_IF_T_U8; *out_len = 1; return MC_IF_OD_OK;
    }
    case 0x3062: {
        out_data[0] = led_indicator_get_b();
        *out_type = MC_IF_T_U8; *out_len = 1; return MC_IF_OD_OK;
    }

    default:
        return MC_IF_OD_ERR_NO_OBJECT;
    }
}

/*----------------------------------------------------------------------------
 * Write dispatch
 *---------------------------------------------------------------------------*/

static MC_IfOdResult_t check_write_size(MC_IfOdType_t t, uint8_t len)
{
    uint8_t expected = type_size_bytes(t);
    if (expected == 0)     return MC_IF_OD_ERR_TYPE;
    if (len != expected)   return MC_IF_OD_ERR_SIZE;
    return MC_IF_OD_OK;
}

#define WRITE_OK_OR(rc) do { if (!(rc)) return MC_IF_OD_ERR_RANGE; return MC_IF_OD_OK; } while (0)

/* Helper: human-readable OD result code (for debug logs). */
static const char *od_result_name(MC_IfOdResult_t r)
{
    switch (r) {
    case MC_IF_OD_OK:             return "OK";
    case MC_IF_OD_ERR_NO_OBJECT:  return "NO_OBJECT";
    case MC_IF_OD_ERR_NO_SUB:     return "NO_SUB";
    case MC_IF_OD_ERR_ACCESS:     return "ACCESS";
    case MC_IF_OD_ERR_TYPE:       return "TYPE";
    case MC_IF_OD_ERR_SIZE:       return "SIZE";
    case MC_IF_OD_ERR_RANGE:      return "RANGE";
    case MC_IF_OD_ERR_CALLBACK:   return "CALLBACK";
    case MC_IF_OD_ERR_NOT_READY:  return "NOT_READY";
    default:                      return "?";
    }
}

/* Indices the operator triggers from the GUI / web. We log every write to
 * these regardless of outcome so the user can trace "GUI sent X" all the
 * way to "axis_manager accepted X". Telemetry-rate writes (joystick raw,
 * etc.) are excluded so the log isn't flooded. */
static bool is_loggable_write(uint16_t idx)
{
    switch (idx) {
    case 0x3010:  /* axis_enable                   */
    case 0x3011:  /* axis_quick_stop               */
    case 0x3012:  /* axis_clear_fault              */
    case 0x3013:  /* axis_start_move               */
    case 0x3020:  /* axis_op_mode                  */
    case 0x3022:  /* axis_joystick_max_velocity    */
    case 0x3023:  /* axis_target_velocity          */
    case 0x3024:  /* axis_target_position          */
    case 0x3025:  /* axis_target_time              */
    case 0x302B:  /* axis_target_current (REQ-0012)*/
    case 0x302C:  /* axis_button_current           */
    case 0x3030:  /* axis_velocity_limit           */
    case 0x3031:  /* axis_position_limit_lo        */
    case 0x3032:  /* axis_position_limit_hi        */
    case 0x3033:  /* axis_accel_limit              */
    case 0x3050:  /* cmc_save_config trigger       */
    case 0x3051:  /* cmc_save_shots trigger        */
    case 0x3060:  /* led_color_r                   */
    case 0x3061:  /* led_color_g                   */
    case 0x3062:  /* led_color_b                   */
        return true;
    default:
        return false;
    }
}

/* Cheap preview of incoming data for the log: shows up to 4 bytes
 * little-endian as a hex value so the operator can see what was
 * actually written without us having to type-switch. */
static uint32_t peek_value_le(const uint8_t *p, uint8_t len)
{
    uint32_t v = 0;
    if (len > 4) len = 4;
    for (uint8_t i = 0; i < len; i++) v |= ((uint32_t)p[i]) << (8u * i);
    return v;
}

/* Internal — does the actual dispatch. The public wrapper below logs
 * loggable-write requests + their outcome around this. */
static MC_IfOdResult_t cmc_od_write_inner(uint16_t idx, uint8_t sub,
                                          MC_IfOdType_t in_type,
                                          const uint8_t *in_data, uint8_t in_len)
{
    if (sub != 0) return MC_IF_OD_ERR_NO_SUB;

    /* Validate size against the entry's declared type. The OD declares
     * U8/U16/F32 etc.; the wire format carries the actual length. */
    MC_IfOdResult_t sz;

    switch (idx) {
    /* --- 0x3000-0x300F state (RO) --- */
    case 0x3000: case 0x3001: case 0x3002:
    case 0x3003: case 0x3004: case 0x3005:
        return MC_IF_OD_ERR_ACCESS;

    /* --- 0x3010 axis_enable (U8 RW) --- */
    case 0x3010:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_request_enable(get_u8(in_data) != 0));

    /* --- 0x3011 axis_quick_stop (U8 WO, write 1 to trigger) --- */
    case 0x3011:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        if (get_u8(in_data) != 1)  return MC_IF_OD_ERR_RANGE;
        WRITE_OK_OR(axis_manager_request_quick_stop());

    /* --- 0x3012 axis_clear_fault (U8 WO, write 1 to clear) --- */
    case 0x3012:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        if (get_u8(in_data) != 1)  return MC_IF_OD_ERR_RANGE;
        WRITE_OK_OR(axis_manager_request_clear_fault());

    /* --- 0x3013 axis_start_move (U8 WO, write 1 to fire NEW_SETPOINT) --- */
    case 0x3013:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        if (get_u8(in_data) != 1)  return MC_IF_OD_ERR_RANGE;
        WRITE_OK_OR(axis_manager_request_start_move());

    /* --- 0x3014 axis_auto_fault_clears (U16 RO) — diagnostic counter --- */
    case 0x3014:
        return MC_IF_OD_ERR_ACCESS;

    /* --- 0x3020 axis_op_mode (U8 RW) --- */
    case 0x3020:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_op_mode(get_u8(in_data)));

    /* --- 0x3021..0x3025 mode targets (F32 RW) --- */
    case 0x3021:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_joystick_value(get_f32(in_data)));
    /* 0x3022 axis_joystick_max_velocity — RO since 4.8.0. Derived from
     * velocity_limit × joy_profile_scale; not directly writable. */
    case 0x3022:
        return MC_IF_OD_ERR_ACCESS;
    case 0x3023:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_target_velocity(get_f32(in_data)));
    case 0x3024:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_target_position(get_f32(in_data)));
    case 0x3025:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_target_time(get_f32(in_data)));

    /* --- 0x3026..0x302A joystick raw + cal --- */
    case 0x3026:
        sz = check_write_size(MC_IF_T_I32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_I32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_joystick_raw(get_i32(in_data)));
    case 0x3027:
        sz = check_write_size(MC_IF_T_I32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_I32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_joystick_raw_center(get_i32(in_data)));
    case 0x3028:
        sz = check_write_size(MC_IF_T_I32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_I32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_joystick_raw_full_pos(get_i32(in_data)));
    case 0x3029:
        sz = check_write_size(MC_IF_T_I32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_I32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_joystick_raw_full_neg(get_i32(in_data)));
    case 0x302A:
        sz = check_write_size(MC_IF_T_U32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_joystick_raw_deadband(get_u32(in_data)));
    /* --- 0x302B operator current command (REQ-0012 TORQUE mode) --- */
    case 0x302B:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_target_current(get_f32(in_data)));
    /* --- 0x302C on-board UP/DOWN button current magnitude --- */
    case 0x302C:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_button_current(get_f32(in_data)));

    /* --- 0x3030..0x3033 limits (F32 RW) --- */
    case 0x3030:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_velocity_limit(get_f32(in_data)));
    case 0x3031:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_position_limit_lo(get_f32(in_data)));
    case 0x3032:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_position_limit_hi(get_f32(in_data)));
    case 0x3033:
        sz = check_write_size(MC_IF_T_F32, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_F32) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_accel_limit(get_f32(in_data)));

    /* --- 0x3040 axis_home_command — U8, write 1 to start homing.
     * 0 (idle/abort) currently no-op on CMC; the motor's own home_command
     * would need it for abort, but that path isn't required here. */
    case 0x3040:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        if (get_u8(in_data) != 1u) return MC_IF_OD_ERR_RANGE;
        WRITE_OK_OR(axis_manager_request_home());
    /* 0x3041 axis_home_status + 0x3042 axis_is_homed are RO. */
    case 0x3041:
    case 0x3042:
        return MC_IF_OD_ERR_ACCESS;

    /* --- 0x3070 axis_role — CAMERAD_AXIS_* bitmap (single bit). */
    case 0x3070:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(axis_manager_set_axis_role(get_u8(in_data)));

    /* --- 0x3050 cmc_save_config: write MC_IF_SAVE_MAGIC to commit ---
     * Two-side save: CMC's own persist (axis_persist blob — joystick cal,
     * motion limits, LED) AND the motor's flash (load_factor, vel_accel_*,
     * brushed gains, etc.). The motor-side save runs ASYNC through the
     * axis_manager sequencer (disable -> wait -> SDO -> re-enable, ~500 ms
     * total). We return OK as soon as the CMC-side save succeeds — the
     * motor save logs its own success in the debug stream and the operator
     * sees the motor briefly disabled. */
    case 0x3050:
        sz = check_write_size(MC_IF_T_U16, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U16)            return MC_IF_OD_ERR_TYPE;
        if (get_u16(in_data) != MC_IF_SAVE_MAGIC) return MC_IF_OD_ERR_RANGE;
        if (!axis_manager_save_to_flash()) return MC_IF_OD_ERR_CALLBACK;
        (void)axis_manager_request_motor_save();   /* fire-and-forget */
        return MC_IF_OD_OK;

    /* --- 0x3051 cmc_save_shots: write MC_IF_SAVE_MAGIC to commit --- */
    case 0x3051:
        sz = check_write_size(MC_IF_T_U16, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U16)            return MC_IF_OD_ERR_TYPE;
        if (get_u16(in_data) != MC_IF_SAVE_MAGIC) return MC_IF_OD_ERR_RANGE;
        return cmc_state_save_shots() ? MC_IF_OD_OK
                                      : MC_IF_OD_ERR_CALLBACK;

    /* --- 0x3060/61/62 RGB status LED colour --- */
    case 0x3060:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(led_indicator_set_r(in_data[0]));
    case 0x3061:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(led_indicator_set_g(in_data[0]));
    case 0x3062:
        sz = check_write_size(MC_IF_T_U8, in_len); if (sz != MC_IF_OD_OK) return sz;
        if (in_type != MC_IF_T_U8) return MC_IF_OD_ERR_TYPE;
        WRITE_OK_OR(led_indicator_set_b(in_data[0]));

    default:
        return MC_IF_OD_ERR_NO_OBJECT;
    }
}

/* Public entry point — adds before/after logging so operator-driven writes
 * can be traced end-to-end. Reads stay silent (the GUI polls them).
 * High-frequency setpoint writes (joystick raw, etc.) are excluded from
 * logging via is_loggable_write so the log doesn't get flooded. */
MC_IfOdResult_t cmc_od_write(uint16_t idx, uint8_t sub,
                             MC_IfOdType_t in_type,
                             const uint8_t *in_data, uint8_t in_len)
{
    bool log_it = is_loggable_write(idx);
    uint32_t preview = (in_data && in_len) ? peek_value_le(in_data, in_len) : 0;

    if (log_it) {
        LOG_INFO("cmc_od: WRITE 0x%04X:%u type=%u len=%u value=0x%lX",
                 (unsigned)idx, (unsigned)sub, (unsigned)in_type,
                 (unsigned)in_len, (unsigned long)preview);
    }

    MC_IfOdResult_t res = cmc_od_write_inner(idx, sub, in_type, in_data, in_len);

    if (log_it) {
        if (res == MC_IF_OD_OK) {
            LOG_INFO("cmc_od: WRITE 0x%04X -> OK", (unsigned)idx);
        } else {
            LOG_WARN("cmc_od: WRITE 0x%04X -> %s (0x%02X)",
                     (unsigned)idx, od_result_name(res), (unsigned)res);
        }
    }
    return res;
}
