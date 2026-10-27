#include "logic.h"

#include <stddef.h>

static RR_HOT int16_t clamp_cmd(int32_t value) {
    if (value > RR_COMMAND_MAX) {
        return RR_COMMAND_MAX;
    }
    if (value < -RR_COMMAND_MAX) {
        return -RR_COMMAND_MAX;
    }
    return (int16_t)value;
}

RR_HOT rr_drive_t rr_drive(int16_t left, int16_t right) {
    rr_drive_t command;
    command.left = clamp_cmd(left);
    command.right = clamp_cmd(right);
    return command;
}

RR_HOT bool rr_drive_is_stop(rr_drive_t command) {
    return command.left == 0 && command.right == 0;
}

static RR_HOT uint16_t clamp_percent(uint16_t percent) {
    if (percent < 1) {
        return 1;
    }
    if (percent > 100) {
        return 100;
    }
    return percent;
}

static RR_HOT int16_t scale_mag(int16_t value, uint16_t percent) {
    percent = clamp_percent(percent);
    return (int16_t)(((int32_t)value * (int32_t)percent) / 100);
}

void rr_config_default(rr_config_t *cfg) {
    cfg->drive.forward = 800;
    cfg->drive.reverse = 600;
    cfg->drive.turn_inner = 350;
    cfg->drive.turn_outer = 800;
    cfg->drive.reverse_turn_inner = 350;
    cfg->drive.reverse_turn_outer = 600;
    cfg->drive.accel_per_s = 2000;
    cfg->drive.decel_per_s = 4000;
    cfg->drive.direction_change_coast_ms = 60;

    cfg->motors.min_duty_permille = 200;
    cfg->motors.max_duty_permille = 1000;

    // 2 ms is the fastest period the validator allows. The radio task wakes
    // the loop earlier when a frame arrives; this is only the idle tick.
    cfg->safety.command_timeout_ms = 250;
    cfg->safety.link_timeout_ms = 2000;
    cfg->safety.control_period_ms = 2;

    cfg->battery.cell_count = 3;
    cfg->battery.warn_mv_per_cell = 3400;
    cfg->battery.critical_mv_per_cell = 3300;
    cfg->battery.divider_num = 127;
    cfg->battery.divider_den = 27;
    cfg->battery.calibration_offset_mv = 0;
    cfg->battery.ema_alpha_percent = 20;
    cfg->battery.critical_samples = 3;

    cfg->scales.slow_percent = 50;
    cfg->scales.normal_percent = 100;
    cfg->scales.fast_percent = 100;
}

static bool drive_value_ok(int16_t value) {
    return value > 0 && value <= RR_COMMAND_MAX;
}

int rr_config_validate(const rr_config_t *cfg) {
    const rr_drive_tuning_t *drive = &cfg->drive;
    if (!drive_value_ok(drive->forward) || !drive_value_ok(drive->reverse) ||
        !drive_value_ok(drive->turn_inner) || !drive_value_ok(drive->turn_outer) ||
        !drive_value_ok(drive->reverse_turn_inner) || !drive_value_ok(drive->reverse_turn_outer)) {
        return RR_CFG_INVALID_DRIVE;
    }
    if (drive->turn_inner > drive->turn_outer || drive->reverse_turn_inner > drive->reverse_turn_outer) {
        return RR_CFG_INVALID_TURN;
    }
    if (drive->accel_per_s <= 0 || drive->decel_per_s <= 0) {
        return RR_CFG_INVALID_RATE;
    }
    if (cfg->safety.command_timeout_ms < 50 || cfg->safety.command_timeout_ms > 2000) {
        return RR_CFG_INVALID_COMMAND_TIMEOUT;
    }
    if (cfg->safety.link_timeout_ms < 100 || cfg->safety.link_timeout_ms > 10000) {
        return RR_CFG_INVALID_LINK_TIMEOUT;
    }
    if (cfg->safety.control_period_ms < 2 || cfg->safety.control_period_ms > 100) {
        return RR_CFG_INVALID_PERIOD;
    }
    if (cfg->battery.cell_count == 0) {
        return RR_CFG_INVALID_BATTERY;
    }
    if (cfg->battery.warn_mv_per_cell < 2500 || cfg->battery.warn_mv_per_cell > 4200 ||
        cfg->battery.critical_mv_per_cell < 2500 || cfg->battery.critical_mv_per_cell > 4200 ||
        cfg->battery.critical_mv_per_cell >= cfg->battery.warn_mv_per_cell ||
        cfg->battery.ema_alpha_percent == 0 || cfg->battery.ema_alpha_percent > 100 ||
        cfg->battery.critical_samples == 0) {
        return RR_CFG_INVALID_BATTERY;
    }
    if (cfg->battery.divider_num == 0 || cfg->battery.divider_den == 0) {
        return RR_CFG_INVALID_DIVIDER;
    }
    if (cfg->scales.slow_percent == 0 || cfg->scales.slow_percent > 100 ||
        cfg->scales.normal_percent == 0 || cfg->scales.normal_percent > 100 ||
        cfg->scales.fast_percent == 0 || cfg->scales.fast_percent > 100 ||
        !(cfg->scales.slow_percent <= cfg->scales.normal_percent &&
          cfg->scales.normal_percent <= cfg->scales.fast_percent)) {
        return RR_CFG_INVALID_SCALE;
    }
    // Same rejection the Rust validator uses for a bad duty window, including
    // the historical error code (it reports the divider enum).
    if (cfg->motors.min_duty_permille == 0 ||
        cfg->motors.min_duty_permille > cfg->motors.max_duty_permille ||
        cfg->motors.max_duty_permille > 1000) {
        return RR_CFG_INVALID_DIVIDER;
    }
    return RR_CFG_OK;
}

const char *rr_config_error_str(int error) {
    switch (error) {
    case RR_CFG_OK:
        return "ok";
    case RR_CFG_INVALID_DRIVE:
        return "drive value out of range";
    case RR_CFG_INVALID_TURN:
        return "turn ratio inverted";
    case RR_CFG_INVALID_RATE:
        return "accel or decel is zero";
    case RR_CFG_INVALID_COMMAND_TIMEOUT:
        return "command timeout out of range";
    case RR_CFG_INVALID_LINK_TIMEOUT:
        return "link timeout out of range";
    case RR_CFG_INVALID_PERIOD:
        return "control period out of range";
    case RR_CFG_INVALID_BATTERY:
        return "battery thresholds inconsistent";
    case RR_CFG_INVALID_DIVIDER:
        return "divider or duty window invalid";
    case RR_CFG_INVALID_SCALE:
        return "speed scale out of range";
    default:
        return "unknown configuration error";
    }
}

uint16_t rr_scale_percent(const rr_config_t *cfg, uint8_t mode) {
    switch (mode) {
    case RR_MODE_SLOW:
        return cfg->scales.slow_percent;
    case RR_MODE_FAST:
        return cfg->scales.fast_percent;
    default:
        return cfg->scales.normal_percent;
    }
}

RR_HOT rr_resolved_t rr_steering_resolve(uint8_t mask, const rr_drive_tuning_t *tuning, uint16_t scale_percent) {
    rr_resolved_t resolved;
    resolved.command = rr_drive(0, 0);
    resolved.conflict = false;

    if ((mask & RR_BTN_STOP) != 0) {
        return resolved;
    }

    const bool forward = (mask & RR_BTN_FORWARD) != 0;
    const bool backward = (mask & RR_BTN_BACKWARD) != 0;
    const bool left = (mask & RR_BTN_LEFT) != 0;
    const bool right = (mask & RR_BTN_RIGHT) != 0;

    if ((forward && backward) || (left && right)) {
        resolved.conflict = true;
        return resolved;
    }

    int16_t left_mag = 0;
    int16_t right_mag = 0;
    if (forward) {
        if (left) {
            left_mag = tuning->turn_inner;
            right_mag = tuning->turn_outer;
        } else if (right) {
            left_mag = tuning->turn_outer;
            right_mag = tuning->turn_inner;
        } else {
            left_mag = tuning->forward;
            right_mag = tuning->forward;
        }
    } else if (backward) {
        if (left) {
            left_mag = tuning->reverse_turn_inner;
            right_mag = tuning->reverse_turn_outer;
        } else if (right) {
            left_mag = tuning->reverse_turn_outer;
            right_mag = tuning->reverse_turn_inner;
        } else {
            left_mag = tuning->reverse;
            right_mag = tuning->reverse;
        }
    } else if (left) {
        left_mag = tuning->turn_inner;
        right_mag = tuning->turn_outer;
    } else if (right) {
        left_mag = tuning->turn_outer;
        right_mag = tuning->turn_inner;
    } else {
        return resolved;
    }

    const int16_t sign = backward ? (int16_t)-1 : (int16_t)1;
    resolved.command = rr_drive((int16_t)(scale_mag(left_mag, scale_percent) * sign),
                                (int16_t)(scale_mag(right_mag, scale_percent) * sign));
    return resolved;
}

RR_HOT void rr_ramp_reset(rr_ramp_t *ramp) {
    ramp->current = rr_drive(0, 0);
    ramp->coast_remaining_ms = 0;
}

static RR_HOT int16_t approach(int16_t current, int16_t target, uint32_t dt_ms, const rr_drive_tuning_t *tuning) {
    if (current == target) {
        return target;
    }
    const uint16_t cur_mag = (uint16_t)(current < 0 ? -current : current);
    const uint16_t target_mag = (uint16_t)(target < 0 ? -target : target);
    const bool growing = target_mag > cur_mag;
    const int16_t rate = growing ? tuning->accel_per_s : tuning->decel_per_s;
    int32_t step32 = ((int32_t)rate * (int32_t)dt_ms) / 1000;
    if (step32 < 1) {
        step32 = 1;
    }
    const uint16_t step = (uint16_t)step32;
    uint16_t new_mag;
    if (growing) {
        const uint32_t sum = (uint32_t)cur_mag + step;
        new_mag = sum > target_mag ? target_mag : (uint16_t)sum;
    } else if (cur_mag > step) {
        const uint16_t dropped = (uint16_t)(cur_mag - step);
        new_mag = dropped < target_mag ? target_mag : dropped;
    } else {
        new_mag = target_mag;
    }
    if (new_mag > RR_COMMAND_MAX) {
        new_mag = RR_COMMAND_MAX;
    }
    const int16_t sign = target < 0 ? (int16_t)-1 : (int16_t)1;
    return (int16_t)(sign * (int16_t)new_mag);
}

RR_HOT rr_drive_t rr_ramp_update(rr_ramp_t *ramp, rr_drive_t target, uint32_t dt_ms, const rr_drive_tuning_t *tuning) {
    if (dt_ms == 0) {
        return ramp->current;
    }

    // A reversal coasts to zero. It never drives through zero under power.
    if (target.left != 0 && ramp->current.left != 0 && (target.left < 0) != (ramp->current.left < 0)) {
        ramp->current.left = 0;
        ramp->coast_remaining_ms = tuning->direction_change_coast_ms;
    }
    if (target.right != 0 && ramp->current.right != 0 && (target.right < 0) != (ramp->current.right < 0)) {
        ramp->current.right = 0;
        ramp->coast_remaining_ms = tuning->direction_change_coast_ms;
    }

    if (ramp->coast_remaining_ms > 0) {
        ramp->coast_remaining_ms = ramp->coast_remaining_ms > dt_ms ? ramp->coast_remaining_ms - dt_ms : 0;
        ramp->current = rr_drive(0, 0);
        return ramp->current;
    }

    ramp->current.left = approach(ramp->current.left, target.left, dt_ms, tuning);
    ramp->current.right = approach(ramp->current.right, target.right, dt_ms, tuning);
    return ramp->current;
}

void rr_safety_init(rr_safety_t *safety, const rr_safety_tuning_t *tuning) {
    safety->state = RR_STATE_BOOT;
    safety->has_fault = false;
    safety->fault = 0;
    safety->last_command = rr_drive(0, 0);
    safety->has_command_ms = false;
    safety->last_command_ms = 0;
    safety->link_up = false;
    safety->battery_ok = true;
    safety->tuning = *tuning;
}

RR_HOT bool rr_safety_enabled(const rr_safety_t *safety) {
    return safety->state == RR_STATE_ARMED || safety->state == RR_STATE_DRIVING;
}

RR_HOT void rr_safety_boot_complete(rr_safety_t *safety) {
    if (safety->state == RR_STATE_BOOT) {
        safety->state = RR_STATE_DISARMED;
    }
}

RR_HOT int rr_safety_arm(rr_safety_t *safety) {
    if (safety->state == RR_STATE_BOOT) {
        return RR_ARM_NOT_INITIALISED;
    }
    if (safety->state == RR_STATE_FAULT) {
        return RR_ARM_FAULT_LATCHED;
    }
    if (safety->state == RR_STATE_ARMED || safety->state == RR_STATE_DRIVING) {
        return RR_ARM_ALREADY;
    }
    safety->state = RR_STATE_ARMED;
    safety->last_command = rr_drive(0, 0);
    safety->has_command_ms = false;
    return RR_ARM_OK;
}

RR_HOT void rr_safety_disarm(rr_safety_t *safety) {
    safety->last_command = rr_drive(0, 0);
    safety->has_command_ms = false;
    // Matches the Rust supervisor: disarm leaves the fault *value* in place
    // but the state becomes disarmed, so a later arm is allowed. The phone
    // only renders the fault value while the state byte is FAULT.
    if (safety->state != RR_STATE_BOOT) {
        safety->state = RR_STATE_DISARMED;
    }
}

RR_HOT int rr_safety_acknowledge(rr_safety_t *safety) {
    if (safety->state != RR_STATE_FAULT) {
        return RR_ACK_OK;
    }
    if (!safety->link_up) {
        return RR_ACK_LINK_STILL_DOWN;
    }
    if (!safety->battery_ok) {
        return RR_ACK_BATTERY_STILL_BAD;
    }
    safety->has_fault = false;
    safety->fault = 0;
    safety->state = RR_STATE_DISARMED;
    safety->last_command = rr_drive(0, 0);
    safety->has_command_ms = false;
    return RR_ACK_OK;
}

RR_HOT void rr_safety_latch(rr_safety_t *safety, uint8_t fault) {
    safety->has_fault = true;
    safety->fault = fault;
    safety->state = RR_STATE_FAULT;
    safety->last_command = rr_drive(0, 0);
    safety->has_command_ms = false;
}

RR_HOT void rr_safety_note_link(rr_safety_t *safety, bool up) {
    if (safety->link_up && !up && rr_safety_enabled(safety)) {
        rr_safety_latch(safety, RR_FAULT_LINK_LOST);
    }
    safety->link_up = up;
}

RR_HOT void rr_safety_note_battery(rr_safety_t *safety, bool ok) {
    if (!ok && rr_safety_enabled(safety)) {
        rr_safety_latch(safety, RR_FAULT_LOW_BATTERY);
    }
    safety->battery_ok = ok;
}

RR_HOT void rr_safety_estop(rr_safety_t *safety) {
    rr_safety_latch(safety, RR_FAULT_ESTOP);
}

RR_HOT void rr_safety_note_command(rr_safety_t *safety, uint32_t now_ms, rr_drive_t command) {
    if (!rr_safety_enabled(safety)) {
        return;
    }
    safety->last_command = command;
    safety->has_command_ms = true;
    safety->last_command_ms = now_ms;
    safety->state = rr_drive_is_stop(command) ? RR_STATE_ARMED : RR_STATE_DRIVING;
}

RR_HOT rr_output_t rr_safety_update(rr_safety_t *safety, uint32_t now_ms) {
    // A stopped robot is allowed to sit armed. Only a command that asked for
    // motion can time out and latch a fault.
    if (rr_safety_enabled(safety) && !rr_drive_is_stop(safety->last_command) && safety->has_command_ms) {
        const uint32_t age = now_ms - safety->last_command_ms;
        if (age > safety->tuning.command_timeout_ms) {
            rr_safety_latch(safety, RR_FAULT_COMMAND_TIMEOUT);
        }
    }

    rr_output_t output;
    output.kind = RR_OUTPUT_DISABLED;
    output.command = rr_drive(0, 0);
    if ((safety->state == RR_STATE_ARMED || safety->state == RR_STATE_DRIVING) &&
        !rr_drive_is_stop(safety->last_command)) {
        output.kind = RR_OUTPUT_MOVE;
        output.command = safety->last_command;
    }
    return output;
}

static void write_u16(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)((value >> 8) & 0xffu);
}

static void write_u32(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)((value >> 8) & 0xffu);
    dst[2] = (uint8_t)((value >> 16) & 0xffu);
    dst[3] = (uint8_t)((value >> 24) & 0xffu);
}

static uint16_t read_u16(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t read_u32(const uint8_t *src) {
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

int rr_frame_decode(const uint8_t *buf, uint32_t len, rr_client_frame_t *out) {
    if (len < RR_CLIENT_FRAME_LEN) {
        return RR_PROTO_TOO_SHORT;
    }
    if (len > RR_CLIENT_FRAME_LEN) {
        return RR_PROTO_TOO_LONG;
    }
    if (buf[0] != RR_MAGIC) {
        return RR_PROTO_BAD_MAGIC;
    }
    if (buf[1] != RR_VERSION) {
        return RR_PROTO_BAD_VERSION;
    }
    const uint8_t kind = buf[2];
    if (kind != RR_KIND_DRIVE && kind != RR_KIND_ARM && kind != RR_KIND_DISARM && kind != RR_KIND_PING &&
        kind != RR_KIND_ACK) {
        return RR_PROTO_UNKNOWN_KIND;
    }
    if ((buf[3] & (uint8_t)~RR_BTN_MASK) != 0) {
        return RR_PROTO_INVALID_BUTTONS;
    }
    if (buf[4] > RR_MODE_FAST) {
        return RR_PROTO_INVALID_MODE;
    }
    out->kind = kind;
    out->buttons = buf[3];
    out->mode = buf[4];
    out->flags = buf[5];
    out->seq = read_u16(buf + 6);
    out->token = read_u32(buf + 8);
    return RR_PROTO_OK;
}

rr_decoder_t rr_decoder_new(uint32_t token) {
    rr_decoder_t decoder;
    decoder.token = token;
    decoder.has_seq = false;
    decoder.last_seq = 0;
    return decoder;
}

int rr_decoder_decode(rr_decoder_t *decoder, const uint8_t *buf, uint32_t len, rr_client_frame_t *out) {
    const int structural = rr_frame_decode(buf, len, out);
    if (structural != RR_PROTO_OK) {
        return structural;
    }
    if (out->token != decoder->token) {
        return RR_PROTO_BAD_TOKEN;
    }
    if (decoder->has_seq) {
        const uint16_t distance = (uint16_t)(out->seq - decoder->last_seq);
        if (distance == 0 || distance > 0x8000u) {
            return RR_PROTO_STALE_SEQUENCE;
        }
    }
    decoder->has_seq = true;
    decoder->last_seq = out->seq;
    return RR_PROTO_OK;
}

static void write_header(uint8_t *out, uint8_t kind) {
    out[0] = RR_MAGIC;
    out[1] = RR_VERSION;
    out[2] = kind;
}

void rr_encode_status(uint8_t state, bool has_fault, uint8_t fault, uint8_t flags, uint16_t battery_mv, int16_t left,
                      int16_t right, uint16_t last_seq, uint8_t mode, uint8_t out[RR_STATUS_FRAME_LEN]) {
    write_header(out, RR_SERVER_STATUS);
    out[3] = state;
    out[4] = has_fault ? fault : 0;
    out[5] = flags;
    write_u16(out + 6, battery_mv);
    write_u16(out + 8, (uint16_t)left);
    write_u16(out + 10, (uint16_t)right);
    write_u16(out + 12, last_seq);
    out[14] = mode;
    out[15] = 0;
}

void rr_encode_hello(uint32_t token, uint32_t uptime_ms, uint8_t out[RR_HELLO_FRAME_LEN]) {
    write_header(out, RR_SERVER_HELLO);
    out[3] = 0;
    write_u32(out + 4, token);
    write_u32(out + 8, uptime_ms);
}

void rr_encode_pong(uint32_t client_ts, uint32_t uptime_ms, uint8_t out[RR_PONG_FRAME_LEN]) {
    write_header(out, RR_SERVER_PONG);
    out[3] = 0;
    write_u32(out + 4, client_ts);
    write_u32(out + 8, uptime_ms);
}

void rr_encode_error(uint8_t code, uint8_t out[RR_ERROR_FRAME_LEN]) {
    write_header(out, RR_SERVER_ERROR);
    out[3] = code;
    write_u32(out + 4, 0);
}

void rr_battery_init(rr_battery_t *battery) {
    battery->has_mv = false;
    battery->filtered_mv = 0;
    battery->critical_streak = 0;
}

RR_HOT uint32_t rr_battery_warn_mv(const rr_battery_cfg_t *cfg) {
    return (uint32_t)cfg->warn_mv_per_cell * cfg->cell_count;
}

RR_HOT uint32_t rr_battery_critical_mv(const rr_battery_cfg_t *cfg) {
    return (uint32_t)cfg->critical_mv_per_cell * cfg->cell_count;
}

RR_HOT uint32_t rr_adc_to_pack_mv(uint32_t adc_mv, const rr_battery_cfg_t *cfg) {
    const uint64_t scaled = ((uint64_t)adc_mv * cfg->divider_num) / cfg->divider_den;
    int64_t pack = (int64_t)scaled + cfg->calibration_offset_mv;
    if (pack < 0) {
        pack = 0;
    }
    if ((uint64_t)pack > 0xffffffffull) {
        return 0xffffffffu;
    }
    return (uint32_t)pack;
}

static RR_HOT uint8_t classify(uint32_t pack_mv, const rr_battery_cfg_t *cfg) {
    if (pack_mv < rr_battery_critical_mv(cfg)) {
        return RR_BATT_CRITICAL;
    }
    if (pack_mv < rr_battery_warn_mv(cfg)) {
        return RR_BATT_WARNING;
    }
    return RR_BATT_OK;
}

RR_HOT uint8_t rr_battery_update(rr_battery_t *battery, uint32_t adc_mv, const rr_battery_cfg_t *cfg) {
    const uint32_t pack_mv = rr_adc_to_pack_mv(adc_mv, cfg);
    if (!battery->has_mv) {
        battery->filtered_mv = pack_mv;
        battery->has_mv = true;
    } else {
        const int64_t delta = (int64_t)pack_mv - (int64_t)battery->filtered_mv;
        int64_t stepped = (int64_t)battery->filtered_mv + (delta * (int64_t)cfg->ema_alpha_percent) / 100;
        if (stepped < 0) {
            stepped = 0;
        }
        if ((uint64_t)stepped > 0xffffffffull) {
            stepped = 0xffffffffll;
        }
        battery->filtered_mv = (uint32_t)stepped;
    }
    const uint8_t level = classify(battery->filtered_mv, cfg);
    if (level == RR_BATT_CRITICAL) {
        if (battery->critical_streak < 255) {
            battery->critical_streak++;
        }
    } else {
        battery->critical_streak = 0;
    }
    return level;
}

RR_HOT bool rr_battery_critical_confirmed(const rr_battery_t *battery, const rr_battery_cfg_t *cfg) {
    return battery->critical_streak >= cfg->critical_samples;
}
