// Hardware-free control logic for the robo racer.
//
// This is a straight port of the host-tested Rust crate in core/. The wire
// format, the state machine and the ramp rules are the same on purpose: the
// phone app and the bench tests depend on them. Nothing in this file touches
// a peripheral, so the same source builds into the firmware and into the
// host test at idf/host/test_logic.c.

#ifndef ROBO_LOGIC_H
#define ROBO_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define RR_HOT IRAM_ATTR
#else
#define RR_HOT
#endif

#define RR_COMMAND_MAX 1000

#define RR_BTN_FORWARD  (1u << 0)
#define RR_BTN_BACKWARD (1u << 1)
#define RR_BTN_LEFT     (1u << 2)
#define RR_BTN_RIGHT    (1u << 3)
#define RR_BTN_STOP     (1u << 4)
#define RR_BTN_MASK     (RR_BTN_FORWARD | RR_BTN_BACKWARD | RR_BTN_LEFT | RR_BTN_RIGHT | RR_BTN_STOP)

#define RR_MAGIC 0xA5u
#define RR_VERSION 1u

#define RR_KIND_DRIVE 1u
#define RR_KIND_ARM 2u
#define RR_KIND_DISARM 3u
#define RR_KIND_PING 4u
#define RR_KIND_ACK 5u

#define RR_SERVER_HELLO 0x80u
#define RR_SERVER_STATUS 0x81u
#define RR_SERVER_PONG 0x82u
#define RR_SERVER_ERROR 0x83u

#define RR_FLAG_ESTOP 0x01u

#define RR_STATUS_ARMED 0x01u
#define RR_STATUS_LINK_UP 0x02u
#define RR_STATUS_BATTERY_OK 0x04u
#define RR_STATUS_SESSION_OK 0x08u

#define RR_CLIENT_FRAME_LEN 12
#define RR_STATUS_FRAME_LEN 16
#define RR_PONG_FRAME_LEN 12
#define RR_ERROR_FRAME_LEN 8
#define RR_HELLO_FRAME_LEN 12

// Wire values. The app indexes these directly, so the numeric order is part
// of the protocol and must not be rearranged.
enum {
    RR_STATE_BOOT = 0,
    RR_STATE_DISARMED = 1,
    RR_STATE_ARMED = 2,
    RR_STATE_DRIVING = 3,
    RR_STATE_FAULT = 4
};

enum {
    RR_FAULT_COMMAND_TIMEOUT = 0,
    RR_FAULT_LINK_LOST = 1,
    RR_FAULT_LOW_BATTERY = 2,
    RR_FAULT_DRIVER = 3,
    RR_FAULT_ESTOP = 4,
    RR_FAULT_INTERNAL = 5
};

enum {
    RR_MODE_SLOW = 0,
    RR_MODE_NORMAL = 1,
    RR_MODE_FAST = 2
};

enum {
    RR_CFG_OK = 0,
    RR_CFG_INVALID_DRIVE,
    RR_CFG_INVALID_TURN,
    RR_CFG_INVALID_RATE,
    RR_CFG_INVALID_COMMAND_TIMEOUT,
    RR_CFG_INVALID_LINK_TIMEOUT,
    RR_CFG_INVALID_PERIOD,
    RR_CFG_INVALID_BATTERY,
    RR_CFG_INVALID_DIVIDER,
    RR_CFG_INVALID_SCALE
};

enum {
    RR_PROTO_OK = 0,
    RR_PROTO_TOO_SHORT = 1,
    RR_PROTO_TOO_LONG = 2,
    RR_PROTO_BAD_MAGIC = 3,
    RR_PROTO_BAD_VERSION = 4,
    RR_PROTO_UNKNOWN_KIND = 5,
    RR_PROTO_INVALID_BUTTONS = 6,
    RR_PROTO_INVALID_MODE = 7,
    RR_PROTO_BAD_TOKEN = 8,
    RR_PROTO_STALE_SEQUENCE = 9
};

enum {
    RR_ARM_OK = 0,
    RR_ARM_NOT_INITIALISED,
    RR_ARM_FAULT_LATCHED,
    RR_ARM_ALREADY
};

enum {
    RR_ACK_OK = 0,
    RR_ACK_LINK_STILL_DOWN,
    RR_ACK_BATTERY_STILL_BAD
};

enum {
    RR_BATT_UNKNOWN = 0,
    RR_BATT_OK,
    RR_BATT_WARNING,
    RR_BATT_CRITICAL
};

typedef struct {
    int16_t left;
    int16_t right;
} rr_drive_t;

typedef struct {
    int16_t forward;
    int16_t reverse;
    int16_t turn_inner;
    int16_t turn_outer;
    int16_t reverse_turn_inner;
    int16_t reverse_turn_outer;
    int16_t accel_per_s;
    int16_t decel_per_s;
    uint16_t direction_change_coast_ms;
} rr_drive_tuning_t;

typedef struct {
    uint16_t min_duty_permille;
    uint16_t max_duty_permille;
} rr_motor_tuning_t;

typedef struct {
    uint32_t command_timeout_ms;
    uint32_t link_timeout_ms;
    uint32_t control_period_ms;
} rr_safety_tuning_t;

typedef struct {
    uint8_t cell_count;
    uint16_t warn_mv_per_cell;
    uint16_t critical_mv_per_cell;
    uint32_t divider_num;
    uint32_t divider_den;
    int32_t calibration_offset_mv;
    uint8_t ema_alpha_percent;
    uint8_t critical_samples;
} rr_battery_cfg_t;

typedef struct {
    uint16_t slow_percent;
    uint16_t normal_percent;
    uint16_t fast_percent;
} rr_speed_scales_t;

typedef struct {
    rr_drive_tuning_t drive;
    rr_motor_tuning_t motors;
    rr_safety_tuning_t safety;
    rr_battery_cfg_t battery;
    rr_speed_scales_t scales;
} rr_config_t;

typedef struct {
    rr_drive_t command;
    bool conflict;
} rr_resolved_t;

typedef struct {
    rr_drive_t current;
    uint32_t coast_remaining_ms;
} rr_ramp_t;

typedef struct {
    uint8_t state;
    bool has_fault;
    uint8_t fault;
    rr_drive_t last_command;
    bool has_command_ms;
    uint32_t last_command_ms;
    bool link_up;
    bool battery_ok;
    rr_safety_tuning_t tuning;
} rr_safety_t;

enum {
    RR_OUTPUT_DISABLED = 0,
    RR_OUTPUT_MOVE = 1
};

typedef struct {
    uint8_t kind;
    rr_drive_t command;
} rr_output_t;

typedef struct {
    uint8_t kind;
    uint8_t buttons;
    uint8_t mode;
    uint8_t flags;
    uint16_t seq;
    uint32_t token;
} rr_client_frame_t;

typedef struct {
    uint32_t token;
    bool has_seq;
    uint16_t last_seq;
} rr_decoder_t;

typedef struct {
    bool has_mv;
    uint32_t filtered_mv;
    uint8_t critical_streak;
} rr_battery_t;

RR_HOT rr_drive_t rr_drive(int16_t left, int16_t right);
RR_HOT bool rr_drive_is_stop(rr_drive_t command);

void rr_config_default(rr_config_t *cfg);
int rr_config_validate(const rr_config_t *cfg);
const char *rr_config_error_str(int error);
uint16_t rr_scale_percent(const rr_config_t *cfg, uint8_t mode);

RR_HOT rr_resolved_t rr_steering_resolve(uint8_t mask, const rr_drive_tuning_t *tuning, uint16_t scale_percent);

RR_HOT void rr_ramp_reset(rr_ramp_t *ramp);
RR_HOT rr_drive_t rr_ramp_update(rr_ramp_t *ramp, rr_drive_t target, uint32_t dt_ms, const rr_drive_tuning_t *tuning);

void rr_safety_init(rr_safety_t *safety, const rr_safety_tuning_t *tuning);
RR_HOT void rr_safety_boot_complete(rr_safety_t *safety);
RR_HOT int rr_safety_arm(rr_safety_t *safety);
RR_HOT void rr_safety_disarm(rr_safety_t *safety);
RR_HOT int rr_safety_acknowledge(rr_safety_t *safety);
RR_HOT void rr_safety_note_link(rr_safety_t *safety, bool up);
RR_HOT void rr_safety_note_battery(rr_safety_t *safety, bool ok);
RR_HOT void rr_safety_latch(rr_safety_t *safety, uint8_t fault);
RR_HOT void rr_safety_estop(rr_safety_t *safety);
RR_HOT void rr_safety_note_command(rr_safety_t *safety, uint32_t now_ms, rr_drive_t command);
RR_HOT rr_output_t rr_safety_update(rr_safety_t *safety, uint32_t now_ms);
RR_HOT bool rr_safety_enabled(const rr_safety_t *safety);

int rr_frame_decode(const uint8_t *buf, uint32_t len, rr_client_frame_t *out);
rr_decoder_t rr_decoder_new(uint32_t token);
int rr_decoder_decode(rr_decoder_t *decoder, const uint8_t *buf, uint32_t len, rr_client_frame_t *out);

void rr_encode_status(uint8_t state, bool has_fault, uint8_t fault, uint8_t flags,
                      uint16_t battery_mv, int16_t left, int16_t right,
                      uint16_t last_seq, uint8_t mode, uint8_t out[RR_STATUS_FRAME_LEN]);
void rr_encode_hello(uint32_t token, uint32_t uptime_ms, uint8_t out[RR_HELLO_FRAME_LEN]);
void rr_encode_pong(uint32_t client_ts, uint32_t uptime_ms, uint8_t out[RR_PONG_FRAME_LEN]);
void rr_encode_error(uint8_t code, uint8_t out[RR_ERROR_FRAME_LEN]);

void rr_battery_init(rr_battery_t *battery);
RR_HOT uint32_t rr_adc_to_pack_mv(uint32_t adc_mv, const rr_battery_cfg_t *cfg);
RR_HOT uint8_t rr_battery_update(rr_battery_t *battery, uint32_t adc_mv, const rr_battery_cfg_t *cfg);
RR_HOT bool rr_battery_critical_confirmed(const rr_battery_t *battery, const rr_battery_cfg_t *cfg);
RR_HOT uint32_t rr_battery_warn_mv(const rr_battery_cfg_t *cfg);
RR_HOT uint32_t rr_battery_critical_mv(const rr_battery_cfg_t *cfg);

#endif
