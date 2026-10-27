// Host checks for the C port of core/. Numbers match the Rust tests.
#include <stdio.h>
#include <string.h>

#include "logic.h"

static int g_failed;
static int g_passed;

#define EXPECT(cond)                                                                 \
    do {                                                                             \
        if (cond) {                                                                  \
            g_passed++;                                                              \
        } else {                                                                     \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
            g_failed++;                                                              \
        }                                                                            \
    } while (0)

static rr_drive_tuning_t tuning(void) {
    rr_config_t cfg;
    rr_config_default(&cfg);
    return cfg.drive;
}

static rr_battery_cfg_t battery_cfg(void) {
    rr_battery_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cell_count = 3;
    cfg.warn_mv_per_cell = 3400;
    cfg.critical_mv_per_cell = 3300;
    cfg.divider_num = 1000;
    cfg.divider_den = 1000;
    cfg.ema_alpha_percent = 100;
    cfg.critical_samples = 3;
    return cfg;
}

static void encode_client(uint8_t kind, uint8_t buttons, uint8_t mode, uint8_t flags, uint16_t seq, uint32_t token,
                          uint8_t out[12]) {
    memset(out, 0, 12);
    out[0] = RR_MAGIC;
    out[1] = RR_VERSION;
    out[2] = kind;
    out[3] = buttons;
    out[4] = mode;
    out[5] = flags;
    out[6] = (uint8_t)(seq & 0xffu);
    out[7] = (uint8_t)(seq >> 8);
    out[8] = (uint8_t)(token & 0xffu);
    out[9] = (uint8_t)((token >> 8) & 0xffu);
    out[10] = (uint8_t)((token >> 16) & 0xffu);
    out[11] = (uint8_t)((token >> 24) & 0xffu);
}

static void test_steering(void) {
    const rr_drive_tuning_t drive = tuning();
    rr_resolved_t r = rr_steering_resolve(RR_BTN_FORWARD, &drive, 100);
    EXPECT(r.command.left == 800 && r.command.right == 800 && !r.conflict);
    r = rr_steering_resolve(RR_BTN_BACKWARD, &drive, 100);
    EXPECT(r.command.left == -600 && r.command.right == -600);
    r = rr_steering_resolve(RR_BTN_LEFT, &drive, 100);
    EXPECT(r.command.left == 350 && r.command.right == 800);
    r = rr_steering_resolve(RR_BTN_RIGHT, &drive, 100);
    EXPECT(r.command.left == 800 && r.command.right == 350);
    r = rr_steering_resolve((uint8_t)(RR_BTN_STOP | RR_BTN_FORWARD | RR_BTN_LEFT), &drive, 100);
    EXPECT(r.command.left == 0 && r.command.right == 0 && !r.conflict);
    r = rr_steering_resolve((uint8_t)(RR_BTN_FORWARD | RR_BTN_BACKWARD), &drive, 100);
    EXPECT(r.command.left == 0 && r.conflict);
    r = rr_steering_resolve((uint8_t)(RR_BTN_LEFT | RR_BTN_RIGHT), &drive, 100);
    EXPECT(r.conflict && rr_drive_is_stop(r.command));
    r = rr_steering_resolve((uint8_t)(RR_BTN_FORWARD | RR_BTN_LEFT), &drive, 100);
    EXPECT(r.command.left == 350 && r.command.right == 800);
    r = rr_steering_resolve((uint8_t)(RR_BTN_BACKWARD | RR_BTN_LEFT), &drive, 100);
    EXPECT(r.command.left == -350 && r.command.right == -600);
    r = rr_steering_resolve((uint8_t)(RR_BTN_BACKWARD | RR_BTN_RIGHT), &drive, 100);
    EXPECT(r.command.left == -600 && r.command.right == -350);
    r = rr_steering_resolve(RR_BTN_FORWARD, &drive, 50);
    EXPECT(r.command.left == 400 && r.command.right == 400);
    r = rr_steering_resolve(0, &drive, 100);
    EXPECT(rr_drive_is_stop(r.command));
    rr_drive_t clamped = rr_drive(5000, -9000);
    EXPECT(clamped.left == RR_COMMAND_MAX && clamped.right == -RR_COMMAND_MAX);
}

static void test_ramp(void) {
    const rr_drive_tuning_t drive = tuning();
    rr_ramp_t ramp;
    rr_ramp_reset(&ramp);
    rr_drive_t out = rr_ramp_update(&ramp, rr_drive(800, 800), 10, &drive);
    EXPECT(out.left == 20 && out.right == 20);

    rr_ramp_reset(&ramp);
    for (int i = 0; i < 200; i++) {
        out = rr_ramp_update(&ramp, rr_drive(800, 800), 10, &drive);
    }
    EXPECT(out.left == 800 && out.right == 800);

    out = rr_ramp_update(&ramp, rr_drive(0, 0), 10, &drive);
    EXPECT(out.left == 760 && out.right == 760);

    rr_ramp_reset(&ramp);
    for (int i = 0; i < 100; i++) {
        rr_ramp_update(&ramp, rr_drive(800, 800), 10, &drive);
    }
    out = rr_ramp_update(&ramp, rr_drive(-600, -600), 10, &drive);
    EXPECT(rr_drive_is_stop(out) && ramp.coast_remaining_ms > 0);
    for (int i = 0; i < 5; i++) {
        out = rr_ramp_update(&ramp, rr_drive(-600, -600), 10, &drive);
        EXPECT(rr_drive_is_stop(out));
    }
    out = rr_ramp_update(&ramp, rr_drive(-600, -600), 10, &drive);
    EXPECT(out.left < 0 && out.right < 0 && ramp.coast_remaining_ms == 0);

    rr_ramp_reset(&ramp);
    for (int i = 0; i < 100; i++) {
        rr_ramp_update(&ramp, rr_drive(800, 800), 10, &drive);
    }
    int ticks = 0;
    out = rr_drive(0, 0);
    while (rr_drive_is_stop(out) && ticks < 20) {
        out = rr_ramp_update(&ramp, rr_drive(-600, -600), 10, &drive);
        ticks++;
    }
    EXPECT(ticks == 7);
    EXPECT(out.left < 0 && out.right < 0);

    rr_ramp_reset(&ramp);
    out = rr_ramp_update(&ramp, rr_drive(800, 800), 0, &drive);
    EXPECT(rr_drive_is_stop(out));
}

static rr_safety_t armed(void) {
    rr_config_t cfg;
    rr_config_default(&cfg);
    rr_safety_t safety;
    rr_safety_init(&safety, &cfg.safety);
    rr_safety_boot_complete(&safety);
    EXPECT(rr_safety_arm(&safety) == RR_ARM_OK);
    return safety;
}

static void test_safety(void) {
    rr_config_t cfg;
    rr_config_default(&cfg);
    rr_safety_t safety;
    rr_safety_init(&safety, &cfg.safety);
    EXPECT(safety.state == RR_STATE_BOOT);
    EXPECT(rr_safety_update(&safety, 0).kind == RR_OUTPUT_DISABLED);
    EXPECT(rr_safety_arm(&safety) == RR_ARM_NOT_INITIALISED);

    rr_safety_boot_complete(&safety);
    rr_safety_note_command(&safety, 0, rr_drive(800, 800));
    EXPECT(rr_safety_update(&safety, 0).kind == RR_OUTPUT_DISABLED);

    safety = armed();
    rr_safety_note_command(&safety, 10, rr_drive(800, 800));
    EXPECT(safety.state == RR_STATE_DRIVING);
    rr_output_t output = rr_safety_update(&safety, 10);
    EXPECT(output.kind == RR_OUTPUT_MOVE && output.command.left == 800);

    rr_safety_note_command(&safety, 20, rr_drive(0, 0));
    EXPECT(safety.state == RR_STATE_ARMED);
    EXPECT(rr_safety_update(&safety, 20).kind == RR_OUTPUT_DISABLED);

    safety = armed();
    rr_safety_note_command(&safety, 10, rr_drive(800, 800));
    rr_safety_note_command(&safety, 20, rr_drive(0, 0));
    EXPECT(rr_safety_update(&safety, 10020).kind == RR_OUTPUT_DISABLED);
    EXPECT(safety.state == RR_STATE_ARMED && !safety.has_fault);

    safety = armed();
    rr_safety_note_command(&safety, 0, rr_drive(800, 800));
    EXPECT(rr_safety_update(&safety, 200).kind == RR_OUTPUT_MOVE);
    EXPECT(rr_safety_update(&safety, 300).kind == RR_OUTPUT_DISABLED);
    EXPECT(safety.state == RR_STATE_FAULT && safety.fault == RR_FAULT_COMMAND_TIMEOUT);

    safety = armed();
    rr_safety_note_command(&safety, 0, rr_drive(800, 800));
    rr_safety_update(&safety, 1000);
    EXPECT(rr_safety_arm(&safety) == RR_ARM_FAULT_LATCHED);
    rr_safety_note_link(&safety, true);
    EXPECT(rr_safety_acknowledge(&safety) == RR_ACK_OK);
    EXPECT(safety.state == RR_STATE_DISARMED);
    EXPECT(rr_safety_arm(&safety) == RR_ARM_OK);

    safety = armed();
    rr_safety_note_link(&safety, true);
    rr_safety_note_link(&safety, false);
    EXPECT(safety.state == RR_STATE_FAULT && safety.fault == RR_FAULT_LINK_LOST);
    EXPECT(rr_safety_acknowledge(&safety) == RR_ACK_LINK_STILL_DOWN);

    safety = armed();
    rr_safety_note_link(&safety, true);
    rr_safety_note_battery(&safety, false);
    EXPECT(safety.state == RR_STATE_FAULT && safety.fault == RR_FAULT_LOW_BATTERY);

    safety = armed();
    rr_safety_latch(&safety, RR_FAULT_DRIVER);
    rr_safety_note_command(&safety, 0, rr_drive(800, 800));
    EXPECT(rr_safety_update(&safety, 0).kind == RR_OUTPUT_DISABLED);

    safety = armed();
    rr_safety_note_command(&safety, 0, rr_drive(800, 800));
    rr_safety_disarm(&safety);
    EXPECT(rr_safety_update(&safety, 0).kind == RR_OUTPUT_DISABLED);
    EXPECT(safety.state == RR_STATE_DISARMED);

    safety = armed();
    rr_safety_note_link(&safety, true);
    rr_safety_note_command(&safety, 0, rr_drive(800, 800));
    rr_safety_estop(&safety);
    EXPECT(safety.state == RR_STATE_FAULT && safety.fault == RR_FAULT_ESTOP);
    EXPECT(rr_safety_update(&safety, 10).kind == RR_OUTPUT_DISABLED);
    rr_safety_note_command(&safety, 20, rr_drive(0, 0));
    EXPECT(rr_safety_arm(&safety) == RR_ARM_FAULT_LATCHED);
    EXPECT(rr_safety_acknowledge(&safety) == RR_ACK_OK);
    EXPECT(safety.state == RR_STATE_DISARMED);
}

static void test_protocol(void) {
    const uint32_t token = 0xdeadbeefu;
    uint8_t bytes[16];
    encode_client(RR_KIND_DRIVE, RR_BTN_FORWARD, RR_MODE_NORMAL, 0, 7, token, bytes);
    rr_client_frame_t frame;
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_OK);
    EXPECT(frame.kind == RR_KIND_DRIVE && frame.buttons == RR_BTN_FORWARD);
    EXPECT(frame.mode == RR_MODE_NORMAL && frame.seq == 7 && frame.token == token);

    bytes[0] = 0;
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_BAD_MAGIC);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 1, token, bytes);
    bytes[1] = 9;
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_BAD_VERSION);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 1, token, bytes);
    bytes[2] = 42;
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_UNKNOWN_KIND);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 1, token, bytes);
    bytes[3] = 0x80;
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_INVALID_BUTTONS);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 1, token, bytes);
    bytes[4] = 7;
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_INVALID_MODE);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 1, token, bytes);
    EXPECT(rr_frame_decode(bytes, 11, &frame) == RR_PROTO_TOO_SHORT);
    EXPECT(rr_frame_decode(bytes, 13, &frame) == RR_PROTO_TOO_LONG);

    rr_decoder_t decoder = rr_decoder_new(token);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 1, token ^ 1u, bytes);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_BAD_TOKEN);

    decoder = rr_decoder_new(token);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 10, token, bytes);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_OK);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_STALE_SEQUENCE);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 9, token, bytes);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_STALE_SEQUENCE);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 11, token, bytes);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_OK);

    decoder = rr_decoder_new(token);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 65535, token, bytes);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_OK);
    encode_client(RR_KIND_DRIVE, 0, RR_MODE_NORMAL, 0, 0, token, bytes);
    EXPECT(rr_decoder_decode(&decoder, bytes, 12, &frame) == RR_PROTO_OK);

    encode_client(RR_KIND_DRIVE, RR_BTN_STOP, RR_MODE_NORMAL, RR_FLAG_ESTOP, 1, token, bytes);
    EXPECT(rr_frame_decode(bytes, 12, &frame) == RR_PROTO_OK);
    EXPECT((frame.flags & RR_FLAG_ESTOP) != 0);

    uint8_t status[16];
    rr_encode_status(RR_STATE_ARMED, false, 0, RR_STATUS_ARMED, 12345, -100, 200, 3, RR_MODE_NORMAL, status);
    EXPECT(status[0] == RR_MAGIC && status[2] == RR_SERVER_STATUS && status[3] == RR_STATE_ARMED);
    EXPECT(status[6] == (12345 & 0xff) && status[7] == (12345 >> 8));

    uint8_t pong[12];
    rr_encode_pong(0xdeadbeefu, 1234, pong);
    EXPECT(pong[4] == 0xef && pong[5] == 0xbe && pong[6] == 0xad && pong[7] == 0xde);
}

static void test_battery(void) {
    rr_battery_cfg_t cfg = battery_cfg();
    cfg.divider_num = 127;
    cfg.divider_den = 27;
    EXPECT(rr_adc_to_pack_mv(2700, &cfg) == 12700);

    cfg = battery_cfg();
    rr_battery_t battery;
    rr_battery_init(&battery);
    EXPECT(rr_battery_warn_mv(&cfg) == 10200);
    EXPECT(rr_battery_critical_mv(&cfg) == 9900);

    rr_battery_init(&battery);
    EXPECT(rr_battery_update(&battery, 12000, &cfg) == RR_BATT_OK);
    EXPECT(rr_battery_update(&battery, 10200, &cfg) == RR_BATT_OK);
    EXPECT(rr_battery_update(&battery, 10199, &cfg) == RR_BATT_WARNING);
    EXPECT(rr_battery_update(&battery, 9900, &cfg) == RR_BATT_WARNING);
    EXPECT(rr_battery_update(&battery, 9899, &cfg) == RR_BATT_CRITICAL);

    rr_battery_init(&battery);
    rr_battery_update(&battery, 9000, &cfg);
    EXPECT(battery.critical_streak == 1 && !rr_battery_critical_confirmed(&battery, &cfg));
    rr_battery_update(&battery, 9000, &cfg);
    rr_battery_update(&battery, 9000, &cfg);
    EXPECT(rr_battery_critical_confirmed(&battery, &cfg));

    rr_battery_init(&battery);
    rr_battery_update(&battery, 12000, &cfg);
    rr_battery_update(&battery, 9000, &cfg);
    EXPECT(!rr_battery_critical_confirmed(&battery, &cfg));
    rr_battery_update(&battery, 12000, &cfg);
    EXPECT(battery.critical_streak == 0);

    cfg.ema_alpha_percent = 50;
    rr_battery_init(&battery);
    rr_battery_update(&battery, 10000, &cfg);
    rr_battery_update(&battery, 12000, &cfg);
    EXPECT(battery.filtered_mv == 11000);
}

static void test_config(void) {
    rr_config_t cfg;
    rr_config_default(&cfg);
    EXPECT(rr_config_validate(&cfg) == RR_CFG_OK);
    EXPECT(cfg.safety.control_period_ms == 2);
    cfg.drive.forward = 1001;
    EXPECT(rr_config_validate(&cfg) == RR_CFG_INVALID_DRIVE);
    rr_config_default(&cfg);
    cfg.drive.turn_inner = 900;
    cfg.drive.turn_outer = 100;
    EXPECT(rr_config_validate(&cfg) == RR_CFG_INVALID_TURN);
    rr_config_default(&cfg);
    cfg.safety.command_timeout_ms = 10;
    EXPECT(rr_config_validate(&cfg) == RR_CFG_INVALID_COMMAND_TIMEOUT);
    rr_config_default(&cfg);
    cfg.scales.slow_percent = 90;
    cfg.scales.normal_percent = 40;
    EXPECT(rr_config_validate(&cfg) == RR_CFG_INVALID_SCALE);
    rr_config_default(&cfg);
    cfg.battery.warn_mv_per_cell = 3300;
    cfg.battery.critical_mv_per_cell = 3400;
    EXPECT(rr_config_validate(&cfg) == RR_CFG_INVALID_BATTERY);
    rr_config_default(&cfg);
    cfg.battery.cell_count = 2;
    EXPECT(rr_battery_warn_mv(&cfg.battery) == 6800);
    EXPECT(rr_battery_critical_mv(&cfg.battery) == 6600);
}

int main(void) {
    test_steering();
    test_ramp();
    test_safety();
    test_protocol();
    test_battery();
    test_config();
    printf("%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
