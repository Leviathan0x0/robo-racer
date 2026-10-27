#include "motors.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "logic.h"

static const char *TAG = "motors";

static bool s_ready;
static uint16_t s_min_permille;
static uint16_t s_max_permille;
static uint32_t s_full_scale;
static int s_bits;

static uint32_t duty_raw(int16_t command) {
    int32_t magnitude = command < 0 ? -(int32_t)command : (int32_t)command;
    if (magnitude > RR_COMMAND_MAX) {
        magnitude = RR_COMMAND_MAX;
    }
    const uint32_t span = (uint32_t)(s_max_permille - s_min_permille);
    const uint32_t permille = (uint32_t)s_min_permille + span * (uint32_t)magnitude / RR_COMMAND_MAX;
    uint32_t duty = permille * s_full_scale / 1000u;
    if (duty > s_full_scale - 1u) {
        duty = s_full_scale - 1u;
    }
    return duty;
}

static void write_duty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void side_disable(gpio_num_t en_r, gpio_num_t en_l, ledc_channel_t ch_r, ledc_channel_t ch_l) {
    gpio_set_level(en_r, 0);
    gpio_set_level(en_l, 0);
    if (s_ready) {
        write_duty(ch_r, 0);
        write_duty(ch_l, 0);
    }
}

static void side_drive(int16_t command, gpio_num_t en_r, gpio_num_t en_l, ledc_channel_t ch_r, ledc_channel_t ch_l) {
    if (command == 0) {
        side_disable(en_r, en_l, ch_r, ch_l);
        return;
    }
    gpio_set_level(en_r, 1);
    gpio_set_level(en_l, 1);
    const uint32_t duty = duty_raw(command);
    if (command > 0) {
        write_duty(ch_l, 0);
        write_duty(ch_r, duty);
    } else {
        write_duty(ch_r, 0);
        write_duty(ch_l, duty);
    }
}

void motors_hold_disabled(void) {
    const uint64_t mask = (1ULL << RR_GPIO_LEFT_REN) | (1ULL << RR_GPIO_LEFT_LEN) | (1ULL << RR_GPIO_RIGHT_REN) |
                          (1ULL << RR_GPIO_RIGHT_LEN) | (1ULL << RR_GPIO_STATUS_LED);
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(RR_GPIO_LEFT_REN, 0);
    gpio_set_level(RR_GPIO_LEFT_LEN, 0);
    gpio_set_level(RR_GPIO_RIGHT_REN, 0);
    gpio_set_level(RR_GPIO_RIGHT_LEN, 0);
    gpio_set_level(RR_GPIO_STATUS_LED, 0);
}

static bool configure_timer(uint32_t freq_hz, ledc_timer_bit_t bits) {
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = bits,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    return ledc_timer_config(&timer) == ESP_OK;
}

static bool configure_channel(ledc_channel_t channel, int gpio) {
    const ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    if (ledc_channel_config(&cfg) != ESP_OK) {
        return false;
    }
    gpio_set_drive_capability((gpio_num_t)gpio, GPIO_DRIVE_CAP_3);
    return true;
}

bool motors_init(uint16_t min_duty_permille, uint16_t max_duty_permille) {
    s_min_permille = min_duty_permille;
    s_max_permille = max_duty_permille;

    // 20 kHz is above hearing and under the BTS7960's 25 kHz switching limit.
    // 11-bit keeps an integer-friendly LEDC divider from the 80 MHz APB clock.
    bool fast = configure_timer(20000, LEDC_TIMER_11_BIT);
    if (fast) {
        s_bits = 11;
        ESP_LOGI(TAG, "PWM 20 kHz, 11-bit");
    } else if (configure_timer(10000, LEDC_TIMER_12_BIT)) {
        s_bits = 12;
        ESP_LOGW(TAG, "20 kHz timer refused, using 10 kHz 12-bit");
    } else {
        ESP_LOGE(TAG, "LEDC timer configuration failed");
        return false;
    }
    s_full_scale = 1u << s_bits;

    const struct {
        ledc_channel_t channel;
        int gpio;
    } channels[] = {
        {LEDC_CHANNEL_0, RR_GPIO_LEFT_RPWM},
        {LEDC_CHANNEL_1, RR_GPIO_LEFT_LPWM},
        {LEDC_CHANNEL_2, RR_GPIO_RIGHT_RPWM},
        {LEDC_CHANNEL_3, RR_GPIO_RIGHT_LPWM},
    };
    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
        if (!configure_channel(channels[i].channel, channels[i].gpio)) {
            ESP_LOGE(TAG, "LEDC channel %d failed", (int)channels[i].channel);
            return false;
        }
    }
    s_ready = true;
    motors_force_disable();
    return true;
}

void motors_force_disable(void) {
    side_disable(RR_GPIO_LEFT_REN, RR_GPIO_LEFT_LEN, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    side_disable(RR_GPIO_RIGHT_REN, RR_GPIO_RIGHT_LEN, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
}

void motors_apply(int16_t left, int16_t right) {
    if (!s_ready) {
        motors_hold_disabled();
        return;
    }
    if (left == 0 && right == 0) {
        motors_force_disable();
        return;
    }
    side_drive(left, RR_GPIO_LEFT_REN, RR_GPIO_LEFT_LEN, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    side_drive(right, RR_GPIO_RIGHT_REN, RR_GPIO_RIGHT_LEN, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
}
