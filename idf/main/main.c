// ESP-IDF firmware for the robo racer.
//
// Core 0 owns the radio and the HTTP server. Core 1 owns the motors. A drive
// frame notifies the control task, so command-to-PWM latency is not the 2 ms
// tick. The tick still runs so a silent link is noticed without a frame.

#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "logic.h"
#include "mailbox.h"
#include "motors.h"
#include "net.h"

static const char *TAG = "robo";

#if defined(ADC_ATTEN_DB_12)
#define RR_ADC_ATTEN ADC_ATTEN_DB_12
#else
#define RR_ADC_ATTEN ADC_ATTEN_DB_11
#endif

// GPIO34 is ADC1 channel 6, input only.
#define RR_ADC_CHANNEL ADC_CHANNEL_6

static rr_config_t s_cfg;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_calibrated;
static SemaphoreHandle_t s_control_ready;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void halt(const char *why) {
    motors_force_disable();
    while (true) {
        printf("robo-racer: halted, motors disabled (%s)\n", why);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool adc_start(void) {
    const adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit, &s_adc) != ESP_OK) {
        return false;
    }
    const adc_oneshot_chan_cfg_t channel = {
        .atten = RR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_oneshot_config_channel(s_adc, RR_ADC_CHANNEL, &channel) != ESP_OK) {
        return false;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t curve = {
        .unit_id = ADC_UNIT_1,
        .chan = RR_ADC_CHANNEL,
        .atten = RR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    s_calibrated = adc_cali_create_scheme_curve_fitting(&curve, &s_cali) == ESP_OK;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t line = {
        .unit_id = ADC_UNIT_1,
        .atten = RR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
#if CONFIG_IDF_TARGET_ESP32
    line.default_vref = 1100;
#endif
    s_calibrated = adc_cali_create_scheme_line_fitting(&line, &s_cali) == ESP_OK;
#else
    s_calibrated = false;
#endif
    if (s_calibrated) {
        ESP_LOGI(TAG, "ADC calibration active");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable, using 3100 mV full scale");
    }
    return true;
}

static bool adc_read_mv(int *millivolts) {
    int raw = 0;
    if (adc_oneshot_read(s_adc, RR_ADC_CHANNEL, &raw) != ESP_OK) {
        return false;
    }
    if (s_calibrated) {
        int calibrated = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw, &calibrated) == ESP_OK) {
            *millivolts = calibrated;
            return true;
        }
    }
    if (raw < 0) {
        raw = 0;
    }
    *millivolts = (int)(((uint32_t)raw * 3100u) / 4095u);
    return true;
}

static void control_task(void *arg) {
    (void)arg;
    mailbox_set_control_task(xTaskGetCurrentTaskHandle());
    xSemaphoreGive(s_control_ready);

    rr_safety_t safety;
    rr_safety_init(&safety, &s_cfg.safety);
    rr_safety_boot_complete(&safety);

    rr_ramp_t ramp;
    rr_ramp_reset(&ramp);
    rr_battery_t battery;
    rr_battery_init(&battery);

    uint16_t last_seq = 0;
    uint32_t last_ms = now_ms();
    uint32_t last_battery_ms = last_ms;
    uint32_t led_toggle_ms = last_ms;
    int led_on = 0;

    mailbox_publish(safety.state, safety.has_fault, safety.fault, 0, 0);
    ESP_LOGI(TAG, "control loop on core %d, idle tick %" PRIu32 " ms", xPortGetCoreID(), s_cfg.safety.control_period_ms);

    for (;;) {
        TickType_t wait = pdMS_TO_TICKS(s_cfg.safety.control_period_ms);
        if (wait < 1) {
            wait = 1;
        }
        ulTaskNotifyTake(pdTRUE, wait);

        const uint32_t now = now_ms();
        if ((now - last_battery_ms) >= 200u) {
            last_battery_ms = now;
            int pin_mv = 0;
            if (adc_read_mv(&pin_mv) && pin_mv >= 0) {
                rr_battery_update(&battery, (uint32_t)pin_mv, &s_cfg.battery);
            }
            uint32_t pack = battery.has_mv ? battery.filtered_mv : 0;
            if (pack > 65535u) {
                pack = 65535u;
            }
            mailbox_set_battery((uint16_t)pack, !rr_battery_critical_confirmed(&battery, &s_cfg.battery));
        }

        const rr_inputs_t inputs = mailbox_take();
        rr_safety_note_link(&safety, inputs.link_up);
        rr_safety_note_battery(&safety, inputs.battery_ok);
        if (inputs.estop) {
            rr_safety_estop(&safety);
        }
        if (inputs.disarm) {
            rr_safety_disarm(&safety);
            rr_ramp_reset(&ramp);
        }
        if (inputs.ack) {
            (void)rr_safety_acknowledge(&safety);
            rr_ramp_reset(&ramp);
        }
        if (inputs.arm && rr_safety_arm(&safety) == RR_ARM_OK) {
            rr_ramp_reset(&ramp);
        }
        if (inputs.seq != last_seq) {
            last_seq = inputs.seq;
            const rr_resolved_t resolved =
                rr_steering_resolve(inputs.buttons, &s_cfg.drive, rr_scale_percent(&s_cfg, inputs.mode));
            rr_safety_note_command(&safety, now, resolved.command);
        }

        const rr_output_t decision = rr_safety_update(&safety, now);
        rr_drive_t applied;
        if (decision.kind == RR_OUTPUT_MOVE) {
            uint32_t dt = now - last_ms;
            if (dt > 100u) {
                dt = 100u;
            }
            applied = rr_ramp_update(&ramp, decision.command, dt, &s_cfg.drive);
        } else {
            rr_ramp_reset(&ramp);
            applied = rr_drive(0, 0);
        }
        last_ms = now;
        motors_apply(applied.left, applied.right);

        if (safety.state == RR_STATE_ARMED || safety.state == RR_STATE_DRIVING) {
            led_on = 1;
            gpio_set_level(RR_GPIO_STATUS_LED, 1);
        } else if (safety.state == RR_STATE_FAULT) {
            if ((now - led_toggle_ms) >= 250u) {
                led_toggle_ms = now;
                led_on = !led_on;
                gpio_set_level(RR_GPIO_STATUS_LED, led_on);
            }
        } else {
            led_on = 0;
            gpio_set_level(RR_GPIO_STATUS_LED, 0);
        }

        mailbox_publish(safety.state, safety.has_fault, safety.fault, applied.left, applied.right);
    }
}

void app_main(void) {
    printf("robo-racer: boot\n");
    fflush(stdout);

    // Enables go low before PWM, Wi-Fi or anything that can fail.
    motors_hold_disabled();
    mailbox_init();

    rr_config_default(&s_cfg);
    const int config_error = rr_config_validate(&s_cfg);
    if (config_error != RR_CFG_OK) {
        halt(rr_config_error_str(config_error));
    }
    if (!motors_init(s_cfg.motors.min_duty_permille, s_cfg.motors.max_duty_permille)) {
        halt("PWM init failed");
    }
    if (!adc_start()) {
        halt("ADC init failed");
    }

    s_control_ready = xSemaphoreCreateBinary();
    if (s_control_ready == NULL) {
        halt("control semaphore failed");
    }
    // Priority 22, app CPU. The Wi-Fi task stays on the protocol CPU at 23,
    // so the radio is never waiting behind the motor loop.
    if (xTaskCreatePinnedToCore(control_task, "control", 8192, NULL, 22, NULL, 1) != pdPASS) {
        halt("control task failed");
    }
    if (xSemaphoreTake(s_control_ready, pdMS_TO_TICKS(1000)) != pdTRUE) {
        halt("control task did not start");
    }
    if (!net_start(&s_cfg)) {
        motors_force_disable();
        halt("network init failed");
    }
}
