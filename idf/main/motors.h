// BTS7960 outputs. Enable pins are driven low before LEDC is configured.
//
// PWM is 20 kHz / 11-bit when the LEDC clock can produce it, otherwise the
// previous 10 kHz / 12-bit setting. The permille window is unchanged either way.

#ifndef ROBO_MOTORS_H
#define ROBO_MOTORS_H

#include <stdbool.h>
#include <stdint.h>

#define RR_GPIO_LEFT_RPWM 16
#define RR_GPIO_LEFT_LPWM 17
#define RR_GPIO_LEFT_REN 18
#define RR_GPIO_LEFT_LEN 19
#define RR_GPIO_RIGHT_RPWM 21
#define RR_GPIO_RIGHT_LPWM 22
#define RR_GPIO_RIGHT_REN 23
#define RR_GPIO_RIGHT_LEN 25
#define RR_GPIO_STATUS_LED 2

// Drives every enable pin and the status LED low. Safe to call before motors_init.
void motors_hold_disabled(void);

// Configures LEDC. On failure the enable pins stay low and the function returns false.
bool motors_init(uint16_t min_duty_permille, uint16_t max_duty_permille);

// Applies a signed command per side. A full stop disables both drivers.
void motors_apply(int16_t left, int16_t right);

void motors_force_disable(void);

#endif
