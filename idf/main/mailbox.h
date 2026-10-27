// Single-slot mailbox between the HTTP task (core 0) and the control task (core 1).
//
// The network task is the only writer of the input fields. The control task is
// the only writer of the published fields. A short spinlock copies the struct;
// the control loop never waits on the socket.

#ifndef ROBO_MAILBOX_H
#define ROBO_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "logic.h"

typedef struct {
    uint8_t buttons;
    uint8_t mode;
    uint16_t seq;
    bool link_up;
    bool arm;
    bool disarm;
    bool ack;
    bool estop;
    bool battery_ok;
    uint16_t client_seq;
} rr_inputs_t;

typedef struct {
    uint8_t state;
    bool has_fault;
    uint8_t fault;
    uint8_t mode;
    bool link_up;
    bool battery_ok;
    int16_t left;
    int16_t right;
    uint16_t client_seq;
    uint16_t battery_mv;
} rr_published_t;

void mailbox_init(void);
void mailbox_set_control_task(TaskHandle_t task);
void mailbox_set_link(bool up);
void mailbox_post(const rr_client_frame_t *frame);
void mailbox_set_battery(uint16_t pack_mv, bool ok);
void mailbox_publish(uint8_t state, bool has_fault, uint8_t fault, int16_t left, int16_t right);
rr_inputs_t mailbox_take(void);
void mailbox_read(rr_published_t *out);

#endif
