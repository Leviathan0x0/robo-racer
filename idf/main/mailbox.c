#include "mailbox.h"

#include "freertos/portmacro.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_control;

static uint8_t s_buttons;
static uint8_t s_mode = RR_MODE_NORMAL;
static uint16_t s_seq;
static bool s_link_up;
static bool s_arm;
static bool s_disarm;
static bool s_ack;
static bool s_estop;
static uint16_t s_client_seq;

static uint8_t s_state = RR_STATE_BOOT;
static bool s_has_fault;
static uint8_t s_fault;
static int16_t s_left;
static int16_t s_right;
static uint16_t s_battery_mv;
static bool s_battery_ok = true;

static void wake(TaskHandle_t task) {
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

void mailbox_init(void) {
    // Static initialisers already hold the safe defaults. This function exists
    // so boot order in main.c reads as a list of steps.
}

void mailbox_set_control_task(TaskHandle_t task) {
    portENTER_CRITICAL(&s_lock);
    s_control = task;
    portEXIT_CRITICAL(&s_lock);
}

void mailbox_set_link(bool up) {
    TaskHandle_t task;
    portENTER_CRITICAL(&s_lock);
    s_link_up = up;
    task = s_control;
    portEXIT_CRITICAL(&s_lock);
    wake(task);
}

void mailbox_post(const rr_client_frame_t *frame) {
    TaskHandle_t task;
    portENTER_CRITICAL(&s_lock);
    if ((frame->flags & RR_FLAG_ESTOP) != 0) {
        s_estop = true;
    }
    switch (frame->kind) {
    case RR_KIND_DRIVE:
        s_buttons = frame->buttons;
        s_mode = frame->mode;
        s_seq = (uint16_t)(s_seq + 1u);
        s_client_seq = frame->seq;
        break;
    case RR_KIND_ARM:
        s_arm = true;
        break;
    case RR_KIND_DISARM:
        s_disarm = true;
        break;
    case RR_KIND_ACK:
        s_ack = true;
        break;
    default:
        break;
    }
    task = s_control;
    portEXIT_CRITICAL(&s_lock);
    wake(task);
}

void mailbox_set_battery(uint16_t pack_mv, bool ok) {
    portENTER_CRITICAL(&s_lock);
    s_battery_mv = pack_mv;
    s_battery_ok = ok;
    portEXIT_CRITICAL(&s_lock);
}

void mailbox_publish(uint8_t state, bool has_fault, uint8_t fault, int16_t left, int16_t right) {
    portENTER_CRITICAL(&s_lock);
    s_state = state;
    s_has_fault = has_fault;
    s_fault = fault;
    s_left = left;
    s_right = right;
    portEXIT_CRITICAL(&s_lock);
}

rr_inputs_t mailbox_take(void) {
    rr_inputs_t inputs;
    portENTER_CRITICAL(&s_lock);
    inputs.buttons = s_buttons;
    inputs.mode = s_mode;
    inputs.seq = s_seq;
    inputs.link_up = s_link_up;
    inputs.arm = s_arm;
    inputs.disarm = s_disarm;
    inputs.ack = s_ack;
    inputs.estop = s_estop;
    inputs.battery_ok = s_battery_ok;
    inputs.client_seq = s_client_seq;
    s_arm = false;
    s_disarm = false;
    s_ack = false;
    s_estop = false;
    portEXIT_CRITICAL(&s_lock);
    return inputs;
}

void mailbox_read(rr_published_t *out) {
    portENTER_CRITICAL(&s_lock);
    out->state = s_state;
    out->has_fault = s_has_fault;
    out->fault = s_fault;
    out->mode = s_mode;
    out->link_up = s_link_up;
    out->battery_ok = s_battery_ok;
    out->left = s_left;
    out->right = s_right;
    out->client_seq = s_client_seq;
    out->battery_mv = s_battery_mv;
    portEXIT_CRITICAL(&s_lock);
}
