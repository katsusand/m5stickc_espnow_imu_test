#pragma once

#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    gpio_num_t gpio_num;
    bool active_low;
    uint32_t poll_interval_ms;
    uint8_t stable_polls;
} button_config_t;

typedef struct {
    button_config_t config;
    bool stable_pressed;
    bool candidate_pressed;
    uint8_t candidate_count;
} button_t;

void button_init(button_t *button, const button_config_t *config);
bool button_is_pressed(const button_t *button);
bool button_poll(button_t *button, bool *pressed_state_changed);
