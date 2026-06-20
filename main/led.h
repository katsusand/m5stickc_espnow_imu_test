#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    gpio_num_t gpio_num;
    uint32_t active_level;
} led_config_t;

typedef struct {
    led_config_t config;
    uint32_t active_sources;
    portMUX_TYPE lock;
} led_t;

esp_err_t led_init(led_t *led, const led_config_t *config);
void led_set(led_t *led, bool on);
void led_set_source(led_t *led, uint32_t source_mask, bool active);
