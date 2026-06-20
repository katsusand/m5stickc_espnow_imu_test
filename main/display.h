#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_MAX_TRACKED_DEVICES 5

typedef struct {
    bool in_use;
    uint8_t mac_addr[6];
    char event_code;
    uint32_t rx_count;
    uint32_t last_seen_order;
} display_device_info_t;

typedef struct {
    uint32_t used_count;
    uint32_t ignored_count;
    uint8_t self_mac[6];
    display_device_info_t devices[DISPLAY_MAX_TRACKED_DEVICES];
} display_status_t;

esp_err_t display_init(void);
// level: 0=off, 1=minimum, 2=low, 3=normal, 4=bright, 5=maximum
esp_err_t display_set_brightness(uint8_t level);
void display_render_status(const display_status_t *status);
