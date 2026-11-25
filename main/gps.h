#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uart_port_t uart_num;
    gpio_num_t rx_gpio;
    gpio_num_t tx_gpio;
    uint32_t baud_rate;
} gps_config_t;

typedef struct {
    bool receiving;
    bool fix_valid;
    bool time_valid;
    uint8_t satellites;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_knots;
    float course_deg;
    float hdop;
    uint8_t fix_type;
    uint16_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    uint64_t gps_utc_ms;
    uint32_t sentence_count;
    uint32_t checksum_error_count;
    uint32_t last_sentence_age_ms;
    uint32_t bytes_received;
    uint32_t bytes_dropped;
} gps_status_t;

esp_err_t gps_init(const gps_config_t *config);
void gps_process(uint32_t timeout_ms);
void gps_get_status(gps_status_t *status);
