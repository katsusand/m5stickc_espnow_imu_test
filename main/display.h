#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_MAX_TRACKED_DEVICES 5

typedef struct {
    bool status_led_visible;
    bool status_led_on;
    bool imu_available;
    bool imu_recent;
    int16_t imu_accel_mg_avg[3];
    int16_t imu_gyro_centi_dps_avg[3];
    int16_t imu_temperature_centi_c;
    uint32_t imu_data_age_ms;
    bool gps_receiving;
    bool gps_fix_valid;
    bool gps_time_valid;
    uint8_t gps_satellites;
    double gps_latitude_deg;
    double gps_longitude_deg;
    float gps_speed_knots;
    float gps_course_deg;
    uint16_t gps_utc_year;
    uint8_t gps_utc_month;
    uint8_t gps_utc_day;
    uint8_t gps_utc_hour;
    uint8_t gps_utc_minute;
    uint8_t gps_utc_second;
    uint32_t gps_last_sentence_age_ms;
    uint8_t self_mac[6];
} display_status_t;

typedef struct board_display_driver {
    esp_err_t (*init)(void);
    esp_err_t (*set_brightness)(uint8_t level);
    void (*render_status)(const display_status_t *status);
} board_display_driver_t;

esp_err_t display_init(void);
// level: 0=off, 1=minimum, 2=low, 3=normal, 4=bright, 5=maximum
esp_err_t display_set_brightness(uint8_t level);
void display_render_status(const display_status_t *status);
