#pragma once

#include "esp_err.h"

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
} imu_sample_t;

esp_err_t imu_init(void);
esp_err_t imu_read_sample(imu_sample_t *out_sample);
