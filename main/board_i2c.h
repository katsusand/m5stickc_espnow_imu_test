#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

esp_err_t board_i2c_init(void);
esp_err_t board_i2c_add_device(uint8_t device_address,
                               uint32_t scl_speed_hz,
                               i2c_master_dev_handle_t *out_device);
esp_err_t board_i2c_remove_device(i2c_master_dev_handle_t device);
