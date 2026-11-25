#pragma once

#include "display.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "imu.h"
#include <stdbool.h>

typedef struct {
    gpio_num_t gpio_num;
    bool active_low;
    bool present;
} board_button_config_t;

typedef struct {
    gpio_num_t gpio_num;
    uint32_t active_level;
    bool present;
} board_led_config_t;

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    bool enable_internal_pullup;
} board_i2c_config_t;

typedef struct {
    int uart_num;
    gpio_num_t rx_gpio;
    gpio_num_t tx_gpio;
    uint32_t baud_rate;
    bool present;
} board_gps_config_t;

typedef struct {
    const char *board_name;
    uint32_t startup_delay_ms;
    board_i2c_config_t i2c;
    board_gps_config_t gps;
    uint16_t lcd_width;
    uint16_t lcd_height;
    gpio_num_t lcd_mosi_gpio;
    gpio_num_t lcd_sclk_gpio;
    gpio_num_t lcd_cs_gpio;
    gpio_num_t lcd_dc_gpio;
    gpio_num_t lcd_rst_gpio;
    gpio_num_t lcd_bl_gpio;
    bool has_lcd_bl_gpio;
    bool lcd_bl_use_pwm;
    uint32_t lcd_bl_active_level;
    uint8_t lcd_min_brightness_level;
    uint16_t lcd_pwm_duty[6];
    bool lcd_zero_level_pulse;
    uint32_t lcd_zero_pulse_interval_ms;
    uint32_t lcd_zero_pulse_duration_ms;
    uint8_t lcd_zero_pulse_level;
    bool uses_axp192;
    bool uses_m5pm1;
    bool lcd_invert_color;
    bool lcd_swap_xy;
    bool lcd_mirror_x;
    bool lcd_mirror_y;
    int lcd_gap_x;
    int lcd_gap_y;
    gpio_num_t hold_gpio;
    bool has_hold_gpio;
    gpio_num_t imu_tx_select_gpio;
    bool has_imu_tx_select_gpio;
    board_button_config_t button_a;
    board_button_config_t button_b;
    board_button_config_t button_c;
    board_led_config_t red_led;
    bool has_display_status_led;
    const board_display_driver_t *display_driver;
    const board_imu_driver_t *imu_driver;
} board_config_t;

const board_config_t *board_get_config(void);
esp_err_t board_init(void);
