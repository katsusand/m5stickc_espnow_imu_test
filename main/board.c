#include "board.h"

#include "board_m5stickc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "board";

static const board_config_t s_board_config = {
#if CONFIG_IDF_TARGET_ESP32S3
    .board_name = "M5StickS3",
    .startup_delay_ms = 5000,
    .i2c = {
        .sda_gpio = GPIO_NUM_47,
        .scl_gpio = GPIO_NUM_48,
        .enable_internal_pullup = true,
    },
    .gps = {
        .uart_num = 1,
        .tx_gpio = GPIO_NUM_9,
        .rx_gpio = GPIO_NUM_10,
        .baud_rate = 115200,
        .present = true,
    },
    .lcd_width = 240,
    .lcd_height = 135,
    .lcd_mosi_gpio = GPIO_NUM_39,
    .lcd_sclk_gpio = GPIO_NUM_40,
    .lcd_cs_gpio = GPIO_NUM_41,
    .lcd_dc_gpio = GPIO_NUM_45,
    .lcd_rst_gpio = GPIO_NUM_21,
    .lcd_bl_gpio = GPIO_NUM_38,
    .has_lcd_bl_gpio = true,
    .lcd_bl_use_pwm = true,
    .lcd_bl_active_level = 1,
    .lcd_min_brightness_level = 0,
    .lcd_pwm_duty = { 0, 4, 12, 40, 128, 511 },
    .lcd_zero_level_pulse = true,
    .lcd_zero_pulse_interval_ms = 10000,
    .lcd_zero_pulse_duration_ms = 500,
    .lcd_zero_pulse_level = 2,
    .uses_axp192 = false,
    .uses_m5pm1 = true,
    .lcd_invert_color = true,
    .lcd_swap_xy = true,
    .lcd_mirror_x = false,
    .lcd_mirror_y = true,
    .lcd_gap_x = 40,
    .lcd_gap_y = 52,
    .hold_gpio = GPIO_NUM_NC,
    .has_hold_gpio = false,
    .imu_tx_select_gpio = GPIO_NUM_5,
    .has_imu_tx_select_gpio = true,
    .button_a = {
        .gpio_num = GPIO_NUM_11,
        .active_low = true,
        .present = true,
    },
    .button_b = {
        .gpio_num = GPIO_NUM_12,
        .active_low = true,
        .present = true,
    },
    .button_c = {
        .gpio_num = GPIO_NUM_NC,
        .active_low = true,
        .present = false,
    },
    .red_led = {
        .gpio_num = GPIO_NUM_2,
        .active_level = 0,
        .present = true,
    },
    .has_display_status_led = true,
#elif CONFIG_M5STICKC_BOARD_PLUS2
    .board_name = "M5StickC Plus2",
    .startup_delay_ms = 0,
    .i2c = {
        .sda_gpio = GPIO_NUM_21,
        .scl_gpio = GPIO_NUM_22,
        .enable_internal_pullup = true,
    },
    .gps = {
        .uart_num = 1,
        .tx_gpio = GPIO_NUM_32,
        .rx_gpio = GPIO_NUM_33,
        .baud_rate = 115200,
        .present = true,
    },
    .lcd_width = 240,
    .lcd_height = 135,
    .lcd_mosi_gpio = GPIO_NUM_15,
    .lcd_sclk_gpio = GPIO_NUM_13,
    .lcd_cs_gpio = GPIO_NUM_5,
    .lcd_dc_gpio = GPIO_NUM_14,
    .lcd_rst_gpio = GPIO_NUM_12,
    .lcd_bl_gpio = GPIO_NUM_27,
    .has_lcd_bl_gpio = true,
    .lcd_bl_use_pwm = true,
    .lcd_bl_active_level = 1,
    .lcd_min_brightness_level = 0,
    .lcd_pwm_duty = { 0, 96, 160, 256, 384, 511 },
    .lcd_zero_level_pulse = false,
    .lcd_zero_pulse_interval_ms = 0,
    .lcd_zero_pulse_duration_ms = 0,
    .lcd_zero_pulse_level = 0,
    .uses_axp192 = false,
    .uses_m5pm1 = false,
    .lcd_invert_color = true,
    .lcd_swap_xy = true,
    .lcd_mirror_x = false,
    .lcd_mirror_y = true,
    .lcd_gap_x = 40,
    .lcd_gap_y = 52,
    .hold_gpio = GPIO_NUM_4,
    .has_hold_gpio = true,
    .imu_tx_select_gpio = GPIO_NUM_26,
    .has_imu_tx_select_gpio = true,
    .button_a = {
        .gpio_num = GPIO_NUM_37,
        .active_low = true,
        .present = true,
    },
    .button_b = {
        .gpio_num = GPIO_NUM_39,
        .active_low = true,
        .present = true,
    },
    .button_c = {
        .gpio_num = GPIO_NUM_35,
        .active_low = true,
        .present = true,
    },
    .red_led = {
        .gpio_num = GPIO_NUM_19,
        .active_level = 1,
        .present = true,
    },
    .has_display_status_led = false,
#else
    .board_name = "M5StickC Plus",
    .startup_delay_ms = 0,
    .i2c = {
        .sda_gpio = GPIO_NUM_21,
        .scl_gpio = GPIO_NUM_22,
        .enable_internal_pullup = true,
    },
    .gps = {
        .uart_num = 1,
        .tx_gpio = GPIO_NUM_32,
        .rx_gpio = GPIO_NUM_33,
        .baud_rate = 115200,
        .present = true,
    },
    .lcd_width = 240,
    .lcd_height = 135,
    .lcd_mosi_gpio = GPIO_NUM_15,
    .lcd_sclk_gpio = GPIO_NUM_13,
    .lcd_cs_gpio = GPIO_NUM_5,
    .lcd_dc_gpio = GPIO_NUM_23,
    .lcd_rst_gpio = GPIO_NUM_18,
    .lcd_bl_gpio = GPIO_NUM_NC,
    .has_lcd_bl_gpio = false,
    .lcd_bl_use_pwm = false,
    .lcd_bl_active_level = 1,
    .lcd_min_brightness_level = 0,
    .lcd_pwm_duty = { 0 },
    .lcd_zero_level_pulse = false,
    .lcd_zero_pulse_interval_ms = 0,
    .lcd_zero_pulse_duration_ms = 0,
    .lcd_zero_pulse_level = 0,
    .uses_axp192 = true,
    .uses_m5pm1 = false,
    .lcd_invert_color = true,
    .lcd_swap_xy = true,
    .lcd_mirror_x = false,
    .lcd_mirror_y = true,
    .lcd_gap_x = 40,
    .lcd_gap_y = 52,
    .hold_gpio = GPIO_NUM_NC,
    .has_hold_gpio = false,
    .imu_tx_select_gpio = GPIO_NUM_26,
    .has_imu_tx_select_gpio = true,
    .button_a = {
        .gpio_num = GPIO_NUM_37,
        .active_low = true,
        .present = true,
    },
    .button_b = {
        .gpio_num = GPIO_NUM_39,
        .active_low = true,
        .present = true,
    },
    .button_c = {
        .gpio_num = GPIO_NUM_NC,
        .active_low = true,
        .present = false,
    },
    .red_led = {
        .gpio_num = GPIO_NUM_10,
        .active_level = 0,
        .present = true,
    },
    .has_display_status_led = false,
#endif
    .display_driver = &g_board_display_driver_st7789,
#if CONFIG_IDF_TARGET_ESP32S3
    .imu_driver = &g_board_imu_driver_bmi270,
#else
    .imu_driver = &g_board_imu_driver_mpu6886,
#endif
};

const board_config_t *board_get_config(void)
{
    return &s_board_config;
}

esp_err_t board_init(void)
{
    const board_config_t *config = board_get_config();

    if (config->has_hold_gpio) {
        ESP_RETURN_ON_ERROR(gpio_reset_pin(config->hold_gpio), TAG, "Reset HOLD GPIO");
        ESP_RETURN_ON_ERROR(gpio_set_direction(config->hold_gpio, GPIO_MODE_OUTPUT), TAG, "Set HOLD GPIO direction");
        ESP_RETURN_ON_ERROR(gpio_set_level(config->hold_gpio, 1), TAG, "Set HOLD GPIO high");
        ESP_LOGI(TAG, "Set HOLD GPIO%d high", config->hold_gpio);
    }

    return ESP_OK;
}
