#include "display.h"

#include "board.h"
#include "board_i2c.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define LCD_TEXT_SCALE 2
#define LCD_LIST_TEXT_SCALE 1
#define LCD_MARGIN_X 8
#define LCD_MARGIN_Y 6
#define LCD_BG_COLOR 0x0000
#define LCD_FG_COLOR 0xffff
#define LCD_ACCENT_COLOR 0x07e0
#define LCD_STATUS_LED_COLOR 0x00f8
#define LCD_STATUS_LED_RADIUS 8
#define LCD_STATUS_LED_MARGIN 4
#define LCD_GAP_X 40
#define LCD_GAP_Y 52
#define LCD_BRIGHTNESS_LEVEL_COUNT 6
#define LCD_DEFAULT_BRIGHTNESS_LEVEL 3

#define AXP192_I2C_ADDR 0x34
#define AXP192_I2C_SPEED_HZ 400000
#define AXP192_OUTPUT_CONTROL_REG 0x12
#define AXP192_LDO23_VOLTAGE_REG 0x28
#define AXP192_LDO2_ENABLE_MASK 0x04
#define AXP192_LDO2_VOLTAGE_MASK 0xf0

#define M5PM1_I2C_ADDR 0x6e
#define M5PM1_I2C_SPEED_HZ 100000
#define M5PM1_WAKE_DELAY_MS 10
#define M5PM1_RETRY_DELAY_MS 800
#define M5PM1_POWER_CONFIG_REG 0x06
#define M5PM1_BOOST_5V_ENABLE_MASK (1U << 3)
#define M5PM1_I2C_CONFIG_REG 0x09
#define M5PM1_GPIO_MODE_REG 0x10
#define M5PM1_GPIO_OUT_REG 0x11
#define M5PM1_GPIO_DRIVE_REG 0x13
#define M5PM1_GPIO_FUNC0_REG 0x16
#define M5PM1_L3B_GPIO 2
#define M5PM1_L3B_GPIO_MASK (1U << M5PM1_L3B_GPIO)
#define M5PM1_L3B_FUNC_MASK (0x03U << (M5PM1_L3B_GPIO * 2))

#define LCD_BACKLIGHT_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LCD_BACKLIGHT_LEDC_TIMER LEDC_TIMER_0
#define LCD_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BACKLIGHT_LEDC_RESOLUTION LEDC_TIMER_9_BIT
#define LCD_BACKLIGHT_LEDC_RESOLUTION_BITS 9
#define LCD_BACKLIGHT_LEDC_FREQUENCY_HZ 256

static const char *TAG = "display_m5c";
static esp_lcd_panel_handle_t s_lcd_panel = NULL;
static uint16_t *s_lcd_frame_buffer = NULL;
static SemaphoreHandle_t s_lcd_flush_done = NULL;
static i2c_master_dev_handle_t s_axp192 = NULL;
static i2c_master_dev_handle_t s_m5pm1 = NULL;

static bool board_uses_axp192(void)
{
    return board_get_config()->uses_axp192;
}

static esp_err_t axp192_read_register(uint8_t reg_addr, uint8_t *reg_value)
{
    return i2c_master_transmit_receive(s_axp192, &reg_addr, sizeof(reg_addr), reg_value, sizeof(*reg_value), -1);
}

static esp_err_t axp192_write_register(uint8_t reg_addr, uint8_t reg_value)
{
    uint8_t payload[2] = { reg_addr, reg_value };
    return i2c_master_transmit(s_axp192, payload, sizeof(payload), -1);
}

static esp_err_t axp192_update_register(uint8_t reg_addr, uint8_t reg_value, uint8_t mask)
{
    uint8_t current_value = 0;
    ESP_RETURN_ON_ERROR(axp192_read_register(reg_addr, &current_value), TAG, "Read AXP192 register 0x%02x", reg_addr);

    uint8_t next_value = (current_value & (uint8_t)~mask) | (reg_value & mask);
    if (next_value == current_value) {
        return ESP_OK;
    }

    return axp192_write_register(reg_addr, next_value);
}

static esp_err_t m5pm1_read_register(uint8_t reg_addr, uint8_t *reg_value)
{
    return i2c_master_transmit_receive(s_m5pm1, &reg_addr, sizeof(reg_addr), reg_value, sizeof(*reg_value), -1);
}

static esp_err_t m5pm1_write_register(uint8_t reg_addr, uint8_t reg_value)
{
    const uint8_t payload[2] = { reg_addr, reg_value };
    return i2c_master_transmit(s_m5pm1, payload, sizeof(payload), -1);
}

static esp_err_t m5pm1_update_register(uint8_t reg_addr, uint8_t reg_value, uint8_t mask)
{
    uint8_t current_value = 0;
    ESP_RETURN_ON_ERROR(m5pm1_read_register(reg_addr, &current_value), TAG, "Read M5PM1 register 0x%02x", reg_addr);

    const uint8_t next_value = (current_value & (uint8_t)~mask) | (reg_value & mask);
    if (next_value == current_value) {
        return ESP_OK;
    }
    return m5pm1_write_register(reg_addr, next_value);
}

static void m5pm1_send_wake(void)
{
    const esp_err_t wake_err = board_i2c_probe(M5PM1_I2C_ADDR, 10);
    if (wake_err != ESP_OK) {
        ESP_LOGD(TAG, "M5PM1 wake probe returned %s", esp_err_to_name(wake_err));
    }
    vTaskDelay(pdMS_TO_TICKS(M5PM1_WAKE_DELAY_MS));
}

static esp_err_t m5pm1_configure_l3b_power(void)
{
    // BOOST5V_EN_PP changes the Grove/EXT_5V rail from input mode to 5V output.
    ESP_RETURN_ON_ERROR(m5pm1_update_register(M5PM1_POWER_CONFIG_REG,
                                              M5PM1_BOOST_5V_ENABLE_MASK,
                                              M5PM1_BOOST_5V_ENABLE_MASK),
                        TAG,
                        "Enable M5PM1 Grove 5V output");

    // M5PM1 GPIO2 enables the L3B rail used by the LCD backlight.
    ESP_RETURN_ON_ERROR(m5pm1_update_register(M5PM1_GPIO_FUNC0_REG, 0, M5PM1_L3B_FUNC_MASK),
                        TAG,
                        "Select M5PM1 GPIO2 function");
    ESP_RETURN_ON_ERROR(m5pm1_update_register(M5PM1_GPIO_MODE_REG,
                                              M5PM1_L3B_GPIO_MASK,
                                              M5PM1_L3B_GPIO_MASK),
                        TAG,
                        "Set M5PM1 GPIO2 output mode");
    ESP_RETURN_ON_ERROR(m5pm1_update_register(M5PM1_GPIO_DRIVE_REG, 0, M5PM1_L3B_GPIO_MASK),
                        TAG,
                        "Set M5PM1 GPIO2 push-pull drive");
    ESP_RETURN_ON_ERROR(m5pm1_update_register(M5PM1_GPIO_OUT_REG,
                                              M5PM1_L3B_GPIO_MASK,
                                              M5PM1_L3B_GPIO_MASK),
                        TAG,
                        "Set M5PM1 L3B enable high");
    ESP_RETURN_ON_ERROR(m5pm1_update_register(M5PM1_I2C_CONFIG_REG, 0, 0xff),
                        TAG,
                        "Disable M5PM1 I2C idle sleep");
    return ESP_OK;
}

static esp_err_t m5pm1_enable_l3b_power(void)
{
    ESP_RETURN_ON_ERROR(board_i2c_add_device(M5PM1_I2C_ADDR, M5PM1_I2C_SPEED_HZ, &s_m5pm1),
                        TAG,
                        "Add M5PM1 to shared I2C bus");

    vTaskDelay(pdMS_TO_TICKS(100));
    m5pm1_send_wake();

    esp_err_t err = m5pm1_configure_l3b_power();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "M5PM1 initialization failed (%s); retrying", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(M5PM1_RETRY_DELAY_MS));
        m5pm1_send_wake();
        err = m5pm1_configure_l3b_power();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "Configure M5PM1 L3B power");

    ESP_LOGI(TAG, "Enabled StickS3 L3B and Grove 5V output via M5PM1");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t init_display_power(void)
{
    if (board_get_config()->uses_m5pm1) {
        return m5pm1_enable_l3b_power();
    }

    if (!board_uses_axp192()) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(board_i2c_add_device(AXP192_I2C_ADDR, AXP192_I2C_SPEED_HZ, &s_axp192),
                        TAG,
                        "Add AXP192 to shared I2C bus");
    ESP_RETURN_ON_ERROR(axp192_update_register(AXP192_OUTPUT_CONTROL_REG, 0x4D, 0xFF),
                        TAG,
                        "Enable AXP192 LCD power rails");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t init_backlight_pwm(void)
{
    const board_config_t *config = board_get_config();

    if (!config->has_lcd_bl_gpio) {
        return ESP_OK;
    }

    if (!config->lcd_bl_use_pwm) {
        ESP_RETURN_ON_ERROR(gpio_reset_pin(config->lcd_bl_gpio), TAG, "Reset LCD backlight GPIO");
        ESP_RETURN_ON_ERROR(gpio_set_direction(config->lcd_bl_gpio, GPIO_MODE_OUTPUT), TAG, "Configure LCD backlight GPIO");
        ESP_RETURN_ON_ERROR(gpio_set_level(config->lcd_bl_gpio, config->lcd_bl_active_level),
                            TAG,
                            "Enable LCD backlight GPIO");
        ESP_LOGI(TAG,
                 "LCD backlight GPIO%d set to active level %lu",
                 config->lcd_bl_gpio,
                 (unsigned long)config->lcd_bl_active_level);
        return ESP_OK;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LCD_BACKLIGHT_LEDC_MODE,
        .duty_resolution = LCD_BACKLIGHT_LEDC_RESOLUTION,
        .timer_num = LCD_BACKLIGHT_LEDC_TIMER,
        .freq_hz = LCD_BACKLIGHT_LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "Configure LCD backlight PWM timer");

    const ledc_channel_config_t channel_config = {
        .gpio_num = config->lcd_bl_gpio,
        .speed_mode = LCD_BACKLIGHT_LEDC_MODE,
        .channel = LCD_BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LCD_BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&channel_config);
}

static const uint8_t FONT_5X7[][5] = {
    [' '] = { 0x00, 0x00, 0x00, 0x00, 0x00 },
    ['-'] = { 0x08, 0x08, 0x08, 0x08, 0x08 },
    ['.'] = { 0x00, 0x00, 0x00, 0x18, 0x18 },
    ['0'] = { 0x3e, 0x45, 0x49, 0x51, 0x3e },
    ['1'] = { 0x00, 0x21, 0x7f, 0x01, 0x00 },
    ['2'] = { 0x23, 0x45, 0x49, 0x51, 0x21 },
    ['3'] = { 0x22, 0x41, 0x49, 0x49, 0x36 },
    ['4'] = { 0x18, 0x28, 0x48, 0x7f, 0x08 },
    ['5'] = { 0x72, 0x51, 0x51, 0x51, 0x4e },
    ['6'] = { 0x1e, 0x29, 0x49, 0x49, 0x06 },
    ['7'] = { 0x40, 0x47, 0x48, 0x50, 0x60 },
    ['8'] = { 0x36, 0x49, 0x49, 0x49, 0x36 },
    ['9'] = { 0x30, 0x49, 0x49, 0x4a, 0x3c },
    [':'] = { 0x00, 0x36, 0x36, 0x00, 0x00 },
    ['A'] = { 0x1f, 0x24, 0x44, 0x24, 0x1f },
    ['B'] = { 0x7f, 0x49, 0x49, 0x49, 0x36 },
    ['C'] = { 0x3e, 0x41, 0x41, 0x41, 0x22 },
    ['D'] = { 0x7f, 0x41, 0x41, 0x22, 0x1c },
    ['E'] = { 0x7f, 0x49, 0x49, 0x49, 0x41 },
    ['F'] = { 0x7f, 0x48, 0x48, 0x48, 0x40 },
    ['G'] = { 0x3e, 0x41, 0x49, 0x49, 0x2e },
    ['I'] = { 0x00, 0x41, 0x7f, 0x41, 0x00 },
    ['L'] = { 0x7f, 0x01, 0x01, 0x01, 0x01 },
    ['M'] = { 0x7f, 0x20, 0x18, 0x20, 0x7f },
    ['N'] = { 0x7f, 0x20, 0x10, 0x08, 0x7f },
    ['O'] = { 0x3e, 0x41, 0x41, 0x41, 0x3e },
    ['P'] = { 0x7f, 0x48, 0x48, 0x48, 0x30 },
    ['R'] = { 0x7f, 0x48, 0x4c, 0x4a, 0x31 },
    ['S'] = { 0x31, 0x49, 0x49, 0x49, 0x46 },
    ['T'] = { 0x40, 0x40, 0x7f, 0x40, 0x40 },
    ['U'] = { 0x7e, 0x01, 0x01, 0x01, 0x7e },
    ['V'] = { 0x7c, 0x02, 0x01, 0x02, 0x7c },
    ['W'] = { 0x7e, 0x01, 0x0e, 0x01, 0x7e },
    ['X'] = { 0x63, 0x14, 0x08, 0x14, 0x63 },
};

static bool on_lcd_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    (void)panel_io;
    (void)edata;

    BaseType_t high_task_woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &high_task_woken);
    return high_task_woken == pdTRUE;
}

static void lcd_fill_screen(uint16_t color)
{
    if (s_lcd_frame_buffer == NULL) {
        return;
    }

    const board_config_t *config = board_get_config();
    for (int i = 0; i < config->lcd_width * config->lcd_height; ++i) {
        s_lcd_frame_buffer[i] = color;
    }
}

static void lcd_draw_pixel(int x, int y, uint16_t color)
{
    const board_config_t *config = board_get_config();
    if (s_lcd_frame_buffer == NULL || x < 0 || x >= config->lcd_width || y < 0 || y >= config->lcd_height) {
        return;
    }

    s_lcd_frame_buffer[(y * config->lcd_width) + x] = color;
}

static void lcd_fill_circle(int center_x, int center_y, int radius, uint16_t color)
{
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if ((x * x) + (y * y) <= radius * radius) {
                lcd_draw_pixel(center_x + x, center_y + y, color);
            }
        }
    }
}

static void lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    const uint8_t *glyph = NULL;
    if ((unsigned char)c < sizeof(FONT_5X7) / sizeof(FONT_5X7[0])) {
        glyph = FONT_5X7[(unsigned char)c];
    }
    if (glyph == NULL) {
        glyph = FONT_5X7[(unsigned char)' '];
    }

    for (int col = 0; col < 5; ++col) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; ++row) {
            uint16_t color = (bits & (1 << (6 - row))) ? fg : bg;
            for (int dx = 0; dx < scale; ++dx) {
                for (int dy = 0; dy < scale; ++dy) {
                    lcd_draw_pixel(x + (col * scale) + dx, y + (row * scale) + dy, color);
                }
            }
        }
    }

    for (int dx = 0; dx < scale; ++dx) {
        for (int row = 0; row < 7 * scale; ++row) {
            lcd_draw_pixel(x + (5 * scale) + dx, y + row, bg);
        }
    }
}

static void lcd_draw_text(int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    while (*text != '\0') {
        lcd_draw_char(x, y, *text, fg, bg, scale);
        x += 6 * scale;
        ++text;
    }
}

static void lcd_flush(void)
{
    if (s_lcd_panel == NULL || s_lcd_frame_buffer == NULL || s_lcd_flush_done == NULL) {
        return;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
        s_lcd_panel, 0, 0, board_get_config()->lcd_width, board_get_config()->lcd_height, s_lcd_frame_buffer));
    xSemaphoreTake(s_lcd_flush_done, portMAX_DELAY);
}

static void format_mac(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static esp_err_t m5stickc_display_init(void)
{
    const board_config_t *config = board_get_config();

    ESP_RETURN_ON_ERROR(init_display_power(), TAG, "Initialize LCD power");

    spi_bus_config_t bus_config = {
        .sclk_io_num = config->lcd_sclk_gpio,
        .mosi_io_num = config->lcd_mosi_gpio,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = config->lcd_width * config->lcd_height * (int)sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO), TAG, "Initialize LCD SPI bus");

    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = config->lcd_cs_gpio,
        .dc_gpio_num = config->lcd_dc_gpio,
        .spi_mode = 0,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 1,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &panel_io), TAG, "Create LCD IO");

    s_lcd_flush_done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_lcd_flush_done != NULL, ESP_ERR_NO_MEM, TAG, "Create LCD flush semaphore");

    esp_lcd_panel_io_callbacks_t io_callbacks = {
        .on_color_trans_done = on_lcd_color_trans_done,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_register_event_callbacks(panel_io, &io_callbacks, s_lcd_flush_done),
        TAG,
        "Register LCD transfer callback");

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = config->lcd_rst_gpio,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(panel_io, &panel_config, &s_lcd_panel), TAG, "Create LCD panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_lcd_panel), TAG, "Reset LCD");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_lcd_panel), TAG, "Initialize LCD");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_lcd_panel, config->lcd_invert_color), TAG, "Invert LCD color");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_lcd_panel, config->lcd_swap_xy), TAG, "Swap LCD axes");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_lcd_panel, config->lcd_mirror_x, config->lcd_mirror_y), TAG, "Mirror LCD");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_lcd_panel, config->lcd_gap_x, config->lcd_gap_y), TAG, "Set LCD gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_lcd_panel, true), TAG, "Enable LCD");

    ESP_RETURN_ON_ERROR(init_backlight_pwm(), TAG, "Initialize LCD backlight PWM");
    ESP_RETURN_ON_ERROR(display_set_brightness(LCD_DEFAULT_BRIGHTNESS_LEVEL), TAG, "Set LCD brightness");

    s_lcd_frame_buffer = heap_caps_malloc(
        config->lcd_width * config->lcd_height * sizeof(uint16_t), MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_lcd_frame_buffer != NULL, ESP_ERR_NO_MEM, TAG, "Allocate LCD frame buffer");

    lcd_fill_screen(LCD_BG_COLOR);
    lcd_flush();
    return ESP_OK;
}

static esp_err_t m5stickc_display_set_brightness(uint8_t level)
{
    ESP_RETURN_ON_FALSE(level < LCD_BRIGHTNESS_LEVEL_COUNT,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "Brightness level must be between 0 and 5");

    const board_config_t *config = board_get_config();
    if (level < config->lcd_min_brightness_level) {
        level = config->lcd_min_brightness_level;
    }

    if (board_uses_axp192()) {
        static const uint8_t axp192_lcd_brightness_table[LCD_BRIGHTNESS_LEVEL_COUNT] = {
            0, 7, 8, 9, 10, 12,
        };

        ESP_RETURN_ON_FALSE(s_axp192 != NULL, ESP_ERR_INVALID_STATE, TAG, "Display power is not initialized");

        const uint8_t voltage_code = axp192_lcd_brightness_table[level];
        if (voltage_code == 0) {
            return axp192_update_register(AXP192_OUTPUT_CONTROL_REG, 0, AXP192_LDO2_ENABLE_MASK);
        }

        ESP_RETURN_ON_ERROR(axp192_update_register(AXP192_LDO23_VOLTAGE_REG,
                                                   (uint8_t)(voltage_code << 4),
                                                   AXP192_LDO2_VOLTAGE_MASK),
                            TAG,
                            "Set AXP192 LCD backlight voltage");
        return axp192_update_register(AXP192_OUTPUT_CONTROL_REG,
                                      AXP192_LDO2_ENABLE_MASK,
                                      AXP192_LDO2_ENABLE_MASK);
    }

    if (!config->lcd_bl_use_pwm) {
        const int level_on = (level == 0) ? !config->lcd_bl_active_level : config->lcd_bl_active_level;
        ESP_RETURN_ON_ERROR(gpio_set_level(config->lcd_bl_gpio, level_on), TAG, "Set LCD backlight GPIO");
        ESP_LOGI(TAG, "LCD backlight GPIO%d -> %d", config->lcd_bl_gpio, level_on);
        return ESP_OK;
    }

    const uint32_t duty = config->lcd_pwm_duty[level];
    ESP_RETURN_ON_ERROR(ledc_set_duty(LCD_BACKLIGHT_LEDC_MODE, LCD_BACKLIGHT_LEDC_CHANNEL, duty),
                        TAG,
                        "Set LCD backlight PWM duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LCD_BACKLIGHT_LEDC_MODE, LCD_BACKLIGHT_LEDC_CHANNEL),
                        TAG,
                        "Update LCD backlight PWM duty");
    ESP_LOGI(TAG, "LCD backlight PWM level %u -> duty %lu/511", level, (unsigned long)duty);
    return ESP_OK;
}

static void m5stickc_display_render_status(const display_status_t *status)
{
    if (status == NULL) {
        return;
    }

    lcd_fill_screen(LCD_BG_COLOR);

    if (status->status_led_visible && status->status_led_on) {
        const board_config_t *config = board_get_config();
        lcd_fill_circle(config->lcd_width - LCD_STATUS_LED_RADIUS - LCD_STATUS_LED_MARGIN,
                        LCD_STATUS_LED_RADIUS + LCD_STATUS_LED_MARGIN,
                        LCD_STATUS_LED_RADIUS,
                        LCD_STATUS_LED_COLOR);
    }

    char gps_text[20];
    if (!status->gps_receiving) {
        snprintf(gps_text, sizeof(gps_text), "GPS WAIT");
    } else if (!status->gps_fix_valid) {
        snprintf(gps_text, sizeof(gps_text), "GPS NOFIX S%02u", status->gps_satellites);
    } else {
        snprintf(gps_text, sizeof(gps_text), "GPS FIX S%02u", status->gps_satellites);
    }
    lcd_draw_text(LCD_MARGIN_X,
                  LCD_MARGIN_Y,
                  gps_text,
                  status->gps_fix_valid ? LCD_ACCENT_COLOR : LCD_FG_COLOR,
                  LCD_BG_COLOR,
                  LCD_LIST_TEXT_SCALE);

    char coordinate_text[32];
    if (status->gps_fix_valid) {
        snprintf(coordinate_text,
                 sizeof(coordinate_text),
                 "LAT %.5f",
                 status->gps_latitude_deg);
    } else {
        snprintf(coordinate_text, sizeof(coordinate_text), "LAT --.-----");
    }
    lcd_draw_text(LCD_MARGIN_X,
                  LCD_MARGIN_Y + 12,
                  coordinate_text,
                  LCD_FG_COLOR,
                  LCD_BG_COLOR,
                  LCD_LIST_TEXT_SCALE);

    char longitude_text[32];
    if (status->gps_fix_valid) {
        snprintf(longitude_text, sizeof(longitude_text), "LON %.5f", status->gps_longitude_deg);
    } else {
        snprintf(longitude_text, sizeof(longitude_text), "LON ---.-----");
    }
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 24, longitude_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    char gps_detail_text[32];
    snprintf(gps_detail_text,
             sizeof(gps_detail_text),
             "SPD %.1fkn CRS %.0f",
             (double)status->gps_speed_knots,
             (double)status->gps_course_deg);
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 36, gps_detail_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    char date_text[32];
    if (status->gps_time_valid) {
        snprintf(date_text,
                 sizeof(date_text),
                 "%04u-%02u-%02u",
                 status->gps_utc_year,
                 status->gps_utc_month,
                 status->gps_utc_day);
    } else {
        snprintf(date_text, sizeof(date_text), "DATE ---- -- --");
    }
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 48, date_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    char utc_text[32];
    if (status->gps_time_valid) {
        snprintf(utc_text,
                 sizeof(utc_text),
                 "%02u:%02u:%02u AGE %lums",
                 status->gps_utc_hour,
                 status->gps_utc_minute,
                 status->gps_utc_second,
                 (unsigned long)status->gps_last_sentence_age_ms);
    } else {
        snprintf(utc_text, sizeof(utc_text), "--:--:-- AGE %lums", (unsigned long)status->gps_last_sentence_age_ms);
    }
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 60, utc_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    char imu_text[32];
    if (!status->imu_available) {
        snprintf(imu_text, sizeof(imu_text), "IMU UNAVAIL");
    } else if (!status->imu_recent) {
        snprintf(imu_text, sizeof(imu_text), "IMU STALE %lums", (unsigned long)status->imu_data_age_ms);
    } else {
        snprintf(imu_text,
                 sizeof(imu_text),
                 "IMU %.2fC AGE %lums",
                 (double)status->imu_temperature_centi_c / 100.0,
                 (unsigned long)status->imu_data_age_ms);
    }
    lcd_draw_text(LCD_MARGIN_X,
                  LCD_MARGIN_Y + 74,
                  imu_text,
                  status->imu_recent ? LCD_ACCENT_COLOR : LCD_FG_COLOR,
                  LCD_BG_COLOR,
                  LCD_LIST_TEXT_SCALE);

    char accel_text[32];
    snprintf(accel_text,
             sizeof(accel_text),
             "ACC %d %d %d mg",
             status->imu_accel_mg_avg[0],
             status->imu_accel_mg_avg[1],
             status->imu_accel_mg_avg[2]);
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 86, accel_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    char gyro_text[32];
    snprintf(gyro_text,
             sizeof(gyro_text),
             "GYR %d %d %d cdps",
             status->imu_gyro_centi_dps_avg[0],
             status->imu_gyro_centi_dps_avg[1],
             status->imu_gyro_centi_dps_avg[2]);
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 98, gyro_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    char self_mac_str[18];
    format_mac(status->self_mac, self_mac_str, sizeof(self_mac_str));
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 110, "SELF", LCD_ACCENT_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);
    lcd_draw_text(LCD_MARGIN_X + 30, LCD_MARGIN_Y + 110, self_mac_str, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    lcd_flush();
}

const board_display_driver_t g_board_display_driver_st7789 = {
    .init = m5stickc_display_init,
    .set_brightness = m5stickc_display_set_brightness,
    .render_status = m5stickc_display_render_status,
};
