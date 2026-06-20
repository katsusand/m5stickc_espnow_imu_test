#include "display.h"

#include "board_i2c.h"
#include "board_pins.h"
#include "driver/gpio.h"
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
#define LCD_GAP_X 40
#define LCD_GAP_Y 52
#define LCD_BRIGHTNESS_LEVEL_COUNT 6
#define LCD_DEFAULT_BRIGHTNESS_LEVEL 3

#if M5STICKC_HAS_AXP192
#define AXP192_I2C_ADDR 0x34
#define AXP192_I2C_SPEED_HZ 400000
#define AXP192_OUTPUT_CONTROL_REG 0x12
#define AXP192_LDO23_VOLTAGE_REG 0x28
#define AXP192_LDO2_ENABLE_MASK 0x04
#define AXP192_LDO2_VOLTAGE_MASK 0xf0
#endif

#if M5STICKC_HAS_LCD_BL_GPIO
#define LCD_BACKLIGHT_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LCD_BACKLIGHT_LEDC_TIMER LEDC_TIMER_0
#define LCD_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BACKLIGHT_LEDC_RESOLUTION LEDC_TIMER_9_BIT
#define LCD_BACKLIGHT_LEDC_RESOLUTION_BITS 9
#define LCD_BACKLIGHT_LEDC_MAX_DUTY ((1U << LCD_BACKLIGHT_LEDC_RESOLUTION_BITS) - 1)
#define LCD_BACKLIGHT_LEDC_FREQUENCY_HZ 256
#endif

static const char *TAG = "display";
static const int s_lcd_width = 240;
static const int s_lcd_height = 135;
static esp_lcd_panel_handle_t s_lcd_panel = NULL;
static uint16_t *s_lcd_frame_buffer = NULL;
static SemaphoreHandle_t s_lcd_flush_done = NULL;
#if M5STICKC_HAS_AXP192
static i2c_master_dev_handle_t s_axp192 = NULL;
#endif

#if M5STICKC_HAS_AXP192
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

static esp_err_t init_display_power(void)
{
    ESP_RETURN_ON_ERROR(board_i2c_add_device(AXP192_I2C_ADDR, AXP192_I2C_SPEED_HZ, &s_axp192),
                        TAG,
                        "Add AXP192 to shared I2C bus");
    ESP_RETURN_ON_ERROR(axp192_update_register(AXP192_OUTPUT_CONTROL_REG, 0x4D, 0xFF),
                        TAG,
                        "Enable AXP192 LCD power rails");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}
#endif

#if M5STICKC_HAS_LCD_BL_GPIO
static esp_err_t init_backlight_pwm(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LCD_BACKLIGHT_LEDC_MODE,
        .duty_resolution = LCD_BACKLIGHT_LEDC_RESOLUTION,
        .timer_num = LCD_BACKLIGHT_LEDC_TIMER,
        .freq_hz = LCD_BACKLIGHT_LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "Configure LCD backlight PWM timer");

    const ledc_channel_config_t channel_config = {
        .gpio_num = M5STICKC_LCD_BL_GPIO,
        .speed_mode = LCD_BACKLIGHT_LEDC_MODE,
        .channel = LCD_BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LCD_BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&channel_config);
}
#endif

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
    ['L'] = { 0x7f, 0x01, 0x01, 0x01, 0x01 },
    ['M'] = { 0x7f, 0x20, 0x18, 0x20, 0x7f },
    ['N'] = { 0x7f, 0x20, 0x10, 0x08, 0x7f },
    ['O'] = { 0x3e, 0x41, 0x41, 0x41, 0x3e },
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

    for (int i = 0; i < s_lcd_width * s_lcd_height; ++i) {
        s_lcd_frame_buffer[i] = color;
    }
}

static void lcd_draw_pixel(int x, int y, uint16_t color)
{
    if (s_lcd_frame_buffer == NULL || x < 0 || x >= s_lcd_width || y < 0 || y >= s_lcd_height) {
        return;
    }

    s_lcd_frame_buffer[(y * s_lcd_width) + x] = color;
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
        s_lcd_panel, 0, 0, s_lcd_width, s_lcd_height, s_lcd_frame_buffer));
    xSemaphoreTake(s_lcd_flush_done, portMAX_DELAY);
}

static void format_mac(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

esp_err_t display_init(void)
{
#if M5STICKC_HAS_AXP192
    ESP_RETURN_ON_ERROR(init_display_power(), TAG, "Initialize LCD power");
#endif

    spi_bus_config_t bus_config = {
        .sclk_io_num = M5STICKC_LCD_SCLK_GPIO,
        .mosi_io_num = M5STICKC_LCD_MOSI_GPIO,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = s_lcd_width * s_lcd_height * (int)sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO), TAG, "Initialize LCD SPI bus");

    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = M5STICKC_LCD_CS_GPIO,
        .dc_gpio_num = M5STICKC_LCD_DC_GPIO,
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
        .reset_gpio_num = M5STICKC_LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(panel_io, &panel_config, &s_lcd_panel), TAG, "Create LCD panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_lcd_panel), TAG, "Reset LCD");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_lcd_panel), TAG, "Initialize LCD");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_lcd_panel, true), TAG, "Invert LCD color");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_lcd_panel, true), TAG, "Swap LCD axes");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_lcd_panel, false, true), TAG, "Mirror LCD");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_lcd_panel, LCD_GAP_X, LCD_GAP_Y), TAG, "Set LCD gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_lcd_panel, true), TAG, "Enable LCD");

#if M5STICKC_HAS_LCD_BL_GPIO
    ESP_RETURN_ON_ERROR(init_backlight_pwm(), TAG, "Initialize LCD backlight PWM");
#endif

    ESP_RETURN_ON_ERROR(display_set_brightness(LCD_DEFAULT_BRIGHTNESS_LEVEL), TAG, "Set LCD brightness");

    s_lcd_frame_buffer = heap_caps_malloc(
        s_lcd_width * s_lcd_height * sizeof(uint16_t), MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_lcd_frame_buffer != NULL, ESP_ERR_NO_MEM, TAG, "Allocate LCD frame buffer");

    lcd_fill_screen(LCD_BG_COLOR);
    lcd_flush();
    return ESP_OK;
}

esp_err_t display_set_brightness(uint8_t level)
{
    ESP_RETURN_ON_FALSE(level < LCD_BRIGHTNESS_LEVEL_COUNT,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "Brightness level must be between 0 and 5");

#if M5STICKC_HAS_AXP192
    ESP_RETURN_ON_FALSE(s_axp192 != NULL, ESP_ERR_INVALID_STATE, TAG, "Display power is not initialized");

    static const uint8_t axp192_lcd_brightness_table[LCD_BRIGHTNESS_LEVEL_COUNT] = {
        0,   // level 0: off
        7,   // level 1: minimum
        8,   // level 2: low
        9,   // level 3: normal
        10,  // level 4: bright
        12,  // level 5: maximum practical brightness
    };

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
#elif M5STICKC_HAS_LCD_BL_GPIO
    static const uint16_t pwm_lcd_brightness_table[LCD_BRIGHTNESS_LEVEL_COUNT] = {
        0,    // level 0: off
        40,   // level 1: minimum, about 8%
        64,   // level 2: low, about 13%
        102,  // level 3: normal, about 20%
        160,  // level 4: bright, about 31%
        256,  // level 5: maximum practical brightness, about 50%
    };

    const uint32_t duty = pwm_lcd_brightness_table[level];
    ESP_RETURN_ON_ERROR(ledc_set_duty(LCD_BACKLIGHT_LEDC_MODE, LCD_BACKLIGHT_LEDC_CHANNEL, duty),
                        TAG,
                        "Set LCD backlight PWM duty");
    return ledc_update_duty(LCD_BACKLIGHT_LEDC_MODE, LCD_BACKLIGHT_LEDC_CHANNEL);
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

void display_render_status(const display_status_t *status)
{
    if (status == NULL) {
        return;
    }

    lcd_fill_screen(LCD_BG_COLOR);

    char header_text[20];
    snprintf(header_text, sizeof(header_text), "USED %lu MAX %d",
             (unsigned long)status->used_count,
             DISPLAY_MAX_TRACKED_DEVICES);
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y, header_text, LCD_ACCENT_COLOR, LCD_BG_COLOR, LCD_TEXT_SCALE);

    char overflow_text[16];
    snprintf(overflow_text, sizeof(overflow_text), "OVER %lu", (unsigned long)status->ignored_count);
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 16, overflow_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    int row_y = LCD_MARGIN_Y + 28;
    for (int i = 0; i < DISPLAY_MAX_TRACKED_DEVICES; ++i) {
        char row_text[32];
        if (status->devices[i].in_use) {
            char mac_str[18];
            format_mac(status->devices[i].mac_addr, mac_str, sizeof(mac_str));
            snprintf(row_text, sizeof(row_text), "%d %c %s %lu",
                     i + 1,
                     status->devices[i].event_code,
                     mac_str,
                     (unsigned long)status->devices[i].rx_count);
        } else {
            snprintf(row_text, sizeof(row_text), "%d - --:--:--:--:--:-- 0", i + 1);
        }
        lcd_draw_text(LCD_MARGIN_X, row_y + (i * 14), row_text, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);
    }

    char self_mac_str[18];
    format_mac(status->self_mac, self_mac_str, sizeof(self_mac_str));
    lcd_draw_text(LCD_MARGIN_X, LCD_MARGIN_Y + 100, "SELF", LCD_ACCENT_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);
    lcd_draw_text(LCD_MARGIN_X + 30, LCD_MARGIN_Y + 100, self_mac_str, LCD_FG_COLOR, LCD_BG_COLOR, LCD_LIST_TEXT_SCALE);

    lcd_flush();
}
