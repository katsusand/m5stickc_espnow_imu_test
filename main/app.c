#include "app.h"

#include "board_pins.h"
#include "button.h"
#include "display.h"
#include "espnow.h"
#include "imu.h"
#include "led.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ESPNOW_WIFI_CHANNEL 1
#define BUTTON_POLL_INTERVAL_MS 20
#define BUTTON_STABLE_POLLS 2
#define POWER_HEARTBEAT_INTERVAL_MS 5000
#define POWER_HEARTBEAT_PULSE_US 1000
#define DEVICE_EXPIRATION_MS (5 * 60 * 1000)
#define DISPLAY_REFRESH_INTERVAL_MS 200
#define IMU_BROADCAST_INTERVAL_MS 10
#define IMU_LOG_INTERVAL_SAMPLES 100
#define PACKET_MAGIC 0xa5
#define PACKET_VERSION 1
#define PACKET_TYPE_IMU 2
#define LED_SOURCE_REMOTE_BUTTON (1U << 0)
#define LED_SOURCE_POWER_HEARTBEAT (1U << 1)
#define LCD_BRIGHTNESS_LEVEL_COUNT 6
#define LCD_DEFAULT_BRIGHTNESS_LEVEL 3

typedef enum {
    BUTTON_EVENT_DOWN = 1,
    BUTTON_EVENT_RELEASE = 2,
} button_event_t;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t event;
} button_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t reserved;
    uint32_t sequence;
    uint32_t timestamp_ms;
    int16_t accel_mg[3];
    int16_t gyro_centi_dps[3];
    int16_t temperature_centi_c;
} imu_packet_t;

_Static_assert(sizeof(imu_packet_t) == 26, "Unexpected IMU packet size");

typedef struct {
    bool in_use;
    uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN];
    button_event_t last_event;
    uint32_t rx_count;
    uint32_t last_seen_order;
    TickType_t last_seen_tick;
} tracked_device_t;

static const char *TAG = "app";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static tracked_device_t s_tracked_devices[DISPLAY_MAX_TRACKED_DEVICES] = { 0 };
static uint32_t s_rx_order = 0;
static uint32_t s_ignored_new_device_count = 0;
static uint8_t s_self_mac[ESPNOW_MAC_ADDR_LEN] = { 0 };
static bool s_imu_available = false;
static bool s_imu_broadcast_enabled = false;
static button_t s_button_a;
static button_t s_button_b;
static led_t s_red_led;
static uint8_t s_lcd_brightness_level = LCD_DEFAULT_BRIGHTNESS_LEVEL;

static const char *button_event_name(button_event_t event);
static char button_event_code(button_event_t event);
static uint8_t next_brightness_level(uint8_t current_level);
static void clear_tracked_device(tracked_device_t *device);
static void purge_stale_tracked_devices_locked(TickType_t now_tick);

static void imu_task(void *arg)
{
    (void)arg;

    TickType_t next_wake_tick = xTaskGetTickCount();
    uint32_t sequence = 0;

    while (true) {
        imu_sample_t sample;
        esp_err_t ret = imu_read_sample(&sample);
        if (ret == ESP_OK) {
            const imu_packet_t packet = {
                .magic = PACKET_MAGIC,
                .version = PACKET_VERSION,
                .type = PACKET_TYPE_IMU,
                .sequence = sequence,
                .timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                .accel_mg = {
                    (int16_t)lroundf(sample.accel_x_g * 1000.0f),
                    (int16_t)lroundf(sample.accel_y_g * 1000.0f),
                    (int16_t)lroundf(sample.accel_z_g * 1000.0f),
                },
                .gyro_centi_dps = {
                    (int16_t)lroundf(sample.gyro_x_dps * 100.0f),
                    (int16_t)lroundf(sample.gyro_y_dps * 100.0f),
                    (int16_t)lroundf(sample.gyro_z_dps * 100.0f),
                },
                .temperature_centi_c = (int16_t)lroundf(sample.temperature_c * 100.0f),
            };

            ret = espnow_send_broadcast(&packet, sizeof(packet));
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to broadcast IMU sample %lu: %s",
                         (unsigned long)sequence,
                         esp_err_to_name(ret));
            } else if ((sequence % IMU_LOG_INTERVAL_SAMPLES) == 0) {
                ESP_LOGI(TAG,
                         "Broadcast IMU sample %lu acc[g] %.3f %.3f %.3f gyro[dps] %.2f %.2f %.2f",
                         (unsigned long)sequence,
                         sample.accel_x_g,
                         sample.accel_y_g,
                         sample.accel_z_g,
                         sample.gyro_x_dps,
                         sample.gyro_y_dps,
                         sample.gyro_z_dps);
            }
            sequence++;
        } else {
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(ret));
        }

        xTaskDelayUntil(&next_wake_tick, pdMS_TO_TICKS(IMU_BROADCAST_INTERVAL_MS));
    }
}

static const char *button_event_name(button_event_t event)
{
    switch (event) {
    case BUTTON_EVENT_DOWN:
        return "down";
    case BUTTON_EVENT_RELEASE:
        return "release";
    default:
        return "unknown";
    }
}

static char button_event_code(button_event_t event)
{
    switch (event) {
    case BUTTON_EVENT_DOWN:
        return 'D';
    case BUTTON_EVENT_RELEASE:
        return 'R';
    default:
        return '-';
    }
}

static uint8_t next_brightness_level(uint8_t current_level)
{
    return (uint8_t)((current_level + 1) % LCD_BRIGHTNESS_LEVEL_COUNT);
}

static void clear_tracked_device(tracked_device_t *device)
{
    memset(device, 0, sizeof(*device));
}

static void purge_stale_tracked_devices_locked(TickType_t now_tick)
{
    const TickType_t expiration_ticks = pdMS_TO_TICKS(DEVICE_EXPIRATION_MS);

    for (int i = 0; i < DISPLAY_MAX_TRACKED_DEVICES; ++i) {
        if (!s_tracked_devices[i].in_use) {
            continue;
        }

        if ((TickType_t)(now_tick - s_tracked_devices[i].last_seen_tick) >= expiration_ticks) {
            clear_tracked_device(&s_tracked_devices[i]);
        }
    }
}

static int find_tracked_device_index(const uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN])
{
    for (int i = 0; i < DISPLAY_MAX_TRACKED_DEVICES; ++i) {
        if (s_tracked_devices[i].in_use &&
            memcmp(s_tracked_devices[i].mac_addr, mac_addr, ESPNOW_MAC_ADDR_LEN) == 0) {
            return i;
        }
    }

    return -1;
}

static int find_free_tracked_device_index(void)
{
    for (int i = 0; i < DISPLAY_MAX_TRACKED_DEVICES; ++i) {
        if (!s_tracked_devices[i].in_use) {
            return i;
        }
    }

    return -1;
}

static void fill_display_status(display_status_t *status)
{
    memset(status, 0, sizeof(*status));

    TickType_t now_tick = xTaskGetTickCount();
    portENTER_CRITICAL(&s_state_lock);
    purge_stale_tracked_devices_locked(now_tick);
    status->ignored_count = s_ignored_new_device_count;
    memcpy(status->self_mac, s_self_mac, sizeof(status->self_mac));

    for (int i = 0; i < DISPLAY_MAX_TRACKED_DEVICES; ++i) {
        const tracked_device_t *source = &s_tracked_devices[i];
        display_device_info_t *dest = &status->devices[i];
        dest->in_use = source->in_use;
        dest->event_code = button_event_code(source->last_event);
        dest->rx_count = source->rx_count;
        dest->last_seen_order = source->last_seen_order;
        memcpy(dest->mac_addr, source->mac_addr, sizeof(dest->mac_addr));
        if (source->in_use) {
            status->used_count++;
        }
    }
    portEXIT_CRITICAL(&s_state_lock);

    for (int i = 0; i < DISPLAY_MAX_TRACKED_DEVICES - 1; ++i) {
        for (int j = i + 1; j < DISPLAY_MAX_TRACKED_DEVICES; ++j) {
            if (status->devices[j].last_seen_order > status->devices[i].last_seen_order) {
                display_device_info_t temp = status->devices[i];
                status->devices[i] = status->devices[j];
                status->devices[j] = temp;
            }
        }
    }
}

static void on_espnow_recv(const uint8_t src_addr[ESPNOW_MAC_ADDR_LEN], const uint8_t *data, size_t data_len)
{
    if (data_len != sizeof(button_packet_t)) {
        ESP_LOGD(TAG,
                 "Ignored non-button packet from " MACSTR ": length=%u",
                 MAC2STR(src_addr),
                 (unsigned)data_len);
        return;
    }

    const button_packet_t *packet = (const button_packet_t *)data;
    if (packet->magic != PACKET_MAGIC || packet->version != PACKET_VERSION) {
        ESP_LOGD(TAG,
                 "Ignored unknown packet from " MACSTR ": magic=0x%02x version=%u",
                 MAC2STR(src_addr),
                 packet->magic,
                 packet->version);
        return;
    }

    switch ((button_event_t)packet->event) {
    case BUTTON_EVENT_DOWN:
        led_set_source(&s_red_led, LED_SOURCE_REMOTE_BUTTON, true);
        break;
    case BUTTON_EVENT_RELEASE:
        led_set_source(&s_red_led, LED_SOURCE_REMOTE_BUTTON, false);
        break;
    default:
        ESP_LOGW(TAG, "Ignored unknown button event: %u", packet->event);
        return;
    }

    bool ignored_new_device = false;
    TickType_t now_tick = xTaskGetTickCount();

    portENTER_CRITICAL(&s_state_lock);
    purge_stale_tracked_devices_locked(now_tick);
    int device_index = find_tracked_device_index(src_addr);
    if (device_index < 0) {
        device_index = find_free_tracked_device_index();
        if (device_index >= 0) {
            s_tracked_devices[device_index].in_use = true;
            memcpy(s_tracked_devices[device_index].mac_addr, src_addr, ESPNOW_MAC_ADDR_LEN);
            s_tracked_devices[device_index].rx_count = 0;
        } else {
            s_ignored_new_device_count++;
            ignored_new_device = true;
        }
    }

    if (!ignored_new_device) {
        s_rx_order++;
        s_tracked_devices[device_index].last_event = (button_event_t)packet->event;
        s_tracked_devices[device_index].rx_count++;
        s_tracked_devices[device_index].last_seen_order = s_rx_order;
        s_tracked_devices[device_index].last_seen_tick = now_tick;
    }
    portEXIT_CRITICAL(&s_state_lock);

    if (ignored_new_device) {
        ESP_LOGW(TAG, "Ignored new device " MACSTR " because tracked device limit is %d",
                 MAC2STR(src_addr),
                 DISPLAY_MAX_TRACKED_DEVICES);
    } else {
        ESP_LOGI(TAG, "Received button %s from " MACSTR,
                 button_event_name((button_event_t)packet->event),
                 MAC2STR(src_addr));
    }
}

static void on_espnow_send(const uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN], bool success)
{
    ESP_LOGD(TAG, "Send to " MACSTR ": %s",
             MAC2STR(mac_addr),
             success ? "success" : "fail");
}

static void send_button_event(button_event_t event)
{
    const button_packet_t packet = {
        .magic = PACKET_MAGIC,
        .version = PACKET_VERSION,
        .event = event,
    };

    esp_err_t ret = espnow_send_broadcast(&packet, sizeof(packet));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Broadcast button %s", button_event_name(event));
    } else {
        ESP_LOGE(TAG, "Failed to broadcast button %s: %s", button_event_name(event), esp_err_to_name(ret));
    }
}

static esp_err_t init_board_gpio(void)
{
#if M5STICKC_HAS_HOLD_GPIO
    gpio_reset_pin(M5STICKC_HOLD_GPIO);
    gpio_set_direction(M5STICKC_HOLD_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(M5STICKC_HOLD_GPIO, 1);
    ESP_LOGI(TAG, "Set HOLD GPIO%d high", M5STICKC_HOLD_GPIO);
#endif

    const gpio_config_t imu_tx_select_config = {
        .pin_bit_mask = 1ULL << M5STICKC_IMU_TX_SELECT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&imu_tx_select_config), TAG, "Configure IMU TX select GPIO");
    s_imu_broadcast_enabled = gpio_get_level(M5STICKC_IMU_TX_SELECT_GPIO) == 0;
    ESP_LOGI(TAG,
             "IMU broadcast select GPIO%d=%s: 100 Hz broadcast %s",
             M5STICKC_IMU_TX_SELECT_GPIO,
             s_imu_broadcast_enabled ? "LOW" : "HIGH",
             s_imu_broadcast_enabled ? "enabled" : "disabled");

    led_config_t red_led_config = {
        .gpio_num = M5STICKC_RED_LED_GPIO,
        .active_level = M5STICKC_RED_LED_ACTIVE_LEVEL,
    };
    ESP_RETURN_ON_ERROR(led_init(&s_red_led, &red_led_config), TAG, "Initialize red LED");

    button_config_t button_a_config = {
        .gpio_num = M5STICKC_BUTTON_A_GPIO,
        .active_low = true,
        .poll_interval_ms = BUTTON_POLL_INTERVAL_MS,
        .stable_polls = BUTTON_STABLE_POLLS,
    };
    button_init(&s_button_a, &button_a_config);

    button_config_t button_b_config = {
        .gpio_num = M5STICKC_BUTTON_B_GPIO,
        .active_low = true,
        .poll_interval_ms = BUTTON_POLL_INTERVAL_MS,
        .stable_polls = BUTTON_STABLE_POLLS,
    };
    button_init(&s_button_b, &button_b_config);
    return ESP_OK;
}

static void power_heartbeat_task(void *arg)
{
    (void)arg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(POWER_HEARTBEAT_INTERVAL_MS));

        led_set_source(&s_red_led, LED_SOURCE_POWER_HEARTBEAT, true);
        esp_rom_delay_us(POWER_HEARTBEAT_PULSE_US);
        led_set_source(&s_red_led, LED_SOURCE_POWER_HEARTBEAT, false);
    }
}

static void display_task(void *arg)
{
    (void)arg;

    display_status_t previous_status = { 0 };
    bool first_frame = true;

    while (true) {
        display_status_t current_status;
        fill_display_status(&current_status);

        if (first_frame || memcmp(&current_status, &previous_status, sizeof(current_status)) != 0) {
            display_render_status(&current_status);
            previous_status = current_status;
            first_frame = false;
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_INTERVAL_MS));
    }
}

static void app_task(void *arg)
{
    (void)arg;

    while (true) {
        bool button_a_state_changed = false;
        bool button_a_pressed = button_poll(&s_button_a, &button_a_state_changed);
        if (button_a_state_changed) {
            send_button_event(button_a_pressed ? BUTTON_EVENT_DOWN : BUTTON_EVENT_RELEASE);
        }

        bool button_b_state_changed = false;
        bool button_b_pressed = button_poll(&s_button_b, &button_b_state_changed);
        if (button_b_state_changed && button_b_pressed) {
            uint8_t next_level = next_brightness_level(s_lcd_brightness_level);
            esp_err_t ret = display_set_brightness(next_level);
            if (ret == ESP_OK) {
                s_lcd_brightness_level = next_level;
                ESP_LOGI(TAG, "LCD brightness level -> %u", s_lcd_brightness_level);
            } else {
                ESP_LOGW(TAG, "Failed to set LCD brightness level %u: %s",
                         next_level,
                         esp_err_to_name(ret));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(s_button_a.config.poll_interval_ms));
    }
}

esp_err_t app_start(void)
{
    ESP_LOGI(TAG, "Starting %s ESP-NOW button demo", M5STICKC_BOARD_NAME);
    ESP_LOGI(TAG, "LCD SPI: MOSI=%d CLK=%d CS=%d DC=%d RST=%d BL=%d",
             M5STICKC_LCD_MOSI_GPIO,
             M5STICKC_LCD_SCLK_GPIO,
             M5STICKC_LCD_CS_GPIO,
             M5STICKC_LCD_DC_GPIO,
             M5STICKC_LCD_RST_GPIO,
             M5STICKC_LCD_BL_GPIO);
    ESP_LOGI(TAG, "Buttons: A=%d B=%d C=%d", M5STICKC_BUTTON_A_GPIO, M5STICKC_BUTTON_B_GPIO, M5STICKC_BUTTON_C_GPIO);
    ESP_LOGI(TAG, "Red LED: GPIO%d (active %s), ESP-NOW channel: %d",
             M5STICKC_RED_LED_GPIO,
             M5STICKC_RED_LED_ACTIVE_LEVEL ? "high" : "low",
             ESPNOW_WIFI_CHANNEL);

    ESP_RETURN_ON_ERROR(init_board_gpio(), TAG, "Initialize board GPIO");
    ESP_RETURN_ON_ERROR(display_init(), TAG, "Initialize display");
    const esp_err_t imu_err = imu_init();
    if (imu_err == ESP_OK) {
        s_imu_available = true;
    } else {
        ESP_LOGW(TAG, "IMU initialization failed (%s); continuing without IMU", esp_err_to_name(imu_err));
    }
    const espnow_config_t espnow_config = {
        .wifi_channel = ESPNOW_WIFI_CHANNEL,
        .receive_callback = on_espnow_recv,
        .send_callback = on_espnow_send,
    };
    ESP_RETURN_ON_ERROR(espnow_init(&espnow_config), TAG, "Initialize ESP-NOW");
    ESP_RETURN_ON_ERROR(espnow_get_self_mac(s_self_mac), TAG, "Read station MAC");

    xTaskCreate(power_heartbeat_task, "power_heartbeat", 2048, NULL, 1, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 1, NULL);
    if (s_imu_available && s_imu_broadcast_enabled) {
        xTaskCreate(imu_task, "imu_task", 4096, NULL, 1, NULL);
    }
    xTaskCreate(app_task, "app_task", 4096, NULL, 1, NULL);

    return ESP_OK;
}
