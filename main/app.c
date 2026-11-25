#include "app.h"

#include "board.h"
#include "button.h"
#include "display.h"
#include "espnow.h"
#include "gps.h"
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

#define ESPNOW_WIFI_CHANNEL 13
#define BUTTON_POLL_INTERVAL_MS 20
#define BUTTON_STABLE_POLLS 2
#define POWER_HEARTBEAT_INTERVAL_MS 5000
#define POWER_HEARTBEAT_PULSE_US 1000
#define DEVICE_EXPIRATION_MS (5 * 60 * 1000)
#define DISPLAY_REFRESH_INTERVAL_MS 500
#define IMU_BROADCAST_INTERVAL_MS 10
#define IMU_LOG_INTERVAL_SAMPLES 100
#define IMU_BATCH_SAMPLE_COUNT 10
#define GPS_READ_TIMEOUT_MS 200
#define GPS_LOG_INTERVAL_MS 5000
#define IMU_DISPLAY_STALE_MS 1000
#define ESPNOW_BUTTON_MAGIC 0xA5
#define ESPNOW_SENSOR_MAGIC 0xA55A
#define ESPNOW_PAYLOAD_VERSION 2
#define ESPNOW_TYPED_IMU_TYPE 1
#define ESPNOW_TYPED_IMU_BATCH_TYPE 4
#define ESPNOW_TYPED_GPS_TYPE 2
#define ESPNOW_GPS_TIME_TYPE 3
#define ESPNOW_IMU_CONTROL_TYPE 5
#define ESPNOW_IMU_CONTROL_START 1
#define ESPNOW_IMU_CONTROL_STOP 2
#define LED_SOURCE_REMOTE_BUTTON (1U << 0)
#define LED_SOURCE_POWER_HEARTBEAT (1U << 1)
#define LCD_BRIGHTNESS_LEVEL_COUNT 6
#define LCD_DEFAULT_BRIGHTNESS_LEVEL 3

typedef enum {
    BUTTON_EVENT_DOWN = 1,
    BUTTON_EVENT_RELEASE = 2,
} button_event_t;

/* Version 2 wire formats consumed by espnow_logger. */
typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t event;
} button_packet_v1_t;

typedef struct __attribute__((packed)) {
    int16_t accel_mg[3];
    int16_t gyro_centi_dps[3];
} imu_raw_sample_v1_t;

typedef struct __attribute__((packed)) {
    int16_t accel_mg_avg[3];
    int16_t accel_mg_min[3];
    int16_t accel_mg_max[3];
    int16_t gyro_centi_dps_avg[3];
    int16_t gyro_centi_dps_min[3];
    int16_t gyro_centi_dps_max[3];
} imu_batch_summary_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    uint8_t sample_count;
    uint8_t sample_period_ms;
    uint16_t flags;
    int16_t temperature_centi_c;
    uint64_t gps_utc_ms;
    imu_batch_summary_v1_t summary;
    imu_raw_sample_v1_t raw[IMU_BATCH_SAMPLE_COUNT];
} sensor_imu_batch_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    int16_t accel_mg[3];
    int16_t gyro_centi_dps[3];
    int16_t temperature_centi_c;
    uint16_t flags;
} sensor_imu_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    uint32_t speed_mmps;
    uint16_t course_cdeg;
    uint16_t hdop_centi;
    uint8_t satellites;
    uint8_t fix_type;
    uint16_t flags;
    uint64_t gps_utc_ms;
} sensor_gps_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
} sensor_utc_time_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t command;
} imu_control_packet_v1_t;

_Static_assert(sizeof(button_packet_v1_t) == 3, "Button packet must be 3 bytes");
_Static_assert(sizeof(imu_raw_sample_v1_t) == 12, "IMU raw sample must be 12 bytes");
_Static_assert(sizeof(imu_batch_summary_v1_t) == 36, "IMU batch summary must be 36 bytes");
_Static_assert(sizeof(sensor_imu_batch_packet_v1_t) == 178, "IMU batch packet must be 178 bytes");
_Static_assert(sizeof(sensor_imu_packet_v1_t) == 24, "IMU packet must be 24 bytes");
_Static_assert(sizeof(sensor_gps_packet_v1_t) == 40, "GPS packet must be 40 bytes");
_Static_assert(sizeof(sensor_utc_time_packet_v1_t) == 11, "GPS time packet must be 11 bytes");
_Static_assert(sizeof(imu_control_packet_v1_t) == 5, "IMU control packet must be 5 bytes");

typedef struct {
    bool in_use;
    uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN];
    button_event_t last_event;
    uint32_t rx_count;
    uint32_t last_seen_order;
    TickType_t last_seen_tick;
} tracked_device_t;

typedef struct {
    bool valid;
    int16_t accel_mg_avg[3];
    int16_t gyro_centi_dps_avg[3];
    int16_t temperature_centi_c;
    TickType_t last_update_tick;
} imu_display_summary_t;

static const char *TAG = "app";
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static tracked_device_t s_tracked_devices[DISPLAY_MAX_TRACKED_DEVICES] = { 0 };
static uint32_t s_rx_order = 0;
static uint32_t s_ignored_new_device_count = 0;
static uint32_t s_status_led_active_sources = 0;
static uint8_t s_self_mac[ESPNOW_MAC_ADDR_LEN] = { 0 };
static bool s_imu_available = false;
static bool s_imu_streaming_enabled = false;
static bool s_gps_available = false;
static bool s_button_a_available = false;
static bool s_button_b_available = false;
static bool s_red_led_available = false;
static bool s_lcd_activity_requested = false;
static bool s_lcd_zero_pulse_active = false;
static TickType_t s_lcd_zero_next_pulse_tick = 0;
static TickType_t s_lcd_zero_pulse_end_tick = 0;
static button_t s_button_a;
static button_t s_button_b;
static led_t s_red_led;
static uint8_t s_lcd_brightness_level = LCD_DEFAULT_BRIGHTNESS_LEVEL;
static uint8_t s_imu_stream_dest_mac[ESPNOW_MAC_ADDR_LEN] = { 0 };
static imu_display_summary_t s_latest_imu_summary = { 0 };

static const char *button_event_name(button_event_t event);
static char button_event_code(button_event_t event);
static uint8_t next_brightness_level(uint8_t current_level);
static void service_lcd_zero_level_mode(void);
static void reset_lcd_zero_level_mode(void);
static void clear_tracked_device(tracked_device_t *device);
static void purge_stale_tracked_devices_locked(TickType_t now_tick);
static void imu_sample_to_raw(const imu_sample_t *sample, imu_raw_sample_v1_t *out_sample);
static void imu_build_batch_summary(const imu_raw_sample_v1_t *samples,
                                    size_t sample_count,
                                    imu_batch_summary_v1_t *out_summary);
static bool imu_get_stream_destination(uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN]);
static void imu_set_stream_destination(const uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN]);
static bool imu_clear_stream_destination_if_matches(const uint8_t src_addr[ESPNOW_MAC_ADDR_LEN]);
static const char *imu_control_command_name(uint8_t command);
static void update_imu_display_summary(const imu_batch_summary_v1_t *summary, int16_t temperature_centi_c);

static void imu_task(void *arg)
{
    (void)arg;

    TickType_t next_wake_tick = xTaskGetTickCount();
    uint32_t sequence = 0;
    imu_raw_sample_v1_t batch_samples[IMU_BATCH_SAMPLE_COUNT];
    uint8_t batch_sample_count = 0;
    uint32_t batch_sequence_start = 0;
    int16_t batch_temperature_centi_c = 0;
    uint64_t batch_gps_utc_ms = 0;

    while (true) {
        imu_sample_t sample;
        esp_err_t ret = imu_read_sample(&sample);
        if (ret == ESP_OK) {
            imu_raw_sample_v1_t raw_sample;
            imu_sample_to_raw(&sample, &raw_sample);
            if (batch_sample_count == 0) {
                batch_sequence_start = sequence;
                batch_temperature_centi_c = (int16_t)lroundf(sample.temperature_c * 100.0f);
                gps_status_t gps_status;
                gps_get_status(&gps_status);
                batch_gps_utc_ms = gps_status.gps_utc_ms;
            }
            batch_samples[batch_sample_count] = raw_sample;
            batch_sample_count++;
            sequence++;
            if (batch_sample_count >= IMU_BATCH_SAMPLE_COUNT) {
                imu_batch_summary_v1_t batch_summary;
                imu_build_batch_summary(batch_samples, IMU_BATCH_SAMPLE_COUNT, &batch_summary);
                update_imu_display_summary(&batch_summary, batch_temperature_centi_c);

                uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN];
                if (!imu_get_stream_destination(dest_addr)) {
                    batch_sample_count = 0;
                    continue;
                }

                const sensor_imu_batch_packet_v1_t packet = {
                    .magic = ESPNOW_SENSOR_MAGIC,
                    .version = ESPNOW_PAYLOAD_VERSION,
                    .type = ESPNOW_TYPED_IMU_BATCH_TYPE,
                    .sequence = batch_sequence_start,
                    .sample_count = IMU_BATCH_SAMPLE_COUNT,
                    .sample_period_ms = IMU_BROADCAST_INTERVAL_MS,
                    .flags = 0,
                    .temperature_centi_c = batch_temperature_centi_c,
                    .gps_utc_ms = batch_gps_utc_ms,
                    .summary = batch_summary,
                };

                sensor_imu_batch_packet_v1_t send_packet = packet;
                memcpy(send_packet.raw, batch_samples, sizeof(send_packet.raw));
                ret = espnow_send(dest_addr, &send_packet, sizeof(send_packet));
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG,
                             "Failed to send IMU batch seq=%lu count=%u to " MACSTR ": %s",
                             (unsigned long)send_packet.sequence,
                             send_packet.sample_count,
                             MAC2STR(dest_addr),
                             esp_err_to_name(ret));
                } else if ((send_packet.sequence % IMU_LOG_INTERVAL_SAMPLES) == 0) {
                    ESP_LOGI(TAG,
                             "Send IMU batch seq=%lu count=%u to " MACSTR " temp=%.2fC gps_utc_ms=%llu avg[mg] %d %d %d avg[cdeg/s] %d %d %d",
                             (unsigned long)send_packet.sequence,
                             send_packet.sample_count,
                             MAC2STR(dest_addr),
                             (double)send_packet.temperature_centi_c / 100.0,
                             (unsigned long long)send_packet.gps_utc_ms,
                             send_packet.summary.accel_mg_avg[0],
                             send_packet.summary.accel_mg_avg[1],
                             send_packet.summary.accel_mg_avg[2],
                             send_packet.summary.gyro_centi_dps_avg[0],
                             send_packet.summary.gyro_centi_dps_avg[1],
                             send_packet.summary.gyro_centi_dps_avg[2]);
                }
                batch_sample_count = 0;
            }
        } else {
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(ret));
        }

        xTaskDelayUntil(&next_wake_tick, pdMS_TO_TICKS(IMU_BROADCAST_INTERVAL_MS));
    }
}

static void update_imu_display_summary(const imu_batch_summary_v1_t *summary, int16_t temperature_centi_c)
{
    if (summary == NULL) {
        return;
    }

    TickType_t now_tick = xTaskGetTickCount();
    portENTER_CRITICAL(&s_state_lock);
    memcpy(s_latest_imu_summary.accel_mg_avg, summary->accel_mg_avg, sizeof(s_latest_imu_summary.accel_mg_avg));
    memcpy(s_latest_imu_summary.gyro_centi_dps_avg,
           summary->gyro_centi_dps_avg,
           sizeof(s_latest_imu_summary.gyro_centi_dps_avg));
    s_latest_imu_summary.temperature_centi_c = temperature_centi_c;
    s_latest_imu_summary.last_update_tick = now_tick;
    s_latest_imu_summary.valid = true;
    portEXIT_CRITICAL(&s_state_lock);
}

static void imu_sample_to_raw(const imu_sample_t *sample, imu_raw_sample_v1_t *out_sample)
{
    if (sample == NULL || out_sample == NULL) {
        return;
    }

    out_sample->accel_mg[0] = (int16_t)lroundf(sample->accel_x_g * 1000.0f);
    out_sample->accel_mg[1] = (int16_t)lroundf(sample->accel_y_g * 1000.0f);
    out_sample->accel_mg[2] = (int16_t)lroundf(sample->accel_z_g * 1000.0f);
    out_sample->gyro_centi_dps[0] = (int16_t)lroundf(sample->gyro_x_dps * 100.0f);
    out_sample->gyro_centi_dps[1] = (int16_t)lroundf(sample->gyro_y_dps * 100.0f);
    out_sample->gyro_centi_dps[2] = (int16_t)lroundf(sample->gyro_z_dps * 100.0f);
}

static void imu_build_batch_summary(const imu_raw_sample_v1_t *samples,
                                    size_t sample_count,
                                    imu_batch_summary_v1_t *out_summary)
{
    if (samples == NULL || out_summary == NULL || sample_count == 0) {
        return;
    }

    memset(out_summary, 0, sizeof(*out_summary));

    int32_t accel_sum[3] = { 0, 0, 0 };
    int32_t gyro_sum[3] = { 0, 0, 0 };

    for (size_t i = 0; i < sample_count; ++i) {
        const imu_raw_sample_v1_t *sample = &samples[i];
        for (size_t axis = 0; axis < 3; ++axis) {
            const int16_t accel = sample->accel_mg[axis];
            const int16_t gyro = sample->gyro_centi_dps[axis];
            if (i == 0) {
                out_summary->accel_mg_min[axis] = accel;
                out_summary->accel_mg_max[axis] = accel;
                out_summary->gyro_centi_dps_min[axis] = gyro;
                out_summary->gyro_centi_dps_max[axis] = gyro;
            } else {
                if (accel < out_summary->accel_mg_min[axis]) {
                    out_summary->accel_mg_min[axis] = accel;
                }
                if (accel > out_summary->accel_mg_max[axis]) {
                    out_summary->accel_mg_max[axis] = accel;
                }
                if (gyro < out_summary->gyro_centi_dps_min[axis]) {
                    out_summary->gyro_centi_dps_min[axis] = gyro;
                }
                if (gyro > out_summary->gyro_centi_dps_max[axis]) {
                    out_summary->gyro_centi_dps_max[axis] = gyro;
                }
            }
            accel_sum[axis] += accel;
            gyro_sum[axis] += gyro;
        }
    }

    for (size_t axis = 0; axis < 3; ++axis) {
        out_summary->accel_mg_avg[axis] = (int16_t)(accel_sum[axis] / (int32_t)sample_count);
        out_summary->gyro_centi_dps_avg[axis] = (int16_t)(gyro_sum[axis] / (int32_t)sample_count);
    }
}

static void gps_task(void *arg)
{
    (void)arg;

    bool previous_receiving = false;
    bool previous_fix_valid = false;
    bool previous_time_valid = false;
    uint16_t previous_utc_year = 0;
    uint8_t previous_utc_month = 0;
    uint8_t previous_utc_day = 0;
    uint8_t previous_utc_hour = 0;
    uint8_t previous_utc_minute = 0;
    uint8_t previous_utc_second = 0;
    int last_time_broadcast_slot = -1;
    bool previous_gps_packet_valid = false;
    sensor_gps_packet_v1_t previous_gps_packet = { 0 };
    uint32_t gps_sequence = 0;
    TickType_t next_log_tick = 0;

    while (true) {
        gps_process(GPS_READ_TIMEOUT_MS);

        gps_status_t status;
        gps_get_status(&status);
        const TickType_t now = xTaskGetTickCount();
        const bool time_changed =
            status.time_valid &&
            (!previous_time_valid ||
             status.utc_year != previous_utc_year ||
             status.utc_month != previous_utc_month ||
             status.utc_day != previous_utc_day ||
             status.utc_hour != previous_utc_hour ||
             status.utc_minute != previous_utc_minute ||
             status.utc_second != previous_utc_second);
        const int current_time_slot = status.utc_second / 20;
        const bool aligned_broadcast_time =
            status.time_valid &&
            (status.utc_second % 20 == 0);
        if (aligned_broadcast_time &&
            (time_changed || current_time_slot != last_time_broadcast_slot)) {
            const sensor_utc_time_packet_v1_t packet = {
                .magic = ESPNOW_SENSOR_MAGIC,
                .version = ESPNOW_PAYLOAD_VERSION,
                .type = ESPNOW_GPS_TIME_TYPE,
                .utc_year = status.utc_year,
                .utc_month = status.utc_month,
                .utc_day = status.utc_day,
                .utc_hour = status.utc_hour,
                .utc_minute = status.utc_minute,
                .utc_second = status.utc_second,
            };

            const esp_err_t ret = espnow_send_broadcast(&packet, sizeof(packet));
            if (ret != ESP_OK) {
                ESP_LOGW(TAG,
                         "Failed to broadcast GPS time %04u-%02u-%02uT%02u:%02u:%02uZ: %s",
                         packet.utc_year,
                         packet.utc_month,
                         packet.utc_day,
                         packet.utc_hour,
                         packet.utc_minute,
                         packet.utc_second,
                         esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG,
                         "Broadcast GPS time %04u-%02u-%02uT%02u:%02u:%02uZ",
                         packet.utc_year,
                         packet.utc_month,
                         packet.utc_day,
                         packet.utc_hour,
                         packet.utc_minute,
                         packet.utc_second);
            }
            previous_time_valid = true;
            previous_utc_year = status.utc_year;
            previous_utc_month = status.utc_month;
            previous_utc_day = status.utc_day;
            previous_utc_hour = status.utc_hour;
            previous_utc_minute = status.utc_minute;
            previous_utc_second = status.utc_second;
            last_time_broadcast_slot = current_time_slot;
        } else if (!status.time_valid) {
            previous_time_valid = false;
            last_time_broadcast_slot = -1;
        }

        const bool gps_packet_ready = status.fix_valid && status.time_valid && status.gps_utc_ms != 0;
        if (gps_packet_ready) {
            const sensor_gps_packet_v1_t packet = {
                .magic = ESPNOW_SENSOR_MAGIC,
                .version = ESPNOW_PAYLOAD_VERSION,
                .type = ESPNOW_TYPED_GPS_TYPE,
                .sequence = 0,
                .latitude_e7 = (int32_t)lround(status.latitude_deg * 10000000.0),
                .longitude_e7 = (int32_t)lround(status.longitude_deg * 10000000.0),
                .altitude_mm = (int32_t)lroundf(status.altitude_m * 1000.0f),
                .speed_mmps = (uint32_t)lroundf(status.speed_knots * 514.444f),
                .course_cdeg = (uint16_t)lroundf(status.course_deg * 100.0f),
                .hdop_centi = (uint16_t)lroundf(status.hdop * 100.0f),
                .satellites = status.satellites,
                .fix_type = status.fix_type,
                .flags = 0,
                .gps_utc_ms = status.gps_utc_ms,
            };

            if (!previous_gps_packet_valid || memcmp(&packet, &previous_gps_packet, sizeof(packet)) != 0) {
                sensor_gps_packet_v1_t send_packet = packet;
                send_packet.sequence = gps_sequence;
                const esp_err_t ret = espnow_send_broadcast(&send_packet, sizeof(send_packet));
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG,
                             "Failed to broadcast GPS fix seq=%lu lat=%.6f lon=%.6f: %s",
                             (unsigned long)send_packet.sequence,
                             status.latitude_deg,
                             status.longitude_deg,
                             esp_err_to_name(ret));
                } else {
                    ESP_LOGI(TAG,
                             "Broadcast GPS fix seq=%lu lat=%.6f lon=%.6f alt=%.1fm speed=%.2fkn course=%.1f hdop=%.2f time=%04u-%02u-%02uT%02u:%02u:%02uZ",
                             (unsigned long)send_packet.sequence,
                             status.latitude_deg,
                             status.longitude_deg,
                             status.altitude_m,
                             status.speed_knots,
                             status.course_deg,
                             status.hdop,
                             status.utc_year,
                             status.utc_month,
                             status.utc_day,
                             status.utc_hour,
                             status.utc_minute,
                             status.utc_second);
                }
                previous_gps_packet = packet;
                previous_gps_packet_valid = true;
                gps_sequence++;
            }
        } else {
            previous_gps_packet_valid = false;
        }

        const bool state_changed =
            status.receiving != previous_receiving ||
            status.fix_valid != previous_fix_valid;
        if (state_changed || (int32_t)(now - next_log_tick) >= 0) {
            if (!status.receiving) {
                ESP_LOGI(TAG,
                         "GPS: waiting for NMEA data (bytes=%lu dropped=%lu time=%s)",
                         (unsigned long)status.bytes_received,
                         (unsigned long)status.bytes_dropped,
                         status.time_valid ? "yes" : "no");
            } else if (!status.fix_valid) {
                ESP_LOGI(TAG,
                         "GPS: NMEA received, waiting for fix (satellites=%u sentences=%lu checksum_errors=%lu bytes=%lu dropped=%lu time=%s)",
                         status.satellites,
                         (unsigned long)status.sentence_count,
                         (unsigned long)status.checksum_error_count,
                         (unsigned long)status.bytes_received,
                         (unsigned long)status.bytes_dropped,
                         status.time_valid ? "yes" : "no");
            } else {
                ESP_LOGI(TAG,
                         "GPS: fix lat=%.6f lon=%.6f alt=%.1fm satellites=%u bytes=%lu dropped=%lu time=%04u-%02u-%02uT%02u:%02u:%02uZ",
                         status.latitude_deg,
                         status.longitude_deg,
                         status.altitude_m,
                         status.satellites,
                         (unsigned long)status.bytes_received,
                         (unsigned long)status.bytes_dropped,
                         status.utc_year,
                         status.utc_month,
                         status.utc_day,
                         status.utc_hour,
                         status.utc_minute,
                         status.utc_second);
            }
            previous_receiving = status.receiving;
            previous_fix_valid = status.fix_valid;
            next_log_tick = now + pdMS_TO_TICKS(GPS_LOG_INTERVAL_MS);
        }
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
    const uint8_t minimum_level = board_get_config()->lcd_min_brightness_level;
    const uint8_t next_level = current_level + 1;
    return next_level < LCD_BRIGHTNESS_LEVEL_COUNT ? next_level : minimum_level;
}

static bool imu_get_stream_destination(uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN])
{
    bool enabled = false;

    portENTER_CRITICAL(&s_state_lock);
    enabled = s_imu_streaming_enabled;
    if (enabled && dest_addr != NULL) {
        memcpy(dest_addr, s_imu_stream_dest_mac, ESPNOW_MAC_ADDR_LEN);
    }
    portEXIT_CRITICAL(&s_state_lock);

    return enabled;
}

static void imu_set_stream_destination(const uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN])
{
    if (dest_addr == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_state_lock);
    memcpy(s_imu_stream_dest_mac, dest_addr, ESPNOW_MAC_ADDR_LEN);
    s_imu_streaming_enabled = true;
    portEXIT_CRITICAL(&s_state_lock);
}

static bool imu_clear_stream_destination_if_matches(const uint8_t src_addr[ESPNOW_MAC_ADDR_LEN])
{
    bool cleared = false;

    if (src_addr == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_state_lock);
    if (s_imu_streaming_enabled &&
        memcmp(s_imu_stream_dest_mac, src_addr, ESPNOW_MAC_ADDR_LEN) == 0) {
        memset(s_imu_stream_dest_mac, 0, sizeof(s_imu_stream_dest_mac));
        s_imu_streaming_enabled = false;
        cleared = true;
    }
    portEXIT_CRITICAL(&s_state_lock);

    return cleared;
}

static const char *imu_control_command_name(uint8_t command)
{
    switch (command) {
    case ESPNOW_IMU_CONTROL_START:
        return "start";
    case ESPNOW_IMU_CONTROL_STOP:
        return "stop";
    default:
        return "unknown";
    }
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

    if (s_gps_available) {
        gps_status_t gps_status;
        gps_get_status(&gps_status);
        status->gps_receiving = gps_status.receiving;
        status->gps_fix_valid = gps_status.receiving && gps_status.fix_valid;
        status->gps_time_valid = gps_status.receiving && gps_status.time_valid;
        status->gps_satellites = gps_status.satellites;
        status->gps_latitude_deg = gps_status.latitude_deg;
        status->gps_longitude_deg = gps_status.longitude_deg;
        status->gps_speed_knots = gps_status.speed_knots;
        status->gps_course_deg = gps_status.course_deg;
        status->gps_utc_year = gps_status.utc_year;
        status->gps_utc_month = gps_status.utc_month;
        status->gps_utc_day = gps_status.utc_day;
        status->gps_utc_hour = gps_status.utc_hour;
        status->gps_utc_minute = gps_status.utc_minute;
        status->gps_utc_second = gps_status.utc_second;
        status->gps_last_sentence_age_ms = gps_status.last_sentence_age_ms;
    }

    TickType_t now_tick = xTaskGetTickCount();
    portENTER_CRITICAL(&s_state_lock);
    purge_stale_tracked_devices_locked(now_tick);
    status->status_led_visible = board_get_config()->has_display_status_led;
    status->status_led_on = s_status_led_active_sources != 0;
    status->imu_available = s_imu_available;
    if (s_latest_imu_summary.valid) {
        memcpy(status->imu_accel_mg_avg,
               s_latest_imu_summary.accel_mg_avg,
               sizeof(status->imu_accel_mg_avg));
        memcpy(status->imu_gyro_centi_dps_avg,
               s_latest_imu_summary.gyro_centi_dps_avg,
               sizeof(status->imu_gyro_centi_dps_avg));
        status->imu_temperature_centi_c = s_latest_imu_summary.temperature_centi_c;
        status->imu_data_age_ms = (uint32_t)pdTICKS_TO_MS(now_tick - s_latest_imu_summary.last_update_tick);
        status->imu_recent = status->imu_data_age_ms <= IMU_DISPLAY_STALE_MS;
    }
    memcpy(status->self_mac, s_self_mac, sizeof(status->self_mac));
    portEXIT_CRITICAL(&s_state_lock);
}

static void set_status_led_source(uint32_t source_mask, bool active)
{
    portENTER_CRITICAL(&s_state_lock);
    if (active) {
        s_status_led_active_sources |= source_mask;
    } else {
        s_status_led_active_sources &= ~source_mask;
    }
    portEXIT_CRITICAL(&s_state_lock);

    if (s_red_led_available) {
        led_set_source(&s_red_led, source_mask, active);
    }
}

static void on_espnow_recv(const uint8_t src_addr[ESPNOW_MAC_ADDR_LEN], const uint8_t *data, size_t data_len)
{
    if (data_len == sizeof(imu_control_packet_v1_t)) {
        const imu_control_packet_v1_t *packet = (const imu_control_packet_v1_t *)data;
        if (packet->magic == ESPNOW_SENSOR_MAGIC &&
            packet->version == ESPNOW_PAYLOAD_VERSION &&
            packet->type == ESPNOW_IMU_CONTROL_TYPE) {
            if (!s_imu_available) {
                ESP_LOGW(TAG, "Ignored IMU control %s from " MACSTR " because IMU is unavailable",
                         imu_control_command_name(packet->command),
                         MAC2STR(src_addr));
                return;
            }

            if (packet->command == ESPNOW_IMU_CONTROL_START) {
                imu_set_stream_destination(src_addr);
                ESP_LOGI(TAG, "IMU stream start requested by " MACSTR, MAC2STR(src_addr));
            } else if (packet->command == ESPNOW_IMU_CONTROL_STOP) {
                const bool stopped = imu_clear_stream_destination_if_matches(src_addr);
                ESP_LOGI(TAG,
                         "IMU stream stop requested by " MACSTR "%s",
                         MAC2STR(src_addr),
                         stopped ? "" : " (not active sender)");
            } else {
                ESP_LOGW(TAG, "Ignored unknown IMU control command %u from " MACSTR,
                         packet->command,
                         MAC2STR(src_addr));
            }
            return;
        }
    }

    if (data_len != sizeof(button_packet_v1_t)) {
        ESP_LOGD(TAG,
                 "Ignored non-button packet from " MACSTR ": length=%u",
                 MAC2STR(src_addr),
                 (unsigned)data_len);
        return;
    }

    const button_packet_v1_t *packet = (const button_packet_v1_t *)data;
    if (packet->magic != ESPNOW_BUTTON_MAGIC || packet->version != ESPNOW_PAYLOAD_VERSION) {
        ESP_LOGD(TAG,
                 "Ignored unknown packet from " MACSTR ": magic=0x%02x version=%u",
                 MAC2STR(src_addr),
                 packet->magic,
                 packet->version);
        return;
    }

    switch ((button_event_t)packet->event) {
    case BUTTON_EVENT_DOWN:
        set_status_led_source(LED_SOURCE_REMOTE_BUTTON, true);
        break;
    case BUTTON_EVENT_RELEASE:
        set_status_led_source(LED_SOURCE_REMOTE_BUTTON, false);
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
    const button_packet_v1_t packet = {
        .magic = ESPNOW_BUTTON_MAGIC,
        .version = ESPNOW_PAYLOAD_VERSION,
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
    const board_config_t *config = board_get_config();

    ESP_RETURN_ON_ERROR(board_init(), TAG, "Initialize board power state");

    if (config->has_imu_tx_select_gpio) {
        const gpio_config_t imu_tx_select_config = {
            .pin_bit_mask = 1ULL << config->imu_tx_select_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&imu_tx_select_config), TAG, "Configure IMU TX select GPIO");
        ESP_LOGI(TAG,
                 "IMU control select GPIO%d sampled as %s (remote IMU control is authoritative)",
                 config->imu_tx_select_gpio,
                 gpio_get_level(config->imu_tx_select_gpio) == 0 ? "LOW" : "HIGH");
    } else {
        ESP_LOGI(TAG, "Board does not expose an IMU control select GPIO");
    }

    s_red_led_available = config->red_led.present;
    if (s_red_led_available) {
        led_config_t red_led_config = {
            .gpio_num = config->red_led.gpio_num,
            .active_level = config->red_led.active_level,
        };
        ESP_RETURN_ON_ERROR(led_init(&s_red_led, &red_led_config), TAG, "Initialize red LED");
    } else {
        ESP_LOGI(TAG, "Board does not expose a red LED");
    }

    s_button_a_available = config->button_a.present;
    if (s_button_a_available) {
        button_config_t button_a_config = {
            .gpio_num = config->button_a.gpio_num,
            .active_low = config->button_a.active_low,
            .poll_interval_ms = BUTTON_POLL_INTERVAL_MS,
            .stable_polls = BUTTON_STABLE_POLLS,
        };
        button_init(&s_button_a, &button_a_config);
    }

    s_button_b_available = config->button_b.present;
    if (s_button_b_available) {
        button_config_t button_b_config = {
            .gpio_num = config->button_b.gpio_num,
            .active_low = config->button_b.active_low,
            .poll_interval_ms = BUTTON_POLL_INTERVAL_MS,
            .stable_polls = BUTTON_STABLE_POLLS,
        };
        button_init(&s_button_b, &button_b_config);
    }
    return ESP_OK;
}

static void power_heartbeat_task(void *arg)
{
    (void)arg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(POWER_HEARTBEAT_INTERVAL_MS));

        if (!s_red_led_available) {
            continue;
        }

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

static bool tick_reached(TickType_t now, TickType_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void reset_lcd_zero_level_mode(void)
{
    const board_config_t *config = board_get_config();
    s_lcd_zero_pulse_active = false;
    s_lcd_zero_next_pulse_tick = xTaskGetTickCount() + pdMS_TO_TICKS(config->lcd_zero_pulse_interval_ms);

    portENTER_CRITICAL(&s_state_lock);
    s_lcd_activity_requested = false;
    portEXIT_CRITICAL(&s_state_lock);

    if (config->lcd_zero_level_pulse && s_lcd_brightness_level == 0) {
        ESP_LOGI(TAG, "LCD level 0 pulse mode armed");
    }
}

static void service_lcd_zero_level_mode(void)
{
    const board_config_t *config = board_get_config();
    if (!config->lcd_zero_level_pulse || s_lcd_brightness_level != 0) {
        return;
    }

    bool activity_requested = false;
    portENTER_CRITICAL(&s_state_lock);
    activity_requested = s_lcd_activity_requested;
    s_lcd_activity_requested = false;
    portEXIT_CRITICAL(&s_state_lock);

    const TickType_t now = xTaskGetTickCount();
    const bool periodic_pulse_due = !s_lcd_zero_pulse_active && tick_reached(now, s_lcd_zero_next_pulse_tick);
    if (activity_requested || periodic_pulse_due) {
        if (!s_lcd_zero_pulse_active) {
            const esp_err_t err = display_set_brightness(config->lcd_zero_pulse_level);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to start LCD status pulse: %s", esp_err_to_name(err));
                return;
            }
            s_lcd_zero_pulse_active = true;
            ESP_LOGI(TAG,
                     "LCD level 0 pulse on (%s)",
                     activity_requested ? "ESP-NOW activity" : "periodic");
        }
        s_lcd_zero_pulse_end_tick = now + pdMS_TO_TICKS(config->lcd_zero_pulse_duration_ms);
        s_lcd_zero_next_pulse_tick = now + pdMS_TO_TICKS(config->lcd_zero_pulse_interval_ms);
    }

    if (s_lcd_zero_pulse_active && tick_reached(now, s_lcd_zero_pulse_end_tick)) {
        const esp_err_t err = display_set_brightness(0);
        if (err == ESP_OK) {
            s_lcd_zero_pulse_active = false;
            ESP_LOGI(TAG, "LCD level 0 pulse off");
        } else {
            ESP_LOGW(TAG, "Failed to finish LCD status pulse: %s", esp_err_to_name(err));
        }
    }
}

static void app_task(void *arg)
{
    (void)arg;

    while (true) {
        if (s_button_a_available) {
            bool button_a_state_changed = false;
            bool button_a_pressed = button_poll(&s_button_a, &button_a_state_changed);
            if (button_a_state_changed) {
                send_button_event(button_a_pressed ? BUTTON_EVENT_DOWN : BUTTON_EVENT_RELEASE);
            }
        }

        if (s_button_b_available) {
            bool button_b_state_changed = false;
            bool button_b_pressed = button_poll(&s_button_b, &button_b_state_changed);
            if (button_b_state_changed && button_b_pressed) {
                uint8_t next_level = next_brightness_level(s_lcd_brightness_level);
                esp_err_t ret = display_set_brightness(next_level);
                if (ret == ESP_OK) {
                    s_lcd_brightness_level = next_level;
                    reset_lcd_zero_level_mode();
                    ESP_LOGI(TAG, "LCD brightness level -> %u", s_lcd_brightness_level);
                } else {
                    ESP_LOGW(TAG, "Failed to set LCD brightness level %u: %s",
                             next_level,
                             esp_err_to_name(ret));
                }
            }
        }

        service_lcd_zero_level_mode();
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
    }
}

esp_err_t app_start(void)
{
    const board_config_t *config = board_get_config();

    if (config->startup_delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(config->startup_delay_ms));
    }

    ESP_LOGI(TAG, "Starting %s ESP-NOW button demo", config->board_name);
    ESP_LOGI(TAG, "LCD SPI: MOSI=%d CLK=%d CS=%d DC=%d RST=%d BL=%d",
             config->lcd_mosi_gpio,
             config->lcd_sclk_gpio,
             config->lcd_cs_gpio,
             config->lcd_dc_gpio,
             config->lcd_rst_gpio,
             config->lcd_bl_gpio);
    ESP_LOGI(TAG, "Buttons: A=%d B=%d C=%d",
             config->button_a.gpio_num,
             config->button_b.gpio_num,
             config->button_c.gpio_num);
    ESP_LOGI(TAG, "Red LED: GPIO%d (present=%s active=%s), ESP-NOW channel: %d",
             config->red_led.gpio_num,
             config->red_led.present ? "yes" : "no",
             config->red_led.active_level ? "high" : "low",
             ESPNOW_WIFI_CHANNEL);
    ESP_LOGI(TAG, "GPS UART%d: RX=GPIO%d TX=GPIO%d baud=%lu",
             config->gps.uart_num,
             config->gps.rx_gpio,
             config->gps.tx_gpio,
             (unsigned long)config->gps.baud_rate);

    ESP_RETURN_ON_ERROR(init_board_gpio(), TAG, "Initialize board GPIO");
    ESP_RETURN_ON_ERROR(display_init(), TAG, "Initialize display");
    if (config->gps.present) {
        const gps_config_t gps_config = {
            .uart_num = (uart_port_t)config->gps.uart_num,
            .rx_gpio = config->gps.rx_gpio,
            .tx_gpio = config->gps.tx_gpio,
            .baud_rate = config->gps.baud_rate,
        };
        const esp_err_t gps_err = gps_init(&gps_config);
        if (gps_err == ESP_OK) {
            s_gps_available = true;
        } else {
            ESP_LOGW(TAG, "GPS initialization failed (%s); continuing without GPS", esp_err_to_name(gps_err));
        }
    }
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
    if (s_gps_available) {
        xTaskCreate(gps_task, "gps_task", 4096, NULL, 2, NULL);
    }
    if (s_imu_available) {
        xTaskCreate(imu_task, "imu_task", 4096, NULL, 1, NULL);
    }
    xTaskCreate(app_task, "app_task", 4096, NULL, 1, NULL);

    return ESP_OK;
}
