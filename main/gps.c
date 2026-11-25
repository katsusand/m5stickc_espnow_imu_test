#include "gps.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define GPS_UART_RX_BUFFER_SIZE 2048
#define GPS_READ_BUFFER_SIZE 256
#define GPS_NMEA_LINE_SIZE 128
#define GPS_MAX_FIELDS 20
#define GPS_RECEIVE_TIMEOUT_MS 2000

static const char *TAG = "gps";
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static uart_port_t s_uart_num = UART_NUM_MAX;
static gps_status_t s_status = { 0 };
static TickType_t s_last_sentence_tick = 0;
static char s_line[GPS_NMEA_LINE_SIZE];
static size_t s_line_length = 0;
static uint32_t s_total_bytes_received = 0;
static uint32_t s_total_bytes_dropped = 0;

static bool parse_uint_range(const char *value, unsigned min_value, unsigned max_value, unsigned *out_value)
{
    if (value == NULL || value[0] == '\0' || out_value == NULL) {
        return false;
    }

    char *end = NULL;
    const long parsed = strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed < (long)min_value || parsed > (long)max_value) {
        return false;
    }

    *out_value = (unsigned)parsed;
    return true;
}

static bool parse_nmea_time(const char *value, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    if (value == NULL || value[0] == '\0' || hour == NULL || minute == NULL || second == NULL) {
        return false;
    }

    const size_t length = strlen(value);
    if (length < 6) {
        return false;
    }

    char hh[3] = { value[0], value[1], '\0' };
    char mm[3] = { value[2], value[3], '\0' };
    char ss[3] = { value[4], value[5], '\0' };

    unsigned parsed = 0;
    if (!parse_uint_range(hh, 0, 23, &parsed)) {
        return false;
    }
    *hour = (uint8_t)parsed;

    if (!parse_uint_range(mm, 0, 59, &parsed)) {
        return false;
    }
    *minute = (uint8_t)parsed;

    if (!parse_uint_range(ss, 0, 60, &parsed)) {
        return false;
    }
    *second = (uint8_t)parsed;
    return true;
}

static bool parse_nmea_date(const char *value, uint16_t *year, uint8_t *month, uint8_t *day)
{
    if (value == NULL || value[0] == '\0' || year == NULL || month == NULL || day == NULL) {
        return false;
    }

    if (strlen(value) < 6) {
        return false;
    }

    char dd[3] = { value[0], value[1], '\0' };
    char mo[3] = { value[2], value[3], '\0' };
    char yy[3] = { value[4], value[5], '\0' };
    unsigned parsed = 0;

    if (!parse_uint_range(dd, 1, 31, &parsed)) {
        return false;
    }
    *day = (uint8_t)parsed;

    if (!parse_uint_range(mo, 1, 12, &parsed)) {
        return false;
    }
    *month = (uint8_t)parsed;

    if (!parse_uint_range(yy, 0, 99, &parsed)) {
        return false;
    }
    *year = (uint16_t)(2000U + (uint16_t)parsed);
    return true;
}

static int64_t days_from_civil(int64_t year, unsigned month, unsigned day)
{
    year -= (month <= 2) ? 1 : 0;
    const int64_t era = (year >= 0 ? year : year - 399) / 400;
    const unsigned year_of_era = (unsigned)(year - era * 400);
    const unsigned month_index = month > 2 ? month - 3 : month + 9;
    const unsigned day_of_year = (153U * month_index + 2U) / 5U + day - 1U;
    const unsigned day_of_era = year_of_era * 365U + year_of_era / 4U - year_of_era / 100U + day_of_year;
    return era * 146097 + (int64_t)day_of_era - 719468;
}

static uint64_t utc_datetime_to_unix_ms(uint16_t year,
                                        uint8_t month,
                                        uint8_t day,
                                        uint8_t hour,
                                        uint8_t minute,
                                        uint8_t second)
{
    if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 60) {
        return 0;
    }

    const int64_t days = days_from_civil((int64_t)year, month, day);
    if (days < 0) {
        return 0;
    }

    const uint64_t seconds_since_epoch =
        (uint64_t)days * 86400ULL +
        (uint64_t)hour * 3600ULL +
        (uint64_t)minute * 60ULL +
        (uint64_t)second;
    return seconds_since_epoch * 1000ULL;
}

static void set_utc_time_locked(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    s_status.time_valid = true;
    s_status.utc_hour = hour;
    s_status.utc_minute = minute;
    s_status.utc_second = second;
    s_status.utc_day = day;
    s_status.utc_month = month;
    s_status.utc_year = year;
    s_status.gps_utc_ms = utc_datetime_to_unix_ms(year, month, day, hour, minute, second);
}

static bool parse_zda_time(char *fields[], size_t field_count)
{
    if (field_count < 5) {
        return false;
    }

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t day = 0;
    uint8_t month = 0;
    uint16_t year = 0;
    if (!parse_nmea_time(fields[1], &hour, &minute, &second)) {
        return false;
    }
    if (!parse_nmea_date(fields[2], &year, &month, &day)) {
        return false;
    }

    portENTER_CRITICAL(&s_status_lock);
    set_utc_time_locked(year, month, day, hour, minute, second);
    portEXIT_CRITICAL(&s_status_lock);
    return true;
}

static bool parse_rmc_time(char *fields[], size_t field_count)
{
    if (field_count < 10 || fields[2][0] != 'A') {
        return false;
    }

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t day = 0;
    uint8_t month = 0;
    uint16_t year = 0;
    if (!parse_nmea_time(fields[1], &hour, &minute, &second)) {
        return false;
    }
    if (!parse_nmea_date(fields[9], &year, &month, &day)) {
        return false;
    }

    portENTER_CRITICAL(&s_status_lock);
    set_utc_time_locked(year, month, day, hour, minute, second);
    portEXIT_CRITICAL(&s_status_lock);
    return true;
}

static int hex_value(char value)
{
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    if (value >= 'A' && value <= 'F') {
        return value - 'A' + 10;
    }
    if (value >= 'a' && value <= 'f') {
        return value - 'a' + 10;
    }
    return -1;
}

static bool validate_checksum(char *line)
{
    char *asterisk = strchr(line, '*');
    if (line[0] != '$' || asterisk == NULL || asterisk[1] == '\0' || asterisk[2] == '\0') {
        return false;
    }

    uint8_t checksum = 0;
    for (char *cursor = line + 1; cursor < asterisk; ++cursor) {
        checksum ^= (uint8_t)*cursor;
    }

    const int high = hex_value(asterisk[1]);
    const int low = hex_value(asterisk[2]);
    if (high < 0 || low < 0 || checksum != (uint8_t)((high << 4) | low)) {
        return false;
    }

    *asterisk = '\0';
    return true;
}

static size_t split_fields(char *line, char *fields[], size_t max_fields)
{
    size_t field_count = 0;
    char *field = line;

    while (field_count < max_fields) {
        fields[field_count++] = field;
        char *comma = strchr(field, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        field = comma + 1;
    }

    return field_count;
}

static bool sentence_type_is(const char *field, const char *type)
{
    const size_t length = strlen(field);
    return length >= 3 && strcmp(field + length - 3, type) == 0;
}

static bool parse_coordinate(const char *value, const char *hemisphere, double *coordinate)
{
    if (value[0] == '\0' || hemisphere[0] == '\0') {
        return false;
    }

    char *end = NULL;
    const double degrees_minutes = strtod(value, &end);
    if (end == value || *end != '\0') {
        return false;
    }

    const int degrees = (int)(degrees_minutes / 100.0);
    const double minutes = degrees_minutes - ((double)degrees * 100.0);
    const bool is_latitude = hemisphere[0] == 'N' || hemisphere[0] == 'S';
    const bool is_longitude = hemisphere[0] == 'E' || hemisphere[0] == 'W';
    if ((!is_latitude && !is_longitude) ||
        minutes < 0.0 ||
        minutes >= 60.0 ||
        (is_latitude && degrees > 90) ||
        (is_longitude && degrees > 180)) {
        return false;
    }

    double result = (double)degrees + (minutes / 60.0);
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        result = -result;
    }

    *coordinate = result;
    return true;
}

static void update_gga(char *fields[], size_t field_count)
{
    if (field_count < 10) {
        return;
    }

    const int fix_quality = atoi(fields[6]);
    const int satellites = atoi(fields[7]);
    const float hdop = strtof(fields[8], NULL);
    double latitude = 0.0;
    double longitude = 0.0;
    const bool coordinates_valid =
        parse_coordinate(fields[2], fields[3], &latitude) &&
        parse_coordinate(fields[4], fields[5], &longitude);

    portENTER_CRITICAL(&s_status_lock);
    s_status.fix_valid = fix_quality > 0 && coordinates_valid;
    s_status.fix_type = (uint8_t)(fix_quality < 0 ? 0 : (fix_quality > 255 ? 255 : fix_quality));
    s_status.satellites = (uint8_t)(satellites < 0 ? 0 : (satellites > 255 ? 255 : satellites));
    s_status.hdop = hdop > 0.0f ? hdop : 0.0f;
    if (s_status.fix_valid) {
        s_status.latitude_deg = latitude;
        s_status.longitude_deg = longitude;
        s_status.altitude_m = strtof(fields[9], NULL);
    }
    portEXIT_CRITICAL(&s_status_lock);
}

static void update_rmc(char *fields[], size_t field_count)
{
    if (field_count < 10) {
        return;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    const float speed_knots = strtof(fields[7], NULL);
    const float course_deg = strtof(fields[8], NULL);
    const bool fix_valid =
        fields[2][0] == 'A' &&
        parse_coordinate(fields[3], fields[4], &latitude) &&
        parse_coordinate(fields[5], fields[6], &longitude);

    portENTER_CRITICAL(&s_status_lock);
    s_status.fix_valid = fix_valid;
    s_status.speed_knots = speed_knots > 0.0f ? speed_knots : 0.0f;
    s_status.course_deg = course_deg >= 0.0f ? course_deg : 0.0f;
    if (fix_valid && s_status.fix_type == 0) {
        s_status.fix_type = 1;
    }
    if (fix_valid) {
        s_status.latitude_deg = latitude;
        s_status.longitude_deg = longitude;
    }
    portEXIT_CRITICAL(&s_status_lock);
}

static void process_sentence(char *line)
{
    if (!validate_checksum(line)) {
        portENTER_CRITICAL(&s_status_lock);
        s_status.checksum_error_count++;
        portEXIT_CRITICAL(&s_status_lock);
        return;
    }

    char *fields[GPS_MAX_FIELDS];
    const size_t field_count = split_fields(line + 1, fields, GPS_MAX_FIELDS);
    if (field_count == 0) {
        return;
    }

    if (sentence_type_is(fields[0], "GGA")) {
        update_gga(fields, field_count);
    } else if (sentence_type_is(fields[0], "RMC")) {
        update_rmc(fields, field_count);
        parse_rmc_time(fields, field_count);
    } else if (sentence_type_is(fields[0], "ZDA")) {
        parse_zda_time(fields, field_count);
    }

    portENTER_CRITICAL(&s_status_lock);
    s_status.sentence_count++;
    s_status.receiving = true;
    s_last_sentence_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_status_lock);
}

static void process_byte(uint8_t value)
{
    if (value == '$') {
        s_line[0] = '$';
        s_line_length = 1;
        return;
    }

    if (s_line_length == 0) {
        portENTER_CRITICAL(&s_status_lock);
        s_total_bytes_dropped++;
        portEXIT_CRITICAL(&s_status_lock);
        return;
    }

    if (value == '\r' || value == '\n') {
        s_line[s_line_length] = '\0';
        process_sentence(s_line);
        s_line_length = 0;
        return;
    }

    if (s_line_length + 1 >= sizeof(s_line)) {
        s_line_length = 0;
        portENTER_CRITICAL(&s_status_lock);
        s_total_bytes_dropped++;
        portEXIT_CRITICAL(&s_status_lock);
        return;
    }

    s_line[s_line_length++] = (char)value;
}

esp_err_t gps_init(const gps_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "GPS config is required");
    ESP_RETURN_ON_FALSE(config->uart_num >= UART_NUM_0 && config->uart_num < UART_NUM_MAX,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "Invalid UART port");

    const uart_config_t uart_config = {
        .baud_rate = (int)config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(config->uart_num, &uart_config), TAG, "Configure GPS UART");
    ESP_RETURN_ON_ERROR(uart_set_pin(config->uart_num,
                                     config->tx_gpio,
                                     config->rx_gpio,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG,
                        "Route GPS UART pins");
    ESP_RETURN_ON_ERROR(uart_driver_install(config->uart_num,
                                            GPS_UART_RX_BUFFER_SIZE,
                                            0,
                                            0,
                                            NULL,
                                            0),
                        TAG,
                        "Install GPS UART driver");

    s_uart_num = config->uart_num;
    ESP_LOGI(TAG,
             "GPS UART%d ready: RX=GPIO%d TX=GPIO%d %lu 8N1",
             config->uart_num,
             config->rx_gpio,
             config->tx_gpio,
             (unsigned long)config->baud_rate);
    return ESP_OK;
}

void gps_process(uint32_t timeout_ms)
{
    if (s_uart_num == UART_NUM_MAX) {
        return;
    }

    uint8_t data[GPS_READ_BUFFER_SIZE];
    const int bytes_read = uart_read_bytes(s_uart_num,
                                           data,
                                           sizeof(data),
                                           pdMS_TO_TICKS(timeout_ms));
    if (bytes_read > 0) {
        portENTER_CRITICAL(&s_status_lock);
        s_total_bytes_received += (uint32_t)bytes_read;
        portEXIT_CRITICAL(&s_status_lock);
    }
    for (int i = 0; i < bytes_read; ++i) {
        process_byte(data[i]);
    }
}

void gps_get_status(gps_status_t *status)
{
    if (status == NULL) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_status_lock);
    *status = s_status;
    status->bytes_received = s_total_bytes_received;
    status->bytes_dropped = s_total_bytes_dropped;
    if (s_status.sentence_count > 0) {
        status->last_sentence_age_ms = (uint32_t)((now - s_last_sentence_tick) * portTICK_PERIOD_MS);
        status->receiving = status->last_sentence_age_ms <= GPS_RECEIVE_TIMEOUT_MS;
    }
    portEXIT_CRITICAL(&s_status_lock);
}
