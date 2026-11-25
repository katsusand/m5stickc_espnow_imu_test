#include "imu.h"

#include "bmi270.h"
#include "board_i2c.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BMI270_I2C_ADDR 0x68
#define BMI270_I2C_SPEED_HZ 400000
#define BMI270_MAX_TRANSFER_BYTES 32
#define BMI270_ACCEL_LSB_PER_G 16384.0f
#define BMI270_GYRO_LSB_PER_DPS 131.072f
#define BMI270_TEMP_LSB_PER_DEG_C 512.0f
#define BMI270_TEMP_OFFSET_DEG_C 23.0f

#define IMU_CALIBRATION_DISCARD_SAMPLES 8
#define IMU_CALIBRATION_TARGET_SAMPLES 60
#define IMU_CALIBRATION_MAX_ATTEMPTS 240
#define IMU_CALIBRATION_SAMPLE_DELAY_MS 20
#define IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G 0.05f
#define IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS 3.5f

static const char *TAG = "imu_bmi270";
static i2c_master_dev_handle_t s_imu = NULL;
static struct bmi2_dev s_bmi;
static bool s_initialized = false;
static float s_gyro_bias_x_dps = 0.0f;
static float s_gyro_bias_y_dps = 0.0f;
static float s_gyro_bias_z_dps = 0.0f;

static BMI2_INTF_RETURN_TYPE bmi270_i2c_read(uint8_t reg_addr,
                                             uint8_t *reg_data,
                                             uint32_t length,
                                             void *intf_ptr)
{
    i2c_master_dev_handle_t device = *(i2c_master_dev_handle_t *)intf_ptr;
    return i2c_master_transmit_receive(device, &reg_addr, sizeof(reg_addr), reg_data, length, -1) == ESP_OK
               ? BMI2_INTF_RET_SUCCESS
               : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE bmi270_i2c_write(uint8_t reg_addr,
                                              const uint8_t *reg_data,
                                              uint32_t length,
                                              void *intf_ptr)
{
    if (length > BMI270_MAX_TRANSFER_BYTES) {
        return BMI2_E_COM_FAIL;
    }

    i2c_master_dev_handle_t device = *(i2c_master_dev_handle_t *)intf_ptr;
    uint8_t payload[BMI270_MAX_TRANSFER_BYTES + 1] = { reg_addr };
    memcpy(&payload[1], reg_data, length);
    return i2c_master_transmit(device, payload, length + 1, -1) == ESP_OK
               ? BMI2_INTF_RET_SUCCESS
               : BMI2_E_COM_FAIL;
}

static void bmi270_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

static esp_err_t bmi270_result(int8_t result, const char *operation)
{
    if (result == BMI2_OK) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "%s failed with Bosch BMI2 error %d", operation, result);
    return ESP_FAIL;
}

static esp_err_t bmi270_read_raw_sample(struct bmi2_sens_data *sensor_data, int16_t *temperature)
{
    ESP_RETURN_ON_ERROR(bmi270_result(bmi2_get_sensor_data(sensor_data, &s_bmi), "Read accel/gyro data"),
                        TAG,
                        "Read BMI270 accel/gyro data");
    ESP_RETURN_ON_ERROR(bmi270_result(bmi2_get_temperature_data(temperature, &s_bmi), "Read temperature"),
                        TAG,
                        "Read BMI270 temperature");
    return ESP_OK;
}

static esp_err_t bmi270_calibrate_gyro_bias(void)
{
    for (int i = 0; i < IMU_CALIBRATION_DISCARD_SAMPLES; ++i) {
        struct bmi2_sens_data data = { 0 };
        int16_t temperature = 0;
        ESP_RETURN_ON_ERROR(bmi270_read_raw_sample(&data, &temperature), TAG, "Discard BMI270 startup sample");
        vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_SAMPLE_DELAY_MS));
    }

    float sum_gyro_x_dps = 0.0f;
    float sum_gyro_y_dps = 0.0f;
    float sum_gyro_z_dps = 0.0f;
    float prev_accel_x_g = 0.0f;
    float prev_accel_y_g = 0.0f;
    float prev_accel_z_g = 0.0f;
    float prev_gyro_x_dps = 0.0f;
    float prev_gyro_y_dps = 0.0f;
    float prev_gyro_z_dps = 0.0f;
    bool have_previous_sample = false;
    int stable_samples = 0;

    for (int attempt = 0; attempt < IMU_CALIBRATION_MAX_ATTEMPTS; ++attempt) {
        struct bmi2_sens_data data = { 0 };
        int16_t temperature = 0;
        ESP_RETURN_ON_ERROR(bmi270_read_raw_sample(&data, &temperature), TAG, "Read BMI270 calibration sample");

        const float accel_x_g = (float)data.acc.x / BMI270_ACCEL_LSB_PER_G;
        const float accel_y_g = (float)data.acc.y / BMI270_ACCEL_LSB_PER_G;
        const float accel_z_g = (float)data.acc.z / BMI270_ACCEL_LSB_PER_G;
        const float gyro_x_dps = (float)data.gyr.x / BMI270_GYRO_LSB_PER_DPS;
        const float gyro_y_dps = (float)data.gyr.y / BMI270_GYRO_LSB_PER_DPS;
        const float gyro_z_dps = (float)data.gyr.z / BMI270_GYRO_LSB_PER_DPS;

        const bool accel_stable = have_previous_sample &&
                                  fabsf(accel_x_g - prev_accel_x_g) <= IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G &&
                                  fabsf(accel_y_g - prev_accel_y_g) <= IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G &&
                                  fabsf(accel_z_g - prev_accel_z_g) <= IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G;
        const bool gyro_stable = have_previous_sample &&
                                 fabsf(gyro_x_dps - prev_gyro_x_dps) <= IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS &&
                                 fabsf(gyro_y_dps - prev_gyro_y_dps) <= IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS &&
                                 fabsf(gyro_z_dps - prev_gyro_z_dps) <= IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS;

        prev_accel_x_g = accel_x_g;
        prev_accel_y_g = accel_y_g;
        prev_accel_z_g = accel_z_g;
        prev_gyro_x_dps = gyro_x_dps;
        prev_gyro_y_dps = gyro_y_dps;
        prev_gyro_z_dps = gyro_z_dps;
        have_previous_sample = true;

        if (accel_stable && gyro_stable) {
            sum_gyro_x_dps += gyro_x_dps;
            sum_gyro_y_dps += gyro_y_dps;
            sum_gyro_z_dps += gyro_z_dps;
            ++stable_samples;
            if (stable_samples >= IMU_CALIBRATION_TARGET_SAMPLES) {
                s_gyro_bias_x_dps = sum_gyro_x_dps / stable_samples;
                s_gyro_bias_y_dps = sum_gyro_y_dps / stable_samples;
                s_gyro_bias_z_dps = sum_gyro_z_dps / stable_samples;
                ESP_LOGI(TAG,
                         "Gyro bias calibrated from %d stable samples: %.3f %.3f %.3f dps",
                         stable_samples,
                         s_gyro_bias_x_dps,
                         s_gyro_bias_y_dps,
                         s_gyro_bias_z_dps);
                return ESP_OK;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_SAMPLE_DELAY_MS));
    }

    ESP_LOGW(TAG,
             "Gyro bias calibration skipped: only %d/%d stable samples collected in %d attempts",
             stable_samples,
             IMU_CALIBRATION_TARGET_SAMPLES,
             IMU_CALIBRATION_MAX_ATTEMPTS);
    return ESP_OK;
}

static esp_err_t bmi270_driver_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(board_i2c_init(), TAG, "Initialize shared I2C bus");
    ESP_RETURN_ON_ERROR(board_i2c_probe(BMI270_I2C_ADDR, 20), TAG, "Probe BMI270");
    if (s_imu == NULL) {
        ESP_RETURN_ON_ERROR(board_i2c_add_device(BMI270_I2C_ADDR, BMI270_I2C_SPEED_HZ, &s_imu),
                            TAG,
                            "Add BMI270 to shared I2C bus");
    }

    memset(&s_bmi, 0, sizeof(s_bmi));
    s_bmi.intf = BMI2_I2C_INTF;
    s_bmi.intf_ptr = &s_imu;
    s_bmi.read = bmi270_i2c_read;
    s_bmi.write = bmi270_i2c_write;
    s_bmi.delay_us = bmi270_delay_us;
    s_bmi.read_write_len = BMI270_MAX_TRANSFER_BYTES;
    s_bmi.config_file_ptr = NULL;

    ESP_RETURN_ON_ERROR(bmi270_result(bmi270_init(&s_bmi), "Initialize sensor"), TAG, "Initialize BMI270");

    struct bmi2_sens_config config[2] = {
        { .type = BMI2_ACCEL },
        { .type = BMI2_GYRO },
    };
    ESP_RETURN_ON_ERROR(bmi270_result(bmi2_get_sensor_config(config, 2, &s_bmi), "Read sensor config"),
                        TAG,
                        "Read BMI270 configuration");

    config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_250;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    ESP_RETURN_ON_ERROR(bmi270_result(bmi2_set_sensor_config(config, 2, &s_bmi), "Write sensor config"),
                        TAG,
                        "Configure BMI270");

    const uint8_t sensors[] = { BMI2_ACCEL, BMI2_GYRO, BMI2_TEMP };
    ESP_RETURN_ON_ERROR(bmi270_result(bmi2_sensor_enable(sensors, sizeof(sensors), &s_bmi), "Enable sensors"),
                        TAG,
                        "Enable BMI270 sensors");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(bmi270_calibrate_gyro_bias(), TAG, "Calibrate BMI270 gyro bias");

    s_initialized = true;
    ESP_LOGI(TAG,
             "BMI270 initialized (chip ID=0x%02x, accel=+/-2g, gyro=+/-250dps, ODR=100Hz)",
             s_bmi.chip_id);
    return ESP_OK;
}

static esp_err_t bmi270_read_sample(imu_sample_t *out_sample)
{
    ESP_RETURN_ON_FALSE(out_sample != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing IMU sample output");
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "BMI270 is not initialized");

    struct bmi2_sens_data data = { 0 };
    int16_t temperature = 0;
    ESP_RETURN_ON_ERROR(bmi270_read_raw_sample(&data, &temperature), TAG, "Read BMI270 sample");

    out_sample->accel_x_g = (float)data.acc.x / BMI270_ACCEL_LSB_PER_G;
    out_sample->accel_y_g = (float)data.acc.y / BMI270_ACCEL_LSB_PER_G;
    out_sample->accel_z_g = (float)data.acc.z / BMI270_ACCEL_LSB_PER_G;
    out_sample->gyro_x_dps = ((float)data.gyr.x / BMI270_GYRO_LSB_PER_DPS) - s_gyro_bias_x_dps;
    out_sample->gyro_y_dps = ((float)data.gyr.y / BMI270_GYRO_LSB_PER_DPS) - s_gyro_bias_y_dps;
    out_sample->gyro_z_dps = ((float)data.gyr.z / BMI270_GYRO_LSB_PER_DPS) - s_gyro_bias_z_dps;
    out_sample->temperature_c = ((float)temperature / BMI270_TEMP_LSB_PER_DEG_C) + BMI270_TEMP_OFFSET_DEG_C;
    return ESP_OK;
}

const board_imu_driver_t g_board_imu_driver_bmi270 = {
    .init = bmi270_driver_init,
    .read_sample = bmi270_read_sample,
};
