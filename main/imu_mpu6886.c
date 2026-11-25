#include "imu.h"

#include "board_i2c.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define MPU6886_I2C_ADDR 0x68
#define SH200Q_I2C_ADDR 0x6C
#define MPU6886_I2C_SPEED_HZ 400000
#define MPU6886_WHO_AM_I_REG 0x75
#define MPU6886_WHO_AM_I_VALUE 0x19
#define MPU6886_PWR_MGMT_1_REG 0x6B
#define MPU6886_SMPLRT_DIV_REG 0x19
#define MPU6886_CONFIG_REG 0x1A
#define MPU6886_GYRO_CONFIG_REG 0x1B
#define MPU6886_ACCEL_CONFIG_REG 0x1C
#define MPU6886_ACCEL_CONFIG2_REG 0x1D
#define MPU6886_ACCEL_XOUT_H_REG 0x3B

#define MPU6886_RESET_DELAY_MS 100
#define MPU6886_WRITE_RETRY_DELAY_MS 10
#define MPU6886_WRITE_RETRY_COUNT 5

#define MPU6886_ACCEL_LSB_PER_G 16384.0f
#define MPU6886_GYRO_LSB_PER_DPS 131.0f
#define MPU6886_TEMP_LSB_PER_DEG_C 326.8f
#define MPU6886_TEMP_OFFSET_DEG_C 25.0f

#define IMU_CALIBRATION_DISCARD_SAMPLES 8
#define IMU_CALIBRATION_TARGET_SAMPLES 60
#define IMU_CALIBRATION_MAX_ATTEMPTS 240
#define IMU_CALIBRATION_SAMPLE_DELAY_MS 20
#define IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G 0.05f
#define IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS 3.5f

static const char *TAG = "imu_mpu6886";
static i2c_master_dev_handle_t s_imu = NULL;
static bool s_initialized = false;
static float s_gyro_bias_x_dps = 0.0f;
static float s_gyro_bias_y_dps = 0.0f;
static float s_gyro_bias_z_dps = 0.0f;

static esp_err_t imu_read_register(uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    return i2c_master_transmit_receive(s_imu, &reg_addr, sizeof(reg_addr), data, data_len, -1);
}

static esp_err_t imu_write_register(uint8_t reg_addr, uint8_t reg_value)
{
    uint8_t payload[2] = { reg_addr, reg_value };
    return i2c_master_transmit(s_imu, payload, sizeof(payload), -1);
}

static esp_err_t imu_write_register_with_retry(uint8_t reg_addr, uint8_t reg_value)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < MPU6886_WRITE_RETRY_COUNT; ++attempt) {
        err = imu_write_register(reg_addr, reg_value);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(MPU6886_WRITE_RETRY_DELAY_MS));
    }
    return err;
}

static esp_err_t imu_read_register_from(i2c_master_dev_handle_t device,
                                        uint8_t reg_addr,
                                        uint8_t *data,
                                        size_t data_len)
{
    return i2c_master_transmit_receive(device, &reg_addr, sizeof(reg_addr), data, data_len, -1);
}

static void imu_log_probe_result(const char *label, uint8_t addr, uint8_t reg_addr)
{
    i2c_master_dev_handle_t probe_device = NULL;
    const esp_err_t add_err = board_i2c_add_device(addr, MPU6886_I2C_SPEED_HZ, &probe_device);
    if (add_err != ESP_OK) {
        ESP_LOGI(TAG,
                 "IMU probe %s addr=0x%02x add_device failed: %s",
                 label,
                 addr,
                 esp_err_to_name(add_err));
        return;
    }

    uint8_t value = 0;
    const esp_err_t read_err = imu_read_register_from(probe_device, reg_addr, &value, sizeof(value));
    if (read_err == ESP_OK) {
        ESP_LOGI(TAG,
                 "IMU probe %s addr=0x%02x reg=0x%02x -> 0x%02x",
                 label,
                 addr,
                 reg_addr,
                 value);
    } else {
        ESP_LOGI(TAG,
                 "IMU probe %s addr=0x%02x reg=0x%02x read failed: %s",
                 label,
                 addr,
                 reg_addr,
                 esp_err_to_name(read_err));
    }

    const esp_err_t remove_err = board_i2c_remove_device(probe_device);
    if (remove_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "IMU probe %s addr=0x%02x remove_device failed: %s",
                 label,
                 addr,
                 esp_err_to_name(remove_err));
    }
}

static void imu_log_startup_diagnostics(void)
{
    ESP_LOGI(TAG, "IMU startup diagnostics begin");
    imu_log_probe_result("mpu6886_who_am_i", MPU6886_I2C_ADDR, MPU6886_WHO_AM_I_REG);
    imu_log_probe_result("sh200q_who_am_i", SH200Q_I2C_ADDR, 0x30);
    imu_log_probe_result("sh200q_chip_id_alt", SH200Q_I2C_ADDR, 0x2B);
    ESP_LOGI(TAG, "IMU startup diagnostics end");
}

static void imu_log_runtime_registers(void)
{
    static const uint8_t registers[] = {
        MPU6886_PWR_MGMT_1_REG,
        MPU6886_SMPLRT_DIV_REG,
        MPU6886_CONFIG_REG,
        MPU6886_GYRO_CONFIG_REG,
        MPU6886_ACCEL_CONFIG_REG,
        MPU6886_ACCEL_CONFIG2_REG,
    };

    for (size_t i = 0; i < sizeof(registers); ++i) {
        uint8_t value = 0;
        const esp_err_t err = imu_read_register(registers[i], &value, sizeof(value));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "IMU reg[0x%02x] = 0x%02x", registers[i], value);
        } else {
            ESP_LOGI(TAG,
                     "IMU reg[0x%02x] read failed: %s",
                     registers[i],
                     esp_err_to_name(err));
        }
    }
}

static int16_t read_be_i16(const uint8_t *buffer)
{
    return (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
}

static esp_err_t imu_read_raw_sample(int16_t *accel_x,
                                     int16_t *accel_y,
                                     int16_t *accel_z,
                                     int16_t *temp,
                                     int16_t *gyro_x,
                                     int16_t *gyro_y,
                                     int16_t *gyro_z)
{
    ESP_RETURN_ON_FALSE(accel_x != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing accel_x output");
    ESP_RETURN_ON_FALSE(accel_y != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing accel_y output");
    ESP_RETURN_ON_FALSE(accel_z != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing accel_z output");
    ESP_RETURN_ON_FALSE(temp != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing temp output");
    ESP_RETURN_ON_FALSE(gyro_x != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing gyro_x output");
    ESP_RETURN_ON_FALSE(gyro_y != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing gyro_y output");
    ESP_RETURN_ON_FALSE(gyro_z != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing gyro_z output");

    uint8_t raw_data[14] = { 0 };
    ESP_RETURN_ON_ERROR(imu_read_register(MPU6886_ACCEL_XOUT_H_REG, raw_data, sizeof(raw_data)),
                        TAG,
                        "Read MPU6886 sample");

    *accel_x = read_be_i16(&raw_data[0]);
    *accel_y = read_be_i16(&raw_data[2]);
    *accel_z = read_be_i16(&raw_data[4]);
    *temp = read_be_i16(&raw_data[6]);
    *gyro_x = read_be_i16(&raw_data[8]);
    *gyro_y = read_be_i16(&raw_data[10]);
    *gyro_z = read_be_i16(&raw_data[12]);
    return ESP_OK;
}

static esp_err_t imu_calibrate_gyro_bias(void)
{
    for (int i = 0; i < IMU_CALIBRATION_DISCARD_SAMPLES; ++i) {
        int16_t accel_x = 0;
        int16_t accel_y = 0;
        int16_t accel_z = 0;
        int16_t temp = 0;
        int16_t gyro_x = 0;
        int16_t gyro_y = 0;
        int16_t gyro_z = 0;
        ESP_RETURN_ON_ERROR(imu_read_raw_sample(&accel_x, &accel_y, &accel_z, &temp, &gyro_x, &gyro_y, &gyro_z),
                            TAG,
                            "Discard MPU6886 startup sample");
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
    bool have_prev_accel = false;
    int stable_samples = 0;

    for (int attempt = 0; attempt < IMU_CALIBRATION_MAX_ATTEMPTS; ++attempt) {
        int16_t accel_x = 0;
        int16_t accel_y = 0;
        int16_t accel_z = 0;
        int16_t temp = 0;
        int16_t gyro_x = 0;
        int16_t gyro_y = 0;
        int16_t gyro_z = 0;
        ESP_RETURN_ON_ERROR(imu_read_raw_sample(&accel_x, &accel_y, &accel_z, &temp, &gyro_x, &gyro_y, &gyro_z),
                            TAG,
                            "Read MPU6886 calibration sample");

        const float accel_x_g = (float)accel_x / MPU6886_ACCEL_LSB_PER_G;
        const float accel_y_g = (float)accel_y / MPU6886_ACCEL_LSB_PER_G;
        const float accel_z_g = (float)accel_z / MPU6886_ACCEL_LSB_PER_G;
        const float gyro_x_dps = (float)gyro_x / MPU6886_GYRO_LSB_PER_DPS;
        const float gyro_y_dps = (float)gyro_y / MPU6886_GYRO_LSB_PER_DPS;
        const float gyro_z_dps = (float)gyro_z / MPU6886_GYRO_LSB_PER_DPS;

        bool accel_stable = have_prev_accel &&
                            (fabsf(accel_x_g - prev_accel_x_g) <= IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G) &&
                            (fabsf(accel_y_g - prev_accel_y_g) <= IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G) &&
                            (fabsf(accel_z_g - prev_accel_z_g) <= IMU_CALIBRATION_ACCEL_DELTA_TOLERANCE_G);
        bool gyro_stable = have_prev_accel &&
                           (fabsf(gyro_x_dps - prev_gyro_x_dps) <= IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS) &&
                           (fabsf(gyro_y_dps - prev_gyro_y_dps) <= IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS) &&
                           (fabsf(gyro_z_dps - prev_gyro_z_dps) <= IMU_CALIBRATION_GYRO_DELTA_TOLERANCE_DPS);

        prev_accel_x_g = accel_x_g;
        prev_accel_y_g = accel_y_g;
        prev_accel_z_g = accel_z_g;
        prev_gyro_x_dps = gyro_x_dps;
        prev_gyro_y_dps = gyro_y_dps;
        prev_gyro_z_dps = gyro_z_dps;
        have_prev_accel = true;

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

static esp_err_t mpu6886_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(board_i2c_init(), TAG, "Initialize shared I2C bus");
    imu_log_startup_diagnostics();

    if (s_imu == NULL) {
        ESP_RETURN_ON_ERROR(board_i2c_add_device(MPU6886_I2C_ADDR, MPU6886_I2C_SPEED_HZ, &s_imu),
                            TAG,
                            "Add MPU6886 to shared I2C bus");
    }

    uint8_t who_am_i = 0;
    ESP_RETURN_ON_ERROR(imu_read_register(MPU6886_WHO_AM_I_REG, &who_am_i, sizeof(who_am_i)),
                        TAG,
                        "Read MPU6886 WHO_AM_I");
    ESP_RETURN_ON_FALSE(who_am_i == MPU6886_WHO_AM_I_VALUE,
                        ESP_ERR_NOT_FOUND,
                        TAG,
                        "Unexpected MPU6886 WHO_AM_I value: 0x%02x",
                        who_am_i);

    ESP_RETURN_ON_ERROR(imu_write_register(MPU6886_PWR_MGMT_1_REG, 0x80), TAG, "Reset MPU6886");
    vTaskDelay(pdMS_TO_TICKS(MPU6886_RESET_DELAY_MS));
    ESP_RETURN_ON_ERROR(imu_write_register_with_retry(MPU6886_PWR_MGMT_1_REG, 0x01), TAG, "Wake MPU6886");
    vTaskDelay(pdMS_TO_TICKS(MPU6886_WRITE_RETRY_DELAY_MS));
    ESP_RETURN_ON_ERROR(imu_write_register(MPU6886_SMPLRT_DIV_REG, 0x04), TAG, "Set MPU6886 sample divider");
    ESP_RETURN_ON_ERROR(imu_write_register(MPU6886_CONFIG_REG, 0x01), TAG, "Set MPU6886 DLPF");
    ESP_RETURN_ON_ERROR(imu_write_register(MPU6886_GYRO_CONFIG_REG, 0x00), TAG, "Set MPU6886 gyro range");
    ESP_RETURN_ON_ERROR(imu_write_register(MPU6886_ACCEL_CONFIG_REG, 0x00), TAG, "Set MPU6886 accel range");
    ESP_RETURN_ON_ERROR(imu_write_register(MPU6886_ACCEL_CONFIG2_REG, 0x01), TAG, "Set MPU6886 accel filter");
    ESP_RETURN_ON_ERROR(imu_calibrate_gyro_bias(), TAG, "Calibrate MPU6886 gyro bias");

    s_initialized = true;
    imu_log_runtime_registers();
    ESP_LOGI(TAG, "MPU6886 initialized (WHO_AM_I=0x%02x)", who_am_i);
    return ESP_OK;
}

static esp_err_t mpu6886_read_sample(imu_sample_t *out_sample)
{
    ESP_RETURN_ON_FALSE(out_sample != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing IMU sample output");
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "MPU6886 is not initialized");

    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    int16_t temp = 0;
    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
    ESP_RETURN_ON_ERROR(imu_read_raw_sample(&accel_x, &accel_y, &accel_z, &temp, &gyro_x, &gyro_y, &gyro_z),
                        TAG,
                        "Read MPU6886 sample");

    out_sample->accel_x_g = (float)accel_x / MPU6886_ACCEL_LSB_PER_G;
    out_sample->accel_y_g = (float)accel_y / MPU6886_ACCEL_LSB_PER_G;
    out_sample->accel_z_g = (float)accel_z / MPU6886_ACCEL_LSB_PER_G;
    out_sample->gyro_x_dps = ((float)gyro_x / MPU6886_GYRO_LSB_PER_DPS) - s_gyro_bias_x_dps;
    out_sample->gyro_y_dps = ((float)gyro_y / MPU6886_GYRO_LSB_PER_DPS) - s_gyro_bias_y_dps;
    out_sample->gyro_z_dps = ((float)gyro_z / MPU6886_GYRO_LSB_PER_DPS) - s_gyro_bias_z_dps;
    out_sample->temperature_c = ((float)temp / MPU6886_TEMP_LSB_PER_DEG_C) + MPU6886_TEMP_OFFSET_DEG_C;
    return ESP_OK;
}

const board_imu_driver_t g_board_imu_driver_mpu6886 = {
    .init = mpu6886_init,
    .read_sample = mpu6886_read_sample,
};
