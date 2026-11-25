#include "imu.h"

#include "board.h"
#include "esp_check.h"

static const char *TAG = "imu";

esp_err_t imu_init(void)
{
    const board_imu_driver_t *driver = board_get_config()->imu_driver;
    ESP_RETURN_ON_FALSE(driver != NULL && driver->init != NULL,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "Board does not provide an IMU driver");
    return driver->init();
}

esp_err_t imu_read_sample(imu_sample_t *out_sample)
{
    const board_imu_driver_t *driver = board_get_config()->imu_driver;
    ESP_RETURN_ON_FALSE(driver != NULL && driver->read_sample != NULL,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "Board does not provide IMU sample reads");
    return driver->read_sample(out_sample);
}
