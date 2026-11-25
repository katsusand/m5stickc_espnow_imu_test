#include "board_i2c.h"

#include "board.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "board_i2c";
static i2c_master_bus_handle_t s_i2c_bus = NULL;

esp_err_t board_i2c_init(void)
{
    if (s_i2c_bus != NULL) {
        return ESP_OK;
    }

    const board_config_t *config = board_get_config();
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = config->i2c.scl_gpio,
        .sda_io_num = config->i2c.sda_gpio,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = config->i2c.enable_internal_pullup,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus), TAG, "Create I2C master bus");

    ESP_LOGI(TAG, "I2C master initialized on port %d (SCL=%d SDA=%d)",
             I2C_NUM_0,
             config->i2c.scl_gpio,
             config->i2c.sda_gpio);
    return ESP_OK;
}

esp_err_t board_i2c_add_device(uint8_t device_address,
                               uint32_t scl_speed_hz,
                               i2c_master_dev_handle_t *out_device)
{
    ESP_RETURN_ON_FALSE(out_device != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing output device handle");
    ESP_RETURN_ON_ERROR(board_i2c_init(), TAG, "Initialize shared I2C bus");

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = scl_speed_hz,
    };
    return i2c_master_bus_add_device(s_i2c_bus, &device_config, out_device);
}

esp_err_t board_i2c_probe(uint8_t device_address, int timeout_ms)
{
    ESP_RETURN_ON_ERROR(board_i2c_init(), TAG, "Initialize shared I2C bus");
    return i2c_master_probe(s_i2c_bus, device_address, timeout_ms);
}

esp_err_t board_i2c_remove_device(i2c_master_dev_handle_t device)
{
    ESP_RETURN_ON_FALSE(device != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing I2C device handle");
    return i2c_master_bus_rm_device(device);
}
