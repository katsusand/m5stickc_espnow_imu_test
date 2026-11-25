#include "display.h"

#include "board.h"
#include "esp_check.h"

static const char *TAG = "display";

esp_err_t display_init(void)
{
    const board_display_driver_t *driver = board_get_config()->display_driver;
    ESP_RETURN_ON_FALSE(driver != NULL && driver->init != NULL,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "Board does not provide a display driver");
    return driver->init();
}

esp_err_t display_set_brightness(uint8_t level)
{
    const board_display_driver_t *driver = board_get_config()->display_driver;
    ESP_RETURN_ON_FALSE(driver != NULL && driver->set_brightness != NULL,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "Board does not provide display brightness control");
    return driver->set_brightness(level);
}

void display_render_status(const display_status_t *status)
{
    const board_display_driver_t *driver = board_get_config()->display_driver;
    if (driver == NULL || driver->render_status == NULL) {
        return;
    }

    driver->render_status(status);
}
