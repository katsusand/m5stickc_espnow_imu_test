#include "app.h"

#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    const esp_err_t err = app_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Application initialization stopped: %s", esp_err_to_name(err));
    }
}
