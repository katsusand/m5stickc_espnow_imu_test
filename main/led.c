#include "led.h"

#include "esp_check.h"
#include <string.h>

static const char *TAG = "led";
#define LED_DIRECT_SOURCE (1U << 31)

esp_err_t led_init(led_t *led, const led_config_t *config)
{
    ESP_RETURN_ON_FALSE(led != NULL && config != NULL, ESP_ERR_INVALID_ARG, TAG, "LED and configuration are required");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->gpio_num), ESP_ERR_INVALID_ARG, TAG, "Invalid output GPIO");
    ESP_RETURN_ON_FALSE(config->active_level <= 1, ESP_ERR_INVALID_ARG, TAG, "Invalid active level");

    memset(led, 0, sizeof(*led));
    led->config = *config;
    led->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    ESP_RETURN_ON_ERROR(gpio_reset_pin(config->gpio_num), TAG, "Reset LED GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_direction(config->gpio_num, GPIO_MODE_OUTPUT), TAG, "Configure LED GPIO");
    return gpio_set_level(config->gpio_num, !config->active_level);
}

void led_set(led_t *led, bool on)
{
    led_set_source(led, LED_DIRECT_SOURCE, on);
}

void led_set_source(led_t *led, uint32_t source_mask, bool active)
{
    if (led == NULL || source_mask == 0) {
        return;
    }

    portENTER_CRITICAL(&led->lock);
    if (active) {
        led->active_sources |= source_mask;
    } else {
        led->active_sources &= ~source_mask;
    }
    gpio_set_level(led->config.gpio_num,
                   led->active_sources != 0 ? led->config.active_level : !led->config.active_level);
    portEXIT_CRITICAL(&led->lock);
}
