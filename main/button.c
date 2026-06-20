#include "button.h"

#include "esp_check.h"

static bool button_read_raw(const button_t *button)
{
    int level = gpio_get_level(button->config.gpio_num);
    return button->config.active_low ? (level == 0) : (level != 0);
}

void button_init(button_t *button, const button_config_t *config)
{
    assert(button != NULL);
    assert(config != NULL);

    button->config = *config;

    gpio_reset_pin(config->gpio_num);
    gpio_set_direction(config->gpio_num, GPIO_MODE_INPUT);

    button->stable_pressed = button_read_raw(button);
    button->candidate_pressed = button->stable_pressed;
    button->candidate_count = 0;
}

bool button_is_pressed(const button_t *button)
{
    return button->stable_pressed;
}

bool button_poll(button_t *button, bool *pressed_state_changed)
{
    assert(button != NULL);
    assert(pressed_state_changed != NULL);

    bool pressed = button_read_raw(button);
    *pressed_state_changed = false;

    if (pressed != button->candidate_pressed) {
        button->candidate_pressed = pressed;
        button->candidate_count = 1;
    } else if (button->candidate_count < button->config.stable_polls) {
        button->candidate_count++;
    }

    if (button->candidate_count >= button->config.stable_polls &&
        button->candidate_pressed != button->stable_pressed) {
        button->stable_pressed = button->candidate_pressed;
        *pressed_state_changed = true;
    }

    return button->stable_pressed;
}
