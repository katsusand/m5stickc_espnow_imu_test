idf_component_register(SRCS "main.c" "app.c" "board_i2c.c" "button.c" "display.c" "espnow.c" "imu.c" "led.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_spi esp_event esp_lcd esp_netif esp_rom esp_wifi nvs_flash)
