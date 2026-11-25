idf_component_register(SRCS "gps.c" "main.c" "app.c" "board.c" "board_i2c.c" "button.c" "display.c" "display_m5stickc.c" "espnow.c" "imu.c" "imu_bmi270.c" "imu_mpu6886.c" "led.c"
                    INCLUDE_DIRS "."
                    REQUIRES bmi270_bosch esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_spi esp_driver_uart esp_event esp_lcd esp_netif esp_rom esp_wifi nvs_flash)
