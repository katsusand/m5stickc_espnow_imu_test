#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESPNOW_MAC_ADDR_LEN 6

typedef void (*espnow_receive_callback_t)(const uint8_t src_addr[ESPNOW_MAC_ADDR_LEN],
                                          const uint8_t *data,
                                          size_t data_len);
typedef void (*espnow_send_callback_t)(const uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN], bool success);

typedef struct {
    uint8_t wifi_channel;
    espnow_receive_callback_t receive_callback;
    espnow_send_callback_t send_callback;
} espnow_config_t;

esp_err_t espnow_init(const espnow_config_t *config);
esp_err_t espnow_send_broadcast(const void *data, size_t data_len);
esp_err_t espnow_send(const uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN], const void *data, size_t data_len);
esp_err_t espnow_get_self_mac(uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN]);
