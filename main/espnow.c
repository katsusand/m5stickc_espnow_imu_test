#include "espnow.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "espnow";
static const uint8_t BROADCAST_ADDR[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static espnow_config_t s_config;

static esp_err_t ensure_peer(const uint8_t peer_addr[ESP_NOW_ETH_ALEN])
{
    if (esp_now_is_peer_exist(peer_addr)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, peer_addr, sizeof(peer.peer_addr));
    peer.ifidx = WIFI_IF_STA;
    peer.channel = s_config.wifi_channel;
    peer.encrypt = false;
    return esp_now_add_peer(&peer);
}

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "Erase NVS");
        ret = nvs_flash_init();
    }
    return ret;
}

static void receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int data_len)
{
    if (s_config.receive_callback == NULL || info == NULL || data_len < 0) {
        return;
    }

    s_config.receive_callback(info->src_addr, data, (size_t)data_len);
}

/* ESP-IDF 5.5 changed the send callback's first argument from the destination MAC
 * to an esp_now_send_info_t (which carries des_addr). */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (s_config.send_callback != NULL && tx_info != NULL) {
        s_config.send_callback(tx_info->des_addr, status == ESP_NOW_SEND_SUCCESS);
    }
}
#else
static void send_callback(const uint8_t *dest_addr, esp_now_send_status_t status)
{
    if (s_config.send_callback != NULL) {
        s_config.send_callback(dest_addr, status == ESP_NOW_SEND_SUCCESS);
    }
}
#endif

esp_err_t espnow_init(const espnow_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Configuration is required");
    ESP_RETURN_ON_FALSE(config->wifi_channel > 0 && config->wifi_channel <= 14,
                        ESP_ERR_INVALID_ARG, TAG, "Invalid Wi-Fi channel");

    s_config = *config;

    ESP_RETURN_ON_ERROR(init_nvs(), TAG, "Initialize NVS");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Initialize esp-netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Create event loop");

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_config), TAG, "Initialize Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Set Wi-Fi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Set Wi-Fi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Start Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(config->wifi_channel, WIFI_SECOND_CHAN_NONE), TAG, "Set Wi-Fi channel");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "Initialize ESP-NOW");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(receive_callback), TAG, "Register receive callback");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_callback), TAG, "Register send callback");

    ESP_RETURN_ON_ERROR(ensure_peer(BROADCAST_ADDR), TAG, "Add broadcast peer");

    return ESP_OK;
}

esp_err_t espnow_send_broadcast(const void *data, size_t data_len)
{
    ESP_RETURN_ON_FALSE(data != NULL && data_len > 0, ESP_ERR_INVALID_ARG, TAG, "Payload is required");
    return esp_now_send(BROADCAST_ADDR, data, data_len);
}

esp_err_t espnow_send(const uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN], const void *data, size_t data_len)
{
    ESP_RETURN_ON_FALSE(dest_addr != NULL, ESP_ERR_INVALID_ARG, TAG, "Destination MAC is required");
    ESP_RETURN_ON_FALSE(data != NULL && data_len > 0, ESP_ERR_INVALID_ARG, TAG, "Payload is required");
    ESP_RETURN_ON_ERROR(ensure_peer(dest_addr), TAG, "Add peer");
    return esp_now_send(dest_addr, data, data_len);
}

esp_err_t espnow_get_self_mac(uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN])
{
    ESP_RETURN_ON_FALSE(mac_addr != NULL, ESP_ERR_INVALID_ARG, TAG, "MAC address buffer is required");
    return esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
}
