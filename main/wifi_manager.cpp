#include "wifi_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "wifi_manager";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static wifi_status_t s_wifi_status           = WIFI_STATUS_DISCONNECTED;
static bool s_wifi_initialized               = false;
static esp_netif_t *s_sta_netif              = NULL;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected");
        s_wifi_status = WIFI_STATUS_DISCONNECTED;
        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_status = WIFI_STATUS_CONNECTED;
        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

wifi_status_t wifi_manager_init()
{
    if (s_wifi_initialized) {
        ESP_LOGI(TAG, "WiFi already initialized");
        return s_wifi_status;
    }

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to init netif: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret                    = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    s_wifi_initialized = true;
    s_wifi_status      = WIFI_STATUS_DISCONNECTED;
    ESP_LOGI(TAG, "WiFi manager initialized");
    return WIFI_STATUS_DISCONNECTED;
}

wifi_status_t wifi_manager_connect(const char *ssid, const char *password, uint32_t timeout_ms)
{
    if (!s_wifi_initialized) {
        wifi_manager_init();
    }

    if (s_wifi_status == WIFI_STATUS_CONNECTED) {
        ESP_LOGI(TAG, "Already connected to WiFi");
        return WIFI_STATUS_CONNECTED;
    }

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
    }

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect: %s", esp_err_to_name(ret));
        return WIFI_STATUS_FAILED;
    }

    s_wifi_status = WIFI_STATUS_CONNECTING;
    ESP_LOGI(TAG, "Connecting to %s...", ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to %s", ssid);
        s_wifi_status = WIFI_STATUS_CONNECTED;
        return WIFI_STATUS_CONNECTED;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to %s", ssid);
        s_wifi_status = WIFI_STATUS_FAILED;
        return WIFI_STATUS_FAILED;
    } else {
        ESP_LOGE(TAG, "Connection timeout");
        s_wifi_status = WIFI_STATUS_FAILED;
        return WIFI_STATUS_FAILED;
    }
}

bool wifi_manager_is_connected()
{
    return s_wifi_status == WIFI_STATUS_CONNECTED;
}

void wifi_manager_disconnect()
{
    if (s_wifi_initialized && s_wifi_status == WIFI_STATUS_CONNECTED) {
        esp_wifi_disconnect();
        s_wifi_status = WIFI_STATUS_DISCONNECTED;
        ESP_LOGI(TAG, "WiFi disconnected");
    }
}

void wifi_manager_deinit()
{
    if (!s_wifi_initialized) {
        return;
    }

    wifi_manager_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    s_wifi_initialized = false;
    s_wifi_status      = WIFI_STATUS_DISCONNECTED;
    ESP_LOGI(TAG, "WiFi manager deinitialized");
}

wifi_status_t wifi_manager_get_status()
{
    return s_wifi_status;
}

bool wifi_manager_get_ip(char *ip_str, size_t len)
{
    if (!wifi_manager_is_connected() || !s_sta_netif) {
        return false;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
        snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip));
        return true;
    }
    return false;
}

