#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_STATUS_DISCONNECTED,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_FAILED
} wifi_status_t;

wifi_status_t wifi_manager_init();
wifi_status_t wifi_manager_connect(const char *ssid, const char *password, uint32_t timeout_ms);
bool wifi_manager_is_connected();
void wifi_manager_disconnect();
void wifi_manager_deinit();
wifi_status_t wifi_manager_get_status();
bool wifi_manager_get_ip(char *ip_str, size_t len);

#ifdef __cplusplus
}
#endif

#endif  // WIFI_MANAGER_H
