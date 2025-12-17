#include "tools.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include <dirent.h>
#include <string.h>
#include "lwip/inet.h"
#include "rx8130.h"
#include "bmi270_tools.h"
#include <lwip/tcpip.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "freertos/event_groups.h"

static const char *TAG = "tools";

i2c_bus_handle_t g_i2c_bus = NULL; // I2C bus handle
i2c_bus_device_handle_t i2c_device_all[0xFF] = {NULL};

// i2c_bus
void register_i2c_bus_device()
{
    // Create I2C bus configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        },
        .clk_flags = 0,
    };
    // Create the I2C bus
    g_i2c_bus = i2c_bus_create(I2C_NUM_0, &conf);
}

void scan_i2c_bus_device()
{
    uint8_t i2c_device_addr[256] = {0};
    uint8_t i2c_device_num = 0;
    i2c_device_num = i2c_bus_scan(g_i2c_bus,i2c_device_addr,0xFF);
    ESP_LOGI(TAG, "I2C devices found:");
    for (int i = 0; i < i2c_device_num; i++) {
        ESP_LOGI(TAG, "I2C device address: 0x%02X", i2c_device_addr[i]);
        i2c_device_all[i2c_device_addr[i]] = i2c_bus_device_create(g_i2c_bus, i2c_device_addr[i], 100000); //创建I2C设备
    }
    ESP_LOGI(TAG, "Total I2C devices found: %d", i2c_device_num);
}

