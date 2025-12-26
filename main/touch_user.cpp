#include "touch_user.h"
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "power_management.h"

static const char *TAG                    = "touch_user";
static i2c_bus_device_handle_t cst820_dev = NULL;
static uint8_t chip_id;
static uint8_t soft_ver;
static uint8_t i2c_buf[10] = {0};
static uint8_t finger_num  = 0;
uint16_t cst820_x          = 0xFFFF;
uint16_t cst820_y          = 0xFFFF;
uint8_t cst820_status      = 0xFF;

/**
 * @brief Reset touch
 */
esp_err_t cst820_tp_reset(void)
{
    ESP_LOGI(TAG, "Reset touch");

    io_expander.digitalWrite(PY32_TP_RST_PIN, 0);

    vTaskDelay(pdMS_TO_TICKS(10));  // Delay 10ms

    io_expander.digitalWrite(PY32_TP_RST_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(50));  // Delay 50ms wait for reset complete

    return ESP_OK;
}

int cst820_init(i2c_bus_handle_t i2c_bus)
{
    cst820_tp_reset();
    cst820_dev = i2c_bus_device_create(i2c_bus, CST820_ADDR, 100000);
    if (cst820_dev == NULL) {
        ESP_LOGE(TAG, "cst820_dev create failed");
        return -1;
    } else {
        ESP_LOGI(TAG, "cst820_dev create success");
    }

    if (cst820_read_tpinfo() == -1) {
        ESP_LOGE(TAG, "cst820_read_tpinfo failed");
        return -1;
    }
    return 0;
}

int cst820_read_tpinfo()
{
    i2c_bus_read_bytes(cst820_dev, CST820_CHIP_ID, 1, &chip_id);
    i2c_bus_read_bytes(cst820_dev, CST820_SOFT_VER, 1, &soft_ver);
    ESP_LOGI(TAG, "chip_id: %d, soft_ver: %d", chip_id, soft_ver);
    if (chip_id != 0 && soft_ver != 0) {
        ESP_LOGI(TAG, "cst820_read_tpinfo success");
        return 0;
    } else {
        ESP_LOGI(TAG, "cst820_read_tpinfo failed");
        return -1;
    }
}

int cst820_update()
{
    i2c_bus_read_bytes(cst820_dev, CST820_REG_STATUS, 7, i2c_buf);

    finger_num    = i2c_buf[2];
    cst820_x      = ((uint16_t)(i2c_buf[3] & 0x0F) << 8) | i2c_buf[4];
    cst820_y      = ((uint16_t)(i2c_buf[5] & 0x0F) << 8) | i2c_buf[6];
    cst820_status = (i2c_buf[3] & 0xC0) >> 6;  // 0 down 1 up 2 keep or move
    // ESP_LOGI(TAG, "cst820_update: %d, %d, %2X", cst820_x, cst820_y, cst820_status);
    return 0;
}

void cst820_sleep()
{
    uint8_t i2c_buf[1] = {0x03};
    i2c_bus_write_bytes(cst820_dev, 0xE5, 1, i2c_buf);  // Enter sleep
}

void cst820_loop()
{
    cst820_update();
    if ((cst820_status == 0 && cst820_x != 0 && cst820_y != 0) || cst820_status == 2) {
        ESP_LOGI(TAG, "X: %d, Y: %d, status: %d", cst820_x, cst820_y, cst820_status);
    }
}