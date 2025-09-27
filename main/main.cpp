#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tools.h"
#include "power_management.h"
#include "es8311_user.h"
#include "cst820_user.h"
#include "nvs_flash.h"
static const char *TAG = "main";

extern "C" void app_main(void)
{
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Hello World! This is M5STACK Stop Watch.");

    //I2C TEST
    ESP_LOGI(TAG, "I2C -> register");
    register_i2c_bus_device();
    ESP_LOGI(TAG, "I2C -> register -> END.");

    ESP_LOGI(TAG, "PMIC / PY32 IO EXPANDER -> START");
    pmic_init();
    py32_io_expander_init();
    ESP_LOGI(TAG, "PMIC / PY32 IO EXPANDER -> END.");

    // scan I2C after L3B enable
    scan_i2c_bus_device();

#if 1
    ESP_LOGI(TAG, "M5GFX SHOW -> START");
    display_init();
    ESP_LOGI(TAG, "M5GFX SHOW -> END.");
#endif

#if 1
    ESP_LOGI(TAG, "CST820 -> init");
    cst820_init(g_i2c_bus);
    ESP_LOGI(TAG, "CST820 -> END.");
#endif

    // ES8311 TEST
#if 1
    ESP_LOGI(TAG, "ES8311 -> init");
    es8311_init();
    ESP_LOGI(TAG, "ES8311 -> END.");
#endif

    // BMI270 TEST
#if 1
    ESP_LOGI(TAG, "BMI270 -> show accel, gyro, mag 10s,and sleep");
    bmi270_test();
    ESP_LOGI(TAG, "BMI270 -> END.");
#endif

    // RX8130 TEST, it will use pm1 shutdown or esp32s3 sleep, so that code will end in this void
#if 0
    ESP_LOGI(TAG, "RX8130 -> show time, set time,30s timer up");
    rx8130_recovery();
    rx8130_wakeup_test();
    ESP_LOGI(TAG, "RX8130 -> END.");
#endif

    while (1)
    {
        display_gfx_loop();
        cst820_loop();
    }
}