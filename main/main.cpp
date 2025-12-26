#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tools.h"
#include "power_management.h"
#include "audio_user.h"
#include "touch_user.h"
#include "nvs_flash.h"
#include "display_user.h"
#include "imu_user.h"
#include "ui.h"
static const char *TAG = "main";

extern "C" void app_main(void)
{
    // NVS Init (Wifi needed)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "M5STACK StopWatch Test v0.1 Starting...");
    gpio_deep_sleep_hold_dis();

    // I2C TEST
    register_i2c_bus_device();

    ESP_LOGI(TAG, "Init PMIC & IO Expander");
    pmic_init();
    py32_io_expander_init();

    pm1_wake_src_t wake_src;
    pm1_wake_src_read(&wake_src, PM1_ADDR_WAKE_FLAG_ALL_CLEAN);
    if (wake_src == PM1_WAKE_SRC_UNKNOWN || wake_src == PM1_WAKE_SRC_NULL)
    {
        ESP_LOGE(TAG, "wake_src is unknown or null : %d", wake_src);
    }
    else
    {
        if (wake_src & PM1_WAKE_SRC_TIM) ESP_LOGI(TAG, "wake_src is TIMER");
        if (wake_src & PM1_WAKE_SRC_VIN) ESP_LOGI(TAG, "wake_src is VIN");
        if (wake_src & PM1_WAKE_SRC_PWRBTN) ESP_LOGI(TAG, "wake_src is PWRBTN");
        if (wake_src & PM1_WAKE_SRC_RSTBTN) ESP_LOGI(TAG, "wake_src is RSTBTN");
        if (wake_src & PM1_WAKE_SRC_CMD_RST) ESP_LOGI(TAG, "wake_src is CMD_RST");
        if (wake_src & PM1_WAKE_SRC_EXT_WAKE) ESP_LOGI(TAG, "wake_src is EXT_WAKE");
        if (wake_src & PM1_WAKE_SRC_5VINOUT) ESP_LOGI(TAG, "wake_src is 5VINOUT");
    }
    clean_irq_flags();

    // scan I2C after L3B enable
    scan_i2c_bus_device();

    ESP_LOGI(TAG, "Init Display");
    display_init();

    ESP_LOGI(TAG, "Init Touch");
    cst820_init(g_i2c_bus);

    app.init();

    while (1) {
        app.loop();
    }
}