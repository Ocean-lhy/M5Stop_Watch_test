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
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "spi_flash_chip_driver.h"
static const char *TAG = "main";

extern "C" void app_main(void)
{
    // NVS Init (Wifi needed)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // print spi flash information
    ESP_LOGI(TAG, "========== SPI Flash Information ==========");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip Model: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Chip Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Chip Revision: %d", chip_info.revision);

    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash Size: %lu MB (%lu bytes)", flash_size / (1024 * 1024), flash_size);
    }

    uint32_t flash_id = 0;
    if (esp_flash_read_id(NULL, &flash_id) == ESP_OK) {
        ESP_LOGI(TAG, "Flash ID: 0x%08lX (Manufacturer: 0x%02lX, Type: 0x%02lX, Capacity: 0x%02lX)", flash_id,
                 flash_id & 0xFF, (flash_id >> 8) & 0xFF, (flash_id >> 16) & 0xFF);
    }

    if (esp_flash_default_chip && esp_flash_default_chip->chip_drv) {
        ESP_LOGI(TAG, "Flash Chip: %s", esp_flash_default_chip->chip_drv->name);
    }

    ESP_LOGI(TAG, "Flash Type: %s", (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded" : "External");
    ESP_LOGI(TAG, "Flash Mode: %s", CONFIG_ESPTOOLPY_FLASHMODE);
    ESP_LOGI(TAG, "Flash Speed: %s MHz", CONFIG_ESPTOOLPY_FLASHFREQ);

    ESP_LOGI(TAG, "===========================================");

    // print PSRAM information
    ESP_LOGI(TAG, "========== PSRAM Information ==========");
    size_t psram_size    = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

    if (psram_size > 0) {
        ESP_LOGI(TAG, "PSRAM Total: %d bytes (%.2f MB)", psram_size, psram_size / (1024.0 * 1024.0));
        ESP_LOGI(TAG, "PSRAM Free: %d bytes (%.2f MB)", psram_free, psram_free / (1024.0 * 1024.0));
        ESP_LOGI(TAG, "PSRAM Largest Block: %d bytes (%.2f MB)", psram_largest, psram_largest / (1024.0 * 1024.0));
        ESP_LOGI(TAG, "PSRAM Status: Available");
    } else {
        ESP_LOGW(TAG, "PSRAM Status: Not detected or not enabled!");
    }

    size_t internal_size = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Internal RAM Total: %d bytes (%.2f KB)", internal_size, internal_size / 1024.0);
    ESP_LOGI(TAG, "Internal RAM Free: %d bytes (%.2f KB)", internal_free, internal_free / 1024.0);
    ESP_LOGI(TAG, "===========================================");

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
    if (wake_src == PM1_WAKE_SRC_UNKNOWN || wake_src == PM1_WAKE_SRC_NULL) {
        ESP_LOGE(TAG, "wake_src is unknown or null : %d", wake_src);
    } else {
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

    ESP_LOGI(TAG, "Init Audio");
    audio_init();
    audio_set_volume(100);
    audio_speaker_enable(true);

    ESP_LOGI(TAG, "Init Motor");
    motor_init();

    ESP_LOGI(TAG, "Init Touch");
    cst820_init(g_i2c_bus);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    audio_play_demo(AUDIO_VOICE_START);
    trigger_motor(300, 50);

    app.init();

    while (1) {
        app.loop();
    }
}