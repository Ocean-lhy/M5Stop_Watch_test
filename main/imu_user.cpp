#include "imu_user.h"

#include "esp_log.h"
#include "bmi270_tools.h"
#include "i2c_bus.h"
#include "setting.h"

static const char* TAG = "IMU_USER";

// bmi270
bmi270_tools* bmi270_sensor = nullptr;
int accel_x                 = 0;
int accel_y                 = 0;
int accel_z                 = 0;
int gyro_x                  = 0;
int gyro_y                  = 0;
int gyro_z                  = 0;
int32_t mag_x               = 0;
int32_t mag_y               = 0;
int32_t mag_z               = 0;
bool mag_valid              = false;

void bmi270_test(i2c_bus_handle_t i2c_bus)
{
    ESP_LOGI(TAG, "BMI270 init");
    bmi270_tools bmi270_sensor;
    i2c_bus_device_handle_t bmi270_device_handle = i2c_bus_device_create(i2c_bus, I2C_BMI270_ADDR, 100000);
    esp_err_t ret = bmi270_sensor.init(i2c_bus, &bmi270_device_handle, true, bmi270_tools::MODE_CONTEXT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270初始化失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "BMI270初始化成功，磁力计已启用");
        // 启用默认传感器配置
        ret = bmi270_sensor.enable_default_sensors();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BMI270传感器启用失败: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "BMI270传感器配置完成");
        }
    }

    uint8_t ct = 10;
    while (ct--) {
        bmi270_tools::sensor_data_t bmi270_data;
        ret = bmi270_sensor.get_sensor_data(bmi270_data);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "BMI270数据获取成功-----------------");
            if (bmi270_data.acc_valid) {
                ESP_LOGI(TAG, "加速度计: %.3f, %.3f, %.3f", bmi270_data.acc_x, bmi270_data.acc_y, bmi270_data.acc_z);
            } else {
                ESP_LOGI(TAG, "BMI270加速度计数据无效");
            }
            if (bmi270_data.gyr_valid) {
                ESP_LOGI(TAG, "陀螺仪: %.3f, %.3f, %.3f", bmi270_data.gyr_x, bmi270_data.gyr_y, bmi270_data.gyr_z);
            } else {
                ESP_LOGI(TAG, "BMI270陀螺仪数据无效");
            }
        } else {
            ESP_LOGE(TAG, "BMI270数据获取失败: %s", esp_err_to_name(ret));
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
