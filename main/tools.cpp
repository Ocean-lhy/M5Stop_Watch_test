#include "tools.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include <dirent.h>
#include <string.h>
#include "rx8130.h"
#include "bmi270_tools.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "power_management.h"

static const char *TAG = "tools";

i2c_bus_handle_t g_i2c_bus                   = NULL;  // I2C bus handle
i2c_bus_device_handle_t i2c_device_all[0xFF] = {NULL};

// Motor
void motor_task(void *arg);
TaskHandle_t motor_task_handle = NULL;
static SemaphoreHandle_t motor_mutex = NULL;
static struct {
    uint32_t end_tick;
    uint8_t intensity;
    bool active;
} motor_state = {0, 0, false};

// i2c_bus
void register_i2c_bus_device()
{
    // Create I2C bus configuration
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = I2C_SDA_PIN,
        .scl_io_num    = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master =
            {
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
    uint8_t i2c_device_num       = 0;
    i2c_device_num               = i2c_bus_scan(g_i2c_bus, i2c_device_addr, 0xFF);
    ESP_LOGI(TAG, "I2C devices found:");
    for (int i = 0; i < i2c_device_num; i++) {
        ESP_LOGI(TAG, "I2C device address: 0x%02X", i2c_device_addr[i]);
        i2c_device_all[i2c_device_addr[i]] =
            i2c_bus_device_create(g_i2c_bus, i2c_device_addr[i], 100000);  // 创建I2C设备
    }
    ESP_LOGI(TAG, "Total I2C devices found: %d", i2c_device_num);
}

void motor_init()
{
    if (motor_mutex == NULL) {
        motor_mutex = xSemaphoreCreateMutex();
    }
    
    if (motor_task_handle == NULL) {
        xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, &motor_task_handle);
    }
}

void motor_task(void *arg)
{
    uint8_t last_intensity = 0;

    while (1) {
        uint8_t target_intensity = 0;
        uint32_t current_tick = xTaskGetTickCount();

        if (motor_mutex != NULL && xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (motor_state.active) {
                // 使用 (int32_t) 类型强转处理 Tick 溢出回零的情况
                if ((int32_t)(motor_state.end_tick - current_tick) > 0) {
                    target_intensity = motor_state.intensity;
                } else {
                    motor_state.active = false;
                    target_intensity = 0;
                }
            }
            xSemaphoreGive(motor_mutex);
        }

        // 只有当强度发生变化时才调用底层驱动
        if (target_intensity != last_intensity) {
            io_expander.setPwmDuty(PY32_MOTOR_PWM_CHANNEL, target_intensity, false, true);
            last_intensity = target_intensity;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void trigger_motor(uint32_t duration_ms, uint8_t duty_percentage)
{
    if (motor_mutex == NULL) return;

    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_ms > 0 && duty_percentage > 0) {
            // 计算结束时间点：当前系统 Tick + 持续时间的 Tick
            motor_state.end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
            motor_state.intensity = duty_percentage;
            motor_state.active = true;
        } else {
            // 如果时间或强度为0，立即停止
            motor_state.active = false;
        }
        xSemaphoreGive(motor_mutex);
    }
}