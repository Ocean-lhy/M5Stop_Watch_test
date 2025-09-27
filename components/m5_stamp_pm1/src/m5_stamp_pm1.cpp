#include "m5_stamp_pm1.h"
#include "m5_stamp_pm1_i2c_compat.h"

// I2C 适配宏：将底层直接调用统一重定向到中间层
#ifndef PM1_WRAP_I2C_DEFINED
#define PM1_WRAP_I2C_DEFINED
#undef i2c_bus_read_byte
#undef i2c_bus_write_byte
#define i2c_bus_read_byte(dev, reg, pdata)  PM1_I2C_READ_BYTE((dev), (reg), (pdata))
#define i2c_bus_write_byte(dev, reg, data)  PM1_I2C_WRITE_BYTE((dev), (reg), (data))
#endif

#include "esp_log.h"
#include "esp_err.h"

#ifdef ARDUINO
// Arduino 环境下的兼容定义
#include "Arduino.h"
#define vTaskDelay(ticks) delay(ticks)
#define pdMS_TO_TICKS(ms) (ms)
#define portMAX_DELAY 0xFFFFFFFF
#define xTaskGetTickCount() millis()
#define pdTICKS_TO_MS(ticks) (ticks)
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// 全局记录上次设置的PWM频率
static uint32_t pm1_last_pwm_freq = 0;

// 全局标记是否为内部管理模式（通过i2c_device是否为NULL判断
static bool pm1_internal_handle_mode = false;

//记录引脚模式，状态（带默认值）
typedef struct {
    pm1_gpio_func_t       pin_func      = PM1_GPIO_FUNC_GPIO;           // 引脚模式，默认GPIO
    pm1_gpio_state_t      pin_state     = PM1_GPIO_INPUT_NC;           // 引脚状态，默认未连接/输入
    pm1_gpio_pupd_t       pin_pupd      = PM1_GPIO_PUPD_NC;            // 上下拉配置，默认无
    pm1_gpio_drv_t        drv_mode      = PM1_GPIO_DRV_OPEN_DRAIN;     // 推挽/开漏配置，默认开漏
    pm1_gpio_wake_t       wake_en       = PM1_GPIO_WAKE_DISABLE;       // 唤醒使能，默认禁用
    pm1_gpio_wake_edge_t  wake_cfg      = PM1_GPIO_WAKE_FALLING;       // 唤醒边沿配置，默认下降沿
    pm1_gpio_power_hold_t power_hold    = PM1_GPIO_POWER_HOLD_DISABLE; // 电源保持，默认禁用
} pm1_pin_status_t;

pm1_pin_status_t pm1_pin_status[5];

#ifdef ARDUINO

//ARDUINO 情况下的初始化
esp_err_t pm1_init(TwoWire *wire) 
{
    if (wire == NULL) {
        ESP_LOGE(TAG, "Wire pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Store the Wire interface
    pm1_i2c_device = wire;
    pm1_i2c_bus = wire;

    esp_err_t ret;

    // Initialize I2C interface with specified pins and frequency
    wire->begin(CONFIG_PM1_I2C_SDA_PIN, CONFIG_PM1_I2C_SCL_PIN, CONFIG_PM1_I2C_FREQUENCY);
    ESP_LOGI(TAG, "I2C initialized with SDA=%d, SCL=%d, Frequency=%dHz", 
             CONFIG_PM1_I2C_SDA_PIN, CONFIG_PM1_I2C_SCL_PIN, CONFIG_PM1_I2C_FREQUENCY);

    // Read hardware version
    uint8_t hw_version = 0;
    ret = pm1_get_hw_version(&hw_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read hardware version, device may not be connected");
        return ret;
    }
    
    // Read software version
    uint8_t sw_version = 0;
    ret = pm1_get_sw_version(&sw_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read software version");
        return ret;
    }
    
    // Read chip UID
    uint16_t chip_id = 0;
    ret = pm1_get_chip_id(&chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }
    
    // Read and initialize pin status array from device registers
    ret = pm1_read_and_check_pin_status(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read and initialize pin status");
        return ret;
    }
    
    // Reset global variables
    pm1_last_pwm_freq = 0;
    
    ESP_LOGI(TAG, "PM1 Hardware Version: %d, Software Version: %d, Chip ID: 0x%04X", hw_version, sw_version, chip_id);
    ESP_LOGI(TAG, "pm1 initialized successfully (Arduino mode)");
    
    return ESP_OK;
}

esp_err_t pm1_deinit(void)
{
    // Clear Wire pointers
    pm1_i2c_device = NULL;
    pm1_i2c_bus = NULL;
    
    // Reset global variables
    pm1_last_pwm_freq = 0;
    
    // Clear pin status
    for (int i = 0; i < 5; i++) {
        pm1_pin_status[i].pin_func = PM1_GPIO_FUNC_GPIO;
        pm1_pin_status[i].pin_state = PM1_GPIO_INPUT_NC;
        pm1_pin_status[i].pin_pupd = PM1_GPIO_PUPD_NC;
        pm1_pin_status[i].drv_mode = PM1_GPIO_DRV_OPEN_DRAIN;
        pm1_pin_status[i].wake_en = PM1_GPIO_WAKE_DISABLE;
        pm1_pin_status[i].wake_cfg = PM1_GPIO_WAKE_FALLING;
    }
    
    ESP_LOGI(TAG, "pm1 deinitialized successfully (Arduino mode)");
    return ESP_OK;
}

#else
// pm1 I2C Init
esp_err_t pm1_init(i2c_bus_handle_t i2c_bus, i2c_bus_device_handle_t *i2c_device, uint32_t i2c_freq_hz) 
{
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Store the I2C bus handle
    pm1_i2c_bus = i2c_bus;

    esp_err_t ret = ESP_OK;

    // 根据i2c_device是否为NULL判断管理模式
    if (i2c_device == NULL || *i2c_device == NULL) {
        // 内部管理模式：创建并管理设备句柄
        pm1_internal_handle_mode = true;
        
        // Create I2C device with 100KHz (PM1 default required frequency)
        pm1_i2c_device = i2c_bus_device_create(pm1_i2c_bus, pm1_i2c_address, 100000);
        if (pm1_i2c_device == NULL) {
            ESP_LOGE(TAG, "Failed to create pm1 I2C device");
            return ESP_ERR_NO_MEM;
        }

        // Return the created handle to caller if requested
        if (i2c_device != NULL) {
            *i2c_device = pm1_i2c_device;
        }

        ESP_LOGI(TAG, "PM1 initialized with internal device handle (addr: 0x%02X)", pm1_i2c_address);
    } else {
        // 外部管理模式：使用提供的设备句柄
        pm1_internal_handle_mode = false;
        pm1_i2c_device = *i2c_device;
        
        // Store the I2C device address
        pm1_i2c_address = i2c_bus_device_get_address(*i2c_device);
        
        ESP_LOGI(TAG, "PM1 initialized with external device handle");
    }

    // I2C频率管理逻辑
    // 配置置 I2C频率 - PM1模块默认必须100KHz才能驱动
    uint32_t pm1_current_clk = i2c_bus_get_current_clk_speed(pm1_i2c_bus);
    if (pm1_current_clk != 100000) {
        // 无论什么模式，都销毁并重新创建句柄以确定100KHz频率
        ret = i2c_bus_device_delete(&pm1_i2c_device);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete existing pm1 I2C device: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // 创建新的 I2C 设备以100KHz频率
        pm1_i2c_device = i2c_bus_device_create(pm1_i2c_bus, pm1_i2c_address, 100000);
        if (pm1_i2c_device == NULL) {
            ESP_LOGE(TAG, "Failed to create pm1 I2C device at 100KHz");
            return ESP_ERR_NO_MEM;
        }
        
        // 如果是外部管理模式，需要将重建后的句柄传回去
        if (!pm1_internal_handle_mode && i2c_device != NULL) {
            *i2c_device = pm1_i2c_device;
            ESP_LOGI(TAG, "External handle mode: recreated device handle with 100KHz, updated caller's handle");
        }
    }

    // 根据用户指定的频率进行相应处理
    if (i2c_freq_hz == 400000) {
        // 用户要求 400KHz，需要先设置 PM1 时钟速度，然后重新创建设备句柄
        ret = pm1_set_clk_speed(PM1_CLK_SPEED_400KHZ);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set PM1 clock speed to 400KHz: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the device to be ready
        
        // 统一重新创建400KHz的设备句柄，不区分模式
        ret = i2c_bus_device_delete(&pm1_i2c_device);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete pm1 I2C device: %s", esp_err_to_name(ret));
            return ret;
        }
        
        pm1_i2c_device = i2c_bus_device_create(pm1_i2c_bus, pm1_i2c_address, 400000);
        if (pm1_i2c_device == NULL) {
            ESP_LOGE(TAG, "Failed to create pm1 I2C device at 400KHz");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "pm1 I2C device created at 400KHz");
        
        // 如果是外部管理模式，需要将重建后的句柄传回去
        if (!pm1_internal_handle_mode && i2c_device != NULL) {
            *i2c_device = pm1_i2c_device;
            ESP_LOGI(TAG, "External handle mode: recreated device handle with 400KHz, updated caller's handle");
        }
    } else if (i2c_freq_hz == 100000) {
        // 100KHz 是PM1 的默认必需频率，设备已经在上面以100KHz 创建了
        ESP_LOGI(TAG, "Using 100KHz I2C frequency (PM1 default required frequency)");
    } else {
        // 对于其他频率，给出错误提示，因为 PM1 只支持100KHz 和400KHz
        ESP_LOGE(TAG, "Unsupported I2C frequency: %lu Hz. PM1 only supports 100000 Hz and 400000 Hz.", i2c_freq_hz);
        if (pm1_internal_handle_mode) {
            i2c_bus_device_delete(&pm1_i2c_device);
            pm1_i2c_device = NULL;
        }
        return ESP_ERR_INVALID_ARG;
    }

    // 获取 硬件版本
    uint8_t hw_version = 0;
    ret = pm1_get_hw_version(&hw_version);
    if (ret != ESP_OK) {
        if (pm1_internal_handle_mode) {
            i2c_bus_device_delete(&pm1_i2c_device);
            pm1_i2c_device = NULL;
        }
        return ret;
    }
    
    // 获取 软件版本
    uint8_t sw_version = 0;
    ret = pm1_get_sw_version(&sw_version);
    if (ret != ESP_OK) {
        if (pm1_internal_handle_mode) {
            i2c_bus_device_delete(&pm1_i2c_device);
            pm1_i2c_device = NULL;
        }
        return ret;
    }
    
    // 获取芯片UID
    uint16_t chip_id = 0;
    ret = pm1_get_chip_id(&chip_id);
    if (ret != ESP_OK) {
        if (pm1_internal_handle_mode) {
            i2c_bus_device_delete(&pm1_i2c_device);
            pm1_i2c_device = NULL;
        }
        return ret;
    }
    
    // Read and initialize pin status array from device registers
    ret = pm1_read_and_check_pin_status(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read and initialize pin status");
        if (pm1_internal_handle_mode) {
            i2c_bus_device_delete(&pm1_i2c_device);
            pm1_i2c_device = NULL;
        }
        return ret;
    }
    
    ESP_LOGI(TAG, "PM1 Hardware Version: %d, Software Version: %d, Chip ID: 0x%04X", hw_version, sw_version, chip_id);
    ESP_LOGI(TAG, "PM1 initialized successfully");
    
    return ESP_OK;
}

// pm1 Deinit
esp_err_t pm1_deinit(void)
{
    esp_err_t ret = ESP_OK;
    
    if (pm1_i2c_device != NULL) {
        // Only delete the device handle if we're managing it internally
        if (pm1_internal_handle_mode) {
            ret = i2c_bus_device_delete(&pm1_i2c_device);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to delete internally managed pm1 I2C device: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "Internally managed pm1 I2C device deleted successfully");
            }
        } else {
            ESP_LOGI(TAG, "External I2C device handle released (not deleted)");
        }
        pm1_i2c_device = NULL;
    }
    
    pm1_i2c_bus = NULL;
    pm1_internal_handle_mode = false; // Reset to default
    pm1_last_pwm_freq = 0;
    
    // Clear pin status
    for (int i = 0; i < 5; i++) {
        pm1_pin_status[i].pin_func = PM1_GPIO_FUNC_GPIO;
        pm1_pin_status[i].pin_state = PM1_GPIO_INPUT_NC;
        pm1_pin_status[i].pin_pupd = PM1_GPIO_PUPD_NC;
        pm1_pin_status[i].drv_mode = PM1_GPIO_DRV_OPEN_DRAIN;
        pm1_pin_status[i].wake_en = PM1_GPIO_WAKE_DISABLE;
        pm1_pin_status[i].wake_cfg = PM1_GPIO_WAKE_FALLING;
    }
    
    ESP_LOGI(TAG, "pm1 deinitialized successfully");
    return ret;
}

#endif

// [PM1_ADDR_HW_REV]
// 读取硬件版本号
// INPUT: uint8_t *hw_version (用来存储读取的硬件版本号)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_get_hw_version(uint8_t *hw_version)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "pm1_get_hw_version -> pm1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (hw_version == NULL) {
        ESP_LOGE(TAG, "Hardware version pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_HW_REV, hw_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PM1 hardware version: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// [PM1_ADDR_SW_REV]
// 读取软件版本号
// INPUT: uint8_t *sw_version (用来存储读取的软件版本号)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_get_sw_version(uint8_t *sw_version)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "pm1_get_sw_version -> pm1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sw_version == NULL) {
        ESP_LOGE(TAG, "Software version pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_SW_REV, sw_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PM1 software version: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// [PM1_ADDR_UID_L/H]
// 读取芯片唯一ID
// INPUT: uint16_t *chip_id (用来存储读取的芯片ID)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_get_chip_id(uint16_t *chip_id)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "pm1_get_chip_id -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (chip_id == NULL) {
        ESP_LOGE(TAG, "Chip ID pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t chip_id_l = 0;
    uint8_t chip_id_h = 0;
    esp_err_t ret;

    // 读取芯片ID低字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_UID_L, &chip_id_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PM1 chip ID low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取芯片ID高字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_UID_H, &chip_id_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PM1 chip ID high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 组合16位芯片ID
    *chip_id = (chip_id_h << 8) | chip_id_l;
    
    return ESP_OK;
}

// [UTILITY FUNCTION]
// 读取并检查引脚状态数组
// INPUT: bool enable_print (是否打印引脚状态信息)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_read_and_check_pin_status(bool enable_print)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "pm1_read_and_check_pin_status -> pm1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    
    // Read and check pin status array from device registers
    for (int i = 0; i < 5; i++) {
        // Store current values for comparison
        pm1_pin_status_t current_status = pm1_pin_status[i];
        
        // Read GPIO function configuration
        uint8_t func_reg, read_buf;
        uint8_t shift_bits;
        
        if (i <= 3) {
            func_reg = PM1_ADDR_GPIO_FUNC0;
            shift_bits = i * 2;
        } else {
            func_reg = PM1_ADDR_GPIO_FUNC1;
            shift_bits = (i - 4) * 2;
        }
        
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, func_reg, &read_buf);
        if (ret == ESP_OK) {
            pm1_gpio_func_t new_func = (pm1_gpio_func_t)((read_buf >> shift_bits) & 0x03);
            if (current_status.pin_func != new_func) {
                ESP_LOGW(TAG, "GPIO%d function changed: %d -> %d", i, current_status.pin_func, new_func);
            }
            pm1_pin_status[i].pin_func = new_func;
        } else {
            ESP_LOGW(TAG, "Failed to read GPIO%d function, using default", i);
            pm1_pin_status[i].pin_func = PM1_GPIO_FUNC_GPIO;
        }
        
        // Read GPIO mode configuration
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_MODE, &read_buf);
        if (ret == ESP_OK) {
            pm1_gpio_state_t new_state;
            if (read_buf & (1 << i)) {
                // Output mode - read output state
                uint8_t out_buf;
                ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_OUT, &out_buf);
                if (ret == ESP_OK) {
                    new_state = (out_buf & (1 << i)) ? PM1_GPIO_OUTPUT_HIGH : PM1_GPIO_OUTPUT_LOW;
                } else {
                    new_state = PM1_GPIO_OUTPUT_LOW;
                }
            } else {
                // Input mode
                new_state = PM1_GPIO_INPUT_NC;
            }
            
            if (current_status.pin_state != new_state) {
                ESP_LOGW(TAG, "GPIO%d state changed: %d -> %d", i, current_status.pin_state, new_state);
            }
            pm1_pin_status[i].pin_state = new_state;
        } else {
            ESP_LOGW(TAG, "Failed to read GPIO%d mode, using default", i);
            pm1_pin_status[i].pin_state = PM1_GPIO_INPUT_NC;
        }
        
        // Read GPIO pull-up/pull-down configuration
        uint8_t pupd_reg;
        if (i <= 3) {
            pupd_reg = PM1_ADDR_GPIO_PUPD0;
            shift_bits = i * 2;
        } else {
            pupd_reg = PM1_ADDR_GPIO_PUPD1;
            shift_bits = (i - 4) * 2;
        }
        
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, pupd_reg, &read_buf);
        if (ret == ESP_OK) {
            pm1_gpio_pupd_t new_pupd = (pm1_gpio_pupd_t)((read_buf >> shift_bits) & 0x03);
            if (current_status.pin_pupd != new_pupd) {
                ESP_LOGW(TAG, "GPIO%d pull config changed: %d -> %d", i, current_status.pin_pupd, new_pupd);
            }
            pm1_pin_status[i].pin_pupd = new_pupd;
        } else {
            ESP_LOGW(TAG, "Failed to read GPIO%d pull config, using default", i);
            pm1_pin_status[i].pin_pupd = PM1_GPIO_PUPD_NC;
        }
        
        // Read GPIO drive mode configuration
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, &read_buf);
        if (ret == ESP_OK) {
            pm1_gpio_drv_t new_drv = (read_buf & (1 << i)) ? PM1_GPIO_DRV_OPEN_DRAIN : PM1_GPIO_DRV_PUSH_PULL;
            if (current_status.drv_mode != new_drv) {
                ESP_LOGW(TAG, "GPIO%d drive mode changed: %d -> %d", i, current_status.drv_mode, new_drv);
            }
            pm1_pin_status[i].drv_mode = new_drv;
        } else {
            ESP_LOGW(TAG, "Failed to read GPIO%d drive mode, using default", i);
            pm1_pin_status[i].drv_mode = PM1_GPIO_DRV_OPEN_DRAIN;
        }
        
        // Read GPIO wake enable configuration
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_EN, &read_buf);
        if (ret == ESP_OK) {
            pm1_gpio_wake_t new_wake_en = (read_buf & (1 << i)) ? PM1_GPIO_WAKE_ENABLE : PM1_GPIO_WAKE_DISABLE;
            if (current_status.wake_en != new_wake_en) {
                ESP_LOGW(TAG, "GPIO%d wake enable changed: %d -> %d", i, current_status.wake_en, new_wake_en);
            }
            pm1_pin_status[i].wake_en = new_wake_en;
        } else {
            ESP_LOGW(TAG, "Failed to read GPIO%d wake enable, using default", i);
            pm1_pin_status[i].wake_en = PM1_GPIO_WAKE_DISABLE;
        }
        
        // Read GPIO wake edge configuration (skip GPIO1 as it doesn't support wake)
        if (i != 1) {
            ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_CFG, &read_buf);
            if (ret == ESP_OK) {
                pm1_gpio_wake_edge_t new_wake_cfg = (read_buf & (1 << i)) ? PM1_GPIO_WAKE_RISING : PM1_GPIO_WAKE_FALLING;
                if (current_status.wake_cfg != new_wake_cfg) {
                    ESP_LOGW(TAG, "GPIO%d wake config changed: %d -> %d", i, current_status.wake_cfg, new_wake_cfg);
                }
                pm1_pin_status[i].wake_cfg = new_wake_cfg;
            } else {
                ESP_LOGW(TAG, "Failed to read GPIO%d wake config, using default", i);
                pm1_pin_status[i].wake_cfg = PM1_GPIO_WAKE_FALLING;
            }
        } else {
            pm1_pin_status[i].wake_cfg = PM1_GPIO_WAKE_FALLING;
        }
        
        // Read GPIO power hold configuration
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, &read_buf);
        if (ret == ESP_OK) {
            pm1_gpio_power_hold_t new_power_hold = (read_buf & (1 << i)) ? PM1_GPIO_POWER_HOLD_ENABLE : PM1_GPIO_POWER_HOLD_DISABLE;
            if (current_status.power_hold != new_power_hold) {
                ESP_LOGW(TAG, "GPIO%d power hold changed: %d -> %d", i, current_status.power_hold, new_power_hold);
            }
            pm1_pin_status[i].power_hold = new_power_hold;
        } else {
            ESP_LOGW(TAG, "Failed to read GPIO%d power hold, using default", i);
            pm1_pin_status[i].power_hold = PM1_GPIO_POWER_HOLD_DISABLE;
        }
        
        // Print pin status if enabled
        if (enable_print) {
            ESP_LOGI(TAG, "GPIO%d status: func=%d, state=%d, pupd=%d, drv=%d, wake_en=%d, wake_cfg=%d, power_hold=%d", 
                     i, pm1_pin_status[i].pin_func, pm1_pin_status[i].pin_state, 
                     pm1_pin_status[i].pin_pupd, pm1_pin_status[i].drv_mode,
                     pm1_pin_status[i].wake_en, pm1_pin_status[i].wake_cfg, pm1_pin_status[i].power_hold);
        }
    }
    
    return ESP_OK;
}

// [PM1_ADDR_I2C_CFG]
// 配置I2C时钟速度
// INPUIT: pm1_clk_speed_t clk_speed_flag
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_set_clk_speed(pm1_clk_speed_t clk_speed_flag)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "pm1_set_clk_speed -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t read_buf = 0;
    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_I2C_CFG, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PM1 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI(TAG, "Current PM1 I2C configuration: 0x%02X:0x%02X", PM1_ADDR_I2C_CFG, read_buf);
    }

    // 配置置第[4]位，0=100KHz, 1=400KHz
    if (clk_speed_flag == PM1_CLK_SPEED_400KHZ) {
        read_buf |= (1 << 4);
    } else {
        read_buf &= ~(1 << 4);
    }

    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_I2C_CFG, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PM1 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PM1 I2C configuration set to %s successfully: 0x%02X:0x%02X",
             (clk_speed_flag == PM1_CLK_SPEED_400KHZ) ? "400K" : "100K",
             PM1_ADDR_I2C_CFG, read_buf);

    return ESP_OK;
}

// [PM1_ADDR_I2C_CFG]
// 配置I2C进入休眠的时长，休眠后，需要将SDA引脚拉低，才能唤醒
// INPUIT: uint8_t sleep_time_sec [0-15]
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_set_i2c_sleep_time(uint8_t sleep_time_sec)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_set_i2c_sleep_time -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查参数有效性
    if (sleep_time_sec > 15) {
        ESP_LOGE(TAG, "Invalid sleep time: %d. Valid range: 0-15 seconds", sleep_time_sec);
        return ESP_ERR_INVALID_ARG;
    }

    if(sleep_time_sec != 0)
    {
        // 警告不支持自动休眠唤醒，需要手动通过PM1_i2c_try_wake函数唤醒
        ESP_LOGW(TAG, "Automatic wake-up after sleep is not supported. Please use PM1_i2c_try_wake to wake up the PM1 device.");
    }

    uint8_t read_buf = 0;
    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_I2C_CFG, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PM1 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI(TAG, "Current PM1 I2C configuration: 0x%02X:0x%02X", PM1_ADDR_I2C_CFG, read_buf);
        ESP_LOGI(TAG, "Current sleep timeout: %d seconds", read_buf & 0x0F);
    }

    // 配置置第[3-0]位，SLP_TO(4-bit 0-15 s，0=禁用)
    // 保持第[4]位SPD速度控制位不变
    read_buf &= 0xF0;  // 清除低4位，保持高4位
    read_buf |= (sleep_time_sec & 0x0F);  // 设置新的休眠时间

    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_I2C_CFG, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PM1 I2C configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    if (sleep_time_sec == 0) {
        ESP_LOGI(TAG, "PM1 I2C sleep disabled successfully: 0x%02X:0x%02X", PM1_ADDR_I2C_CFG, read_buf);
    } else {
        ESP_LOGI(TAG, "PM1 I2C sleep timeout set to %d seconds successfully: 0x%02X:0x%02X", 
                 sleep_time_sec, PM1_ADDR_I2C_CFG, read_buf);
    }

    return ESP_OK;
}

// [I2C_TOOL_FUNCTION]
// I2C工具函数，用于唤醒I2C总线
// INPUIT: pm1_i2c_ack_check_t ack_check_type uint32_t timeout_ms_or_try_times
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_i2c_try_wake(pm1_i2c_ack_check_t ack_check_type, uint32_t timeout_ms_or_try_times)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_i2c_try_wake -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_FAIL;
    uint8_t dummy_read = 0;
    uint32_t try_count = 0;

    if (ack_check_type == PM1_I2C_ACK_CHECK_UNTIL_WAKE) {
        // 模式0: 持续尝试直到唤醒成功，支持超时设置
        if (timeout_ms_or_try_times == 0) {
            ESP_LOGI(TAG, "Trying to wake I2C device until success (no timeout)...");
        } else {
            ESP_LOGI(TAG, "Trying to wake I2C device until success (timeout: %lu ms)...", timeout_ms_or_try_times);
        }
        
        uint32_t start_time = xTaskGetTickCount();
        uint32_t timeout_ticks = (timeout_ms_or_try_times == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms_or_try_times);
        
        while (1) {
            try_count++;
            // 尝试读取硬件版本寄存器器来唤醒设备
            ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_HW_REV, &dummy_read);
            
            if (ret == ESP_OK) {
                uint32_t elapsed_time = pdTICKS_TO_MS(xTaskGetTickCount() - start_time);
                ESP_LOGI(TAG, "I2C device wake up successful after %lu attempts (%lu ms)", try_count, elapsed_time);
                break;
            }
            
            // 检查超时
            if (timeout_ms_or_try_times > 0) {
                uint32_t elapsed_time = xTaskGetTickCount() - start_time;
                if (elapsed_time >= timeout_ticks) {
                    ESP_LOGE(TAG, "I2C device wake up timeout after %lu attempts (%lu ms)", try_count, pdTICKS_TO_MS(elapsed_time));
                    ret = ESP_ERR_TIMEOUT;
                    break;
                }
            }
            
            // 延迟一段时间再尝试
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else if (ack_check_type == PM1_I2C_ACK_CHECK_TRY_TIMES) {
        // 模式1: 尝试指定次数，每次间隔500ms
        ESP_LOGI(TAG, "Trying to wake I2C device %lu times with 500ms interval...", timeout_ms_or_try_times);
        
        if (timeout_ms_or_try_times == 0) {
            ESP_LOGE(TAG, "Invalid try times: 0");
            return ESP_ERR_INVALID_ARG;
        }
        
        for (try_count = 1; try_count <= timeout_ms_or_try_times; try_count++) {
            // 尝试读取硬件版本寄存器器来唤醒设备
            ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_HW_REV, &dummy_read);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "I2C device wake up successful on attempt %lu/%lu", try_count, timeout_ms_or_try_times);
                break;
            }
            
            ESP_LOGW(TAG, "I2C wake attempt %lu/%lu failed: %s", try_count, timeout_ms_or_try_times, esp_err_to_name(ret));
            
            // 如果不是最后一次尝试，则延迟500ms
            if (try_count < timeout_ms_or_try_times) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C device wake up failed after %lu attempts", timeout_ms_or_try_times);
        }
    } else {
        ESP_LOGE(TAG, "Invalid ack_check_type: %d", ack_check_type);
        return ESP_ERR_INVALID_ARG;
    }

    return ret;
}

// [PM1_ADDR_GPIO_FUNC]
// 配置置GPIO模式
// INPUIT: pm1_gpio_num_t gpio_num
//        pm1_gpio_func_t func
// OUTPUIT: ESP_OK on success, error code on failure
// TIPS: pm1_pin_status 会记录变化
esp_err_t pm1_gpio_set_func(pm1_gpio_num_t gpio_num, pm1_gpio_func_t func)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_func -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr, read_buf;
    uint8_t shift_bits;
    esp_err_t ret;

    // 确定寄存器和位偏移
    if (gpio_num <= PM1_GPIO_NUM_3) {
        reg_addr = PM1_ADDR_GPIO_FUNC0;
        shift_bits = gpio_num * 2;
    } else {
        reg_addr = PM1_ADDR_GPIO_FUNC1;
        shift_bits = (gpio_num - PM1_GPIO_NUM_4) * 2;
    }

    // 读取当前寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, reg_addr, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO function register (0x%02X): %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d function: 0x%02X (bits %d-%d: %d)", 
             gpio_num, read_buf, shift_bits+1, shift_bits, (read_buf >> shift_bits) & 0x03);

    // 修改对应的位
    read_buf &= ~(0x03 << shift_bits);  // 清除对应的两位
    read_buf |= (func << shift_bits);   // 设置新的功能位

    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, reg_addr, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO function register (0x%02X): %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d function to %d successfully: 0x%02X", gpio_num, func, read_buf);

    // 更新引脚状态记录
    pm1_pin_status[gpio_num].pin_func = func;
    
    ESP_LOGI(TAG, "GPIO%d status: function=%d, state=%d, pupd=%d", 
             gpio_num, pm1_pin_status[gpio_num].pin_func, 
             pm1_pin_status[gpio_num].pin_state, pm1_pin_status[gpio_num].pin_pupd);

    // Check pin status after setting function
    ret = pm1_read_and_check_pin_status(false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to check pin status after setting GPIO%d function", gpio_num);
    }

    return ESP_OK;
}

// [PM1_ADDR_GPIO_MODE]
// 配置置GPIO输出输入
// INPUIT: pm1_gpio_num_t gpio_num
//        pm1_gpio_mode_t mode
//        pm1_gpio_state_t state
//        pm1_gpio_pupd_t pupd
//        pm1_gpio_drv_t drv_mode
// OUTPUIT: ESP_OK on success, error code on failure
// TIPS: pm1_pin_status 会记录变化
esp_err_t pm1_gpio_set(pm1_gpio_num_t gpio_num, pm1_gpio_mode_t mode, pm1_gpio_state_t state, pm1_gpio_pupd_t pupd, pm1_gpio_drv_t drv_mode)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_mode, read_out;
    esp_err_t ret;

    // 读取当前GPIO模式
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_MODE, &read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d mode: 0x%02X (bit %d: %s)", 
             gpio_num, read_mode, gpio_num, (read_mode & (1 << gpio_num)) ? "Output" : "Input");

    // 设置GPIO模式
    if (mode == PM1_GPIO_MODE_OUTPUT) {
        read_mode |= (1 << gpio_num);   // 设置为输出模式
        
        // 如果是输出模式，还需要设置输出状态
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_OUT, &read_out);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read GPIO output register: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Current GPIO%d output: 0x%02X (bit %d: %s)", 
                 gpio_num, read_out, gpio_num, (read_out & (1 << gpio_num)) ? "High" : "Low");

        // 设置输出状态
        if (state == PM1_GPIO_OUTPUT_HIGH) {
            read_out |= (1 << gpio_num);  // 设置高电平
        } else {
            read_out &= ~(1 << gpio_num); // 设置低电平
        }
        
        // 写入输出状态
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_OUT, read_out);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write GPIO output register: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Set GPIO%d output to %s: 0x%02X", 
                 gpio_num, (state == PM1_GPIO_OUTPUT_HIGH) ? "High" : "Low", read_out);
    } else {
        read_mode &= ~(1 << gpio_num);  // 设置为输入模式
    }

    // 写入GPIO模式
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_MODE, read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d mode to %s: 0x%02X", 
             gpio_num, (mode == PM1_GPIO_MODE_OUTPUT) ? "Output" : "Input", read_mode);

    // 配置GPIO上下拉
    if (pupd != PM1_GPIO_PUPD_NC) {
        uint8_t pupd_reg_addr, read_pupd;
        uint8_t shift_bits;
        
        // 确定寄存器和位偏移
        if (gpio_num <= PM1_GPIO_NUM_3) {
            pupd_reg_addr = PM1_ADDR_GPIO_PUPD0;
            shift_bits = gpio_num * 2;
        } else {
            pupd_reg_addr = PM1_ADDR_GPIO_PUPD1;
            shift_bits = (gpio_num - PM1_GPIO_NUM_4) * 2;
        }
        
        // 读取当前上下拉配置
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, pupd_reg_addr, &read_pupd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Current GPIO%d pull config: 0x%02X (bits %d-%d: %d)", 
                 gpio_num, read_pupd, shift_bits+1, shift_bits, (read_pupd >> shift_bits) & 0x03);
        
        // 修改对应的位
        read_pupd &= ~(0x03 << shift_bits);  // 清除对应的两位
        read_pupd |= (pupd << shift_bits);   // 设置新的上下拉配置
        
        // 写入修改后的值
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, pupd_reg_addr, read_pupd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
            return ret;
        }
        
        const char *pupd_str = (pupd == PM1_GPIO_PUPD_PULLUP) ? "Pull-up" : 
                               (pupd == PM1_GPIO_PUPD_PULLDOWN) ? "Pull-down" : "No-pull";
        ESP_LOGI(TAG, "Set GPIO%d pull config to %s: 0x%02X", gpio_num, pupd_str, read_pupd);
    }

    // 配置GPIO驱动模式
    uint8_t drv_reg;
    
    // 读取当前GPIO驱动模式寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, &drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO drive register: 0x%02X", drv_reg);

    // 修改对应GPIO的驱动模式位
    if (drv_mode == PM1_GPIO_DRV_OPEN_DRAIN) {
        drv_reg |= (1 << gpio_num);   // 设置为开漏模式
    } else {
        drv_reg &= ~(1 << gpio_num);  // 设置为推挽模式
    }

    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set GPIO%d drive mode to %s: 0x%02X", 
             gpio_num, 
             (drv_mode == PM1_GPIO_DRV_OPEN_DRAIN) ? "Open-Drain" : "Push-Pull", 
             drv_reg);

    // 更新引脚状态记录
    if (mode == PM1_GPIO_MODE_OUTPUT) {
        pm1_pin_status[gpio_num].pin_state = state;
    }
    pm1_pin_status[gpio_num].pin_pupd = pupd;
    pm1_pin_status[gpio_num].drv_mode = drv_mode;
    
    ESP_LOGI(TAG, "GPIO%d status updated: function=%d, state=%d, pupd=%d, drv_mode=%d", 
             gpio_num, pm1_pin_status[gpio_num].pin_func, 
             pm1_pin_status[gpio_num].pin_state, pm1_pin_status[gpio_num].pin_pupd,
             pm1_pin_status[gpio_num].drv_mode);

    return ESP_OK;
}

// [PM1_ADDR_GPIO_DRV]
// 配置GPIO驱动模式（推挽/开漏）
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号)
//        pm1_gpio_drv_t drv_mode (驱动模式：推挽或开漏)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_gpio_set_drv(pm1_gpio_num_t gpio_num, pm1_gpio_drv_t drv_mode)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_drv -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t drv_reg;
    esp_err_t ret;

    // 读取当前GPIO驱动模式寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, &drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO drive register: 0x%02X", drv_reg);

    // 修改对应GPIO的驱动模式位
    if (drv_mode == PM1_GPIO_DRV_OPEN_DRAIN) {
        drv_reg |= (1 << gpio_num);   // 设置为开漏模式
    } else {
        drv_reg &= ~(1 << gpio_num);  // 设置为推挽模式
    }

    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set GPIO%d drive mode to %s: 0x%02X", 
             gpio_num, 
             (drv_mode == PM1_GPIO_DRV_OPEN_DRAIN) ? "Open-Drain" : "Push-Pull", 
             drv_reg);

    // 更新引脚状态记录
    pm1_pin_status[gpio_num].drv_mode = drv_mode;

    return ESP_OK;
}

// [PM1_ADDR_GPIO_DRV]
// 配置LED_EN驱动模式（推挽/开漏）
// INPUT: pm1_gpio_drv_t drv_mode (驱动模式：推挽或开漏)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_led_en_set_drv(pm1_gpio_drv_t drv_mode)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_led_en_set_drv -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t drv_reg;
    esp_err_t ret;

    // 读取当前GPIO驱动模式寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, &drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO drive register: 0x%02X", drv_reg);

    // 修改第5位（LED_EN_DRV位）
    if (drv_mode == PM1_GPIO_DRV_OPEN_DRAIN) {
        drv_reg |= (1 << 5);   // 设置第5位为1（开漏模式）
    } else {
        drv_reg &= ~(1 << 5);  // 清除第5位为0（推挽模式）
    }

    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_DRV, drv_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO drive register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set LED_EN drive mode to %s: 0x%02X", 
             (drv_mode == PM1_GPIO_DRV_OPEN_DRAIN) ? "Open-Drain" : "Push-Pull", 
             drv_reg);

    return ESP_OK;
}

// [PM1_ADDR_GPIO_MODE]
// 单独配置GPIO模式（输入/输出）
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号)
//        pm1_gpio_mode_t mode (模式：输入或输出)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: pm1_pin_status 会记录变化
esp_err_t pm1_gpio_set_mode(pm1_gpio_num_t gpio_num, pm1_gpio_mode_t mode)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_mode -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_mode;
    esp_err_t ret;

    // 读取当前GPIO模式
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_MODE, &read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d mode: 0x%02X (bit %d: %s)", 
             gpio_num, read_mode, gpio_num, (read_mode & (1 << gpio_num)) ? "Output" : "Input");

    // 设置GPIO模式
    if (mode == PM1_GPIO_MODE_OUTPUT) {
        read_mode |= (1 << gpio_num);   // 设置为输出模式
    } else {
        read_mode &= ~(1 << gpio_num);  // 设置为输入模式
    }

    // 写入GPIO模式
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_MODE, read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d mode to %s: 0x%02X", 
             gpio_num, (mode == PM1_GPIO_MODE_OUTPUT) ? "Output" : "Input", read_mode);

    ESP_LOGI(TAG, "GPIO%d mode updated successfully", gpio_num);
    return ESP_OK;
}

// [PM1_ADDR_GPIO_OUT]
// 单独配置GPIO输出状态（仅在输出模式下有效）
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号)
//        pm1_gpio_state_t state (输出状态：高电平或低电平)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: pm1_pin_status 会记录变化
esp_err_t pm1_gpio_set_state(pm1_gpio_num_t gpio_num, pm1_gpio_state_t state)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_state -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_mode, read_out;
    esp_err_t ret;

    // 检查GPIO是否配置为输出模式
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_MODE, &read_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO mode register: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!(read_mode & (1 << gpio_num))) {
        ESP_LOGW(TAG, "GPIO%d is not in output mode, state setting may not take effect", gpio_num);
    }

    // 读取当前输出状态
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_OUT, &read_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO output register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d output: 0x%02X (bit %d: %s)", 
             gpio_num, read_out, gpio_num, (read_out & (1 << gpio_num)) ? "High" : "Low");

    // 设置输出状态
    if (state == PM1_GPIO_OUTPUT_HIGH) {
        read_out |= (1 << gpio_num);  // 设置高电平
    } else {
        read_out &= ~(1 << gpio_num); // 设置低电平
    }
    
    // 写入输出状态
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_OUT, read_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO output register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d output to %s: 0x%02X", 
             gpio_num, (state == PM1_GPIO_OUTPUT_HIGH) ? "High" : "Low", read_out);

    // 更新引脚状态记录
    pm1_pin_status[gpio_num].pin_state = state;
    
    ESP_LOGI(TAG, "GPIO%d state updated successfully", gpio_num);
    
    // Check pin status after setting state
    ret = pm1_read_and_check_pin_status(false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to check pin status after setting GPIO%d state", gpio_num);
    }
    
    return ESP_OK;
}

// [PM1_ADDR_GPIO_PUPD0/1]
// 单独配置GPIO上下拉
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号)
//        pm1_gpio_pupd_t pupd (上下拉配置：无配置/上拉/下拉)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: pm1_pin_status 会记录变化
esp_err_t pm1_gpio_set_pupd(pm1_gpio_num_t gpio_num, pm1_gpio_pupd_t pupd)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_pupd -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pupd_reg_addr, read_pupd;
    uint8_t shift_bits;
    esp_err_t ret;
    
    // 确定寄存器和位偏移
    if (gpio_num <= PM1_GPIO_NUM_3) {
        pupd_reg_addr = PM1_ADDR_GPIO_PUPD0;
        shift_bits = gpio_num * 2;
    } else {
        pupd_reg_addr = PM1_ADDR_GPIO_PUPD1;
        shift_bits = (gpio_num - PM1_GPIO_NUM_4) * 2;
    }
    
    // 读取当前上下拉配置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, pupd_reg_addr, &read_pupd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current GPIO%d pull config: 0x%02X (bits %d-%d: %d)", 
             gpio_num, read_pupd, shift_bits+1, shift_bits, (read_pupd >> shift_bits) & 0x03);
    
    // 修改对应的位
    read_pupd &= ~(0x03 << shift_bits);  // 清除对应的两位
    read_pupd |= (pupd << shift_bits);   // 设置新的上下拉配置
    
    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, pupd_reg_addr, read_pupd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO pull-up/pull-down register (0x%02X): %s", pupd_reg_addr, esp_err_to_name(ret));
        return ret;
    }
    
    const char *pupd_str = (pupd == PM1_GPIO_PUPD_PULLUP) ? "Pull-up" : 
                           (pupd == PM1_GPIO_PUPD_PULLDOWN) ? "Pull-down" : "No-pull";
    ESP_LOGI(TAG, "Set GPIO%d pull config to %s: 0x%02X", gpio_num, pupd_str, read_pupd);

    // 更新引脚状态记录
    pm1_pin_status[gpio_num].pin_pupd = pupd;
    
    ESP_LOGI(TAG, "GPIO%d pull configuration updated successfully", gpio_num);
    return ESP_OK;
}

// [PM1_ADDR_GPIO_WAKE_EN]
// 配置置GPIO唤醒使能
// INPUIT: pm1_gpio_num_t gpio_num
//        pm1_gpio_wake_t wake_en
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_gpio_set_wake_en(pm1_gpio_num_t gpio_num, pm1_gpio_wake_t wake_en)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_wake_en -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    // 检查是否可设置唤醒: 根据CSV配置限制和pm1_pin_status状态
    if (wake_en == PM1_GPIO_WAKE_ENABLE) {
        // GPIO1不支持唤醒
        if (gpio_num == PM1_GPIO_NUM_1) {
            ESP_LOGE(TAG, "GPIO1 does not support WAKE functionality");
            return ESP_ERR_INVALID_ARG;
        }
        
        // 冲突检查：0<->2, 3<->4
        pm1_gpio_num_t conflict_pin = PM1_GPIO_NUM_NC;  // 使用无效的GPIO编号表示无冲突
        if ((gpio_num == PM1_GPIO_NUM_0 && pm1_pin_status[PM1_GPIO_NUM_2].wake_en == PM1_GPIO_WAKE_ENABLE)) {
            conflict_pin = PM1_GPIO_NUM_2;
        } else if ((gpio_num == PM1_GPIO_NUM_2 && pm1_pin_status[PM1_GPIO_NUM_0].wake_en == PM1_GPIO_WAKE_ENABLE)) {
            conflict_pin = PM1_GPIO_NUM_0;
        } else if ((gpio_num == PM1_GPIO_NUM_3 && pm1_pin_status[PM1_GPIO_NUM_4].wake_en == PM1_GPIO_WAKE_ENABLE)) {
            conflict_pin = PM1_GPIO_NUM_4;
        } else if ((gpio_num == PM1_GPIO_NUM_4 && pm1_pin_status[PM1_GPIO_NUM_3].wake_en == PM1_GPIO_WAKE_ENABLE)) {
            conflict_pin = PM1_GPIO_NUM_3;
        }
        
        if (conflict_pin <= PM1_GPIO_NUM_4) {
            ESP_LOGW(TAG, "GPIO%d wake conflicts with GPIO%d, disabling GPIO%d wake first", gpio_num, conflict_pin, conflict_pin);
            // 先读取当前寄存器值
            uint8_t conflict_reg_val;
            esp_err_t conflict_ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_EN, &conflict_reg_val);
            if (conflict_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read GPIO wake enable register for conflict resolution: %s", esp_err_to_name(conflict_ret));
                return conflict_ret;
            }
            // 关闭冲突引脚的wake功能
            conflict_reg_val &= ~(1 << conflict_pin);
            conflict_ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_EN, conflict_reg_val);
            if (conflict_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to disable conflicting GPIO%d wake: %s", conflict_pin, esp_err_to_name(conflict_ret));
                return conflict_ret;
            }
            // 更新内部状态
            pm1_pin_status[conflict_pin].wake_en = PM1_GPIO_WAKE_DISABLE;
            ESP_LOGI(TAG, "Disabled GPIO%d wake due to conflict", conflict_pin);
        }
    }
    uint8_t reg_val;
    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_EN, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO wake enable register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应位
    if (wake_en == PM1_GPIO_WAKE_ENABLE) {
        reg_val |= (1 << gpio_num);
    } else {
        reg_val &= ~(1 << gpio_num);
    }

    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_EN, reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO wake enable register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d wake enable to %s: 0x%02X", gpio_num,
             (wake_en == PM1_GPIO_WAKE_ENABLE) ? "Enable" : "Disable", reg_val);
    // 更新唤醒使能状态
    pm1_pin_status[gpio_num].wake_en = wake_en;

    return ESP_OK;
}

// [PM1_ADDR_GPIO_WAKE_CFG]
// 配置置GPIO唤醒边沿配置置
// INPUIT: pm1_gpio_num_t gpio_num
//        pm1_gpio_wake_edge_t wake_cfg
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_gpio_set_wake_cfg(pm1_gpio_num_t gpio_num, pm1_gpio_wake_edge_t wake_cfg)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_wake_cfg -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    //不支持GPIO1的唤醒边沿配置
    if (gpio_num == PM1_GPIO_NUM_1) {
        ESP_LOGE(TAG, "GPIO1 does not support wake edge configuration");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_val;
    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_CFG, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO wake config register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应位
    if (wake_cfg == PM1_GPIO_WAKE_RISING) {
        reg_val |= (1 << gpio_num);
    } else {
        reg_val &= ~(1 << gpio_num);
    }

    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_WAKE_CFG, reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO wake config register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set GPIO%d wake edge to %s: 0x%02X", gpio_num,
             (wake_cfg == PM1_GPIO_WAKE_RISING) ? "Rising" : "Falling", reg_val);
    // 更新唤醒边沿配置状态
    pm1_pin_status[gpio_num].wake_cfg = wake_cfg;

    return ESP_OK;
}

// [PM1_ADDR_GPIO_IN]
// 读取GPIO输入状态
// INPUIT: pm1_gpio_num_t gpio_num
//          pm1_gpio_in_state_t *state (用来存储读取的状态)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_gpio_get_in_state(pm1_gpio_num_t gpio_num, pm1_gpio_in_state_t *state)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_get_in_state -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (state == NULL) {
        ESP_LOGE(TAG, "State pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_buf;
    esp_err_t ret;

    // 读取GPIO输入状态
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_IN, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO input register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 提取指定GPIO的输入状态
    *state = (read_buf & (1 << gpio_num)) ? PM1_GPIO_IN_STATE_HIGH : PM1_GPIO_IN_STATE_LOW;
    
    ESP_LOGI(TAG, "GPIO%d input state: %s (0x%02X, bit %d: %d)", 
             gpio_num, (*state == PM1_GPIO_IN_STATE_HIGH) ? "High" : "Low", 
             read_buf, gpio_num, (*state == PM1_GPIO_IN_STATE_HIGH) ? 1 : 0);
    
    return ESP_OK;
}

// [PM1_ADDR_ADC_RES] Read Only
// 读取ADC值
// INPUIT: pm1_adc_channel_t channel
//          uint16_t *adc_12bit_value (用来存储读取的ADC值)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_adc_read(pm1_adc_channel_t channel, uint16_t *adc_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_adc_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (adc_value == NULL) {
        ESP_LOGE(TAG, "ADC value pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    // 若channel为1或2，检查对应GPIO模式
    if (channel == PM1_ADC_CHANNEL_1 || channel == PM1_ADC_CHANNEL_2) {
        pm1_gpio_num_t gpio = (channel == PM1_ADC_CHANNEL_1) ? PM1_GPIO_NUM_1 : PM1_GPIO_NUM_2;
        if (pm1_pin_status[gpio].pin_func != PM1_GPIO_FUNC_OTHER) {
            ESP_LOGE(TAG, "GPIO%d not configured for ADC function", gpio);
            return ESP_ERR_INVALID_STATE;
        }
    }
    // 如果请求的是大于6的通道，报错不支持
    if (channel == PM1_ADC_CHANNEL_0 || channel == PM1_ADC_CHANNEL_3 ||
        channel == PM1_ADC_CHANNEL_4 || channel == PM1_ADC_CHANNEL_5 ||
        channel > PM1_ADC_CHANNEL_TEMP) {
        ESP_LOGE(TAG, "ADC channel %d is not supported", channel);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t read_buf;
    esp_err_t ret;

    // 读取当前ADC控制寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_ADC_CTRL, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC control register: %s", esp_err_to_name(ret));
        return ret;
    }
    // 设置ADC通道并启动转换
    read_buf = ((channel & 0x07) << 1) | 0x01;
    
    // 写入ADC控制寄存器器以启动转换
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_ADC_CTRL, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ADC control register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Started ADC conversion on channel %d: 0x%02X", channel, read_buf);

    // 等待ADC转换完成（START位清零）
    int retry = 10;  // 最多等待10次
    while (retry--) {
        vTaskDelay(pdMS_TO_TICKS(20));  // 等待20ms
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_ADC_CTRL, &read_buf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC control register during wait: %s", esp_err_to_name(ret));
            return ret;
        }
        
        if ((read_buf & 0x01) == 0) {  // START位被清零，表示转换完成
            break;
        }
        
        if (retry == 0) {
            ESP_LOGE(TAG, "ADC conversion timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    // 读取ADC结果寄存器
    uint8_t adc_l, adc_h;
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_ADC_RES_L, &adc_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC result low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_ADC_RES_H, &adc_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC result high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 组合ADC结果（12位）
    *adc_value = ((adc_h & 0x0F) << 8) | adc_l;
    
    ESP_LOGI(TAG, "ADC channel %d result: %u (0x%03X)", channel, *adc_value, *adc_value);
    
    return ESP_OK;
}

// [PM1_ADDR_PWM]
// 配置PWM参数
// INPUIT: pm1_pwm_channel_t channel
//          pm1_pwm_ctrl_t ctrl (PWM使能控制)
//          pm1_pwm_polarity_t polarity (PWM极性)
//          uint32_t pwm_freq_16bit_value (16位频率值 0-65535)
//          uint32_t duty_cycle_12bit_value (12位占空比值 0-4095)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_pwm_set(pm1_pwm_channel_t channel, pm1_pwm_ctrl_t ctrl, pm1_pwm_polarity_t polarity, uint32_t pwm_freq_16bit_value, uint32_t duty_cycle_12bit_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_pwm_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // 检查PWM对应的GPIO引脚模式：PWM0->GPIO3, PWM1->GPIO4
    if (channel == PM1_PWM_CHANNEL_0) {
        if (pm1_pin_status[PM1_GPIO_NUM_3].pin_func != PM1_GPIO_FUNC_OTHER) {
            ESP_LOGW(TAG, "GPIO3 not configured for PWM function (OTHER mode)");
        }
    } else {
        if (pm1_pin_status[PM1_GPIO_NUM_4].pin_func != PM1_GPIO_FUNC_OTHER) {
            ESP_LOGW(TAG, "GPIO4 not configured for PWM function (OTHER mode)");
        }
    }

    if (duty_cycle_12bit_value > 4095) {
        ESP_LOGE(TAG, "Invalid duty cycle value: %lu (max 4095)", duty_cycle_12bit_value);
        return ESP_ERR_INVALID_ARG;
    }

    if (pwm_freq_16bit_value > 0xFFFF) {
        ESP_LOGE(TAG, "Invalid PWM frequency value: %lu (max 65535)", pwm_freq_16bit_value);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pwm_l_addr, pwm_hc_addr;
    uint8_t read_hc;
    esp_err_t ret;

    // 确定寄存器器地址
    if (channel == PM1_PWM_CHANNEL_0) {
        pwm_l_addr = PM1_ADDR_PWM0_L;
        pwm_hc_addr = PM1_ADDR_PWM0_HC;
    } else {
        pwm_l_addr = PM1_ADDR_PWM1_L;
        pwm_hc_addr = PM1_ADDR_PWM1_HC;
    }

    // 读取当前PWM控制/高位寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, pwm_hc_addr, &read_hc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PWM%d control register: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current PWM%d control: 0x%02X (enable: %d, polarity: %d, duty high bits: 0x%X)", 
             channel, read_hc, (read_hc >> 4) & 0x01, (read_hc >> 5) & 0x01, read_hc & 0x0F);

    // 设置PWM占空比低8位
    uint8_t pwm_l = duty_cycle_12bit_value & 0xFF;
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, pwm_l_addr, pwm_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM%d duty cycle low byte: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    
    // 设置PWM控制/高位寄存器
    uint8_t pwm_hc = (polarity << 5) | (ctrl << 4) | ((duty_cycle_12bit_value >> 8) & 0x0F);
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, pwm_hc_addr, pwm_hc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM%d control register: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    
    // 检查频率改变对另一路PWM的影响
    if (pm1_last_pwm_freq != 0 && pwm_freq_16bit_value != pm1_last_pwm_freq) {
        ESP_LOGW(TAG, "PWM frequency changed from %lu to %lu, other channel will also be updated", pm1_last_pwm_freq, pwm_freq_16bit_value);
    }
    // 更新全局记录
    pm1_last_pwm_freq = pwm_freq_16bit_value;

    // 设置PWM频率寄存器
    uint8_t freq_l = pwm_freq_16bit_value & 0xFF;
    uint8_t freq_h = (pwm_freq_16bit_value >> 8) & 0xFF;
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_PWM_FREQ_L, freq_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM frequency low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_PWM_FREQ_H, freq_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM frequency high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set PWM%d: freq=%lu/65535, duty=%lu/4095, %s, polarity=%s", 
             channel, pwm_freq_16bit_value, duty_cycle_12bit_value, 
             (ctrl == PM1_PWM_CTRL_ENABLE) ? "enabled" : "disabled", 
             (polarity == PM1_PWM_POLARITY_NORMAL) ? "normal" : "inverted");
    
    return ESP_OK;
}

// [PM1_ADDR_WDT_SET]
// 配置WDT参数
// INPUIT: pm1_wdt_ctrl_t ctrl (WDT使能控制)
//          uint8_t timeout_s (WDT超时时间, 单位s, 0表示禁用, 1-255为秒)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_wdt_set(pm1_wdt_ctrl_t ctrl, uint8_t timeout_s)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_wdt_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t read_buf;
    esp_err_t ret;

    // 读取当前WDT设置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_WDT_CNT, &read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WDT counter register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current WDT counter: 0x%02X (%d seconds, %s)", 
             read_buf, read_buf, read_buf == 0 ? "disabled" : "enabled");

    // 设置WDT超时时间：0表示禁用，1-255表示超时秒数
    uint8_t wdt_cnt = (ctrl == PM1_WDT_CTRL_ENABLE) ? timeout_s : 0;
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_WDT_CNT, wdt_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write WDT counter register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set WDT to %s with timeout %u seconds", 
             (ctrl == PM1_WDT_CTRL_ENABLE) ? "enabled" : "disabled", 
             (ctrl == PM1_WDT_CTRL_ENABLE) ? timeout_s : 0);
    
    return ESP_OK;
}

// [PM1_ADDR_WDT_CLR] Write Only
// 喂狗
// INPUIT: key (喂狗密钥, 0xA5)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_wdt_feed(uint8_t key)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_wdt_feed -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (key != 0xA5) {
        ESP_LOGE(TAG, "Invalid WDT feed key: 0x%02X (should be 0xA5)", key);
        return ESP_ERR_INVALID_ARG;
    }

    // 写入喂狗密钥
    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_WDT_KEY, key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write WDT key register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WDT feed successful with key 0x%02X", key);
    
    return ESP_OK;
}

// [PM1_ADDR_WDT_CNT] Read Only
// 读取看门狗倒计时值
// INPUT: uint8_t *wdt_cnt (用来存储读取的看门狗倒计时值, 单位s)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_wdt_get_wdt_cnt(uint8_t *wdt_cnt)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_wdt_get_wdt_cnt -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (wdt_cnt == NULL) {
        ESP_LOGE(TAG, "WDT count pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_WDT_CNT, wdt_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WDT count register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WDT count: %u seconds (%s)", *wdt_cnt, 
             (*wdt_cnt == 0) ? "disabled" : "enabled");
    
    return ESP_OK;
}

// [PM1_ADDR_TIM]
// 配置定时器参数
// INPUIT: pm1_tim_ctrl_t ctrl (定时器使能控制)
//          pm1_tim_action_t action (定时器动作配置)
//          uint32_t tim_ct_31bit_value (31位定时器计数值 0-0x7FFFFFFF, 单位s)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_tim_set(pm1_tim_ctrl_t ctrl, pm1_tim_action_t action, uint32_t tim_ct_31bit_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_tim_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (tim_ct_31bit_value > 0x7FFFFFFF) {
        ESP_LOGE(TAG, "Invalid timer count: %lu (max 0x7FFFFFFF)", tim_ct_31bit_value);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t read_cfg;
    esp_err_t ret;

    // 读取当前定时器配置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_TIM_CFG, &read_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read timer configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current timer configuration: 0x%02X (ARM: %d, ACTION: %d)", 
             read_cfg, (read_cfg >> 3) & 0x01, read_cfg & 0x07);

    // 设置计数值 - 使用4个字节寄存器
    uint8_t tim_byte0 = tim_ct_31bit_value & 0xFF;
    uint8_t tim_byte1 = (tim_ct_31bit_value >> 8) & 0xFF;
    uint8_t tim_byte2 = (tim_ct_31bit_value >> 16) & 0xFF;
    uint8_t tim_byte3 = (tim_ct_31bit_value >> 24) & 0x7F; // 只有7位有效

    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_TIM_CNT_BYTE_0, tim_byte0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 0: %s", esp_err_to_name(ret));
               return ret;
    }
    
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_TIM_CNT_BYTE_1, tim_byte1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 1: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_TIM_CNT_BYTE_2, tim_byte2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 2: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_TIM_CNT_BYTE_3, tim_byte3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer counter byte 3: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 设置定时器配置
    // ARM位[3]: 当ctrl为ENABLE时设置为1，表示启用定时器；为DISABLE时设置为0
    // ACTION位[2:0]: 定时器动作配置
    uint8_t tim_cfg;
    if (ctrl == PM1_ADDR_TIM_ENABLE) {
        tim_cfg = (1 << 3) | action;  // ARM=1(启用定时器)，设置ACTION
    } else {
        tim_cfg = 0;  // ARM=0(停止计数)，ACTION=000(停止计数)
    }
    
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_TIM_CFG, tim_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Set timer: count=%lu, %s, ACTION=%s", 
             tim_ct_31bit_value, 
             (ctrl == PM1_ADDR_TIM_ENABLE) ? "enabled" : "disabled", 
             (action == PM1_TIM_ACTION_000) ? "stop" :
             (action == PM1_TIM_ACTION_001) ? "set WAKE flag" :
             (action == PM1_TIM_ACTION_010) ? "restart" :
             (action == PM1_TIM_ACTION_011) ? "power on" :
             (action == PM1_TIM_ACTION_100) ? "power off" : "unknown");

    return ESP_OK;
}

// [PM1_ADDR_TIM_CLR] Write Only
// 清除定时器并重载
// INPUIT: key (清除密钥, 0xA5)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_tim_clear(uint8_t key)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_tim_clear -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (key != 0xA5) {
        ESP_LOGE(TAG, "Invalid timer clear key: 0x%02X (should be 0xA5)", key);
        return ESP_ERR_INVALID_ARG;
    }

    // 写入清除密钥
    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_TIM_KEY, key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timer key register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Timer cleared and reloaded with key 0x%02X", key);
    
    return ESP_OK;
}

// [PM1_ADDR_BTN_DL_LOCK]
// 配置下载锁定模式(控制BTN_CFG寄存器的DL_LOCK位[7])
// INPUT: pm1_download_enable_t enable (下载模式锁定控制)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_download_enable_set(pm1_download_enable_t enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_download_enable_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current button configuration: 0x%02X (DL_LOCK: %d)", 
             btn_cfg, (btn_cfg >> 7) & 0x01);

    // 设置或清除DL_LOCK位[7]
    if (enable == PM1_ADDR_DOWNLOAD_DISABLE) {
        btn_cfg |= (1 << 7);  // 设置DL_LOCK
    } else {
        btn_cfg &= ~(1 << 7); // 清除DL_LOCK
    }
    
    // 写入更新后的配置
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s download lock: %s", 
                 (enable == PM1_ADDR_DOWNLOAD_ENABLE) ? "enable" : "disable", 
                 esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PM1 download lock %s successfully", 
             (enable == PM1_ADDR_DOWNLOAD_ENABLE) ? "enabled" : "disabled");
    
    return ESP_OK;
}

// 获取下载锁定模式状态(读取BTN_CFG寄存器的DL_LOCK位[7])
// INPUT: pm1_download_enable_t *enable (用于返回当前状态的指针)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_download_enable_get(pm1_download_enable_t *enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_download_enable_get -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (enable == NULL) {
        ESP_LOGE(TAG, "Download enable pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 提取DL_LOCK位[7]的状态
    if ((btn_cfg >> 7) & 0x01) {
        *enable = PM1_ADDR_DOWNLOAD_DISABLE; // DL_LOCK=1 表示禁用下载
    } else {
        *enable = PM1_ADDR_DOWNLOAD_ENABLE;  // DL_LOCK=0 表示启用下载
    }

    ESP_LOGI(TAG, "Current download lock status: %s (reg value: 0x%02X)", 
             (*enable == PM1_ADDR_DOWNLOAD_ENABLE) ? "enabled" : "disabled", btn_cfg);
    
    return ESP_OK;
}

// [PM1_ADDR_BTN_CFG - SINGLE_RESET_DIS]
// 配置单击复位禁用功能(控制BTN_CFG寄存器的SINGLE_RESET_DIS位[0])
// INPUT: pm1_single_reset_dis_t enable (单击复位禁用控制)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_single_reset_dis_set(pm1_single_reset_dis_t enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_single_reset_dis_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current button configuration: 0x%02X (SINGLE_RESET_DIS: %d)", 
             btn_cfg, btn_cfg & 0x01);

    // 设置或清除SINGLE_RESET_DIS位[0]
    if (enable == PM1_SINGLE_RESET_DISABLE) {
        btn_cfg |= (1 << 0);  // 设置SINGLE_RESET_DIS=1 (禁止单击复位)
    } else {
        btn_cfg &= ~(1 << 0); // 清除SINGLE_RESET_DIS=0 (启用单击复位)
    }
    
    // 写入更新后的配置
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s single reset: %s", 
                 (enable == PM1_SINGLE_RESET_DISABLE) ? "disable" : "enable", 
                 esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PM1 single reset %s successfully", 
             (enable == PM1_SINGLE_RESET_DISABLE) ? "disabled" : "enabled");
    
    return ESP_OK;
}

// 获取单击复位禁用功能状态(读取BTN_CFG寄存器的SINGLE_RESET_DIS位[0])
// INPUT: pm1_single_reset_dis_t *enable (用于返回当前状态的指针)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_single_reset_dis_get(pm1_single_reset_dis_t *enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_single_reset_dis_get -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (enable == NULL) {
        ESP_LOGE(TAG, "Single reset disable pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 提取SINGLE_RESET_DIS位[0]的状态
    if (btn_cfg & 0x01) {
        *enable = PM1_SINGLE_RESET_DISABLE; // SINGLE_RESET_DIS=1 表示禁止单击复位
    } else {
        *enable = PM1_SINGLE_RESET_ENABLE;  // SINGLE_RESET_DIS=0 表示启用单击复位
    }

    ESP_LOGI(TAG, "Current single reset status: %s (reg value: 0x%02X)", 
             (*enable == PM1_SINGLE_RESET_DISABLE) ? "disabled" : "enabled", btn_cfg);
    
    return ESP_OK;
}

// [PM1_ADDR_BTN_CFG_2 - DOUBLE_POWEROFF_DIS]
// 配置双击关机禁用功能(控制BTN_CFG_2寄存器的DOUBLE_POWEROFF_DIS位[0])
// INPUT: pm1_double_poweroff_dis_t enable (双击关机禁用控制)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_double_poweroff_dis_set(pm1_double_poweroff_dis_t enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_double_poweroff_dis_set -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t btn_cfg2;
    esp_err_t ret;

    // 读取当前按键配置寄存器2
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG_2, &btn_cfg2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration 2 register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current button configuration 2: 0x%02X (DOUBLE_POWEROFF_DIS: %d)", 
             btn_cfg2, btn_cfg2 & 0x01);

    // 设置或清除DOUBLE_POWEROFF_DIS位[0]
    if (enable == PM1_DOUBLE_POWEROFF_DISABLE) {
        btn_cfg2 |= (1 << 0);  // 设置DOUBLE_POWEROFF_DIS=1 (禁止双击关机)
    } else {
        btn_cfg2 &= ~(1 << 0); // 清除DOUBLE_POWEROFF_DIS=0 (启用双击关机)
    }
    
    // 写入更新后的配置
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG_2, btn_cfg2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to %s double poweroff: %s", 
                 (enable == PM1_DOUBLE_POWEROFF_DISABLE) ? "disable" : "enable", 
                 esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PM1 double poweroff %s successfully", 
             (enable == PM1_DOUBLE_POWEROFF_DISABLE) ? "disabled" : "enabled");
    
    return ESP_OK;
}

// 获取双击关机禁用功能状态(读取BTN_CFG_2寄存器的DOUBLE_POWEROFF_DIS位[0])
// INPUT: pm1_double_poweroff_dis_t *enable (用于返回当前状态的指针)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_double_poweroff_dis_get(pm1_double_poweroff_dis_t *enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_double_poweroff_dis_get -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (enable == NULL) {
        ESP_LOGE(TAG, "Double poweroff disable pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t btn_cfg2;
    esp_err_t ret;

    // 读取当前按键配置寄存器2
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG_2, &btn_cfg2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration 2 register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 提取DOUBLE_POWEROFF_DIS位[0]的状态
    if (btn_cfg2 & 0x01) {
        *enable = PM1_DOUBLE_POWEROFF_DISABLE; // DOUBLE_POWEROFF_DIS=1 表示禁止双击关机
    } else {
        *enable = PM1_DOUBLE_POWEROFF_ENABLE;  // DOUBLE_POWEROFF_DIS=0 表示启用双击关机
    }

    ESP_LOGI(TAG, "Current double poweroff status: %s (reg value: 0x%02X)", 
             (*enable == PM1_DOUBLE_POWEROFF_DISABLE) ? "disabled" : "enabled", btn_cfg2);
    
    return ESP_OK;
}

// [PM1_ADDR_BTN_CFG]
// 按键响应延迟配置
// INPUIT: pm1_btn_type_t btn_type (按键类型)
//          pm1_btn_delay_t delay (延迟配置)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_btn_set_cfg(pm1_btn_type_t btn_type, pm1_btn_delay_t delay)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_btn_set_cfg -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 验证按键类型参数
    if (btn_type > PM1_ADDR_BTN_TYPE_LONG_PRESS) {
        ESP_LOGE(TAG, "Invalid button type: %d", btn_type);
        return ESP_ERR_INVALID_ARG;
    }

    // 验证延迟配置参数（对应不同按键类型的有效范围
    // 修改[2-1]位 - SINGLE配置
    // 修改[6-5]位 - DBL配置
    // 修改[4-3]位 - LONG配置
    if ((btn_type == PM1_ADDR_BTN_TYPE_CLICK && delay > PM1_ADDR_BTN_CLICK_DELAY_1000MS) ||
        (btn_type == PM1_ADDR_BTN_TYPE_DOUBLE_CLICK && delay > PM1_ADDR_BTN_DOUBLE_CLICK_DELAY_1000MS) ||
        (btn_type == PM1_ADDR_BTN_TYPE_LONG_PRESS && delay > PM1_ADDR_BTN_LONG_PRESS_DELAY_4000MS)) {
        ESP_LOGE(TAG, "Invalid delay value %d for button type %d", delay, btn_type);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t btn_cfg;
    esp_err_t ret;

    // 读取当前按键配置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, &btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Current button configuration: 0x%02X", btn_cfg);

    // 根据按键类型修改相应的位
    switch (btn_type) {
        case PM1_ADDR_BTN_TYPE_CLICK:
            // 修改[2-1]位 - SINGLE配置
            btn_cfg &= ~(0x03 << 1);  // 清除原有配置
            btn_cfg |= (delay & 0x03) << 1;  // 设置新配置
            break;
        
        case PM1_ADDR_BTN_TYPE_DOUBLE_CLICK:
            // 修改[6-5]位 - DBL配置
            btn_cfg &= ~(0x03 << 5);  // 清除原有配置
            btn_cfg |= (delay & 0x03) << 5;  // 设置新配置
            break;
        
        case PM1_ADDR_BTN_TYPE_LONG_PRESS:
            // 修改[4-3]位 - LONG配置
            btn_cfg &= ~(0x03 << 3);  // 清除原有配置
            btn_cfg |= (delay & 0x03) << 3;  // 设置新配置
            break;
    }

    // 写入修改后的配置置
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_BTN_CFG, btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write button configuration register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 记录按键类型和延迟的字符串描述，用于日志
    const char *type_str;
    const char *delay_str;

    // 按键类型描述
    switch (btn_type) {
        case PM1_ADDR_BTN_TYPE_CLICK:
            type_str = "Click";
            break;
        case PM1_ADDR_BTN_TYPE_DOUBLE_CLICK:
            type_str = "Double Click";
            break;
        case PM1_ADDR_BTN_TYPE_LONG_PRESS:
            type_str = "Long Press";
            break;
        default:
            type_str = "Unknown";
            break;
    }

    // 延迟配置描述
    if (btn_type == PM1_ADDR_BTN_TYPE_LONG_PRESS) {
        switch (delay) {
            case PM1_ADDR_BTN_LONG_PRESS_DELAY_1000MS:
                delay_str = "1000ms";
                break;
            case PM1_ADDR_BTN_LONG_PRESS_DELAY_2000MS:
                delay_str = "2000ms";
                break;
            case PM1_ADDR_BTN_LONG_PRESS_DELAY_3000MS:
                delay_str = "3000ms";
                break;
            case PM1_ADDR_BTN_LONG_PRESS_DELAY_4000MS:
                delay_str = "4000ms";
                break;
            default:
                delay_str = "Unknown";
                break;
        }
    } else {  // 点击和双击的延迟配置相同
        switch (delay) {
            case PM1_ADDR_BTN_CLICK_DELAY_125MS:
                delay_str = "125ms";
                break;
            case PM1_ADDR_BTN_CLICK_DELAY_250MS:
                delay_str = "250ms";
                break;
            case PM1_ADDR_BTN_CLICK_DELAY_500MS:
                delay_str = "500ms";
                break;
            case PM1_ADDR_BTN_CLICK_DELAY_1000MS:
                delay_str = "1000ms";
                break;
            default:
                delay_str = "Unknown";
                break;
        }
    }

    ESP_LOGI(TAG, "Button %s delay set to %s successfully (reg value: 0x%02X)", 
             type_str, delay_str, btn_cfg);
    
    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS1]
// 读取GPIO中断状态
// INPUIT: pm1_irq_gpio_t *gpio_num (GPIO编号)
//         pm1_irq_gpio_clean_type_t clean_type (中断清除类型)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_status(pm1_irq_gpio_t *gpio_num, pm1_irq_gpio_clean_type_t clean_type)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_status -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num == NULL) {
        ESP_LOGE(TAG, "GPIO number pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t irq_status;
    esp_err_t ret;

    // 读取GPIO中断状态寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1, &irq_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IRQ status register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查哪个GPIO的中断状态位被设置
    *gpio_num = PM1_ADDR_IRQ_NULL;  // 初始化为无效值
    for (int i = 0; i < 5; i++) {
        if (irq_status & (1 << i)) {
            *gpio_num = (pm1_irq_gpio_t)i;
            break;
        }
    }

    ESP_LOGI(TAG, "IRQ status: 0x%02X, triggered GPIO: %d", 
             irq_status, (*gpio_num == PM1_ADDR_IRQ_NULL) ? -1 : (int)*gpio_num);
    
    // Clear interrupts based on clean_type
    switch (clean_type) {
        case PM1_ADDR_IRQ_GPIO_NOT_CLEAN:
            // 不清除中断，直接返回
            break;
        case PM1_ADDR_IRQ_GPIO_ONCE_CLEAN:
            // 清除指定GPIO的中断
            if (*gpio_num != PM1_ADDR_IRQ_NULL) {
                ret = pm1_irq_clear_gpio_flag(*gpio_num);
                if (ret != ESP_OK) {
                    return ret;
                }
            }
            break;
        case PM1_ADDR_IRQ_GPIO_ALL_CLEAN:
            // 清除所有GPIO中断
            ret = pm1_irq_clear_gpio_flag(PM1_ADDR_IRQ_GPIO_ALL);
            if (ret != ESP_OK) {
                return ret;
            }
            break;
        default:
            break;
    }

    return ESP_OK;
}

// 清除GPIO中断标志
// INPUT: pm1_irq_gpio_t gpio_num (GPIO编号，可使用PM1_ADDR_IRQ_GPIO_ALL清除所有)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_clear_gpio_flag(pm1_irq_gpio_t gpio_num)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_clear_gpio_flag -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    if (gpio_num == PM1_ADDR_IRQ_GPIO_ALL) {
        // 清除所有GPIO中断
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear all GPIO IRQs: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "All GPIO IRQ flags cleared");
    } else {
        // 清除指定GPIO的中断
        uint8_t irq_status;
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1, &irq_status);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read IRQ status register: %s", esp_err_to_name(ret));
            return ret;
        }

        uint8_t new_val = irq_status & ~(1 << gpio_num);
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1, new_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear GPIO%d IRQ: %s", gpio_num, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "GPIO%d IRQ flag cleared", gpio_num);
    }

    return ESP_OK;
}

// 清除系统中断标志
// INPUT: pm1_irq_sys_t sys_irq (系统中断类型，可使用PM1_ADDR_IRQ_SYS_ALL清除所有)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_clear_sys_status(pm1_irq_sys_t sys_irq)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_clear_sys_status -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    if (sys_irq == PM1_ADDR_IRQ_SYS_ALL) {
        // 清除所有系统中断
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear all system IRQs: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "All system IRQ flags cleared");
    } else {
        // 清除指定系统中断
        uint8_t irq_status;
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2, &irq_status);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read system IRQ status register: %s", esp_err_to_name(ret));
            return ret;
        }

        uint8_t new_val = irq_status & ~(1 << sys_irq);
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2, new_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear system IRQ %d: %s", sys_irq, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "System IRQ %d flag cleared", sys_irq);
    }

    return ESP_OK;
}

// 清除按钮中断标志
// INPUT: pm1_irq_btn_t btn_irq (按钮中断类型，可使用PM1_ADDR_IRQ_BTN_ALL清除所有)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_clear_btn_status(pm1_irq_btn_t btn_irq)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_clear_btn_status -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    if (btn_irq == PM1_ADDR_IRQ_BTN_ALL) {
        // 清除所有按钮中断
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear all button IRQs: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "All button IRQ flags cleared");
    } else {
        // 清除指定按钮中断
        uint8_t irq_status;
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3, &irq_status);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read button IRQ status register: %s", esp_err_to_name(ret));
            return ret;
        }

        uint8_t new_val = irq_status & ~(1 << btn_irq);
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3, new_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear button IRQ %d: %s", btn_irq, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Button IRQ %d flag cleared", btn_irq);
    }

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS2]
// 读取系统中断状态
// INPUIT: pm1_irq_sys_t *sys_irq (系统中断类型)
//         pm1_irq_sys_clean_type_t clean_type (中断清除类型)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_sys_status(pm1_irq_sys_t *sys_irq, pm1_irq_sys_clean_type_t clean_type)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_sys_status -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sys_irq == NULL) {
        ESP_LOGE(TAG, "System IRQ pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t irq_status;
    esp_err_t ret;

    // 读取系统中断状态寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2, &irq_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read system IRQ status register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查哪个系统中断状态位被设置
    *sys_irq = PM1_ADDR_IRQ_SYS_NULL;  // 初始化为无效值
    for (int i = 0; i <= 5; i++) {
        if (irq_status & (1 << i)) {
            *sys_irq = (pm1_irq_sys_t)i;
            break;
        }
    }

    ESP_LOGI(TAG, "System IRQ status: 0x%02X, event: %d", 
             irq_status, (*sys_irq == PM1_ADDR_IRQ_SYS_NULL) ? -1 : (int)*sys_irq);
    
    // Clear system IRQ based on clean_type
    switch (clean_type) {
        case PM1_ADDR_IRQ_SYS_NOT_CLEAN:
            // 不清除中断，直接返回
            break;
        case PM1_ADDR_IRQ_SYS_ONCE_CLEAN:
            // 清除指定系统中断
            if (*sys_irq != PM1_ADDR_IRQ_SYS_NULL && *sys_irq <= PM1_ADDR_IRQ_SYS_BAT_REMOVE) {
                ret = pm1_irq_clear_sys_status(*sys_irq);
                if (ret != ESP_OK) {
                    return ret;
                }
            }
            break;
        case PM1_ADDR_IRQ_SYS_ALL_CLEAN:
            // 清除所有系统中断
            ret = pm1_irq_clear_sys_status(PM1_ADDR_IRQ_SYS_ALL);
            if (ret != ESP_OK) {
                return ret;
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown system IRQ clean type: %d", clean_type);
            break;
    }
    
    return ESP_OK;
}

// [PM1_ADDR_BATT_LVP]
// 配置电池低压保护阈值 (2000-4000 mV)
// INPUIT: uint16_t lvp_mv (低压保护阈值 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
// 寄存器配置公式: lvp_mv = 2000mV + n * 7.81mV (n为写入寄存器的值)
esp_err_t pm1_batt_set_lvp(uint16_t lvp_mv)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_batt_set_lvp -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查电压范围
    if (lvp_mv < 2000 || lvp_mv > 4000) {
        ESP_LOGE(TAG, "Invalid battery LVP value: %u mV (valid range: 2000-4000 mV)", lvp_mv);
        return ESP_ERR_INVALID_ARG;
    }

    // 计算寄存器值
    uint8_t reg_value = (uint8_t)((lvp_mv - 2000) * 100 / 781);
    
    // 写入LVP寄存器
    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_BATT_LVP, reg_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set battery LVP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Battery LVP set to %u mV (reg value: 0x%02X)", lvp_mv, reg_value);
    
    return ESP_OK;
}

// [PM1_ADDR_VREF] Read Only
// 读取参考电压
// INPUIT: uint16_t *vref_mv (用来存储读取的参考电压值 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_vref_read(uint16_t *vref_mv)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_vref_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vref_mv == NULL) {
        ESP_LOGE(TAG, "VREF pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vref_l, vref_h;
    esp_err_t ret;

    // 读取VREF低字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_VREF_L, &vref_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VREF low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取VREF高字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_VREF_H, &vref_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VREF high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算电压值（单位mV
    *vref_mv = ((uint16_t)vref_h << 8) | vref_l;
    
    ESP_LOGI(TAG, "VREF: %u mV (raw: 0x%04X)", *vref_mv, *vref_mv);
    
    return ESP_OK;
}

// [PM1_ADDR_VBAT] Read Only
// 读取电池电压
// INPUIT: uint16_t *vbat_mv (用来存储读取的电池电压值 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_vbat_read(uint16_t *vbat_mv)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_vbat_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vbat_mv == NULL) {
        ESP_LOGE(TAG, "VBAT pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vbat_l, vbat_h;
    esp_err_t ret;

    // 读取电池电压低字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_VBAT_L, &vbat_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VBAT low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取电池电压高字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_VBAT_H, &vbat_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VBAT high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算电池电压值（单位mV
    *vbat_mv = ((uint16_t)vbat_h << 8) | vbat_l;
    
    ESP_LOGI(TAG, "Battery voltage: %u mV (raw: 0x%03X)", *vbat_mv, *vbat_mv);
    
    return ESP_OK;
}

// [PM1_ADDR_VIN] Read Only
// 读取VIN电压
// INPUIT: uint16_t *vin_mv (用来存储读取的VIN电压值 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_vin_read(uint16_t *vin_mv)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_vin_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vin_mv == NULL) {
        ESP_LOGE(TAG, "VIN pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vin_l, vin_h;
    esp_err_t ret;

    // 读取VIN电压低字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_VIN_L, &vin_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VIN low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取VIN电压高字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_VIN_H, &vin_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VIN high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算VIN电压值（单位mV
    *vin_mv = ((uint16_t)vin_h << 8) | vin_l;
    
    ESP_LOGI(TAG, "VIN voltage: %u mV (raw: 0x%03X)", *vin_mv, *vin_mv);
    
    return ESP_OK;
}

// [PM1_ADDR_5VINOUT] Read Only
// 读取5VINOUT(5VOUT)电压
// INPUIT: uint16_t *vinout_mv (用来存储读取的5VINOUT电压值 单位mV)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_5vinout_read(uint16_t *vinout_mv)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_5vinout_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vinout_mv == NULL) {
        ESP_LOGE(TAG, "5VINOUT pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t vinout_l, vinout_h;
    esp_err_t ret;

    // 读取5VINOUT电压低字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_5VINOUT_L, &vinout_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read 5VINOUT low byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取5VINOUT电压高字节
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_5VINOUT_H, &vinout_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read 5VINOUT high byte: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算5VINOUT电压值（单位mV
    *vinout_mv = ((uint16_t)vinout_h << 8) | vinout_l;
    
    ESP_LOGI(TAG, "5VINOUT voltage: %u mV (raw: 0x%03X)", *vinout_mv, *vinout_mv);
    
    return ESP_OK;
}

// [PM1_ADDR_PWR_SRC] Read Only
// 读取电源来源状态
// INPUIT: pm1_pwr_src_t *pwr_src (用来存储读取的电源来源状态)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_pwr_src_read(pm1_pwr_src_t *pwr_src)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_pwr_src_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pwr_src == NULL) {
        ESP_LOGE(TAG, "Power source pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pwr_src_reg;
    esp_err_t ret;

    // 读取电源来源寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_PWR_SRC, &pwr_src_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read power source register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 解析电源来源 (低3位)
    uint8_t src_bits = pwr_src_reg & 0x07;
    
    if (src_bits & (1 << 0)) {
        *pwr_src = PM1_PWR_SRC_5VIN;
    } else if (src_bits & (1 << 1)) {
        *pwr_src = PM1_PWR_SRC_5VINOUT;
    } else if (src_bits & (1 << 2)) {
        *pwr_src = PM1_PWR_SRC_BAT;
    } else {
        *pwr_src = PM1_PWR_SRC_UNKNOWN;
    }
    
    ESP_LOGI(TAG, "Power source: %d (reg value: 0x%02X, bits: %d)", 
             *pwr_src, pwr_src_reg, src_bits);
    
    return ESP_OK;
}

// [PM1_ADDR_WAKE_SRC] Read Only
// 读取唤醒来源状态
// INPUIT: pm1_wake_src_t *wake_src (用来存储读取的唤醒来源状态)
//         pm1_wake_flag_clean_type_t clean_type (唤醒来源清除类型)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_wake_src_read(pm1_wake_src_t *wake_src, pm1_wake_flag_clean_type_t clean_type)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_wake_src_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (wake_src == NULL) {
        ESP_LOGE(TAG, "Wake source pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t wake_src_reg;
    esp_err_t ret;

    // 读取唤醒源寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_WAKE_SRC, &wake_src_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read wake source register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 解析唤醒源
    uint8_t src_bits = wake_src_reg & 0x7F;  // 只关注低7位
    
    // 初始化为未知唤醒源
    *wake_src = PM1_WAKE_SRC_NULL;
    
    // 找到最低位置1的位（优先级：TIM > VIN > PWRBTN > RSTBTN > CMD_RST > EXT_WAKE > 5VINOUT）
    for (int i = 0; i <= 6; i++) {  // 更新范围到6，包括5VINOUT
        if (src_bits & (1 << i)) {
            // 返回对应的位掩码值
            *wake_src = (pm1_wake_src_t)(1 << i);
            break;
        }
    }
    
    ESP_LOGI(TAG, "Wake source: 0x%02X (reg value: 0x%02X, bits: 0x%02X)", 
             *wake_src, wake_src_reg, src_bits);
    
    // Clear wake flags based on clean_type
    switch (clean_type) {
        case PM1_ADDR_WAKE_FLAG_NOT_CLEAN:
            // 不清除标志
            break;
        case PM1_ADDR_WAKE_FLAG_ONCE_CLEAN:
            // 清除当前唤醒源标志
            if (*wake_src != PM1_WAKE_SRC_UNKNOWN && *wake_src != PM1_WAKE_SRC_NULL) {
                uint8_t new_val = wake_src_reg & ~(*wake_src);
                ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_WAKE_SRC, new_val);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to clear wake flag once: %s", esp_err_to_name(ret));
                    return ret;
                }
                ESP_LOGI(TAG, "Cleared wake flag for source 0x%02X", *wake_src);
            }
            break;
        case PM1_ADDR_WAKE_FLAG_ALL_CLEAN:
            // 清除所有唤醒源标志
            ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_WAKE_SRC, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to clear all wake flags: %s", esp_err_to_name(ret));
                return ret;
            }
            ESP_LOGI(TAG, "Cleared all wake flags");
            break;
        default:
            ESP_LOGW(TAG, "Unknown clean type: %d", clean_type);
            break;
    }
    
    return ESP_OK;
}

// [PM1_ADDR_SYS_CMD] Write Only
// 操作系统命令寄存器
// INPUIT: pm1_sys_cmd_t cmd (系统命令)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_sys_cmd(pm1_sys_cmd_t cmd)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_sys_cmd -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd > PM1_SYS_CMD_REBOOT) {
        ESP_LOGE(TAG, "Invalid system command: %d", cmd);
        return ESP_ERR_INVALID_ARG;
    }

    // 命令格式: [7-4] KEY(0xA) | [3-2] Reserved | [1-0] CMD
    uint8_t cmd_value = (0xA << 4) | (cmd & 0x03);
    
    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_SYS_CMD, cmd_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send system command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    const char *cmd_str;
    switch (cmd) {
        case PM1_SYS_CMD_NULL:
            cmd_str = "NULL";
            break;
        case PM1_SYS_CMD_SHUTDOWN:
            cmd_str = "SHUTDOWN";
            break;
        case PM1_SYS_CMD_REBOOT:
            cmd_str = "REBOOT";
            break;
        default:
            cmd_str = "UNKNOWN";
            break;
    }
    ESP_LOGI(TAG, "System command '%s' sent successfully", cmd_str);
    
    return ESP_OK;
}

// [PM1_ADDR_PWR_CFG]
// 配置电源管理参数（精确控制模式）
// INPUIT: uint8_t mask (要修改的位掩码, 位为1表示该位需要更新)
//         uint8_t value (与mask对应的目标值, 1=设置该位, 0=清除该位)
//         uint8_t *final_cfg (用于返回最终配置置值的指针，可以为NULL)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_pwr_set_cfg(uint8_t mask, uint8_t value, uint8_t *final_cfg)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_pwr_set_cfg -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t current_cfg;
    esp_err_t ret;

    // 先读取当前配置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_PWR_CFG, &current_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current power configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    // 确保只有有效位被设置 (bit 0-4)
    uint8_t valid_mask = mask & 0x1F;  // 只保留低5位的掩码
    uint8_t valid_value = value & 0x1F; // 只保留低5位的值
    
    // 精确控制配置步骤
    // 1. 清除掩码指定的位
    // 2. 设置掩码指定位的新值
    uint8_t new_cfg = (current_cfg & ~valid_mask) | (valid_value & valid_mask);
    
    // 写入新的配置置
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_PWR_CFG, new_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write power configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 等待1秒让配置生效
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 重新读取并验证配置是否成功
    uint8_t actual_cfg;
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_PWR_CFG, &actual_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to verify power configuration after write: %s", esp_err_to_name(ret));
        // 即使验证失败，也继续执行，因为写入可能已经成功
    } else {
        // 检查哪些位配置失败
        uint8_t expected_bits = new_cfg & valid_mask;
        uint8_t actual_bits = actual_cfg & valid_mask;
        
        if (expected_bits != actual_bits) {
            uint8_t failed_bits = expected_bits ^ actual_bits;
            ESP_LOGW(TAG, "Power configuration verification failed:");
            ESP_LOGW(TAG, "  Expected: 0x%02X, Actual: 0x%02X, Failed bits: 0x%02X", 
                     expected_bits, actual_bits, failed_bits);
            
            // 检查具体哪些功能配置失败
            if (failed_bits & PM1_PWR_CFG_CHG_EN) {
                ESP_LOGW(TAG, "  CHG_EN configuration failed");
            }
            if (failed_bits & PM1_PWR_CFG_DCDC_EN) {
                ESP_LOGW(TAG, "  DCDC_EN configuration failed");
            }
            if (failed_bits & PM1_PWR_CFG_LDO_EN) {
                ESP_LOGW(TAG, "  LDO_EN configuration failed");
            }
            if (failed_bits & PM1_PWR_CFG_5V_INOUT) {
                ESP_LOGW(TAG, "  5V_INOUT configuration failed");
            }
            if (failed_bits & PM1_PWR_CFG_LED_CONTROL) {
                ESP_LOGW(TAG, "  LED_CONTROL configuration failed");
            }
        } else {
            ESP_LOGI(TAG, "Power configuration verified successfully");
        }
        
        // 更新final_cfg为实际读取的值
        new_cfg = actual_cfg;
    }
    
    // 如果提供final_cfg指针则返回最终配置
    if (final_cfg != NULL) {
        *final_cfg = new_cfg;
    }
    
    // 记录配置置变化
    ESP_LOGI(TAG, "Power configuration updated:");
    ESP_LOGI(TAG, "  Previous: 0x%02X, Mask: 0x%02X, Value: 0x%02X, Final: 0x%02X", 
             current_cfg, valid_mask, valid_value, new_cfg);
    ESP_LOGI(TAG, "  CHG_EN: %s", (new_cfg & PM1_PWR_CFG_CHG_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  DCDC_EN: %s", (new_cfg & PM1_PWR_CFG_DCDC_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  LDO_EN: %s", (new_cfg & PM1_PWR_CFG_LDO_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  5V_INOUT: %s", (new_cfg & PM1_PWR_CFG_5V_INOUT) ? "output mode" : "input mode");
    ESP_LOGI(TAG, "  LED_CONTROL: %s", (new_cfg & PM1_PWR_CFG_LED_CONTROL) ? "enabled" : "disabled");
    
    return ESP_OK;
}

// [PM1_ADDR_PWR_CFG]
// 清除电源管理参数中的特定位
// INPUIT: uint8_t cfg (要清除的电源配置置位掩码)
//         uint8_t *final_cfg (用于返回最终配置置值的指针，可以为NULL)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_pwr_clear_cfg(uint8_t cfg, uint8_t *final_cfg)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_pwr_clear_cfg -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t current_cfg;
    esp_err_t ret;

    // 先读取当前配置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_PWR_CFG, &current_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current power configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    // 确保只有有效位被清除 (bit 0-4)
    uint8_t valid_cfg = cfg & 0x1F;  // 只保留低5位
    
    // 清除配置：将指定位从当前配置中清除
    uint8_t cleared_cfg = current_cfg & (~valid_cfg);
    
    // 写入清除后的配置置
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_PWR_CFG, cleared_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write power configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 若提供final_cfg则写回最终配置
    if (final_cfg != NULL) {
        *final_cfg = cleared_cfg;
    }
    
    ESP_LOGI(TAG, "Power configuration cleared:");
    ESP_LOGI(TAG, "  Previous: 0x%02X, Cleared: 0x%02X, Final: 0x%02X", current_cfg, valid_cfg, cleared_cfg);
    
    return ESP_OK;
}

// [PM1_ADDR_PWR_CFG]
// 读取当前电源管理配置
// INPUIT: uint8_t *current_cfg (用于返回当前配置置值的指针)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_pwr_get_cfg(uint8_t *current_cfg)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_pwr_get_cfg -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (current_cfg == NULL) {
        ESP_LOGE(TAG, "Configuration pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_PWR_CFG, current_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read power configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Current power configuration: 0x%02X", *current_cfg);
    ESP_LOGI(TAG, "  CHG_EN: %s", (*current_cfg & PM1_PWR_CFG_CHG_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  BOOST_EN: %s", (*current_cfg & PM1_PWR_CFG_DCDC_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  LDO_HOLD: %s", (*current_cfg & PM1_PWR_CFG_LDO_EN) ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  5V_VIN: %s", (*current_cfg & PM1_PWR_CFG_5V_INOUT) ? "output mode" : "input mode");
    ESP_LOGI(TAG, "  LED_CONTROL: %s", (*current_cfg & PM1_PWR_CFG_LED_CONTROL) ? "enabled" : "disabled");
    
    return ESP_OK;
}

// [PM1_ADDR_NEO_REFRESH]
// 刷新NeoPixel数据
// INPUIT: NULL
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_neo_refresh(void)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_neo_refresh -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // 检查GPIO0是否配置为NeoPixel复用功能
    if (pm1_pin_status[PM1_GPIO_NUM_0].pin_func != PM1_GPIO_FUNC_OTHER) {
        ESP_LOGE(TAG, "GPIO0 not configured for NeoPixel function");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t neo_cfg;
    esp_err_t ret;

    // 读取当前NeoPixel配置
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_NEO_CFG, &neo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read NeoPixel configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 设置REFRESH位(bit6)
    neo_cfg |= (1 << 6);
    
    // 写入配置置以触发刷新
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_NEO_CFG, neo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger NeoPixel refresh: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "NeoPixel refresh triggered successfully");
    
    return ESP_OK;
}

// [PM1_ADDR_NEO_CFG]
// 配置NeoPixel参数
// INPUIT: uint8_t neo_num (NeoPixel数量, 1-32)
//         uint16_t *neo_data (NeoPixel数据, RGB565格式, 每个像素16位)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_neo_set_cfg(uint8_t neo_num, uint16_t *neo_data, uint8_t refresh_flag)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_neo_set_cfg -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (neo_num > 32) {
        ESP_LOGE(TAG, "Invalid NeoPixel count: %d (max 32)", neo_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (neo_num > 0 && neo_data == NULL) {
        ESP_LOGE(TAG, "NeoPixel data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 设置NeoPixel数量
    uint8_t neo_cfg = neo_num & 0x1F;  // 仅使用低5位
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_NEO_CFG, neo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NeoPixel count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 如果有像素数据，写入数据
    if (neo_num > 0) {
        // 每个像素占用2字节(RGB565格式)
        for (int i = 0; i < neo_num; i++) {
            uint8_t data_l = neo_data[i] & 0xFF;          // 低字节
            uint8_t data_h = (neo_data[i] >> 8) & 0xFF;   // 高字节
            
            // 写入低字节
            ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_NEO_PIXn_ADDR_START + (i * 2), data_l);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write NeoPixel %d low byte: %s", i, esp_err_to_name(ret));
                return ret;
            }
            
            // 写入高字节
            ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_NEO_PIXn_ADDR_START + (i * 2) + 1, data_h);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write NeoPixel %d high byte: %s", i, esp_err_to_name(ret));
                return ret;
            }
        }
    }
    
    ESP_LOGI(TAG, "NeoPixel configuration set: %d LEDs", neo_num);
    
    // 如果需要立即刷新
    if (refresh_flag) {
        ret = pm1_neo_refresh();
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    return ESP_OK;
}

// [PM1_ADDR_RTC_RAM_WRITE]
// 写RTC RAM
// INPUIT: uint8_t rtc_ram_addr_start (RTC RAM起始地址, 0-31, 实际地址=基址+偏移)
//          uint8_t len (数据长度, 1-32)
//          uint8_t *data (数据缓冲区, 长度=len)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_rtc_ram_write(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_rtc_ram_write -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rtc_ram_addr_start > 31) {
        ESP_LOGE(TAG, "Invalid RTC RAM start address: %d (max 31)", rtc_ram_addr_start);
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0 || len > 32 || rtc_ram_addr_start + len > 32) {
        ESP_LOGE(TAG, "Invalid data length: %d (1-32, and must not exceed address space)", len);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    
    // 写入RTC RAM
    for (int i = 0; i < len; i++) {
        uint8_t addr = PM1_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start + i;
        ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, addr, data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write RTC RAM at address 0x%02X: %s", addr, esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Wrote %d bytes to RTC RAM starting at address %d (0x%02X)", 
             len, rtc_ram_addr_start, PM1_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start);
    
    return ESP_OK;
}

// [PM1_ADDR_RTC_RAM_READ]
// 读RTC RAM
// INPUIT: uint8_t rtc_ram_addr_start (RTC RAM起始地址, 0-31, 实际地址=基址+偏移)
//          uint8_t len (数据长度, 1-32)
//          uint8_t *data (数据缓冲区, 长度=len)
// OUTPUIT: ESP_OK on success, error code on failure
esp_err_t pm1_rtc_ram_read(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_rtc_ram_read -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rtc_ram_addr_start > 31) {
        ESP_LOGE(TAG, "Invalid RTC RAM start address: %d (max 31)", rtc_ram_addr_start);
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0 || len > 32 || rtc_ram_addr_start + len > 32) {
        ESP_LOGE(TAG, "Invalid data length: %d (1-32, and must not exceed address space)", len);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    
    // 读取RTC RAM
    for (int i = 0; i < len; i++) {
        uint8_t addr = PM1_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start + i;
        ret = PM1_I2C_READ_BYTE(pm1_i2c_device, addr, &data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read RTC RAM at address 0x%02X: %s", addr, esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Read %d bytes from RTC RAM starting at address %d (0x%02X)", 
             len, rtc_ram_addr_start, PM1_ADDR_RTC_MEM_ADDR_START + rtc_ram_addr_start);
    
    return ESP_OK;
}

// =======================================
// GPIO Power Hold Functions
// =======================================

// [PM1_ADDR_GPIO_POWER_HOLD]
// 设置单个GPIO电源保持状态
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号, 0-4)
//        pm1_gpio_power_hold_t power_hold (电源保持状态: 使能/禁用)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: 设置GPIO在关机后是否保持状态
esp_err_t pm1_gpio_set_power_hold(pm1_gpio_num_t gpio_num, pm1_gpio_power_hold_t power_hold)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_set_power_hold -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t current_mask;

    // 读取当前电源保持寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO power hold register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Current GPIO power hold mask: 0x%02X (GPIO%d bit: %s)", 
             current_mask, gpio_num, (current_mask & (1 << gpio_num)) ? "Enabled" : "Disabled");

    // 修改对应GPIO的电源保持位
    if (power_hold == PM1_GPIO_POWER_HOLD_ENABLE) {
        current_mask |= (1 << gpio_num);   // 设置为使能
    } else {
        current_mask &= ~(1 << gpio_num);  // 设置为禁用
    }

    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO power hold register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set GPIO%d power hold to %s: 0x%02X", 
             gpio_num, 
             (power_hold == PM1_GPIO_POWER_HOLD_ENABLE) ? "Enabled" : "Disabled", 
             current_mask);

    // 更新引脚状态记录
    pm1_pin_status[gpio_num].power_hold = power_hold;

    return ESP_OK;
}

// [PM1_ADDR_GPIO_POWER_HOLD]
// 获取单个GPIO电源保持状态
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号, 0-4)
//        pm1_gpio_power_hold_t *power_hold (电源保持状态指针)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_gpio_get_power_hold(pm1_gpio_num_t gpio_num, pm1_gpio_power_hold_t *power_hold)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_gpio_get_power_hold -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (power_hold == NULL) {
        ESP_LOGE(TAG, "Power hold pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t current_mask;

    // 读取当前电源保持寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO power hold register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查对应GPIO的电源保持位
    *power_hold = (current_mask & (1 << gpio_num)) ? PM1_GPIO_POWER_HOLD_ENABLE : PM1_GPIO_POWER_HOLD_DISABLE;

    ESP_LOGI(TAG, "GPIO%d power hold status: %s (mask: 0x%02X)", 
             gpio_num, 
             (*power_hold == PM1_GPIO_POWER_HOLD_ENABLE) ? "Enabled" : "Disabled", 
             current_mask);

    return ESP_OK;
}

// =======================================
// LDO Power Hold Functions
// =======================================

// [PM1_ADDR_GPIO_POWER_HOLD]
// 设置3.3V LDO电源保持状态
// INPUT: bool enable (true: 使能LDO电源保持, false: 禁用LDO电源保持)
// OUTPUT: ESP_OK on success, error code on failure
// TIPS: 控制3.3V LDO在关机后是否保持输出
esp_err_t pm1_ldo_set_power_hold(bool enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_ldo_set_power_hold -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    uint8_t current_mask;

    // 读取当前电源保持寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO power hold register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Current power hold mask: 0x%02X (LDO bit5: %s)", 
             current_mask, (current_mask & (1 << 5)) ? "Enabled" : "Disabled");

    // 修改第5位（LDO电源保持位）
    if (enable) {
        current_mask |= (1 << 5);   // 设置第5位为1，使能LDO电源保持
    } else {
        current_mask &= ~(1 << 5);  // 设置第5位为0，禁用LDO电源保持
    }

    // 写入修改后的值
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO power hold register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set 3.3V LDO power hold to %s: 0x%02X", 
             enable ? "Enabled" : "Disabled", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_GPIO_POWER_HOLD]
// 获取3.3V LDO电源保持状态
// INPUT: bool *enable (LDO电源保持状态指针)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_ldo_get_power_hold(bool *enable)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_ldo_get_power_hold -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (enable == NULL) {
        ESP_LOGE(TAG, "Enable pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t current_mask;

    // 读取当前电源保持寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_GPIO_POWER_HOLD, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO power hold register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查第5位（LDO电源保持位）
    *enable = (current_mask & (1 << 5)) ? true : false;

    ESP_LOGI(TAG, "3.3V LDO power hold status: %s (mask: 0x%02X)", 
             *enable ? "Enabled" : "Disabled", current_mask);

    return ESP_OK;
}

// =============================================================================
// IRQ MASK Functions Implementation
// =============================================================================

// [PM1_ADDR_IRQ_STATUS3]
// 读取按键中断状态
// INPUT: pm1_irq_btn_t *btn_irq (按键中断类型)
//        pm1_irq_btn_clean_type_t clean_type (中断清除类型)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_btn_status(pm1_irq_btn_t *btn_irq, pm1_irq_btn_clean_type_t clean_type)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_btn_status -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (btn_irq == NULL) {
        ESP_LOGE(TAG, "Button IRQ pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t irq_status;
    esp_err_t ret;

    // 读取按键中断状态寄存器
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3, &irq_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button IRQ status register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查哪个按键中断状态位被设置 [2:Double click 1:Wakeup 0:Click]
    *btn_irq = PM1_ADDR_IRQ_BTN_NULL;  // 初始化为无效值
    if (irq_status & (1 << 0)) {
        *btn_irq = PM1_ADDR_IRQ_BTN_CLICK;
    } else if (irq_status & (1 << 1)) {
        *btn_irq = PM1_ADDR_IRQ_BTN_WAKEUP;
    } else if (irq_status & (1 << 2)) {
        *btn_irq = PM1_ADDR_IRQ_BTN_DOUBLE_CLICK;
    }

    ESP_LOGI(TAG, "Button IRQ status: 0x%02X, event: %d", 
             irq_status, (*btn_irq == PM1_ADDR_IRQ_BTN_NULL) ? -1 : (int)*btn_irq);
    
    // Clear button IRQ based on clean_type
    switch (clean_type) {
        case PM1_ADDR_IRQ_BTN_NOT_CLEAN:
            // 不清除中断，直接返回
            break;
        case PM1_ADDR_IRQ_BTN_ONCE_CLEAN:
            // 清除指定按钮中断
            if (*btn_irq != PM1_ADDR_IRQ_BTN_NULL) {
                ret = pm1_irq_clear_btn_status(*btn_irq);
                if (ret != ESP_OK) {
                    return ret;
                }
            }
            break;
        case PM1_ADDR_IRQ_BTN_ALL_CLEAN:
            // 清除所有按钮中断
            ret = pm1_irq_clear_btn_status(PM1_ADDR_IRQ_BTN_ALL);
            if (ret != ESP_OK) {
                return ret;
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown button IRQ clean type: %d", clean_type);
            break;
    }
    
    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS1_MASK]
// 设置单个GPIO中断屏蔽
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号)
//        pm1_irq_mask_ctrl_t mask_ctrl (屏蔽控制：0=允许中断，1=屏蔽中断)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_set_gpio_mask(pm1_gpio_num_t gpio_num, pm1_irq_mask_ctrl_t mask_ctrl)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_set_gpio_mask -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_mask;
    esp_err_t ret;

    // 读取当前GPIO中断屏蔽寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1_MASK, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应GPIO的屏蔽位
    if (mask_ctrl == PM1_IRQ_MASK_ENABLE) {
        current_mask |= (1 << gpio_num);  // 设置屏蔽位为1（屏蔽中断）
    } else {
        current_mask &= ~(1 << gpio_num); // 清除屏蔽位为0（允许中断）
    }

    // 写回寄存器
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1_MASK, current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set GPIO%d IRQ mask to %s (mask register: 0x%02X)", 
             gpio_num, (mask_ctrl == PM1_IRQ_MASK_ENABLE) ? "ENABLED" : "DISABLED", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS1_MASK]
// 获取单个GPIO中断屏蔽状态
// INPUT: pm1_gpio_num_t gpio_num (GPIO编号)
//        pm1_irq_mask_ctrl_t *mask_ctrl (屏蔽控制状态)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_gpio_mask(pm1_gpio_num_t gpio_num, pm1_irq_mask_ctrl_t *mask_ctrl)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_gpio_mask -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_num > PM1_GPIO_NUM_4 || mask_ctrl == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: gpio_num=%d, mask_ctrl=%p", gpio_num, mask_ctrl);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_mask;
    esp_err_t ret;

    // 读取当前GPIO中断屏蔽寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1_MASK, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查对应GPIO的屏蔽位
    *mask_ctrl = (current_mask & (1 << gpio_num)) ? PM1_IRQ_MASK_ENABLE : PM1_IRQ_MASK_DISABLE;

    ESP_LOGI(TAG, "GPIO%d IRQ mask status: %s (mask register: 0x%02X)", 
             gpio_num, (*mask_ctrl == PM1_IRQ_MASK_ENABLE) ? "ENABLED" : "DISABLED", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS1_MASK]
// 设置所有GPIO中断屏蔽
// INPUT: uint8_t mask_value (屏蔽值：位=1屏蔽对应中断)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_set_gpio_mask_all(uint8_t mask_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_set_gpio_mask_all -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 只保留低5位，高3位为保留位
    mask_value &= 0x1F;

    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1_MASK, mask_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write GPIO IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set all GPIO IRQ mask: 0x%02X", mask_value);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS1_MASK]
// 获取所有GPIO中断屏蔽状态
// INPUT: uint8_t *mask_value (屏蔽值)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_gpio_mask_all(uint8_t *mask_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_gpio_mask_all -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mask_value == NULL) {
        ESP_LOGE(TAG, "Mask value pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS1_MASK, mask_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All GPIO IRQ mask: 0x%02X", *mask_value);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS2_MASK]
// 设置单个系统中断屏蔽
// INPUT: pm1_irq_sys_t sys_irq (系统中断类型)
//        pm1_irq_mask_ctrl_t mask_ctrl (屏蔽控制：0=允许中断，1=屏蔽中断)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_set_sys_mask(pm1_irq_sys_t sys_irq, pm1_irq_mask_ctrl_t mask_ctrl)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_set_sys_mask -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sys_irq > PM1_ADDR_IRQ_SYS_BAT_REMOVE) {
        ESP_LOGE(TAG, "Invalid system IRQ type: %d", sys_irq);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_mask;
    esp_err_t ret;

    // 读取当前系统中断屏蔽寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2_MASK, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read system IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应系统中断的屏蔽位
    if (mask_ctrl == PM1_IRQ_MASK_ENABLE) {
        current_mask |= (1 << sys_irq);  // 设置屏蔽位为1（屏蔽中断）
    } else {
        current_mask &= ~(1 << sys_irq); // 清除屏蔽位为0（允许中断）
    }

    // 写回寄存器
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2_MASK, current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write system IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set system IRQ%d mask to %s (mask register: 0x%02X)", 
             sys_irq, (mask_ctrl == PM1_IRQ_MASK_ENABLE) ? "ENABLED" : "DISABLED", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS2_MASK]
// 获取单个系统中断屏蔽状态
// INPUT: pm1_irq_sys_t sys_irq (系统中断类型)
//        pm1_irq_mask_ctrl_t *mask_ctrl (屏蔽控制状态)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_sys_mask(pm1_irq_sys_t sys_irq, pm1_irq_mask_ctrl_t *mask_ctrl)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_sys_mask -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sys_irq > PM1_ADDR_IRQ_SYS_BAT_REMOVE || mask_ctrl == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: sys_irq=%d, mask_ctrl=%p", sys_irq, mask_ctrl);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_mask;
    esp_err_t ret;

    // 读取当前系统中断屏蔽寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2_MASK, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read system IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查对应系统中断的屏蔽位
    *mask_ctrl = (current_mask & (1 << sys_irq)) ? PM1_IRQ_MASK_ENABLE : PM1_IRQ_MASK_DISABLE;

    ESP_LOGI(TAG, "System IRQ%d mask status: %s (mask register: 0x%02X)", 
             sys_irq, (*mask_ctrl == PM1_IRQ_MASK_ENABLE) ? "ENABLED" : "DISABLED", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS2_MASK]
// 设置所有系统中断屏蔽
// INPUT: uint8_t mask_value (屏蔽值：位=1屏蔽对应中断)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_set_sys_mask_all(uint8_t mask_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_set_sys_mask_all -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 只保留低6位，高2位为保留位
    mask_value &= 0x3F;

    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2_MASK, mask_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write system IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set all system IRQ mask: 0x%02X", mask_value);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS2_MASK]
// 获取所有系统中断屏蔽状态
// INPUT: uint8_t *mask_value (屏蔽值)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_sys_mask_all(uint8_t *mask_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_sys_mask_all -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mask_value == NULL) {
        ESP_LOGE(TAG, "Mask value pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS2_MASK, mask_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read system IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All system IRQ mask: 0x%02X", *mask_value);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS3_MASK]
// 设置单个按键中断屏蔽
// INPUT: pm1_irq_btn_t btn_irq (按键中断类型)
//        pm1_irq_mask_ctrl_t mask_ctrl (屏蔽控制：0=允许中断，1=屏蔽中断)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_set_btn_mask(pm1_irq_btn_t btn_irq, pm1_irq_mask_ctrl_t mask_ctrl)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_set_btn_mask -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (btn_irq > PM1_ADDR_IRQ_BTN_DOUBLE_CLICK) {
        ESP_LOGE(TAG, "Invalid button IRQ type: %d", btn_irq);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_mask;
    esp_err_t ret;

    // 读取当前按键中断屏蔽寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3_MASK, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置对应按键中断的屏蔽位
    if (mask_ctrl == PM1_IRQ_MASK_ENABLE) {
        current_mask |= (1 << btn_irq);  // 设置屏蔽位为1（屏蔽中断）
    } else {
        current_mask &= ~(1 << btn_irq); // 清除屏蔽位为0（允许中断）
    }

    // 写回寄存器
    ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3_MASK, current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write button IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set button IRQ%d mask to %s (mask register: 0x%02X)", 
             btn_irq, (mask_ctrl == PM1_IRQ_MASK_ENABLE) ? "ENABLED" : "DISABLED", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS3_MASK]
// 获取单个按键中断屏蔽状态
// INPUT: pm1_irq_btn_t btn_irq (按键中断类型)
//        pm1_irq_mask_ctrl_t *mask_ctrl (屏蔽控制状态)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_btn_mask(pm1_irq_btn_t btn_irq, pm1_irq_mask_ctrl_t *mask_ctrl)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_btn_mask -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (btn_irq > PM1_ADDR_IRQ_BTN_DOUBLE_CLICK || mask_ctrl == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: btn_irq=%d, mask_ctrl=%p", btn_irq, mask_ctrl);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_mask;
    esp_err_t ret;

    // 读取当前按键中断屏蔽寄存器值
    ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3_MASK, &current_mask);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查对应按键中断的屏蔽位
    *mask_ctrl = (current_mask & (1 << btn_irq)) ? PM1_IRQ_MASK_ENABLE : PM1_IRQ_MASK_DISABLE;

    ESP_LOGI(TAG, "Button IRQ%d mask status: %s (mask register: 0x%02X)", 
             btn_irq, (*mask_ctrl == PM1_IRQ_MASK_ENABLE) ? "ENABLED" : "DISABLED", current_mask);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS3_MASK]
// 设置所有按键中断屏蔽
// INPUT: uint8_t mask_value (屏蔽值：位=1屏蔽对应中断)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_set_btn_mask_all(uint8_t mask_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_set_btn_mask_all -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 只保留低3位，高5位为保留位
    mask_value &= 0x07;

    esp_err_t ret = PM1_I2C_WRITE_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3_MASK, mask_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write button IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Set all button IRQ mask: 0x%02X", mask_value);

    return ESP_OK;
}

// [PM1_ADDR_IRQ_STATUS3_MASK]
// 获取所有按键中断屏蔽状态
// INPUT: uint8_t *mask_value (屏蔽值)
// OUTPUT: ESP_OK on success, error code on failure
esp_err_t pm1_irq_get_btn_mask_all(uint8_t *mask_value)
{
    if (pm1_i2c_device == NULL) {
        ESP_LOGE(TAG, "PM1_irq_get_btn_mask_all -> PM1 I2C device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mask_value == NULL) {
        ESP_LOGE(TAG, "Mask value pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = PM1_I2C_READ_BYTE(pm1_i2c_device, PM1_ADDR_IRQ_STATUS3_MASK, mask_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read button IRQ mask register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All button IRQ mask: 0x%02X", *mask_value);

    return ESP_OK;
}
