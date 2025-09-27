/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "M5_io_expander.h"

// 无参构造函数，用于后续配置
m5_io_expander::m5_io_expander() {
    // 设置默认值
    this->i2c_port = I2C_NUM_0;
    this->i2c_sda_pin = GPIO_NUM_NC;
    this->i2c_scl_pin = GPIO_NUM_NC;
    this->i2c_freq_hz = 400000;
    this->_device_handle_external = false;
    this->_bus_handle_external = false;
    this->_i2c_bus_handle = NULL;
    this->_i2c_bus_device_handle = NULL;
    
    ESP_LOGI(_M5_io_expander_TAG, "M5 IO Expander created (no parameters), use set_i2c_config() to configure");
}

m5_io_expander::m5_io_expander(gpio_num_t i2c_sda_pin, gpio_num_t i2c_scl_pin, uint32_t i2c_freq_hz, i2c_port_t i2c_port) {

    //save the parameters to the class members
    this->i2c_port = i2c_port;
    this->i2c_sda_pin = i2c_sda_pin;
    this->i2c_scl_pin = i2c_scl_pin;
    this->i2c_freq_hz = i2c_freq_hz;
    this->_device_handle_external = false;
    this->_bus_handle_external = false;

    // Create I2C bus configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = i2c_freq_hz,
        },
        .clk_flags = 0,
    };
    // Create the I2C bus
    _i2c_bus_handle = i2c_bus_create(i2c_port, &conf);
    if (_i2c_bus_handle == NULL) {
        // Handle error: failed to create I2C bus
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create I2C bus");
        return;
    }
    else {
        ESP_LOGI(_M5_io_expander_TAG, "I2C bus created successfully");
    }
}

// 支持传入现有总线句柄和设备句柄的构造函数
m5_io_expander::m5_io_expander(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle) {

    // 设置为默认值
    this->i2c_port = I2C_NUM_0;
    this->i2c_sda_pin = GPIO_NUM_NC;
    this->i2c_scl_pin = GPIO_NUM_NC;
    this->i2c_freq_hz = 400000;
    
    // 处理总线句柄
    this->_i2c_bus_handle = existing_bus_handle;
    this->_bus_handle_external = (existing_bus_handle != NULL);
    
    // 处理设备句柄
    this->_i2c_bus_device_handle = existing_device_handle;
    this->_device_handle_external = (existing_device_handle != NULL);
    
    if (existing_bus_handle != NULL) {
        ESP_LOGI(_M5_io_expander_TAG, "Using external bus handle: %p", existing_bus_handle);
    } else {
        ESP_LOGW(_M5_io_expander_TAG, "External bus handle is NULL, will need to create one later");
    }
    
    if (existing_device_handle != NULL) {
        ESP_LOGI(_M5_io_expander_TAG, "Using external device handle: %p", existing_device_handle);
    } else {
        ESP_LOGI(_M5_io_expander_TAG, "External device handle is NULL, will create when needed");
    }
}

m5_io_expander::~m5_io_expander() {
    // Destructor I2C bus cleanup
    // 只有非外部传入的设备句柄才需要删除
    if (_i2c_bus_device_handle != NULL && !_device_handle_external) {
        i2c_bus_device_delete(&_i2c_bus_device_handle);
        _i2c_bus_device_handle = NULL;
    }
    
    // 只有非外部传入的总线句柄才需要删除
    if (_i2c_bus_handle != NULL && !_bus_handle_external) {
        i2c_bus_delete(&_i2c_bus_handle);
        _i2c_bus_handle = NULL;
    }
    ESP_LOGI(_M5_io_expander_TAG, "I2C resources cleaned up successfully");
}

void m5_io_expander::cleanup() {
    // Cleanup I2C bus
    // 只有非外部传入的设备句柄才需要删除
    if (_i2c_bus_device_handle != NULL && !_device_handle_external) {
        i2c_bus_device_delete(&_i2c_bus_device_handle);
        _i2c_bus_device_handle = NULL;
    }
    
    // 只有非外部传入的总线句柄才需要删除
    if (_i2c_bus_handle != NULL && !_bus_handle_external) {
        i2c_bus_delete(&_i2c_bus_handle);
        _i2c_bus_handle = NULL;
    }
    ESP_LOGI(_M5_io_expander_TAG, "Program error, I2C resources cleaned up successfully");
}

void m5_io_expander::set_log_level(esp_log_level_t level) {
    esp_log_level_set(_M5_io_expander_TAG, level);
    ESP_LOGI(_M5_io_expander_TAG, "Log level set to %d", level);
}

void m5_io_expander::init() {
    ESP_LOGI(_M5_io_expander_TAG, "Initializing M5 IO Expander...");
}

// I2C配置管理功能
void m5_io_expander::set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num) {
    ESP_LOGI(_M5_io_expander_TAG, "Setting I2C config: SDA=%d, SCL=%d, Freq=%lu, Port=%d", sda_pin, scl_pin, frequency, port_num);
    
    // 清理现有资源（如果是内部创建的）
    if (_i2c_bus_device_handle != NULL && !_device_handle_external) {
        i2c_bus_device_delete(&_i2c_bus_device_handle);
        _i2c_bus_device_handle = NULL;
    }
    
    if (_i2c_bus_handle != NULL && !_bus_handle_external) {
        i2c_bus_delete(&_i2c_bus_handle);
        _i2c_bus_handle = NULL;
    }
    
    // 更新参数
    this->i2c_sda_pin = sda_pin;
    this->i2c_scl_pin = scl_pin;
    this->i2c_freq_hz = frequency;
    this->i2c_port = port_num;
    
    // 验证参数
    if (sda_pin == GPIO_NUM_NC || scl_pin == GPIO_NUM_NC) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid GPIO pins: SDA=%d, SCL=%d", sda_pin, scl_pin);
        return;
    }
    
    // 重新创建I2C总线
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = frequency,
        },
        .clk_flags = 0,
    };
    
    _i2c_bus_handle = i2c_bus_create(port_num, &conf);
    if (_i2c_bus_handle == NULL) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create I2C bus after config update");
        return;
    }
    
    // 标记为内部创建
    _bus_handle_external = false;
    _device_handle_external = false;
    
    ESP_LOGI(_M5_io_expander_TAG, "I2C bus created successfully with new config");
}

void m5_io_expander::set_i2c_config(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle) {
    // 清理当前资源（如果是内部创建的）
    if (_i2c_bus_device_handle != NULL && !_device_handle_external) {
        i2c_bus_device_delete(&_i2c_bus_device_handle);
        _i2c_bus_device_handle = NULL;
    }
    
    if (_i2c_bus_handle != NULL && !_bus_handle_external) {
        i2c_bus_delete(&_i2c_bus_handle);
        _i2c_bus_handle = NULL;
    }
    
    // 设置新的句柄
    this->_i2c_bus_handle = existing_bus_handle;
    this->_bus_handle_external = (existing_bus_handle != NULL);
    
    this->_i2c_bus_device_handle = existing_device_handle;
    this->_device_handle_external = (existing_device_handle != NULL);
    
    // 重置其他参数为默认值
    this->i2c_port = I2C_NUM_0;
    this->i2c_sda_pin = GPIO_NUM_NC;
    this->i2c_scl_pin = GPIO_NUM_NC;
    this->i2c_freq_hz = 400000;
    
    ESP_LOGI(_M5_io_expander_TAG, "I2C config updated with external handles: bus=%p, device=%p", 
             existing_bus_handle, existing_device_handle);
}

// 创建设备句柄的方法（支持双向传递）
i2c_bus_device_handle_t m5_io_expander::create_device_handle(uint8_t i2c_addr) {
    // 如果已经有设备句柄，直接返回
    if (_i2c_bus_device_handle != NULL) {
        ESP_LOGI(_M5_io_expander_TAG, "Using existing device handle for address 0x%02X", i2c_addr);
        return _i2c_bus_device_handle;
    }
    
    // 确保有I2C总线句柄
    if (_i2c_bus_handle == NULL) {
        ESP_LOGE(_M5_io_expander_TAG, "I2C bus handle is NULL, cannot create device handle");
        ESP_LOGE(_M5_io_expander_TAG, "Please provide bus handle when using external handle constructor, or use the GPIO constructor");
        return NULL;
    }
    
    // 创建新的设备句柄
    _i2c_bus_device_handle = i2c_bus_device_create(_i2c_bus_handle, i2c_addr, i2c_freq_hz);
    if (_i2c_bus_device_handle == NULL) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create I2C bus device handle for address 0x%02X", i2c_addr);
        return NULL;
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "I2C bus device handle created successfully for address 0x%02X", i2c_addr);
    return _i2c_bus_device_handle;
}