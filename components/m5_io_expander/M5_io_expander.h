/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef M5_IO_EXPANDER_H
#define M5_IO_EXPANDER_H

#include "i2c_bus.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

#ifndef INPUT
#define INPUT 0x01
#endif
#ifndef OUTPUT
#define OUTPUT 0x03 //参考arduinoespressif32的定义,但M5_io_expander将支持 0x02 和 0x03 混合使用
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x05
#endif
#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN 0x09
#endif
#ifndef HIGH
#define HIGH 0x01
#endif
#ifndef LOW
#define LOW 0x00
#endif
#ifndef ENABLE
#define ENABLE 0x01
#endif
#ifndef DISABLE
#define DISABLE 0x00
#endif
#ifndef PULL_UP
#define PULL_UP 0x01
#endif
#ifndef PULL_DOWN
#define PULL_DOWN 0x00
#endif
#ifndef RISING
#define RISING 0x01
#endif
#ifndef FALLING
#define FALLING 0x02
#endif


#define M5_IO_INPUT INPUT
#define M5_IO_OUTPUT OUTPUT
#define M5_IO_INPUT_PULLUP INPUT_PULLUP
#define M5_IO_INPUT_PULLDOWN INPUT_PULLDOWN
#define M5_IO_HIGH HIGH
#define M5_IO_LOW LOW
#define M5_IO_ENABLE ENABLE
#define M5_IO_DISABLE DISABLE
#define M5_IO_PULL_UP PULL_UP
#define M5_IO_PULL_DOWN PULL_DOWN
#define M5_IO_RISING RISING
#define M5_IO_FALLING FALLING


class m5_io_expander {
public:
    m5_io_expander(); // 无参构造函数，用于后续配置
    m5_io_expander(gpio_num_t i2c_sda_pin, gpio_num_t i2c_scl_pin, uint32_t i2c_freq_hz, i2c_port_t i2c_port);
    m5_io_expander(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle);
    ~m5_io_expander();
    void cleanup(); // 主动清理函数

    // 设置日志级别（仅对该模块生效）
    void set_log_level(esp_log_level_t level);

    // 初始化
    virtual void init();
    virtual void init(uint8_t i2c_addr) = 0;
    
    // 创建设备句柄的方法（支持双向传递）
    virtual i2c_bus_device_handle_t create_device_handle(uint8_t i2c_addr);
    
    // 获取当前设备句柄
    i2c_bus_device_handle_t get_device_handle() const { return _i2c_bus_device_handle; }
    
    // 获取当前总线句柄
    i2c_bus_handle_t get_bus_handle() const { return _i2c_bus_handle; }

    // I2C配置管理功能
    void set_i2c_config(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency, i2c_port_t port_num);
    void set_i2c_config(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle);

    //  用户接口（Arduino形式）
    virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
    virtual void digitalWrite(uint8_t pin, uint8_t value) = 0;
    virtual int digitalRead(uint8_t pin) = 0;
    virtual void attachInterrupt(uint8_t pin, void (*callback)(void), uint8_t mode) = 0;
    virtual void attachInterruptArg(uint8_t pin, void (*callback)(void*), void* arg, uint8_t mode) = 0;
    virtual void detachInterrupt(uint8_t pin) = 0;
    virtual void enableInterrupt(uint8_t pin) = 0;
    virtual void disableInterrupt(uint8_t pin) = 0;

    i2c_port_t i2c_port = I2C_NUM_0;
    gpio_num_t i2c_sda_pin = GPIO_NUM_NC;
    gpio_num_t i2c_scl_pin = GPIO_NUM_NC;
    uint32_t i2c_freq_hz = 400000;  // 400kHz






private:

protected:
    const char *_M5_io_expander_TAG = "M5_io_expander";
    i2c_bus_handle_t _i2c_bus_handle = NULL;
    i2c_bus_device_handle_t _i2c_bus_device_handle = NULL;
    bool _device_handle_external = false; // 标记设备句柄是否为外部传入
    bool _bus_handle_external = false; // 标记总线句柄是否为外部传入
};

#endif // M5_IO_EXPANDER_H