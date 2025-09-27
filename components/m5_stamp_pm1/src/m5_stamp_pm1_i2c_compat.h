#ifndef __M5_STAMP_PM1_I2C_COMPAT_H__
#define __M5_STAMP_PM1_I2C_COMPAT_H__

#ifdef ARDUINO

#ifndef CONFIG_PM1_I2C_ADDRESS
#define CONFIG_PM1_I2C_ADDRESS 0x6E
#endif

#ifndef CONFIG_PM1_I2C_FREQUENCY  
#define CONFIG_PM1_I2C_FREQUENCY 100000
#endif

#ifndef CONFIG_PM1_I2C_SDA_PIN
#define CONFIG_PM1_I2C_SDA_PIN 2  // 默认SDA引脚
#endif

#ifndef CONFIG_PM1_I2C_SCL_PIN
#define CONFIG_PM1_I2C_SCL_PIN 1  // 默认SCL引脚
#endif


// I2C Configuration
uint8_t pm1_i2c_address = CONFIG_PM1_I2C_ADDRESS; // I2C设备地址
uint32_t pm1_i2c_frequency = CONFIG_PM1_I2C_FREQUENCY; // I2C频率
uint8_t pm1_i2c_sda_pin = CONFIG_PM1_I2C_SDA_PIN; // I2C SDA引脚
uint8_t pm1_i2c_scl_pin = CONFIG_PM1_I2C_SCL_PIN; // I2C SCL引脚

#include "Wire.h"

TwoWire *pm1_i2c_bus = nullptr;  // Default to nullptr, can be set in Arduino setup
TwoWire *pm1_i2c_device = &Wire;  // Default to Wire, can be changed in Arduino setup

const char *TAG = "pm1";

#ifndef PM1_I2C_READ_BYTE
static inline esp_err_t PM1_I2C_READ_BYTE(TwoWire *wire, uint8_t reg, uint8_t *data) {
    wire->beginTransmission(CONFIG_PM1_I2C_ADDRESS);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) { // false to send a restart
        return ESP_FAIL; // Transmission error
    }
    if (wire->requestFrom(CONFIG_PM1_I2C_ADDRESS, (uint8_t)1) != 1) {
        return ESP_FAIL; // Request error
    }
    *data = wire->read();
    return ESP_OK;
}
#endif
#ifndef PM1_I2C_WRITE_BYTE
static inline esp_err_t PM1_I2C_WRITE_BYTE(TwoWire *wire, uint8_t reg, uint8_t data) {
    wire->beginTransmission(CONFIG_PM1_I2C_ADDRESS);
    wire->write(reg);
    wire->write(data);
    if (wire->endTransmission() != 0) {
        return ESP_FAIL; // Transmission error
    }
    return ESP_OK;
}
#endif

#else

#include <esp_err.h>
#include <stdint.h>
#include <i2c_bus.h>

// I2C Configuration
uint8_t pm1_i2c_address = 0x6E;        // I2C设备地址
uint32_t pm1_i2c_frequency = 100000;   // I2C频率
uint8_t pm1_i2c_sda_pin = -1;           // I2C SDA引脚
uint8_t pm1_i2c_scl_pin = -1;           // I2C SCL引脚

// Internal I2C handles - managed by the component
i2c_bus_handle_t pm1_i2c_bus = NULL;
i2c_bus_device_handle_t pm1_i2c_device = NULL;
const char *TAG = "pm1";

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PM1_I2C_READ_BYTE
static inline esp_err_t PM1_I2C_READ_BYTE(i2c_bus_device_handle_t dev, uint8_t reg, uint8_t *data) {
    return i2c_bus_read_byte(dev, reg, data);
}
#endif

#ifndef PM1_I2C_WRITE_BYTE
static inline esp_err_t PM1_I2C_WRITE_BYTE(i2c_bus_device_handle_t dev, uint8_t reg, uint8_t data) {
    return i2c_bus_write_byte(dev, reg, data);
}
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif // __M5_STAMP_PM1_I2C_COMPAT_H__