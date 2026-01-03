#ifndef TOOLS_H
#define TOOLS_H

#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
// I2C Bus
#include "i2c_bus.h"
#include "setting.h"

// I2C ADDR
extern i2c_bus_handle_t g_i2c_bus;                    // I2C bus handle
extern i2c_bus_device_handle_t i2c_device_all[0xFF];  // I2C device handle

// I2C
void register_i2c_bus_device();
void scan_i2c_bus_device();

// Motor
void motor_init();
void trigger_motor(uint32_t duration_ms, uint8_t duty_percentage);

#endif  // TOOLS_H