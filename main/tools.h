#ifndef TOOLS_H
#define TOOLS_H


#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
// I2C Bus
#include "i2c_bus.h"
#include "setting.h"


// I2C ADDR
extern i2c_bus_handle_t g_i2c_bus; // I2C bus handle
extern i2c_bus_device_handle_t i2c_device_all[0xFF]; // I2C device handle

void display_init();
void display_gfx_loop();

// I2C
void register_i2c_bus_device();
void scan_i2c_bus_device();

// rx8130
void rx8130_wakeup_test();
void rx8130_recovery();

// bmi270
void bmi270_test();


#endif // TOOLS_H