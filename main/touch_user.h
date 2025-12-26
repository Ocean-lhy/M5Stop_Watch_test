#ifndef TOUCH_USER_H
#define TOUCH_USER_H

#include <stdint.h>
#include <string.h>
#include "i2c_bus.h"
#include "setting.h"

#define CST820_REG_STATUS     0x00
#define CST820_REG_GESTURE_ID 0x01
#define CST820_REG_FINGER_NUM 0x02
#define CST820_REG_X_H        0x03
#define CST820_REG_X_L        0x04
#define CST820_REG_Y_H        0x05
#define CST820_REG_Y_L        0x06

#define CST820_CHIP_ID  0xA7
#define CST820_SOFT_VER 0xA9

#define CST820_TOUCH_MODE 0xFA
typedef enum {
    CST820_TOUCH_MODE_DOT             = 0x60,
    CST820_TOUCH_MODE_GESTURE         = 0x11,
    CST820_TOUCH_MODE_GESTURE_AND_DOT = 0x71,
} cst820_touch_mode_t;

extern uint16_t cst820_x;
extern uint16_t cst820_y;
extern uint8_t cst820_status;

int cst820_init(i2c_bus_handle_t i2c_bus);

int cst820_read_tpinfo();

int cst820_update();

void cst820_sleep();

// void cst820_set_workmode(enum work_mode mode, uint8_t enable);

void cst820_loop();

#endif