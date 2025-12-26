#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "m5_io_py32ioexpander.h"
#include "m5_stamp_pm1.h"

// I2C ADDR
#define CST820_ADDR           0x15
#define ES8311_ADDR           0x18
#define RX8130_ADDR           0x32
#define BMI270_ADDR           0x68
#define PMIC_ADDR             0x6E
#define PY32_IO_EXPANDER_ADDR 0x6F

// PMIC IO
#define PMG0_RTC_IMU_INT PM1_GPIO_NUM_0
#define PMG2_CHG_STAT    PM1_GPIO_NUM_2
#define PMG4_PORT_INT    PM1_GPIO_NUM_4
#define PMG3_CHG_PROG    PM1_GPIO_NUM_3
#define PMG1_G12_PY_IRQ  PM1_GPIO_NUM_1

// PY32 IO Expander
#define PY32_MOTOR_EN_PIN      M5_IO_NUM_9   // GPIO_PIN_9 (PA0-IO9): MOTOR ENABLE (PWM)
#define PY32_L3B_EN_PIN        M5_IO_NUM_8   // GPIO_PIN_8 (PB0-IO8): L3B ENABLE
#define PY32_SPK_PA_PIN        M5_IO_NUM_10  // GPIO_PIN_10 (PA1-IO10): SPEAKER PA
#define PY32_TP_RST_PIN        M5_IO_NUM_4   // GPIO_PIN_4 (PA3-IO4): TOUCH RESET
#define PY32_OLED_RST_PIN      M5_IO_NUM_5   // GPIO_PIN_5 (PA4-IO5): OLED RESET
#define PY32_MUX_CTR_PIN       M5_IO_NUM_1   // GPIO_PIN_1 (PB5-IO1): CH442E MUX CONTROL
#define PY32_AU_EN_PIN         M5_IO_NUM_3   // GPIO_PIN_3 (PA1-IO3): AUDIO ENABLE
#define PY32_MOTOR_PWM_CHANNEL 0

// ES8311 I2S
#define I2S_MCLK_PIN     (gpio_num_t)18
#define I2S_BCLK_PIN     (gpio_num_t)17
#define I2S_DADC_IN_PIN  (gpio_num_t)16
#define I2S_LRCK_PIN     (gpio_num_t)15
#define I2S_DDAC_OUT_PIN (gpio_num_t)21
#define SPK_PA_PIN       (gpio_num_t)14

// I2C PIN
#define I2C_SCL_PIN (gpio_num_t)48
#define I2C_SDA_PIN (gpio_num_t)47
#define I2C_FREQ    100000  // if you want to use 400KHz, please setting PMIC and PY32_IO_EXPANDER first

// USER BUTTON
#define USER_BUTTON1_PIN (gpio_num_t)1
#define USER_BUTTON2_PIN (gpio_num_t)2

// IRQ PIN
#define IRQ_PIN        (gpio_num_t)12
#define TP_INT_IRQ_PIN (gpio_num_t)13

// Grove PIN
#define GROVE_3_PIN (gpio_num_t)10
#define GROVE_4_PIN (gpio_num_t)11

// Bottom IO PIN
#define EXT_GPIO3_PIN (gpio_num_t)3
#define EXT_GPIO4_PIN (gpio_num_t)4
#define EXT_GPIO5_PIN (gpio_num_t)5
#define EXT_GPIO6_PIN (gpio_num_t)6
#define EXT_GPIO7_PIN (gpio_num_t)7
#define EXT_GPIO8_PIN (gpio_num_t)8
#define EXT_GPIO9_PIN (gpio_num_t)9

// OLED QSPI PIN
#define QSPI_TE_PIN   (gpio_num_t)38
#define QSPI_CS_PIN   (gpio_num_t)39
#define QSPI_SCLK_PIN (gpio_num_t)40
#define QSPI_D0_PIN   (gpio_num_t)41
#define QSPI_D1_PIN   (gpio_num_t)42
#define QSPI_D2_PIN   (gpio_num_t)46
#define QSPI_D3_PIN   (gpio_num_t)45
#define QSPI_RST_PIN  (gpio_num_t) - 1  // PY32 BIT 14

// TOUCH CST820
#define TOUCH_INT_PIN TP_INT_IRQ_PIN
#define TOUCH_SCL_PIN I2C_SCL_PIN
#define TOUCH_SDA_PIN I2C_SDA_PIN

// OLED HOST and TOUCH HOST
#define LCD_HOST   SPI2_HOST
#define TOUCH_HOST I2C_NUM_0
