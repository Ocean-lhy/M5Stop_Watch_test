/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef M5_IO_PI4IOE5V6408_H
#define M5_IO_PI4IOE5V6408_H

#include "../../M5_io_expander.h"

// M5 I/O map for PI4IOE5V6408
#define M5_IO_NUM_NC -1
#define M5_IO_NUM_0 0
#define M5_IO_NUM_1 1
#define M5_IO_NUM_2 2
#define M5_IO_NUM_3 3
#define M5_IO_NUM_4 4
#define M5_IO_NUM_5 5
#define M5_IO_NUM_6 6
#define M5_IO_NUM_7 7

// PI4IO registers
#define PI4IO_REG_CHIP_RESET 0x01
#define PI4IO_REG_IO_DIR     0x03
#define PI4IO_REG_OUT_SET    0x05
#define PI4IO_REG_OUT_H_IM   0x07
#define PI4IO_REG_IN_DEF_STA 0x09
#define PI4IO_REG_PULL_EN    0x0B
#define PI4IO_REG_PULL_SEL   0x0D
#define PI4IO_REG_IN_STA     0x0F
#define PI4IO_REG_INT_MASK   0x11
#define PI4IO_REG_IRQ_STA    0x13

// Define the number of pins
#define PIN_NUM_MAX 8

// Task stack size and priority for the interrupt task
#define INTER_TASK_STACK_SIZE 2048
#define INTER_TASK_PRIO 10

class m5_io_pi4ioe5v6408 : public m5_io_expander
{
public:
    m5_io_pi4ioe5v6408(); // Default constructor
    m5_io_pi4ioe5v6408(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq, i2c_port_t i2c_port, gpio_num_t int_pin);
    m5_io_pi4ioe5v6408(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle, gpio_num_t int_pin);
    ~m5_io_pi4ioe5v6408();

    void init(uint8_t i2c_addr) override;

    void set_pin_direction(uint8_t pin, uint8_t mode);
    void write_pin_value(uint8_t pin, bool value);
    void set_high_impedance(uint8_t pin, bool value);
    void set_input_default_state(uint8_t pin, bool value);
    void set_pull_enable(uint8_t pin, bool value);
    void set_pull_select(uint8_t pin, bool value);
    bool read_input_status(uint8_t pin);
    void set_interrupt_mask(uint8_t pin, bool value);
    uint8_t set_interrupt_map_mask(uint8_t mask);
    uint8_t read_interrupt_status(void);
    uint8_t read_interrupt_flag(void);
    void soft_reset(void);


    void pinMode(uint8_t pin, uint8_t mode) override;
    void digitalWrite(uint8_t pin, uint8_t value) override;
    int digitalRead(uint8_t pin) override;
    void attachInterrupt(uint8_t pin, void (*callback)(void), uint8_t mode) override;
    void attachInterruptArg(uint8_t pin, void (*callback)(void*), void* arg, uint8_t mode) override;
    void detachInterrupt(uint8_t pin) override;
    void enableInterrupt(uint8_t pin) override;
    void disableInterrupt(uint8_t pin) override;


private:
    gpio_num_t _int_pin = GPIO_NUM_NC;  // INT引脚
    bool _gpio_isr_service_installed = false;   // ISR服务是否已安装
    static void IRAM_ATTR _interrupt_handler(void* arg);
    void interrupt_task(void* arg);
    static void interrupt_task_trampoline(void* arg);
    TaskHandle_t _interrupt_task_handle = NULL; // 中断任务句柄
    uint8_t _intr_check_flag = 0;   // 中断标志位

    typedef void (*isr_callback_t)(void);
    typedef void (*isr_callback_with_arg_t)(void*);
    struct pin_interrupt_config {
        union {
            isr_callback_t callback;
            isr_callback_with_arg_t callback_with_arg;
        };
        void* arg;
        uint8_t mode;
        bool enabled;
        bool has_arg;
    };
    pin_interrupt_config _pin_interrupts[PIN_NUM_MAX] = {};


};

#endif // M5_IO_PI4IOE5V6408_H