/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

 #include "m5_io_pi4ioe5v6408.h"

// Default constructor
m5_io_pi4ioe5v6408::m5_io_pi4ioe5v6408() 
    : m5_io_expander() {
    
    // Set default values
    _int_pin = GPIO_NUM_NC;
}

 m5_io_pi4ioe5v6408::m5_io_pi4ioe5v6408(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq, i2c_port_t i2c_port, gpio_num_t int_pin) 
     // Call the base class constructor
    : m5_io_expander(sda_pin, scl_pin, freq, i2c_port) {
    
    // Save the interrupt pin number
    _int_pin = int_pin;

 }

// 支持传入现有总线句柄和设备句柄的构造函数
m5_io_pi4ioe5v6408::m5_io_pi4ioe5v6408(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle, gpio_num_t int_pin) 
    : m5_io_expander(existing_bus_handle, existing_device_handle) {
    
    // Save the interrupt pin number
    _int_pin = int_pin;
}

// Define the destructor for m5_io_pi4ioe5v6408
m5_io_pi4ioe5v6408::~m5_io_pi4ioe5v6408()
 {
    // Add cleanup code here if necessary

    if(_gpio_isr_service_installed) {
        gpio_uninstall_isr_service(); // Uninstall ISR service
        gpio_isr_handler_remove(_int_pin); // Remove ISR handler
    }
}

// Define the init method for m5_io_pi4ioe5v6408
void m5_io_pi4ioe5v6408::init(uint8_t i2c_addr) {
    // Call the base class init method
    m5_io_expander::init();

    ESP_LOGI(_M5_io_expander_TAG, "Initializing M5 IO Expander -> PI4IOE5V6408[0x%02X]", i2c_addr);

    // Initialize the I2C bus and any other necessary components
    if (_i2c_bus_handle == NULL) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create I2C bus");
        return;
    }

    // 使用基类的双向传递机制创建设备句柄
    i2c_bus_device_handle_t device_handle = create_device_handle(i2c_addr);
    if (device_handle == NULL) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create I2C bus device handle");
        return;
    }
    ESP_LOGI(_M5_io_expander_TAG, "I2C bus device handle created successfully");
    
    //
    uint8_t Manufacture_ID = 0;
    uint8_t Firmware_Revision = 0;
    uint8_t Reset_interrupt_flag = 0;
    uint8_t read_buf = 0;

    ESP_LOGI(_M5_io_expander_TAG, "Chip: Soft Resetting the chip...");
    i2c_bus_write_byte(device_handle, PI4IO_REG_CHIP_RESET, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for 100ms for the chip to reset
    i2c_bus_read_byte(device_handle, PI4IO_REG_CHIP_RESET, &read_buf);
    ESP_LOGI(_M5_io_expander_TAG, "[Reg:0x01] \"Device ID and Control\" -> 0x%02X", read_buf);
    Manufacture_ID = read_buf >> 5;
    Firmware_Revision = (read_buf & 0x1C) >> 2;
    Reset_interrupt_flag = (read_buf & 0x02) >> 1;
    ESP_LOGI(_M5_io_expander_TAG, "Chip: Manufacture ID: 0x%02X,Firmware Revision: 0x%02X,Reset Interrupt Flag: 0x%02X", Manufacture_ID, Firmware_Revision, Reset_interrupt_flag);


    // Add any additional initialization code specific to this class here
    ESP_LOGI(_M5_io_expander_TAG, "Chip: Initialization complete.");

    if(_int_pin == GPIO_NUM_NC) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid interrupt pin number: GPIO_NUM_NC");
        return;
    }

    // Initialize the interrupt pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << _int_pin),  // 选择要配置的 GPIO
        .mode = GPIO_MODE_INPUT,               // 设为输入模式
        .pull_up_en = GPIO_PULLUP_DISABLE,      // 启用上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // 禁用下拉电阻
        .intr_type = GPIO_INTR_NEGEDGE      // 低电平触发中断
    };
    // Configure the GPIO pin
    gpio_config(&io_conf); 
    ESP_LOGI(_M5_io_expander_TAG, "GPIO %d configured as interrupt pin", _int_pin);

    // Register the interrupt handler - default to level 1
    esp_err_t ret = ESP_OK;

    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM); 
    if (ret != ESP_OK) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(_M5_io_expander_TAG, "ISR service installed successfully");
    
    ret = gpio_isr_handler_add(_int_pin, _interrupt_handler, (void*)this); // Add ISR handler
    // ret = gpio_isr_handler_add(_int_pin, _interrupt_handler, NULL); // Add ISR handler
    if (ret != ESP_OK) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        gpio_uninstall_isr_service(); // Uninstall ISR service if adding handler fails
        return;
    }
    ESP_LOGI(_M5_io_expander_TAG, "ISR handler added successfully");

    _gpio_isr_service_installed = true; // Set the flag to indicate that the ISR service is installed

    // Create the interrupt task
    xTaskCreate(
        interrupt_task_trampoline, // Task function
        "InterruptTask", // Task name
        INTER_TASK_STACK_SIZE, // Stack size
        this, // Task parameter (this pointer)
        INTER_TASK_PRIO, // Task priority
        &_interrupt_task_handle); // Task handle
    if (_interrupt_task_handle == NULL) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create interrupt task");
        gpio_isr_handler_remove(_int_pin); // Remove ISR handler
        gpio_uninstall_isr_service(); // Uninstall ISR service
        return;
    }
}

//  输入输出方向设置 
//  Register 03h : I/O Direction
//  0: Input, 1: Output
void m5_io_pi4ioe5v6408::set_pin_direction(uint8_t pin, uint8_t mode) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_IO_DIR, &last);
        if (mode == GPIO_MODE_INPUT) {
            write = last & ~(1 << pin); // Set the bit to 0 for input
        } else if(mode == GPIO_MODE_OUTPUT) {
            write = last | (1 << pin);  // Set the bit to 1 for output
        }
        else {
            ESP_LOGE(_M5_io_expander_TAG, "[set_pin_direction 03h] -> Invalid mode: %d", mode);
            return;
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_IO_DIR, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[set_pin_direction 03h] -> Invalid pin number: %d", pin);
    }
    ESP_LOGI(_M5_io_expander_TAG, "[set_pin_direction 03h] -> pin: %d, mode: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, mode, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

//  输出方向高低电平设置（仅输出模式有效�?
//  Register 05h : Output Port Register
//  0: Low, 1: High
void m5_io_pi4ioe5v6408::write_pin_value(uint8_t pin, bool value) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_OUT_SET, &last);
        if (value) {
            write = last | (1 << pin);  // Set the bit to 1 for high
        } else {
            write = last & ~(1 << pin); // Set the bit to 0 for low
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_OUT_SET, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[write_pin_value 05h] -> Invalid pin number: %d", pin);
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "[write_pin_value 05h] -> pin: %d, value: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, value, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

//  高阻态配置（仅输出模式有效）
//  Register 07h : Output High-Impedance
//  0: Low, 1: High-Z
void m5_io_pi4ioe5v6408::set_high_impedance(uint8_t pin, bool value) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_OUT_H_IM, &last);
        if (value) {
            write = last | (1 << pin);  // Set the bit to 1 for high-Z
        } else {
            write = last & ~(1 << pin); // Set the bit to 0 for low
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_OUT_H_IM, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[set_high_impedance 07h] -> Invalid pin number: %d", pin);
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "[set_high_impedance 07h] -> pin: %d, value: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, value, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

//  输入默认状态配置（仅输入模式有效）
//  Register 09h : Input Default State
//  0: Low, 1: High
void m5_io_pi4ioe5v6408::set_input_default_state(uint8_t pin, bool value) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_IN_DEF_STA, &last);
        if (value) {
            write = last | (1 << pin);  // Set the bit to 1 for high
        } else {
            write = last & ~(1 << pin); // Set the bit to 0 for low
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_IN_DEF_STA, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[set_input_default_state 09h] -> Invalid pin number: %d", pin);
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "[set_input_default_state 09h] -> pin: %d, value: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, value, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

//  Pull-up//-Down 使能配置
//  Register 0Bh : Pull-up/-Down Enable
//  0: Disable, 1: Enable
void m5_io_pi4ioe5v6408::set_pull_enable(uint8_t pin, bool value) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_PULL_EN, &last);
        if (value) {
            write = last | (1 << pin);  // Set the bit to 1 for enable
        } else {
            write = last & ~(1 << pin); // Set the bit to 0 for disable
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_PULL_EN, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[set_pull_enable 0Bh] -> Invalid pin number: %d", pin);
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "[set_pull_enable 0Bh] -> pin: %d, value: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, value, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

//  Pull-up//-Down 选择配置
//  Register 0Dh : Pull-up/-Down Select
//  0: Pull-Down, 1: Pull-Up
void m5_io_pi4ioe5v6408::set_pull_select(uint8_t pin, bool value) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_PULL_SEL, &last);
        if (value) {
            write = last | (1 << pin);  // Set the bit to 1 for pull-up
        } else {
            write = last & ~(1 << pin); // Set the bit to 0 for pull-down
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_PULL_SEL, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[set_pull_select 0Dh] -> Invalid pin number: %d", pin);
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "[set_pull_select 0Dh] -> pin: %d, value: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, value, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

// 读取输入状�?
//  Register 0Fh : Input Status Register
//  0: Low, 1: High
bool m5_io_pi4ioe5v6408::read_input_status(uint8_t pin) {
    uint8_t read_buf = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_IN_STA, &read_buf);
        ESP_LOGI(_M5_io_expander_TAG, "[read_input_status 0Fh] -> pin: %d, value: %d", pin, (read_buf >> pin) & 0x01);
        return ((read_buf >> pin) & 0x01);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[read_input_status 0Fh] -> Invalid pin number: %d", pin);
        return false;
    }
}

//  中断使能配置（启用则触发时INT引脚会被拉低�?
//  Register 11h : Interrupt Mask Register
//  1: Enable, 0: Disable
void m5_io_pi4ioe5v6408::set_interrupt_mask(uint8_t pin, bool value) {
    uint8_t last = 0;
    uint8_t write = 0;
    if (pin < PIN_NUM_MAX) {
        i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_INT_MASK, &last);
        if (value) {
            write = last & ~(1 << pin); // Set the bit to 0 for enable
        } else {
            write = last | (1 << pin);  // Set the bit to 1 for disable
        }
        i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_INT_MASK, write);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "[set_interrupt_mask 11h] -> Invalid pin number: %d", pin);
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "[set_interrupt_mask 11h] -> pin: %d, value: %d, last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        pin, value, BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
}

//  使能配置 多引脚配置（启用则触发时INT引脚会被拉低�?
//  Register 11h : Interrupt Mask Register
//  1: Enable, 0: Disable mask: 0b00000000
//  RETURN: last register value (返回设置前的寄存器的反值，可用于后续复原寄存器状�?
uint8_t m5_io_pi4ioe5v6408::set_interrupt_map_mask(uint8_t mask) {
    uint8_t last = 0;
    uint8_t write = 0;
    i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_INT_MASK, &last);
    write = ~mask;
    i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_INT_MASK, write);
    ESP_LOGI(_M5_io_expander_TAG, "[set_interrupt_map_mask 11h] -> mask: 0b" BYTE_TO_BINARY_PATTERN ", last: 0b" BYTE_TO_BINARY_PATTERN ", write: 0b" BYTE_TO_BINARY_PATTERN,
        BYTE_TO_BINARY(mask), BYTE_TO_BINARY(last), BYTE_TO_BINARY(write));
    return ~last; // Return the last register value
}

//  中断状态读取（与输入默认状态相反则触发高电平，读取寄存器后复位�?
//  Register 13h : Interrupt Status Register
//  RETURN 0: Not Triggered, 1: Triggered
uint8_t m5_io_pi4ioe5v6408::read_interrupt_status(void){
    uint8_t read_buf = 0;
    i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_IRQ_STA, &read_buf);
    ESP_LOGI(_M5_io_expander_TAG, "[read_interrupt_status 13h] -> Interrupt Status: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(read_buf));
    return read_buf;
}

//  读取中断标志�?�?�?复位后，需要读取一次，消除中断复位标志位，才可正常启用中断)
//  Register 01h : Interrupt Flag Register
//  0: Not Triggered, 1: Triggered
uint8_t m5_io_pi4ioe5v6408::read_interrupt_flag(void){
    uint8_t read_buf = 0;
    i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_CHIP_RESET, &read_buf);
    ESP_LOGI(_M5_io_expander_TAG, "[read_interrupt_flag 01h] -> Interrupt Flag: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(read_buf));
    return read_buf;
}

//  软复�?
//  Register 01h : Chip Reset
//  null
void m5_io_pi4ioe5v6408::soft_reset(void) {
    uint8_t read_buf = 0;
    i2c_bus_write_byte(_i2c_bus_device_handle, PI4IO_REG_CHIP_RESET, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for 100ms for the chip to reset
    i2c_bus_read_byte(_i2c_bus_device_handle, PI4IO_REG_CHIP_RESET, &read_buf);
    ESP_LOGI(_M5_io_expander_TAG, "[soft_reset 01h] -> Chip Reset: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(read_buf));
}


//  中断处理函数（ISR�?
void IRAM_ATTR m5_io_pi4ioe5v6408::_interrupt_handler(void* arg) {
    auto* instance = static_cast<m5_io_pi4ioe5v6408*>(arg);
    instance->_intr_check_flag = 1;
}

//  中断事务处理函数
void m5_io_pi4ioe5v6408::interrupt_task(void* arg) {
    auto* instance = static_cast<m5_io_pi4ioe5v6408*>(arg);
    while (true) {
        if (instance->_intr_check_flag) {
            uint8_t interrupt_status = instance->read_interrupt_status();
            //禁用全部中断
            uint8_t last = instance->set_interrupt_map_mask(0x00); // Disable all interrupts
            // read_interrupt_flag();   //DEBUG CHECK
            instance->_intr_check_flag = 0;
            
            for(uint8_t i = 0; i < PIN_NUM_MAX; i++) {
                if ((interrupt_status & (1 << i)) && instance->_pin_interrupts[i].enabled) {
                    ESP_LOGI(_M5_io_expander_TAG, "Interrupt triggered on pin %d", i);
                    
                    // 根据回调类型调用不同的函�?
                    if (instance->_pin_interrupts[i].has_arg) {
                        if (instance->_pin_interrupts[i].callback_with_arg) {
                            instance->_pin_interrupts[i].callback_with_arg(instance->_pin_interrupts[i].arg);
                        }
                    } else {
                        if (instance->_pin_interrupts[i].callback) {
                            instance->_pin_interrupts[i].callback();
                        }
                    }
                }
            }
            instance->set_interrupt_map_mask(last);
        }
        // DEBUG CHECK
        // else
        // {
        //     // No interrupt detected, do nothing
        //     uint8_t interrupt_status = instance->read_interrupt_status();
        //     ESP_LOGI(_M5_io_expander_TAG, "No interrupt detected");
        //     read_interrupt_flag();
        // }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    _interrupt_task_handle = NULL; // Clear the task handle
    vTaskDelete(NULL); // Delete the task when done

}

//  中断事务处理函数的跳板函�?
void m5_io_pi4ioe5v6408::interrupt_task_trampoline(void* arg) {
    auto* instance = static_cast<m5_io_pi4ioe5v6408*>(arg);
    instance->interrupt_task(arg);
}

//  类Arduino函数实现 -> pinMode
//  mode: INPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT
//  null
void m5_io_pi4ioe5v6408::pinMode(uint8_t pin, uint8_t mode) {
    if(pin < PIN_NUM_MAX) {
        switch (mode) {
            case INPUT:
                set_pin_direction(pin, GPIO_MODE_INPUT);
                set_high_impedance(pin, M5_IO_HIGH); // Set to high impedance
                set_pull_enable(pin, M5_IO_DISABLE); // Disable pull-up/down
                break;
            case INPUT_PULLUP:
                set_pin_direction(pin, GPIO_MODE_INPUT);
                set_high_impedance(pin, M5_IO_HIGH); // Set to high impedance
                set_pull_enable(pin, M5_IO_ENABLE); // Enable pull-up
                set_pull_select(pin, M5_IO_PULL_UP); // Select pull-up
                break;
            case INPUT_PULLDOWN:
                set_pin_direction(pin, GPIO_MODE_INPUT);
                set_high_impedance(pin, M5_IO_HIGH); // Set to high impedance
                set_pull_enable(pin, M5_IO_ENABLE); // Enable pull-down
                set_pull_select(pin, M5_IO_PULL_DOWN); // Select pull-down
                break;
            case OUTPUT:
                set_pin_direction(pin, GPIO_MODE_OUTPUT);
                set_high_impedance(pin, M5_IO_LOW); // Set to low impedance
                break;
            default:
                ESP_LOGE(_M5_io_expander_TAG, "Invalid mode: %d", mode);
        }
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
    ESP_LOGI(_M5_io_expander_TAG, "Pin %d set to mode %s", pin, (mode == INPUT ? "INPUT" : (mode == INPUT_PULLUP ? "INPUT_PULLUP" : (mode == INPUT_PULLDOWN ? "INPUT_PULLDOWN" : "OUTPUT"))));
}

//  类Arduino函数实现 -> digitalWrite
//  value: HIGH, LOW
//  null
void m5_io_pi4ioe5v6408::digitalWrite(uint8_t pin, uint8_t value) {
    if(pin < PIN_NUM_MAX) {
        if(value == M5_IO_HIGH) {
            write_pin_value(pin, M5_IO_HIGH); // Set to high
        } else {
            write_pin_value(pin, M5_IO_LOW); // Set to low
        }
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
    ESP_LOGI(_M5_io_expander_TAG, "Pin %d set to value %s", pin, (value == M5_IO_HIGH ? "HIGH" : "LOW"));
}

//  类Arduino函数实现 -> digitalRead
//  null
//  return: -1:ERROR 0:LOW 1:HIGH
int m5_io_pi4ioe5v6408::digitalRead(uint8_t pin) {
    if(pin < PIN_NUM_MAX) {
        bool value = read_input_status(pin); // Read the input status
        ESP_LOGI(_M5_io_expander_TAG, "Pin %d read value: %s", pin, (value ? "HIGH" : "LOW"));
        return value;
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
        return false;
    }
}

//  类Arduino函数实现 -> attachInterrupt
//  mode: M5_IO_RISING, M5_IO_FALLING, function: void(*callback)(void)
//  null
void m5_io_pi4ioe5v6408::attachInterrupt(uint8_t pin, void (*callback)(void), uint8_t mode) {
    if (pin < PIN_NUM_MAX) {
        // 保存回调函数和模�?
        _pin_interrupts[pin].callback = callback;
        _pin_interrupts[pin].mode = mode;
        _pin_interrupts[pin].enabled = true;
        _pin_interrupts[pin].has_arg = false;
        
        // 确保引脚被配置为输入模式
        set_pin_direction(pin, GPIO_MODE_INPUT);
        
        // 根据中断模式设置引脚默认状�?
        // 对于上升沿，默认状态为低电平；对于下降沿，默认状态为高电�?
        if (mode == M5_IO_RISING) {
            set_input_default_state(pin, false);  // 默认低电平，上升到高电平触发中断
        } else if (mode == M5_IO_FALLING) {
            set_input_default_state(pin, true);  // 默认高电平，下降到低电平触发中断
        }
        
        // 启用该引脚的中断
        set_interrupt_mask(pin, true);
        
        ESP_LOGI(_M5_io_expander_TAG, "Interrupt attached to pin %d with mode %s", pin, (mode == M5_IO_RISING ? "RISING" : "FALLING"));
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
}

//  类Arduino函数实现 -> attachInterruptArg
//  mode: M5_IO_RISING, M5_IO_FALLING, function: void(*callback)(void*), arg: void*
//  null
void m5_io_pi4ioe5v6408::attachInterruptArg(uint8_t pin, void (*callback)(void*), void* arg, uint8_t mode) {
    if (pin < PIN_NUM_MAX) {
        // 保存回调函数和模�?
        _pin_interrupts[pin].callback_with_arg = callback;
        _pin_interrupts[pin].arg = arg;         // 保存参数
        _pin_interrupts[pin].mode = mode;
        _pin_interrupts[pin].enabled = true;
        _pin_interrupts[pin].has_arg = true;    // 标记为带参数的回�?
        
        // 确保引脚被配置为输入模式
        set_pin_direction(pin, GPIO_MODE_INPUT);
        
        // 根据中断模式设置引脚默认状�?
        // 对于上升沿，默认状态为低电平；对于下降沿，默认状态为高电�?
        if (mode == M5_IO_RISING) {
            set_input_default_state(pin, false);  // 默认低电平，上升到高电平触发中断
        } else if (mode == M5_IO_FALLING) {
            set_input_default_state(pin, true);  // 默认高电平，下降到低电平触发中断
        }
        
        // 启用该引脚的中断
        set_interrupt_mask(pin, true);
        
        ESP_LOGI(_M5_io_expander_TAG, "Interrupt attached to pin %d with mode %s", pin, (mode == M5_IO_RISING ? "RISING" : "FALLING"));
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
}

//  类Arduino函数实现 -> detachInterrupt
//  null
//  null
void m5_io_pi4ioe5v6408::detachInterrupt(uint8_t pin) {
    if (pin < PIN_NUM_MAX) {
        // 禁用该引脚的中断
        set_interrupt_mask(pin, false);
        
        // 清除回调函数和模式
        _pin_interrupts[pin].callback = nullptr;
        _pin_interrupts[pin].callback_with_arg = nullptr;
        _pin_interrupts[pin].enabled = false;
        _pin_interrupts[pin].has_arg = false;
        
        ESP_LOGI(_M5_io_expander_TAG, "Interrupt detached from pin %d", pin);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
}

//  类Arduino函数实现 -> enableInterrupt （attachInterrupt类函数自动激活中断）
//  null
//  null
void m5_io_pi4ioe5v6408::enableInterrupt(uint8_t pin) {
    if (pin < PIN_NUM_MAX) {
        // 启用该引脚的中断
        set_interrupt_mask(pin, true);

        // 启用回调函数
        if(_pin_interrupts[pin].callback != nullptr || (_pin_interrupts[pin].callback_with_arg != nullptr && _pin_interrupts[pin].has_arg)) {
            _pin_interrupts[pin].enabled = true;
            ESP_LOGI(_M5_io_expander_TAG, "Interrupt enabled on pin %d, has callback.", pin);
        }
        else {
            ESP_LOGE(_M5_io_expander_TAG, "Interrupt not enabled on pin %d, no callback set.", pin);
            _pin_interrupts[pin].enabled = false;
            set_interrupt_mask(pin, false);
        }
        
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
}

//  类Arduino函数实现 -> disableInterrupt （detachInterrupt类函数自动禁用中断）
//  null
//  null
void m5_io_pi4ioe5v6408::disableInterrupt(uint8_t pin) {
    if (pin < PIN_NUM_MAX) {
        // 禁用该引脚的中断
        set_interrupt_mask(pin, false);
        
        // 禁用回调函数
        _pin_interrupts[pin].enabled = false;
        
        ESP_LOGI(_M5_io_expander_TAG, "Interrupt disabled on pin %d", pin);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
    }
}
