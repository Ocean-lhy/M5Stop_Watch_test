/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "m5_io_py32ioexpander.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 不再包含内部C实现头文件；寄存器与位定义已迁移至类头文�?

// Default Constructor
m5_io_py32ioexpander::m5_io_py32ioexpander()
    : m5_io_expander(), _device_addr(PY32_DEFAULT_I2C_ADDR), _int_pin(GPIO_NUM_NC), _initialized_ok(false), 
      _auto_snapshot_enabled(true), _pin_states_valid(false), _pwm_states_valid(false), _adc_state_valid(false)
{
    // Initialize interrupt callbacks array
    memset(_interrupt_callbacks, 0, sizeof(_interrupt_callbacks));
    _clearPinStates();
    _clearPwmStates();
    _clearAdcState();
}

// Constructor
m5_io_py32ioexpander::m5_io_py32ioexpander(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq, i2c_port_t i2c_port, gpio_num_t int_pin)
    : m5_io_expander(sda_pin, scl_pin, freq, i2c_port), _device_addr(PY32_DEFAULT_I2C_ADDR), _int_pin(int_pin), 
      _initialized_ok(false), _auto_snapshot_enabled(true), _pin_states_valid(false), _pwm_states_valid(false), _adc_state_valid(false)
{
    // Initialize interrupt callbacks array
    memset(_interrupt_callbacks, 0, sizeof(_interrupt_callbacks));
    _clearPinStates();
    _clearPwmStates();
    _clearAdcState();
}

// 支持传入现有总线句柄和设备句柄的构造函数
m5_io_py32ioexpander::m5_io_py32ioexpander(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle, gpio_num_t int_pin)
    : m5_io_expander(existing_bus_handle, existing_device_handle), _device_addr(PY32_DEFAULT_I2C_ADDR), _int_pin(int_pin), 
      _initialized_ok(false), _auto_snapshot_enabled(true), _pin_states_valid(false), _pwm_states_valid(false), _adc_state_valid(false)
{
    // Initialize interrupt callbacks array
    memset(_interrupt_callbacks, 0, sizeof(_interrupt_callbacks));
    _clearPinStates();
    _clearPwmStates();
    _clearAdcState();
}

// Destructor
m5_io_py32ioexpander::~m5_io_py32ioexpander()
{
    // 基类会处理设备句柄的清理
    
    // Clean up interrupt handling
    if (_int_pin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(_int_pin);
    }

    if (_intr_task) {
        vTaskDelete(_intr_task);
        _intr_task = nullptr;
    }
    if (_intr_queue) {
        vQueueDelete(_intr_queue);
        _intr_queue = nullptr;
    }
}

// Initialize the device
void m5_io_py32ioexpander::init(uint8_t i2c_addr)
{
    // Call base class init
    m5_io_expander::init();
    
    _device_addr = i2c_addr;
    
    ESP_LOGI(_M5_io_expander_TAG, "Initializing M5 IO Expander -> PY32IOExpander[0x%02X]", i2c_addr);
    
    // PY32 IO Expander 默认需要100KHz频率
    if (_i2c_bus_handle != NULL) {
        uint32_t current_clk = i2c_bus_get_current_clk_speed(_i2c_bus_handle);
        if (current_clk != 100000) {
            ESP_LOGI(_M5_io_expander_TAG, "Current I2C bus frequency: %lu Hz, need to recreate device handle for 100KHz", current_clk);
            
            // 销毁现有设备句柄（如果存在且非外部管理）
            if (_i2c_bus_device_handle != NULL && !_device_handle_external) {
                esp_err_t ret = i2c_bus_device_delete(&_i2c_bus_device_handle);
                if (ret != ESP_OK) {
                    ESP_LOGE(_M5_io_expander_TAG, "Failed to delete existing PY32 I2C device: %s", esp_err_to_name(ret));
                    _initialized_ok = false;
                    return;
                }
                ESP_LOGI(_M5_io_expander_TAG, "Existing device handle deleted for frequency adjustment");
            }
            
            // 创建新的100KHz设备句柄
            _i2c_bus_device_handle = i2c_bus_device_create(_i2c_bus_handle, _device_addr, 100000);
            if (_i2c_bus_device_handle == NULL) {
                ESP_LOGE(_M5_io_expander_TAG, "Failed to create PY32 I2C device at 100KHz");
                _initialized_ok = false;
                return;
            }
            
            // 如果原来是外部管理的设备句柄，现在变成内部管理
            _device_handle_external = false;
            ESP_LOGI(_M5_io_expander_TAG, "PY32 I2C device recreated at 100KHz frequency");
        }
    }
    
    if (!_initializeDevice()) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to initialize PY32 IO Expander device");
        _initialized_ok = false;
        return;
    }
    
    if (_int_pin != GPIO_NUM_NC && !_setupInterrupts()) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to setup interrupts");
    // cleanup any partially created resources
    if (_intr_task) { vTaskDelete(_intr_task); _intr_task = nullptr; }
    if (_intr_queue) { vQueueDelete(_intr_queue); _intr_queue = nullptr; }
        return;
    }
    
    // snapshot all device states after init
    bool gpio_ok = _snapshotIoState();
    bool pwm_ok  = _snapshotPwmState();
    bool adc_ok  = _snapshotAdcState();
    
    _pin_states_valid = gpio_ok;
    _pwm_states_valid = pwm_ok;
    _adc_state_valid  = adc_ok;
    _initialized_ok   = true;
    
    ESP_LOGI(_M5_io_expander_TAG, "PY32 IO Expander initialization complete (GPIO:%s PWM:%s ADC:%s)",
             gpio_ok ? "OK" : "FAIL", pwm_ok ? "OK" : "FAIL", adc_ok ? "OK" : "FAIL");
}

// Arduino-style pinMode function
void m5_io_py32ioexpander::pinMode(uint8_t pin, uint8_t mode)
{
    // Validate configuration before proceeding
    py32_config_type_t config_type = (mode == INPUT) ? PY32_CONFIG_TYPE_GPIO_INPUT : PY32_CONFIG_TYPE_GPIO_OUTPUT;
    py32_config_validation_t validation = _validatePinConfig(pin, config_type, true);
    if (!validation.valid) {
        ESP_LOGE(_M5_io_expander_TAG, "pinMode validation failed: %s", validation.error_msg);
        return;
    }
    
    if (_i2c_bus_device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Device not initialized");
        return;
    }

    // Configure mode register
    uint16_t mode_reg = 0;
    if (!_readRegister16(REG_GPIO_M_L, &mode_reg)) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to read GPIO mode reg");
        return;
    }

    // Configure pull-up/down registers
    uint16_t pu_reg = 0, pd_reg = 0;
    if (!_readRegister16(REG_GPIO_PU_L, &pu_reg)) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to read GPIO pull-up reg");
        return;
    }
    if (!_readRegister16(REG_GPIO_PD_L, &pd_reg)) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to read GPIO pull-down reg");
        return;
    }

    switch (mode) {
        case INPUT:
            mode_reg &= ~(1 << pin);
            pu_reg   &= ~(1 << pin);
            pd_reg   &= ~(1 << pin);
            _pin_states[pin].is_output = false;
            _pin_states[pin].pull      = 0;
            break;
        case INPUT_PULLUP:
            mode_reg &= ~(1 << pin);
            pu_reg   |=  (1 << pin);
            pd_reg   &= ~(1 << pin);
            _pin_states[pin].is_output = false;
            _pin_states[pin].pull      = 1;
            break;
        case INPUT_PULLDOWN:
            mode_reg &= ~(1 << pin);
            pu_reg   &= ~(1 << pin);
            pd_reg   |=  (1 << pin);
            _pin_states[pin].is_output = false;
            _pin_states[pin].pull      = 2;
            break;
        case OUTPUT: {
            // Check if this pin is a PWM pin and disable PWM if enabled
            if (_isPwmPin(pin)) {
                uint8_t pwm_channel = _getPwmChannel(pin);
                uint8_t pwm_reg_addr = REG_PWM1_DUTY_H + (pwm_channel * 2);
                uint8_t pwm_ctrl = 0;
                if (_readRegister(pwm_reg_addr, &pwm_ctrl)) {
                    if (pwm_ctrl & PWM_ENABLE) {
                        ESP_LOGW(_M5_io_expander_TAG, "Pin %d has PWM enabled, disabling PWM for GPIO output", pin);
                        pwm_ctrl &= ~PWM_ENABLE;
                        _writeRegister(pwm_reg_addr, pwm_ctrl);
                    }
                }
            }
            
            mode_reg |=  (1 << pin);
            pu_reg   &= ~(1 << pin);
            pd_reg   &= ~(1 << pin);
            // ensure push-pull drive - this is critical for power control
            uint16_t drv_reg = 0;
            if (_readRegister16(REG_GPIO_DRV_L, &drv_reg)) {
                drv_reg &= ~(1 << pin);  // 0 = push-pull, 1 = open-drain
                if (!_writeRegister16(REG_GPIO_DRV_L, drv_reg)) {
                    ESP_LOGE(_M5_io_expander_TAG, "Failed to set push-pull mode for pin %d", pin);
                }
            } else {
                ESP_LOGE(_M5_io_expander_TAG, "Failed to read drive register for pin %d", pin);
            }
            _pin_states[pin].is_output = true;
            _pin_states[pin].drive     = 0; // push-pull
            break;
        }
        default:
            ESP_LOGE(_M5_io_expander_TAG, "Invalid mode: %d", mode);
            return;
    }

    // Write registers in proper order: first configure pull-up/down, then drive mode, finally direction
    if (!_writeRegister16(REG_GPIO_PU_L, pu_reg)) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to write GPIO pull-up reg for pin %d", pin);
        return;
    }
    if (!_writeRegister16(REG_GPIO_PD_L, pd_reg)) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to write GPIO pull-down reg for pin %d", pin);
        return;
    }
    if (!_writeRegister16(REG_GPIO_M_L, mode_reg)) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to write GPIO mode reg for pin %d", pin);
        return;
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "Pin %d set to mode %s", pin, 
             (mode == INPUT ? "INPUT" : 
              (mode == INPUT_PULLUP ? "INPUT_PULLUP" : 
               (mode == INPUT_PULLDOWN ? "INPUT_PULLDOWN" : "OUTPUT"))));
    
    // Verify registers for debugging with register addresses
    uint16_t verify_mode = 0, verify_drv = 0;
    if (_readRegister16(REG_GPIO_M_L, &verify_mode) && _readRegister16(REG_GPIO_DRV_L, &verify_drv)) {
        ESP_LOGI(_M5_io_expander_TAG, "Pin %d verify: REG[0x%02X-0x%02X]MODE=%d%d%d%d %d%d%d%d %d%d%d%d %d%dXX (bit%d=%d)", 
                 pin, REG_GPIO_M_L, REG_GPIO_M_H,
                 (verify_mode>>0)&1, (verify_mode>>1)&1, (verify_mode>>2)&1, (verify_mode>>3)&1,
                 (verify_mode>>4)&1, (verify_mode>>5)&1, (verify_mode>>6)&1, (verify_mode>>7)&1,
                 (verify_mode>>8)&1, (verify_mode>>9)&1, (verify_mode>>10)&1, (verify_mode>>11)&1,
                 (verify_mode>>12)&1, (verify_mode>>13)&1, pin, (verify_mode >> pin) & 1);
        ESP_LOGI(_M5_io_expander_TAG, "Pin %d verify: REG[0x%02X-0x%02X]DRV=%d%d%d%d %d%d%d%d %d%d%d%d %d%dXX (bit%d=%d)", 
                 pin, REG_GPIO_DRV_L, REG_GPIO_DRV_H,
                 (verify_drv>>0)&1, (verify_drv>>1)&1, (verify_drv>>2)&1, (verify_drv>>3)&1,
                 (verify_drv>>4)&1, (verify_drv>>5)&1, (verify_drv>>6)&1, (verify_drv>>7)&1,
                 (verify_drv>>8)&1, (verify_drv>>9)&1, (verify_drv>>10)&1, (verify_drv>>11)&1,
                 (verify_drv>>12)&1, (verify_drv>>13)&1, pin, (verify_drv >> pin) & 1);
    }
    
    // Auto-snapshot after configuration change
    _autoSnapshot();
}

// Arduino-style digitalWrite function
void m5_io_py32ioexpander::digitalWrite(uint8_t pin, uint8_t value)
{
    if (!_isValidPin(pin)) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
        return;
    }
    
    if (_i2c_bus_device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Device not initialized");
        return;
    }

    // Pre-check: ensure OUTPUT mode before writing
    uint16_t mode_reg = 0;
    if (_readRegister16(REG_GPIO_M_L, &mode_reg)) {
        bool is_output = (mode_reg & (1 << pin)) != 0;
        if (!is_output) {
            ESP_LOGW(_M5_io_expander_TAG, "Pin %d not OUTPUT, switching to OUTPUT before write", pin);
            mode_reg |= (1 << pin);
            _writeRegister16(REG_GPIO_M_L, mode_reg);
        }
    }
    
    uint8_t level = (value == HIGH) ? 1 : 0;
    uint16_t out_reg = 0;
    esp_err_t ret = _readRegister16(REG_GPIO_O_L, &out_reg) ? ESP_OK : ESP_FAIL;
    if (ret == ESP_OK) {
        if (level) out_reg |= (1 << pin); else out_reg &= ~(1 << pin);
        ret = _writeRegister16(REG_GPIO_O_L, out_reg) ? ESP_OK : ESP_FAIL;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(_M5_io_expander_TAG, "Pin %d set to %s", pin, (value == HIGH ? "HIGH" : "LOW"));
        _pin_states[pin].output_level = level;
        
        // Verify output register for debugging with register address
        uint16_t verify_out = 0;
        if (_readRegister16(REG_GPIO_O_L, &verify_out)) {
            ESP_LOGI(_M5_io_expander_TAG, "Pin %d verify: REG[0x%02X-0x%02X]OUT=%d%d%d%d %d%d%d%d %d%d%d%d %d%dXX (bit%d=%d)", 
                     pin, REG_GPIO_O_L, REG_GPIO_O_H,
                     (verify_out>>0)&1, (verify_out>>1)&1, (verify_out>>2)&1, (verify_out>>3)&1,
                     (verify_out>>4)&1, (verify_out>>5)&1, (verify_out>>6)&1, (verify_out>>7)&1,
                     (verify_out>>8)&1, (verify_out>>9)&1, (verify_out>>10)&1, (verify_out>>11)&1,
                     (verify_out>>12)&1, (verify_out>>13)&1, pin, (verify_out >> pin) & 1);
        }
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to set pin %d to %s", pin, (value == HIGH ? "HIGH" : "LOW"));
    }
    
    // Auto-snapshot after configuration change
    _autoSnapshot();
}

// Arduino-style digitalRead function
int m5_io_py32ioexpander::digitalRead(uint8_t pin)
{
    if (!_isValidPin(pin)) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
        return -1;
    }
    
    if (_i2c_bus_device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Device not initialized");
        return -1;
    }
    
    uint16_t in_reg = 0; uint8_t level = 0;
    esp_err_t ret = _readRegister16(REG_GPIO_I_L, &in_reg) ? ESP_OK : ESP_FAIL;
    if (ret == ESP_OK) level = (in_reg & (1 << pin)) ? 1 : 0;
    
    if (ret == ESP_OK) {
        ESP_LOGI(_M5_io_expander_TAG, "Pin %d read value: %s", pin, (level ? "HIGH" : "LOW"));
        return level;
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to read pin %d", pin);
        return -1;
    }
}

// Arduino-style attachInterrupt function
void m5_io_py32ioexpander::attachInterrupt(uint8_t pin, void (*callback)(void), uint8_t mode)
{
    // Validate interrupt configuration before proceeding
    py32_config_validation_t validation = _validatePinConfig(pin, PY32_CONFIG_TYPE_GPIO_INTERRUPT, true);
    if (!validation.valid) {
        ESP_LOGE(_M5_io_expander_TAG, "attachInterrupt validation failed: %s", validation.error_msg);
        return;
    }
    
    if (callback == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Callback function is null");
        return;
    }
    
    if (_i2c_bus_device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Device not initialized");
        return;
    }
    
    // Pre-check interrupt conflict pairs using local snapshot if available
    for (uint8_t i = 0; i < PY32_MAX_GPIO_PINS; ++i) {
        if (i != pin && _pin_states[i].intr_enabled && _pinsConflict(pin, i)) {
            ESP_LOGE(_M5_io_expander_TAG, "Interrupt conflict between IO%u and IO%u", pin + 1, i + 1);
            return;
        }
    }

    // Store callback information
    _interrupt_callbacks[pin].callback = callback;
    _interrupt_callbacks[pin].callback_arg = nullptr;
    _interrupt_callbacks[pin].arg = nullptr;
    _interrupt_callbacks[pin].enabled = true;
    
    // Set pin as input if not already
    uint16_t mode_reg = 0; _readRegister16(REG_GPIO_M_L, &mode_reg);
    mode_reg &= ~(1 << pin);
    _writeRegister16(REG_GPIO_M_L, mode_reg);
    
    // Configure interrupt type
    // Configure interrupt registers
    uint16_t ie_reg = 0, it_reg = 0;
    _readRegister16(REG_GPIO_IE_L, &ie_reg);
    _readRegister16(REG_GPIO_IT_L, &it_reg);
    if (mode == RISING) {
        ie_reg |= (1 << pin);
        it_reg |= (1 << pin);
    } else if (mode == FALLING) {
        ie_reg |= (1 << pin);
        it_reg &= ~(1 << pin);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid interrupt mode: %d", mode);
        return;
    }
    esp_err_t ret = (_writeRegister16(REG_GPIO_IE_L, ie_reg) && _writeRegister16(REG_GPIO_IT_L, it_reg)) ? ESP_OK : ESP_FAIL;
    if (ret == ESP_OK) {
        ESP_LOGI(_M5_io_expander_TAG, "Interrupt attached to pin %d with mode %s", pin, 
                 (mode == RISING ? "RISING" : "FALLING"));
    _pin_states[pin].intr_enabled = true;
    _pin_states[pin].intr_rising  = (mode == RISING);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to attach interrupt to pin %d", pin);
    }
}

// Arduino-style attachInterruptArg function
void m5_io_py32ioexpander::attachInterruptArg(uint8_t pin, void (*callback)(void*), void* arg, uint8_t mode)
{
    // Validate interrupt configuration before proceeding
    py32_config_validation_t validation = _validatePinConfig(pin, PY32_CONFIG_TYPE_GPIO_INTERRUPT, true);
    if (!validation.valid) {
        ESP_LOGE(_M5_io_expander_TAG, "attachInterruptArg validation failed: %s", validation.error_msg);
        return;
    }
    
    if (callback == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Callback function is null");
        return;
    }
    
    if (_i2c_bus_device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Device not initialized");
        return;
    }
    
    // Pre-check interrupt conflict pairs using local snapshot if available
    for (uint8_t i = 0; i < PY32_MAX_GPIO_PINS; ++i) {
        if (i != pin && _pin_states[i].intr_enabled && _pinsConflict(pin, i)) {
            ESP_LOGE(_M5_io_expander_TAG, "Interrupt conflict between IO%u and IO%u", pin + 1, i + 1);
            return;
        }
    }

    // Store callback information
    _interrupt_callbacks[pin].callback = nullptr;
    _interrupt_callbacks[pin].callback_arg = callback;
    _interrupt_callbacks[pin].arg = arg;
    _interrupt_callbacks[pin].enabled = true;
    
    // Set pin as input if not already
    uint16_t mode_reg = 0; _readRegister16(REG_GPIO_M_L, &mode_reg);
    mode_reg &= ~(1 << pin);
    _writeRegister16(REG_GPIO_M_L, mode_reg);
    
    // Configure interrupt type
    // Configure interrupt registers
    uint16_t ie_reg = 0, it_reg = 0;
    _readRegister16(REG_GPIO_IE_L, &ie_reg);
    _readRegister16(REG_GPIO_IT_L, &it_reg);
    if (mode == RISING) {
        ie_reg |= (1 << pin);
        it_reg |= (1 << pin);
    } else if (mode == FALLING) {
        ie_reg |= (1 << pin);
        it_reg &= ~(1 << pin);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid interrupt mode: %d", mode);
        return;
    }
    esp_err_t ret = (_writeRegister16(REG_GPIO_IE_L, ie_reg) && _writeRegister16(REG_GPIO_IT_L, it_reg)) ? ESP_OK : ESP_FAIL;
    if (ret == ESP_OK) {
        ESP_LOGI(_M5_io_expander_TAG, "Interrupt attached to pin %d with mode %s", pin, 
                 (mode == RISING ? "RISING" : "FALLING"));
    _pin_states[pin].intr_enabled = true;
    _pin_states[pin].intr_rising  = (mode == RISING);
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to attach interrupt to pin %d", pin);
    }
}

// Arduino-style detachInterrupt function
void m5_io_py32ioexpander::detachInterrupt(uint8_t pin)
{
    if (!_isValidPin(pin)) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
        return;
    }
    
    if (_i2c_bus_device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Device not initialized");
        return;
    }
    
    // Disable interrupt
    uint16_t ie_reg = 0; _readRegister16(REG_GPIO_IE_L, &ie_reg);
    ie_reg &= ~(1 << pin);
    _writeRegister16(REG_GPIO_IE_L, ie_reg);
    
    // Clear callback information
    _interrupt_callbacks[pin].callback = nullptr;
    _interrupt_callbacks[pin].callback_arg = nullptr;
    _interrupt_callbacks[pin].arg = nullptr;
    _interrupt_callbacks[pin].enabled = false;
    _pin_states[pin].intr_enabled    = false;
    
    ESP_LOGI(_M5_io_expander_TAG, "Interrupt detached from pin %d", pin);
}

// Enable interrupt on a specific pin
void m5_io_py32ioexpander::enableInterrupt(uint8_t pin)
{
    if (!_isValidPin(pin)) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
        return;
    }
    
    _interrupt_callbacks[pin].enabled = true;
    ESP_LOGI(_M5_io_expander_TAG, "Interrupt enabled on pin %d", pin);
}

// Disable interrupt on a specific pin
void m5_io_py32ioexpander::disableInterrupt(uint8_t pin)
{
    if (!_isValidPin(pin)) {
        ESP_LOGE(_M5_io_expander_TAG, "Invalid pin number: %d", pin);
        return;
    }
    
    _interrupt_callbacks[pin].enabled = false;
    ESP_LOGI(_M5_io_expander_TAG, "Interrupt disabled on pin %d", pin);
}

// ====================================================================================
// Extended Functions - Device Information
// ====================================================================================

bool m5_io_py32ioexpander::readDeviceUID(uint16_t* uid)
{
    if (uid == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_UID_L, uid);
}

bool m5_io_py32ioexpander::readVersion(uint8_t* hw_version, uint8_t* fw_version)
{
    if (hw_version == nullptr || fw_version == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint8_t reg = 0;
    if (!_readRegister(REG_VERSION, &reg)) return false;
    *hw_version = (reg >> 4) & 0x0F;
    *fw_version = reg & 0x0F;
    return true;
}

// ====================================================================================
// Extended Functions - Advanced GPIO
// ====================================================================================

bool m5_io_py32ioexpander::setPullMode(uint8_t pin, uint8_t pull_mode)
{
    if (!_isValidPin(pin)) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint16_t pu_reg = 0, pd_reg = 0;
    if (!_readRegister16(REG_GPIO_PU_L, &pu_reg)) return false;
    if (!_readRegister16(REG_GPIO_PD_L, &pd_reg)) return false;

    pu_reg &= ~(1 << pin);
    pd_reg &= ~(1 << pin);
    if (pull_mode == PULL_UP)      pu_reg |= (1 << pin);
    else if (pull_mode == PULL_DOWN) pd_reg |= (1 << pin);

    bool ok = _writeRegister16(REG_GPIO_PU_L, pu_reg) && _writeRegister16(REG_GPIO_PD_L, pd_reg);
    if (ok) {
        _pin_states[pin].pull = (pull_mode == PULL_UP) ? 1 : ((pull_mode == PULL_DOWN) ? 2 : 0);
        // Auto-snapshot after configuration change
        _autoSnapshot();
    }
    return ok;
}

bool m5_io_py32ioexpander::setDriveMode(uint8_t pin, uint8_t drive_mode)
{
    if (!_isValidPin(pin)) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint16_t drv_reg = 0;
    if (!_readRegister16(REG_GPIO_DRV_L, &drv_reg)) return false;
    if (drive_mode == 1) drv_reg |=  (1 << pin);   // open-drain
    else                 drv_reg &= ~(1 << pin);   // push-pull
    bool ok = _writeRegister16(REG_GPIO_DRV_L, drv_reg);
    if (ok) {
        _pin_states[pin].drive = (drive_mode == 1) ? 1 : 0;
        // Auto-snapshot after configuration change
        _autoSnapshot();
    }
    return ok;
}

uint16_t m5_io_py32ioexpander::getInterruptStatus()
{
    if (_i2c_bus_device_handle == nullptr) return 0;
    
    uint16_t status = 0;
    if (_readRegister16(REG_GPIO_IS_L, &status)) return status;
    return 0;
}

bool m5_io_py32ioexpander::clearInterrupt(uint8_t pin)
{
    if (!_isValidPin(pin)) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint16_t is_reg = 0;
    if (!_readRegister16(REG_GPIO_IS_L, &is_reg)) return false;
    is_reg &= ~(1 << pin); // write 0 to clear bit
    return _writeRegister16(REG_GPIO_IS_L, is_reg);
}

// ====================================================================================
// Extended Functions - ADC
// ====================================================================================

bool m5_io_py32ioexpander::analogRead(uint8_t channel, uint16_t* result)
{
    if (result == nullptr || channel < 1 || channel > 4) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Convert channel to pin number for validation (channel 1-4 maps to pins IO2,IO4,IO5,IO7)
    uint8_t pin = (channel == 1) ? 1 : (channel == 2) ? 3 : (channel == 3) ? 4 : 6;  // 0-based pin numbers
    
    // Validate ADC configuration before proceeding
    py32_config_validation_t validation = _validatePinConfig(pin, PY32_CONFIG_TYPE_ADC, true);
    if (!validation.valid) {
        ESP_LOGE(_M5_io_expander_TAG, "analogRead validation failed: %s", validation.error_msg);
        return false;
    }

    // Start conversion
    uint8_t adc_ctrl = (channel & ADC_CTRL_CH_MASK) | ADC_CTRL_START;
    if (!_writeRegister(REG_ADC_CTRL, adc_ctrl)) return false;

    // Wait until not busy (with timeout)
    uint8_t tries = 0; uint8_t reg = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (!_readRegister(REG_ADC_CTRL, &reg)) return false;
        tries++;
    } while ((reg & ADC_CTRL_BUSY) && tries < 20);
    if (reg & ADC_CTRL_BUSY) return false; // timeout

    // Read 16-bit result (12-bit valid)
    bool read_ok = _readRegister16(REG_ADC_D_L, result);
    if (read_ok) {
        // Auto-snapshot after successful ADC conversion
        _autoSnapshot();
    }
    return read_ok;
}

bool m5_io_py32ioexpander::isAdcBusy()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    uint8_t ctrl = 0;
    if (_readRegister(REG_ADC_CTRL, &ctrl)) return (ctrl & ADC_CTRL_BUSY) != 0;
    return false;
}

bool m5_io_py32ioexpander::disableAdc()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    // Disable ADC by writing 0 (channel disable, no START)
    bool ok = _writeRegister(REG_ADC_CTRL, 0);
    if (ok) {
        // Auto-snapshot after ADC configuration change
        _autoSnapshot();
    }
    return ok;
}

// ====================================================================================
// Extended Functions - Temperature Sensor
// ====================================================================================

bool m5_io_py32ioexpander::readTemperature(uint16_t* temperature)
{
    if (temperature == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Start temperature conversion
    if (!_writeRegister(REG_TEMP_CTRL, TEMP_CTRL_START)) return false;

    // Wait until not busy (with timeout)
    uint8_t temp_ctrl = 0; uint8_t tries = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (!_readRegister(REG_TEMP_CTRL, &temp_ctrl)) return false;
        tries++;
    } while ((temp_ctrl & TEMP_CTRL_BUSY) && tries < 20);
    if (temp_ctrl & TEMP_CTRL_BUSY) return false;

    return _readRegister16(REG_TEMP_D_L, temperature);
}

bool m5_io_py32ioexpander::isTemperatureBusy()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    uint8_t temp_ctrl = 0;
    if (_readRegister(REG_TEMP_CTRL, &temp_ctrl)) return (temp_ctrl & TEMP_CTRL_BUSY) != 0;
    return false;
}

// ====================================================================================
// Extended Functions - PWM
// ====================================================================================

bool m5_io_py32ioexpander::setPwmFrequency(uint16_t frequency)
{
    if (_i2c_bus_device_handle == nullptr) return false;

    // Write 16-bit little-endian to PWM frequency registers
    bool ok = _writeRegister16(REG_PWM_FREQ_L, frequency);
    if (ok) {
        // Auto-snapshot after configuration change
        _autoSnapshot();
    }
    return ok;
}

bool m5_io_py32ioexpander::getPwmFrequency(uint16_t* frequency)
{
    if (frequency == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Read 16-bit little-endian from PWM frequency registers
    return _readRegister16(REG_PWM_FREQ_L, frequency);
}

bool m5_io_py32ioexpander::setPwmDuty(uint8_t channel, uint8_t duty_percentage, bool polarity, bool enable)
{
    if (channel > 3 || duty_percentage > 100) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Convert channel to pin number for validation (channel 0-3 maps to pins IO9,IO8,IO11,IO10)
    uint8_t pin = (channel == 0) ? 8 : (channel == 1) ? 7 : (channel == 2) ? 10 : 9;  // 0-based pin numbers
    
    // Validate PWM configuration before proceeding (only validate if enabling)
    if (enable) {
        py32_config_validation_t validation = _validatePinConfig(pin, PY32_CONFIG_TYPE_PWM, true);
        if (!validation.valid) {
            ESP_LOGE(_M5_io_expander_TAG, "setPwmDuty validation failed: %s", validation.error_msg);
            return false;
        }
    }

    // Convert 0-100% to 12-bit value (0-0x0FFF)
    uint16_t duty12 = (uint16_t)((duty_percentage * 0x0FFF) / 100);

    // Add control bits into high byte
    if (enable)   duty12 |= ((uint16_t)PWM_ENABLE << 8);
    if (polarity) duty12 |= ((uint16_t)PWM_POLARITY << 8);

    uint8_t reg_addr_l = REG_PWM1_DUTY_L + (channel * 2);
    bool ok = _writeRegister16(reg_addr_l, duty12);
    if (ok) {
        // Auto-snapshot after configuration change
        _autoSnapshot();
    }
    return ok;
}

bool m5_io_py32ioexpander::setPwmDuty12b(uint8_t channel, uint16_t duty12, bool polarity, bool enable)
{
    if (channel > 3 || duty12 > 0x0FFF) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Convert channel to pin number for validation (channel 0-3 maps to pins IO9,IO8,IO11,IO10)
    uint8_t pin = (channel == 0) ? 8 : (channel == 1) ? 7 : (channel == 2) ? 10 : 9;  // 0-based pin numbers
    
    // Validate PWM configuration before proceeding (only validate if enabling)
    if (enable) {
        py32_config_validation_t validation = _validatePinConfig(pin, PY32_CONFIG_TYPE_PWM, true);
        if (!validation.valid) {
            ESP_LOGE(_M5_io_expander_TAG, "setPwmDuty12b validation failed: %s", validation.error_msg);
            return false;
        }
    }

    // Compose high byte: duty[11:8] | control bits
    uint8_t reg_addr_l = REG_PWM1_DUTY_L + (channel * 2);
    uint8_t new_l = (uint8_t)(duty12 & 0xFF);
    uint8_t new_h = (uint8_t)(((duty12 >> 8) & 0x0F) | (polarity ? (PWM_POLARITY) : 0) | (enable ? (PWM_ENABLE) : 0));
    uint8_t data[2] = {new_l, new_h};
    bool ok = (i2c_bus_write_bytes(_i2c_bus_device_handle, reg_addr_l, 2, data) == ESP_OK);
    if (ok) {
        // Auto-snapshot after configuration change
        _autoSnapshot();
    }
    return ok;
}

bool m5_io_py32ioexpander::getPwmDuty(uint8_t channel, uint8_t* duty, bool* polarity, bool* enable)
{
    if (channel > 3 || duty == nullptr || polarity == nullptr || enable == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint8_t reg_addr_l = REG_PWM1_DUTY_L + (channel * 2);
    uint16_t duty16 = 0;
    if (!_readRegister16(reg_addr_l, &duty16)) return false;

    uint16_t duty12 = duty16 & 0x0FFF;
    *duty = (uint8_t)((duty12 * 100) / 0x0FFF);
    *polarity = (duty16 & ((uint16_t)PWM_POLARITY << 8)) != 0;
    *enable   = (duty16 & ((uint16_t)PWM_ENABLE << 8)) != 0;
    return true;
}

bool m5_io_py32ioexpander::getPwmDutyRaw(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enable)
{
    if (channel > 3 || duty12 == nullptr || polarity == nullptr || enable == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;
    uint8_t reg_addr_l = REG_PWM1_DUTY_L + (channel * 2);
    uint16_t duty16 = 0;
    if (!_readRegister16(reg_addr_l, &duty16)) return false;
    *duty12  = duty16 & 0x0FFF;               // 12-bit raw duty
    *polarity = (duty16 & ((uint16_t)PWM_POLARITY << 8)) != 0;
    *enable   = (duty16 & ((uint16_t)PWM_ENABLE << 8)) != 0;
    return true;
}

// ====================================================================================
// Extended Functions - LED Control
// ====================================================================================

bool m5_io_py32ioexpander::setLedCount(uint8_t num_leds)
{
    if (num_leds > 32) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Validate Neopixel configuration before proceeding (IO14 = pin index 13)
    if (num_leds > 0) {
        py32_config_validation_t validation = _validatePinConfig(13, PY32_CONFIG_TYPE_NEOPIXEL, true);
        if (!validation.valid) {
            ESP_LOGE(_M5_io_expander_TAG, "setLedCount validation failed: %s", validation.error_msg);
            return false;
        }
    }

    uint8_t led_cfg = num_leds & LED_CFG_NUM_MASK;
    return _writeRegister(REG_LED_CFG, led_cfg);
}

bool m5_io_py32ioexpander::setLedColor(uint8_t led_index, py32_rgb_color_t color)
{
    if (led_index > 31) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    // Convert to RGB565
    uint16_t r5 = (color.r >> 3) & 0x1F;
    uint16_t g6 = (color.g >> 2) & 0x3F;
    uint16_t b5 = (color.b >> 3) & 0x1F;
    uint16_t rgb565 = (uint16_t)((r5 << 11) | (g6 << 5) | b5);

    uint8_t reg_addr = REG_LED_RAM_START + (led_index * 2);
    uint8_t data[2] = {(uint8_t)((rgb565 >> 8) & 0xFF), (uint8_t)(rgb565 & 0xFF)}; // high first
    return (i2c_bus_write_bytes(_i2c_bus_device_handle, reg_addr, 2, data) == ESP_OK);
}

bool m5_io_py32ioexpander::setLedColor(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b)
{
    py32_rgb_color_t color = {r, g, b};
    return setLedColor(led_index, color);
}

bool m5_io_py32ioexpander::refreshLeds()
{
    if (_i2c_bus_device_handle == nullptr) return false;
    // Check LED count is set (>0) before refresh
    uint8_t led_cfg = 0;
    if (i2c_bus_read_byte(_i2c_bus_device_handle, REG_LED_CFG, &led_cfg) != ESP_OK) {
        // read failed; proceed but warn
        ESP_LOGW(_M5_io_expander_TAG, "Failed to read LED_CFG before refresh; proceeding");
    }
    if ((led_cfg & LED_CFG_NUM_MASK) == 0) {
        ESP_LOGW(_M5_io_expander_TAG, "LED count is 0; call setLedCount() before refreshLeds()");
    }
    // set REFRESH bit
    led_cfg |= LED_CFG_REFRESH;
    return _writeRegister(REG_LED_CFG, led_cfg);
}

bool m5_io_py32ioexpander::disableLeds()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    // set LED count to 0 to disable
    return _writeRegister(REG_LED_CFG, 0);
}

// ====================================================================================
// Extended Functions - RTC RAM
// ====================================================================================

bool m5_io_py32ioexpander::writeRtcRam(uint8_t offset, const uint8_t* data, uint8_t length)
{
    if (data == nullptr || offset > 31 || length == 0 || (offset + length) > 32) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint8_t reg_addr = REG_RTC_RAM_START + offset;
    return (i2c_bus_write_bytes(_i2c_bus_device_handle, reg_addr, length, (uint8_t*)data) == ESP_OK);
}

bool m5_io_py32ioexpander::readRtcRam(uint8_t offset, uint8_t* data, uint8_t length)
{
    if (data == nullptr || offset > 31 || length == 0 || (offset + length) > 32) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    uint8_t reg_addr = REG_RTC_RAM_START + offset;
    return (i2c_bus_read_bytes(_i2c_bus_device_handle, reg_addr, length, data) == ESP_OK);
}

// ====================================================================================
// Extended Functions - System Configuration
// ====================================================================================

bool m5_io_py32ioexpander::setI2cConfig(uint8_t sleep_time, bool speed_400k, bool wake_mode, bool inter_pull_off)
{
    if (_i2c_bus_device_handle == nullptr) return false;

    if (sleep_time > 15) return false;
    
    // Validate I2C sleep configuration if enabling sleep
    if (sleep_time > 0) {
        py32_config_validation_t validation = _validatePinConfig(4, PY32_CONFIG_TYPE_I2C_SLEEP, true);  // IO5 = pin index 4
        if (!validation.valid) {
            ESP_LOGE(_M5_io_expander_TAG, "setI2cConfig validation failed: %s", validation.error_msg);
            return false;
        }
    }
    
    uint8_t i2c_cfg = (sleep_time & I2C_CFG_SLEEP_MASK) |
                      (speed_400k ? I2C_CFG_SPEED_400K : 0) |
                      (wake_mode ? I2C_CFG_WAKE_RISING : I2C_CFG_WAKE_FALLING) |
                      (inter_pull_off ? I2C_CFG_INTER_PULL_OFF : I2C_CFG_INTER_PULL_ON);
    bool ok = _writeRegister(REG_I2C_CFG, i2c_cfg);
    if (ok) {
        if (sleep_time > 0) {
            ESP_LOGW(_M5_io_expander_TAG, "I2C idle sleep enabled (%u s); IO5 interrupt will be disabled per spec", sleep_time);
        }
        // If any PWM enabled, sleep ineffective per manual
        if (sleep_time > 0) {
            uint8_t any_pwm_enabled = 0;
            for (uint8_t ch = 0; ch < 4; ++ch) {
                uint8_t duty = 0; bool pol = false; bool en = false;
                if (getPwmDuty(ch, &duty, &pol, &en) && en) {
                    any_pwm_enabled = 1; break;
                }
            }
            if (any_pwm_enabled) {
                ESP_LOGW(_M5_io_expander_TAG, "PWM enabled; I2C idle sleep will be ignored by device");
            }
        }
    }
    return ok;
}

bool m5_io_py32ioexpander::getRefVoltage(uint16_t* ref_voltage)
{
    if (ref_voltage == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_REF_VOLTAGE_L, ref_voltage);
}

bool m5_io_py32ioexpander::factoryReset()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    bool ok = _writeRegister(REG_FACTORY_RESET, FACTORY_RESET_TRIGGER);
    if (ok) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (ok) {
        // After factory reset, mark not initialized and clear all states
        // Device will restart and I2C frequency will reset to 100KHz
        // User should call init() again to restore proper configuration
        _initialized_ok    = false;
        _pin_states_valid  = false;
        _pwm_states_valid  = false;
        _adc_state_valid   = false;
        _clearPinStates();
        _clearPwmStates();
        _clearAdcState();
        
        ESP_LOGW(_M5_io_expander_TAG, "Factory reset complete. Device requires re-initialization (call init() again)");
    }
    return ok;
}

// ====================================================================================
// Extended Functions - Debug
// ====================================================================================

bool m5_io_py32ioexpander::getGpioModeReg(uint16_t* mode_reg)
{
    if (mode_reg == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_GPIO_M_L, mode_reg);
}

bool m5_io_py32ioexpander::getGpioOutputReg(uint16_t* output_reg)
{
    if (output_reg == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_GPIO_O_L, output_reg);
}

bool m5_io_py32ioexpander::getGpioInputReg(uint16_t* input_reg)
{
    if (input_reg == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_GPIO_I_L, input_reg);
}

bool m5_io_py32ioexpander::getGpioPullUpReg(uint16_t* pull_up_reg)
{
    if (pull_up_reg == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_GPIO_PU_L, pull_up_reg);
}

bool m5_io_py32ioexpander::getGpioPullDownReg(uint16_t* pull_down_reg)
{
    if (pull_down_reg == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_GPIO_PD_L, pull_down_reg);
}

bool m5_io_py32ioexpander::getGpioDriveReg(uint16_t* drive_reg)
{
    if (drive_reg == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;

    return _readRegister16(REG_GPIO_DRV_L, drive_reg);
}

// ====================================================================================
// Private Helper Functions
// ====================================================================================

bool m5_io_py32ioexpander::_writeRegister(uint8_t reg, uint8_t value)
{
    if (_i2c_bus_device_handle == nullptr) return false;
    return (i2c_bus_write_byte(_i2c_bus_device_handle, reg, value) == ESP_OK);
}

bool m5_io_py32ioexpander::_writeRegister16(uint8_t reg, uint16_t value)
{
    if (_i2c_bus_device_handle == nullptr) return false;
    uint8_t data[2] = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
    return (i2c_bus_write_bytes(_i2c_bus_device_handle, reg, 2, data) == ESP_OK);
}

bool m5_io_py32ioexpander::_readRegister(uint8_t reg, uint8_t* value)
{
    if (value == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;
    return (i2c_bus_read_byte(_i2c_bus_device_handle, reg, value) == ESP_OK);
}

bool m5_io_py32ioexpander::_readRegister16(uint8_t reg, uint16_t* value)
{
    if (value == nullptr) return false;
    if (_i2c_bus_device_handle == nullptr) return false;
    uint8_t data[2];
    if (i2c_bus_read_bytes(_i2c_bus_device_handle, reg, 2, data) == ESP_OK) {
        *value = (data[1] << 8) | data[0];
        return true;
    }
    return false;
}

bool m5_io_py32ioexpander::_isValidPin(uint8_t pin)
{
    return (pin < PY32_MAX_GPIO_PINS);
}

bool m5_io_py32ioexpander::_isAdcPin(uint8_t pin)
{
    return (pin == M5_IO_NUM_2 || pin == M5_IO_NUM_4 || 
            pin == M5_IO_NUM_5 || pin == M5_IO_NUM_7);
}

bool m5_io_py32ioexpander::_isPwmPin(uint8_t pin)
{
    return (pin == M5_IO_NUM_8 || pin == M5_IO_NUM_9 || 
            pin == M5_IO_NUM_10 || pin == M5_IO_NUM_11);
}

uint8_t m5_io_py32ioexpander::_getAdcChannel(uint8_t pin)
{
    switch (pin) {
        case M5_IO_NUM_2: return PY32_ADC_CHANNEL_1;
        case M5_IO_NUM_4: return PY32_ADC_CHANNEL_2;
        case M5_IO_NUM_5: return PY32_ADC_CHANNEL_3;
        case M5_IO_NUM_7: return PY32_ADC_CHANNEL_4;
        default: return 0;
    }
}

uint8_t m5_io_py32ioexpander::_getPwmChannel(uint8_t pin)
{
    switch (pin) {
        case M5_IO_NUM_9: return PY32_PWM_CHANNEL_1;
        case M5_IO_NUM_8: return PY32_PWM_CHANNEL_2;
        case M5_IO_NUM_11: return PY32_PWM_CHANNEL_3;
        case M5_IO_NUM_10: return PY32_PWM_CHANNEL_4;
        default: return 0;
    }
}

// Initialize the internal device
bool m5_io_py32ioexpander::_initializeDevice()
{
    if (_i2c_bus_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "I2C bus handle is null");
        return false;
    }
    
    // 使用基类的双向传递机制创建设备句�?
    i2c_bus_device_handle_t device_handle = create_device_handle(_device_addr);
    if (device_handle == nullptr) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to create I2C device handle");
        return false;
    }

    // Read and display device information
    uint16_t uid;
    uint8_t hw_version, fw_version;
    
    if (readDeviceUID(&uid)) {
        ESP_LOGI(_M5_io_expander_TAG, "Device UID: 0x%04X", uid);
    }
    
    if (readVersion(&hw_version, &fw_version)) {
        ESP_LOGI(_M5_io_expander_TAG, "Device Version - HW: %d, FW: %d", hw_version, fw_version);
    }
    
    // Snapshot IO registers at end of init is handled by caller
    return true;
}

// Setup interrupt handling
bool m5_io_py32ioexpander::_setupInterrupts()
{
    if (_int_pin == GPIO_NUM_NC) {
        ESP_LOGW(_M5_io_expander_TAG, "No interrupt pin specified");
        return true;  // Not an error, just no interrupt support
    }
    
    // Configure interrupt pin: low-active INT, external pull-up required
    // We enable internal pull-up on ESP32 side and use low-level interrupt
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << _int_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to configure interrupt pin: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Create queue & task first
    if (_intr_queue == nullptr) {
        _intr_queue = xQueueCreate(8, sizeof(uint32_t));
        if (_intr_queue == nullptr) {
            ESP_LOGE(_M5_io_expander_TAG, "Failed to create interrupt queue");
            return false;
        }
    }
    if (_intr_task == nullptr) {
        BaseType_t ok = xTaskCreatePinnedToCore(_interruptTask, "py32_intr", 4096, this, configMAX_PRIORITIES - 2, &_intr_task, tskNO_AFFINITY);
        if (ok != pdPASS) {
            ESP_LOGE(_M5_io_expander_TAG, "Failed to create interrupt task");
            vQueueDelete(_intr_queue); _intr_queue = nullptr;
            return false;
        }
    }

    // Install ISR service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // ESP_ERR_INVALID_STATE means already installed
        ESP_LOGE(_M5_io_expander_TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        if (_intr_task) { vTaskDelete(_intr_task); _intr_task = nullptr; }
        if (_intr_queue) { vQueueDelete(_intr_queue); _intr_queue = nullptr; }
        return false;
    }
    
    // Add ISR handler
    ret = gpio_isr_handler_add(_int_pin, _interruptHandler, this);
    if (ret != ESP_OK) {
        ESP_LOGE(_M5_io_expander_TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        if (_intr_task) { vTaskDelete(_intr_task); _intr_task = nullptr; }
        if (_intr_queue) { vQueueDelete(_intr_queue); _intr_queue = nullptr; }
        return false;
    }
    
    ESP_LOGI(_M5_io_expander_TAG, "Interrupt setup complete on pin %d", _int_pin);
    return true;
}

// ISR handler
void IRAM_ATTR m5_io_py32ioexpander::_interruptHandler(void* arg)
{
    m5_io_py32ioexpander* instance = static_cast<m5_io_py32ioexpander*>(arg);
    if (instance && instance->_intr_queue) {
        uint32_t evt = 1;
        BaseType_t hpw = pdFALSE;
        xQueueSendFromISR(instance->_intr_queue, &evt, &hpw);
        if (hpw == pdTRUE) portYIELD_FROM_ISR();
    }
}

// Handle interrupt
void m5_io_py32ioexpander::_handleInterrupt()
{
    // Read interrupt status
    uint16_t status = getInterruptStatus();
    
    // Process each pin
    for (uint8_t pin = 0; pin < PY32_MAX_GPIO_PINS; pin++) {
        if ((status & (1 << pin)) && _interrupt_callbacks[pin].enabled) {
            // Call appropriate callback
            if (_interrupt_callbacks[pin].callback_arg != nullptr) {
                _interrupt_callbacks[pin].callback_arg(_interrupt_callbacks[pin].arg);
            } else if (_interrupt_callbacks[pin].callback != nullptr) {
                _interrupt_callbacks[pin].callback();
            }
            
            // Clear interrupt
            clearInterrupt(pin);
        }
    }
}

void m5_io_py32ioexpander::_interruptTask(void* arg)
{
    m5_io_py32ioexpander* self = static_cast<m5_io_py32ioexpander*>(arg);
    uint32_t evt;
    while (true) {
        if (xQueueReceive(self->_intr_queue, &evt, portMAX_DELAY) == pdTRUE) {
            // Mask ESP32 gpio interrupt during processing (level-trigger)
            gpio_intr_disable(self->_int_pin);
            // Process repeatedly until INT line returns high
            int guard = 0;
            do {
                self->_handleInterrupt();
                // small yield to allow I2C + callbacks to progress
                vTaskDelay(pdMS_TO_TICKS(1));
                guard++;
            } while (gpio_get_level(self->_int_pin) == 0 && guard < 64);
            // Re-enable after handled
            gpio_intr_enable(self->_int_pin);
        }
    }
}

void m5_io_py32ioexpander::_clearPinStates()
{
    memset(_pin_states, 0, sizeof(_pin_states));
}

void m5_io_py32ioexpander::_clearPwmStates()
{
    memset(_pwm_states, 0, sizeof(_pwm_states));
    _pwm_frequency = 0;
}

void m5_io_py32ioexpander::_clearAdcState()
{
    memset(&_adc_state, 0, sizeof(_adc_state));
}

bool m5_io_py32ioexpander::_snapshotIoState()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    uint16_t mode_reg = 0, out_reg = 0, in_reg = 0, pu_reg = 0, pd_reg = 0, drv_reg = 0, ie_reg = 0, it_reg = 0;
    bool ok =
        _readRegister16(REG_GPIO_M_L, &mode_reg) &&
        _readRegister16(REG_GPIO_O_L, &out_reg) &&
        _readRegister16(REG_GPIO_I_L, &in_reg) &&
        _readRegister16(REG_GPIO_PU_L, &pu_reg) &&
        _readRegister16(REG_GPIO_PD_L, &pd_reg) &&
        _readRegister16(REG_GPIO_DRV_L, &drv_reg);

    if (!ok) return false;

    uint8_t tmp2[2] = {0};
    if (i2c_bus_read_bytes(_i2c_bus_device_handle, REG_GPIO_IE_L, 2, tmp2) != ESP_OK) return false;
    ie_reg = (uint16_t)tmp2[0] | ((uint16_t)tmp2[1] << 8);
    if (i2c_bus_read_bytes(_i2c_bus_device_handle, REG_GPIO_IT_L, 2, tmp2) != ESP_OK) return false;
    it_reg = (uint16_t)tmp2[0] | ((uint16_t)tmp2[1] << 8);

    for (uint8_t pin = 0; pin < PY32_MAX_GPIO_PINS; ++pin) {
        _pin_states[pin].is_output    = (mode_reg & (1 << pin)) != 0;
        _pin_states[pin].output_level = (out_reg & (1 << pin)) ? 1 : 0;
        _pin_states[pin].input_level  = (in_reg & (1 << pin)) ? 1 : 0;
        _pin_states[pin].pull         = (pu_reg & (1 << pin)) ? 1 : ((pd_reg & (1 << pin)) ? 2 : 0);
        _pin_states[pin].drive        = (drv_reg & (1 << pin)) ? 1 : 0;
        _pin_states[pin].intr_enabled = (ie_reg & (1 << pin)) != 0;
        _pin_states[pin].intr_rising  = (it_reg & (1 << pin)) != 0;
    }
    return true;
}

bool m5_io_py32ioexpander::_snapshotPwmState()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    // Read PWM frequency (16-bit)
    if (!_readRegister16(REG_PWM_FREQ_L, &_pwm_frequency)) return false;

    // Read each PWM channel state
    for (uint8_t ch = 0; ch < PY32_MAX_PWM_CHANNELS; ++ch) {
        uint8_t reg_addr_l = REG_PWM1_DUTY_L + (ch * 2);
        uint16_t duty16 = 0;
        if (!_readRegister16(reg_addr_l, &duty16)) return false;

        _pwm_states[ch].duty12   = duty16 & 0x0FFF;  // 12-bit duty value
        _pwm_states[ch].enabled  = (duty16 & ((uint16_t)PWM_ENABLE << 8)) != 0;
        _pwm_states[ch].polarity = (duty16 & ((uint16_t)PWM_POLARITY << 8)) != 0;
    }

    return true;
}

bool m5_io_py32ioexpander::_snapshotAdcState()
{
    if (_i2c_bus_device_handle == nullptr) return false;

    // Read ADC control register
    uint8_t adc_ctrl = 0;
    if (!_readRegister(REG_ADC_CTRL, &adc_ctrl)) return false;

    _adc_state.active_channel = adc_ctrl & ADC_CTRL_CH_MASK;  // Current channel (0=disabled)
    _adc_state.busy = (adc_ctrl & ADC_CTRL_BUSY) != 0;        // Conversion in progress

    // Read last ADC value if not busy
    if (!_adc_state.busy) {
        if (!_readRegister16(REG_ADC_D_L, &_adc_state.last_value)) {
            _adc_state.last_value = 0;  // Set to 0 if read fails
        }
    }

    return true;
}

bool m5_io_py32ioexpander::_pinsConflict(uint8_t a, uint8_t b)
{
    // Conflict pairs: (1,6),(2,3),(7,12),(8,9),(10,14),(11,13)
    // Convert to 1-based IO numbers for easier comparison
    uint8_t A = a + 1, B = b + 1;
    auto eq = [&](uint8_t x, uint8_t y, uint8_t m, uint8_t n){ return (x==m && y==n) || (x==n && y==m); };
    return eq(A,B,1,6) || eq(A,B,2,3) || eq(A,B,7,12) || eq(A,B,8,9) || eq(A,B,10,14) || eq(A,B,11,13);
}

// ====================================================================================
// Configuration Validation Functions
// ====================================================================================

bool m5_io_py32ioexpander::_isInterruptMutexPin(uint8_t pin, uint8_t* mutex_pin)
{
    // Convert to 1-based IO numbers and check mutex pairs
    uint8_t io_num = pin + 1;
    switch (io_num) {
        case 1: if (mutex_pin) *mutex_pin = 5; return true;  // IO1 <-> IO6
        case 6: if (mutex_pin) *mutex_pin = 0; return true;  // IO6 <-> IO1
        case 2: if (mutex_pin) *mutex_pin = 2; return true;  // IO2 <-> IO3
        case 3: if (mutex_pin) *mutex_pin = 1; return true;  // IO3 <-> IO2
        case 7: if (mutex_pin) *mutex_pin = 11; return true; // IO7 <-> IO12
        case 12: if (mutex_pin) *mutex_pin = 6; return true; // IO12 <-> IO7
        case 8: if (mutex_pin) *mutex_pin = 8; return true;  // IO8 <-> IO9
        case 9: if (mutex_pin) *mutex_pin = 7; return true;  // IO9 <-> IO8
        case 10: if (mutex_pin) *mutex_pin = 13; return true; // IO10 <-> IO14
        case 14: if (mutex_pin) *mutex_pin = 9; return true;  // IO14 <-> IO10
        case 11: if (mutex_pin) *mutex_pin = 12; return true; // IO11 <-> IO13
        case 13: if (mutex_pin) *mutex_pin = 10; return true; // IO13 <-> IO11
        default: return false;
    }
}

bool m5_io_py32ioexpander::_isAdcPin(uint8_t pin, uint8_t* adc_channel)
{
    switch (pin + 1) {  // Convert to 1-based IO numbers
        case 2: if (adc_channel) *adc_channel = 1; return true;  // IO2 = ADC1
        case 4: if (adc_channel) *adc_channel = 2; return true;  // IO4 = ADC2
        case 5: if (adc_channel) *adc_channel = 3; return true;  // IO5 = ADC3
        case 7: if (adc_channel) *adc_channel = 4; return true;  // IO7 = ADC4
        default: return false;
    }
}

bool m5_io_py32ioexpander::_isPwmPin(uint8_t pin, uint8_t* pwm_channel)
{
    switch (pin + 1) {  // Convert to 1-based IO numbers
        case 9: if (pwm_channel) *pwm_channel = 1; return true;  // IO9 = PWM1
        case 8: if (pwm_channel) *pwm_channel = 2; return true;  // IO8 = PWM2
        case 11: if (pwm_channel) *pwm_channel = 3; return true; // IO11 = PWM3
        case 10: if (pwm_channel) *pwm_channel = 4; return true; // IO10 = PWM4
        default: return false;
    }
}

bool m5_io_py32ioexpander::_isNeopixelPin(uint8_t pin)
{
    return (pin + 1) == 14;  // IO14 = Neopixel
}

bool m5_io_py32ioexpander::_hasActiveInterrupt(uint8_t pin)
{
    if (!_pin_states_valid || pin >= PY32_MAX_GPIO_PINS) return false;
    return _pin_states[pin].intr_enabled;
}

bool m5_io_py32ioexpander::_hasActiveAdc(uint8_t pin)
{
    if (!_adc_state_valid) return false;
    
    uint8_t adc_channel;
    if (!_isAdcPin(pin, &adc_channel)) return false;
    
    return _adc_state.active_channel == adc_channel;
}

bool m5_io_py32ioexpander::_hasActivePwm(uint8_t pin)
{
    if (!_pwm_states_valid) return false;
    
    uint8_t pwm_channel;
    if (!_isPwmPin(pin, &pwm_channel)) return false;
    
    return _pwm_states[pwm_channel - 1].enabled;  // Convert to 0-based array index
}

bool m5_io_py32ioexpander::_hasI2cSleepEnabled()
{
    // Read I2C config register to check if sleep is enabled
    uint8_t i2c_cfg = 0;
    if (_i2c_bus_device_handle && _readRegister(REG_I2C_CFG, &i2c_cfg)) {
        return (i2c_cfg & I2C_CFG_SLEEP_MASK) != 0;
    }
    return false;
}

py32_config_validation_t m5_io_py32ioexpander::_validatePinConfig(uint8_t pin, py32_config_type_t config_type, bool enable_config)
{
    py32_config_validation_t result = {false, {0}, 0xFF};
    
    // Basic pin validation
    if (!_isValidPin(pin)) {
        snprintf(result.error_msg, sizeof(result.error_msg), "Invalid pin %d (valid range: 0-13)", pin);
        return result;
    }
    
    // Check if device is initialized and states are valid
    if (!_initialized_ok) {
        snprintf(result.error_msg, sizeof(result.error_msg), "Device not initialized");
        return result;
    }
    
    uint8_t io_num = pin + 1;  // Convert to 1-based IO numbers for user-friendly messages
    
    switch (config_type) {
        case PY32_CONFIG_TYPE_GPIO_INTERRUPT:
            if (enable_config) {
                // Check interrupt mutex conflicts
                uint8_t mutex_pin;
                if (_isInterruptMutexPin(pin, &mutex_pin)) {
                    if (_hasActiveInterrupt(mutex_pin)) {
                        result.conflicting_pin = mutex_pin;
                        snprintf(result.error_msg, sizeof(result.error_msg), 
                                "IO%d interrupt conflicts with active interrupt on IO%d", io_num, mutex_pin + 1);
                        return result;
                    }
                }
                
                // Check if ADC is active on this pin
                if (_hasActiveAdc(pin)) {
                    snprintf(result.error_msg, sizeof(result.error_msg), 
                            "IO%d interrupt blocked: ADC is active", io_num);
                    return result;
                }
                
                // Check if PWM is active on this pin
                if (_hasActivePwm(pin)) {
                    uint8_t pwm_channel;
                    _isPwmPin(pin, &pwm_channel);
                    snprintf(result.error_msg, sizeof(result.error_msg), 
                            "IO%d interrupt blocked: PWM%d is active", io_num, pwm_channel);
                    return result;
                }
                
                // Check if I2C sleep is enabled and this is IO5
                if (io_num == 5 && _hasI2cSleepEnabled()) {
                    snprintf(result.error_msg, sizeof(result.error_msg), 
                            "IO5 interrupt blocked: I2C idle sleep is enabled");
                    return result;
                }
            }
            break;
            
        case PY32_CONFIG_TYPE_ADC:
            if (enable_config) {
                // Check if this pin supports ADC
                uint8_t adc_channel;
                if (!_isAdcPin(pin, &adc_channel)) {
                    snprintf(result.error_msg, sizeof(result.error_msg), 
                            "IO%d does not support ADC function", io_num);
                    return result;
                }
                
                // ADC will disable interrupt on this pin (this is expected behavior)
                if (_hasActiveInterrupt(pin)) {
                    ESP_LOGW(_M5_io_expander_TAG, "IO%d interrupt will be automatically disabled due to ADC activation (hardware limitation)", io_num);
                }
            }
            break;
            
        case PY32_CONFIG_TYPE_PWM:
            if (enable_config) {
                // Check if this pin supports PWM
                uint8_t pwm_channel;
                if (!_isPwmPin(pin, &pwm_channel)) {
                    snprintf(result.error_msg, sizeof(result.error_msg), 
                            "IO%d does not support PWM function", io_num);
                    return result;
                }
                
                // PWM will disable interrupt on this pin (this is expected behavior)
                if (_hasActiveInterrupt(pin)) {
                    ESP_LOGW(_M5_io_expander_TAG, "IO%d interrupt will be automatically disabled due to PWM activation (hardware limitation)", io_num);
                }
            }
            break;
            
        case PY32_CONFIG_TYPE_NEOPIXEL:
            if (enable_config) {
                // Check if this pin supports Neopixel
                if (!_isNeopixelPin(pin)) {
                    snprintf(result.error_msg, sizeof(result.error_msg), 
                            "IO%d does not support Neopixel function", io_num);
                    return result;
                }
                
                // Check interrupt mutex conflict (IO14 conflicts with IO10)
                uint8_t mutex_pin;
                if (_isInterruptMutexPin(pin, &mutex_pin)) {
                    if (_hasActiveInterrupt(mutex_pin)) {
                        result.conflicting_pin = mutex_pin;
                        snprintf(result.error_msg, sizeof(result.error_msg), 
                                "IO%d Neopixel conflicts with active interrupt on IO%d", io_num, mutex_pin + 1);
                        return result;
                    }
                }
            }
            break;
            
        case PY32_CONFIG_TYPE_I2C_SLEEP:
            if (enable_config) {
                // I2C sleep will disable IO5 interrupt (this is expected behavior)
                if (_hasActiveInterrupt(4)) {  // IO5 = pin index 4
                    ESP_LOGW(_M5_io_expander_TAG, "IO5 interrupt will be automatically disabled due to I2C sleep activation (hardware limitation)");
                }
            }
            break;
            
        case PY32_CONFIG_TYPE_GPIO_INPUT:
        case PY32_CONFIG_TYPE_GPIO_OUTPUT:
            // GPIO input/output generally don't have conflicts, just check special function priority
            if (_hasActiveAdc(pin)) {
                uint8_t adc_channel;
                _isAdcPin(pin, &adc_channel);
                ESP_LOGW(_M5_io_expander_TAG, "IO%d GPIO config may be ineffective: ADC%d is active. Call disableAdc() if you need GPIO control.", io_num, adc_channel);
            }
            if (_hasActivePwm(pin)) {
                uint8_t pwm_channel;
                _isPwmPin(pin, &pwm_channel);
                ESP_LOGW(_M5_io_expander_TAG, "IO%d GPIO config may be ineffective: PWM%d is active. Call setPwmDuty(%d, 0, false, false) if you need GPIO control.", io_num, pwm_channel, pwm_channel - 1);
            }
            break;
    }
    
    result.valid = true;
    return result;
}

// ====================================================================================
// Auto-snapshot functions
// ====================================================================================

void m5_io_py32ioexpander::setAutoSnapshotEnabled(bool enable)
{
    _auto_snapshot_enabled = enable;
    ESP_LOGI(_M5_io_expander_TAG, "Auto-snapshot %s", enable ? "ENABLED" : "DISABLED");
}

bool m5_io_py32ioexpander::isAutoSnapshotEnabled() const
{
    return _auto_snapshot_enabled;
}

bool m5_io_py32ioexpander::triggerSnapshot()
{
    if (!_initialized_ok) {
        ESP_LOGW(_M5_io_expander_TAG, "Device not initialized, cannot trigger snapshot");
        return false;
    }
    
    bool gpio_ok = _snapshotIoState();
    bool pwm_ok  = _snapshotPwmState();
    bool adc_ok  = _snapshotAdcState();
    
    if (gpio_ok) {
        _pin_states_valid = true;
    }
    if (pwm_ok) {
        _pwm_states_valid = true;
    }
    if (adc_ok) {
        _adc_state_valid = true;
    }
    
    bool all_ok = gpio_ok && pwm_ok && adc_ok;
    if (all_ok) {
        ESP_LOGI(_M5_io_expander_TAG, "Manual snapshot triggered successfully");
    } else {
        ESP_LOGE(_M5_io_expander_TAG, "Manual snapshot partially failed (GPIO:%s PWM:%s ADC:%s)", 
                 gpio_ok ? "OK" : "FAIL", pwm_ok ? "OK" : "FAIL", adc_ok ? "OK" : "FAIL");
    }
    return all_ok;
}

void m5_io_py32ioexpander::_autoSnapshot()
{
    if (_auto_snapshot_enabled && _initialized_ok) {
        bool gpio_ok = _snapshotIoState();
        bool pwm_ok  = _snapshotPwmState();
        bool adc_ok  = _snapshotAdcState();
        
        if (gpio_ok) {
            _pin_states_valid = true;
        }
        if (pwm_ok) {
            _pwm_states_valid = true;
        }
        if (adc_ok) {
            _adc_state_valid = true;
        }
        
        if (!gpio_ok || !pwm_ok || !adc_ok) {
            ESP_LOGW(_M5_io_expander_TAG, "Auto-snapshot partially failed (GPIO:%s PWM:%s ADC:%s)", 
                     gpio_ok ? "OK" : "FAIL", pwm_ok ? "OK" : "FAIL", adc_ok ? "OK" : "FAIL");
        }
    }
}

// ====================================================================================
// State Query Functions (Cached Values)
// ====================================================================================

bool m5_io_py32ioexpander::getCachedPwmFrequency(uint16_t* frequency)
{
    if (frequency == nullptr) return false;
    if (!_initialized_ok || !_pwm_states_valid) return false;
    
    *frequency = _pwm_frequency;
    return true;
}

bool m5_io_py32ioexpander::getCachedPwmState(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enabled)
{
    if (channel >= PY32_MAX_PWM_CHANNELS || duty12 == nullptr || polarity == nullptr || enabled == nullptr) return false;
    if (!_initialized_ok || !_pwm_states_valid) return false;
    
    *duty12   = _pwm_states[channel].duty12;
    *polarity = _pwm_states[channel].polarity;
    *enabled  = _pwm_states[channel].enabled;
    return true;
}

bool m5_io_py32ioexpander::getCachedAdcState(uint8_t* active_channel, bool* busy, uint16_t* last_value)
{
    if (active_channel == nullptr || busy == nullptr || last_value == nullptr) return false;
    if (!_initialized_ok || !_adc_state_valid) return false;
    
    *active_channel = _adc_state.active_channel;
    *busy = _adc_state.busy;
    *last_value = _adc_state.last_value;
    return true;
}

// ====================================================================================
// Public Configuration Validation API
// ====================================================================================

py32_config_validation_t m5_io_py32ioexpander::validatePinConfig(uint8_t pin, py32_config_type_t config_type, bool enable_config)
{
    return _validatePinConfig(pin, config_type, enable_config);
}
