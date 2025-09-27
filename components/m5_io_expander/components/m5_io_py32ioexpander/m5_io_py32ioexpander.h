/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef M5_IO_PY32IOEXPANDER_H
#define M5_IO_PY32IOEXPANDER_H

#include "../../M5_io_expander.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "i2c_bus.h"

// M5 I/O map for PY32 IO Expander
#define M5_IO_NUM_NC -1
#define M5_IO_NUM_1   0
#define M5_IO_NUM_2   1
#define M5_IO_NUM_3   2
#define M5_IO_NUM_4   3
#define M5_IO_NUM_5   4
#define M5_IO_NUM_6   5
#define M5_IO_NUM_7   6
#define M5_IO_NUM_8   7
#define M5_IO_NUM_9   8
#define M5_IO_NUM_10  9
#define M5_IO_NUM_11  10
#define M5_IO_NUM_12  11
#define M5_IO_NUM_13  12
#define M5_IO_NUM_14  13

// PY32 IO Expander specific definitions
#define PY32_DEFAULT_I2C_ADDR       0x6F
#define PY32_MAX_GPIO_PINS          14
#define PY32_MAX_ADC_CHANNELS       4
#define PY32_MAX_PWM_CHANNELS       4
#define PY32_MAX_LED_COUNT          32

// ============================
// 寄存器地址定义
// ============================
// System
#define REG_UID_L            0x00
#define REG_UID_H            0x01
#define REG_VERSION          0x02
// GPIO
#define REG_GPIO_M_L         0x03
#define REG_GPIO_M_H         0x04
#define REG_GPIO_O_L         0x05
#define REG_GPIO_O_H         0x06
#define REG_GPIO_I_L         0x07
#define REG_GPIO_I_H         0x08
#define REG_GPIO_PU_L        0x09
#define REG_GPIO_PU_H        0x0A
#define REG_GPIO_PD_L        0x0B
#define REG_GPIO_PD_H        0x0C
#define REG_GPIO_IE_L        0x0D
#define REG_GPIO_IE_H        0x0E
#define REG_GPIO_IT_L        0x0F
#define REG_GPIO_IT_H        0x10
#define REG_GPIO_IS_L        0x11
#define REG_GPIO_IS_H        0x12
#define REG_GPIO_DRV_L       0x13
#define REG_GPIO_DRV_H       0x14
// ADC
#define REG_ADC_CTRL         0x15
#define REG_ADC_D_L          0x16
#define REG_ADC_D_H          0x17
// Temperature
#define REG_TEMP_CTRL        0x18
#define REG_TEMP_D_L         0x19
#define REG_TEMP_D_H         0x1A
// PWM
#define REG_PWM1_DUTY_L      0x1B
#define REG_PWM1_DUTY_H      0x1C
#define REG_PWM2_DUTY_L      0x1D
#define REG_PWM2_DUTY_H      0x1E
#define REG_PWM3_DUTY_L      0x1F
#define REG_PWM3_DUTY_H      0x20
#define REG_PWM4_DUTY_L      0x21
#define REG_PWM4_DUTY_H      0x22
// System Config
#define REG_I2C_CFG          0x23
#define REG_LED_CFG          0x24
#define REG_PWM_FREQ_L       0x25
#define REG_PWM_FREQ_H       0x26
#define REG_REF_VOLTAGE_L    0x27
#define REG_REF_VOLTAGE_H    0x28
#define REG_FACTORY_RESET    0x29
// Data areas
#define REG_LED_RAM_START    0x30
#define REG_LED_RAM_END      0x6F
#define REG_RTC_RAM_START    0x70
#define REG_RTC_RAM_END      0x8F

// ============================
// 配置类型定义
// ============================
typedef enum {
    PY32_CONFIG_TYPE_GPIO_INPUT,
    PY32_CONFIG_TYPE_GPIO_OUTPUT,
    PY32_CONFIG_TYPE_GPIO_INTERRUPT,
    PY32_CONFIG_TYPE_ADC,
    PY32_CONFIG_TYPE_PWM,
    PY32_CONFIG_TYPE_NEOPIXEL,
    PY32_CONFIG_TYPE_I2C_SLEEP
} py32_config_type_t;

// ============================
// 配置校验结果
// ============================
typedef struct {
    bool valid;
    char error_msg[128];
    uint8_t conflicting_pin;
} py32_config_validation_t;

// ============================
// 位定义
// ============================
// ADC
#define ADC_CTRL_CH_MASK     0x07
#define ADC_CTRL_START       (1 << 6)
#define ADC_CTRL_BUSY        (1 << 7)
// Temperature
#define TEMP_CTRL_START      (1 << 6)
#define TEMP_CTRL_BUSY       (1 << 7)
// PWM duty high bits
#define PWM_POLARITY         (1 << 6)
#define PWM_ENABLE           (1 << 7)
// I2C config
#define I2C_CFG_SLEEP_MASK     0x0F
#define I2C_CFG_SPEED_100K     (0 << 4)
#define I2C_CFG_SPEED_400K     (1 << 4)
#define I2C_CFG_WAKE_FALLING   (0 << 5)
#define I2C_CFG_WAKE_RISING    (1 << 5)
#define I2C_CFG_INTER_PULL_ON  (0 << 6)
#define I2C_CFG_INTER_PULL_OFF (1 << 6)
// LED
#define LED_CFG_NUM_MASK     0x3F
#define LED_CFG_REFRESH      (1 << 6)
// Factory reset
#define FACTORY_RESET_TRIGGER 0x3A

// ADC channel definitions
#define PY32_ADC_CHANNEL_1          1   // IO2 (PB1)
#define PY32_ADC_CHANNEL_2          2   // IO4 (PA3)
#define PY32_ADC_CHANNEL_3          3   // IO5 (PA4)
#define PY32_ADC_CHANNEL_4          4   // IO7 (PA6)

// PWM channel definitions
#define PY32_PWM_CHANNEL_1          0   // IO9 (PA0)
#define PY32_PWM_CHANNEL_2          1   // IO8 (PB0)
#define PY32_PWM_CHANNEL_3          2   // IO11 (PB2)
#define PY32_PWM_CHANNEL_4          3   // IO10 (PA7)

// RGB color structure for LED control
typedef struct {
    uint8_t r;  // Red component (0-255)
    uint8_t g;  // Green component (0-255)
    uint8_t b;  // Blue component (0-255)
} py32_rgb_color_t;

class m5_io_py32ioexpander : public m5_io_expander
{
public:
    // Constructor and destructor
    m5_io_py32ioexpander(); // Default constructor
    m5_io_py32ioexpander(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq, i2c_port_t i2c_port, gpio_num_t int_pin = GPIO_NUM_NC);
    m5_io_py32ioexpander(i2c_bus_handle_t existing_bus_handle, i2c_bus_device_handle_t existing_device_handle, gpio_num_t int_pin = GPIO_NUM_NC);
    ~m5_io_py32ioexpander();

    // Override base class virtual functions
    void init(uint8_t i2c_addr = PY32_DEFAULT_I2C_ADDR) override;
    void pinMode(uint8_t pin, uint8_t mode) override;
    void digitalWrite(uint8_t pin, uint8_t value) override;
    int digitalRead(uint8_t pin) override;
    void attachInterrupt(uint8_t pin, void (*callback)(void), uint8_t mode) override;
    void attachInterruptArg(uint8_t pin, void (*callback)(void*), void* arg, uint8_t mode) override;
    void detachInterrupt(uint8_t pin) override;
    void enableInterrupt(uint8_t pin) override;
    void disableInterrupt(uint8_t pin) override;

    // Device information functions
    bool readDeviceUID(uint16_t* uid);
    bool readVersion(uint8_t* hw_version, uint8_t* fw_version);

    // Advanced GPIO functions
    bool setPullMode(uint8_t pin, uint8_t pull_mode);
    bool setDriveMode(uint8_t pin, uint8_t drive_mode);
    uint16_t getInterruptStatus();
    bool clearInterrupt(uint8_t pin);

    // ADC functions
    bool analogRead(uint8_t channel, uint16_t* result);
    bool isAdcBusy();
    bool disableAdc();

    // Temperature sensor functions
    bool readTemperature(uint16_t* temperature);
    bool isTemperatureBusy();

    // PWM functions
    bool setPwmFrequency(uint16_t frequency);
    bool getPwmFrequency(uint16_t* frequency);
    bool setPwmDuty(uint8_t channel, uint8_t duty_percentage, bool polarity = false, bool enable = true);
    bool setPwmDuty12b(uint8_t channel, uint16_t duty12, bool polarity = false, bool enable = true);
    bool getPwmDuty(uint8_t channel, uint8_t* duty, bool* polarity, bool* enable);
    // Diagnostics: get raw 12-bit duty plus control flags (polarity/enable)
    bool getPwmDutyRaw(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enable);

    // LED control functions (using IO14/PB7)
    bool setLedCount(uint8_t num_leds);
    bool setLedColor(uint8_t led_index, py32_rgb_color_t color);
    bool setLedColor(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);
    bool refreshLeds();
    bool disableLeds();

    // RTC RAM functions
    bool writeRtcRam(uint8_t offset, const uint8_t* data, uint8_t length);
    bool readRtcRam(uint8_t offset, uint8_t* data, uint8_t length);

    // System configuration functions
    bool setI2cConfig(uint8_t sleep_time, bool speed_400k, bool wake_mode, bool inter_pull_off);
    bool getRefVoltage(uint16_t* ref_voltage);
    bool factoryReset();

    // Auto-snapshot control functions
    /**
     * @brief 启用或禁用自动状态快照功能
     * 
     * 当启用时，每次GPIO配置更改后会自动调用_snapshotIoState()来更新内部状态缓存
     * 
     * @param enable true=启用自动快照, false=禁用自动快照
     */
    void setAutoSnapshotEnabled(bool enable);

    /**
     * @brief 获取自动状态快照功能的当前状态
     * 
     * @return true=已启用自动快照, false=已禁用自动快照
     */
    bool isAutoSnapshotEnabled() const;

    /**
     * @brief 手动触发状态快照
     * 
     * 无论自动快照是否启用，都可以通过此函数手动更新内部状态缓存
     * 包括GPIO、PWM和ADC状态
     * 
     * @return true=快照成功, false=快照失败
     */
    bool triggerSnapshot();

    // State query functions (cached values from last snapshot)
    /**
     * @brief 获取缓存的PWM频率设置
     * 
     * @param frequency 输出参数，PWM频率(Hz)
     * @return true=成功, false=状态无效或未初始化
     */
    bool getCachedPwmFrequency(uint16_t* frequency);

    /**
     * @brief 获取缓存的PWM通道状态
     * 
     * @param channel PWM通道 (0-3)
     * @param duty12 输出参数，12位占空比值 (0-0x0FFF)
     * @param polarity 输出参数，极性设置
     * @param enabled 输出参数，使能状态
     * @return true=成功, false=通道无效或状态无效
     */
    bool getCachedPwmState(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enabled);

    /**
     * @brief 获取缓存的ADC配置状态
     * 
     * @param active_channel 输出参数，当前ADC通道 (0=禁用, 1-4=通道)
     * @param busy 输出参数，转换状态
     * @param last_value 输出参数，最后转换结果
     * @return true=成功, false=状态无效或未初始化
     */
    bool getCachedAdcState(uint8_t* active_channel, bool* busy, uint16_t* last_value);

    // Configuration validation function (public API)
    /**
     * @brief 手动验证引脚配置是否有效
     * 
     * 根据PY32手册中的约束规则验证配置，包括中断互斥、功能冲突等
     * 
     * @param pin 引脚编号 (0-13)
     * @param config_type 配置类型
     * @param enable_config 是否启用该配置
     * @return py32_config_validation_t 校验结果，包含是否有效和错误信息
     */
    py32_config_validation_t validatePinConfig(uint8_t pin, py32_config_type_t config_type, bool enable_config = true);

    // Debug functions
    bool getGpioModeReg(uint16_t* mode_reg);
    bool getGpioOutputReg(uint16_t* output_reg);
    bool getGpioInputReg(uint16_t* input_reg);
    bool getGpioPullUpReg(uint16_t* pull_up_reg);
    bool getGpioPullDownReg(uint16_t* pull_down_reg);
    bool getGpioDriveReg(uint16_t* drive_reg);

private:
    // Device I2C address
    uint8_t _device_addr;
    
    // Interrupt pin
    gpio_num_t _int_pin;
    
    // Initialization status flag
    bool _initialized_ok;
    
    // Auto-snapshot control flag
    bool _auto_snapshot_enabled;
    
    // Interrupt callback storage
    struct interrupt_callback_t {
        void (*callback)(void);
        void (*callback_arg)(void*);
        void* arg;
        bool enabled;
    };
    interrupt_callback_t _interrupt_callbacks[PY32_MAX_GPIO_PINS];

    // IO pin runtime state (snapshot from device)
    struct io_pin_state_t {
        bool is_output;        // GPIO_M
        uint8_t output_level;  // GPIO_O
        uint8_t input_level;   // GPIO_I
        uint8_t pull;          // 0:none 1:up 2:down
        uint8_t drive;         // 0:push-pull 1:open-drain
        bool intr_enabled;     // GPIO_IE
        bool intr_rising;      // GPIO_IT: 1 rising, 0 falling
    };
    io_pin_state_t _pin_states[PY32_MAX_GPIO_PINS];
    bool _pin_states_valid;
    
    // PWM channel runtime state (snapshot from device)
    struct pwm_channel_state_t {
        uint16_t duty12;       // 12-bit duty value (0-0x0FFF)
        bool enabled;          // PWM enabled
        bool polarity;         // PWM polarity (1=low active, 0=high active)
    };
    pwm_channel_state_t _pwm_states[PY32_MAX_PWM_CHANNELS];
    uint16_t _pwm_frequency;   // PWM frequency in Hz
    bool _pwm_states_valid;
    
    // ADC configuration state
    struct adc_state_t {
        uint8_t active_channel; // Current ADC channel (0=disabled, 1-4=channel)
        bool busy;              // ADC conversion in progress
        uint16_t last_value;    // Last conversion result
    };
    adc_state_t _adc_state;
    bool _adc_state_valid;
    
    // Internal helper functions
    bool _writeRegister(uint8_t reg, uint8_t value);
    bool _writeRegister16(uint8_t reg, uint16_t value);
    bool _readRegister(uint8_t reg, uint8_t* value);
    bool _readRegister16(uint8_t reg, uint16_t* value);
    bool _isValidPin(uint8_t pin);
    bool _isAdcPin(uint8_t pin);
    bool _isPwmPin(uint8_t pin);
    uint8_t _getAdcChannel(uint8_t pin);
    uint8_t _getPwmChannel(uint8_t pin);
    void _clearPinStates();
    bool _snapshotIoState();
    void _clearPwmStates();
    bool _snapshotPwmState();
    void _clearAdcState();
    bool _snapshotAdcState();
    static bool _pinsConflict(uint8_t a, uint8_t b);
    
    // Configuration validation functions
    py32_config_validation_t _validatePinConfig(uint8_t pin, py32_config_type_t config_type, bool enable_config = true);
    bool _isInterruptMutexPin(uint8_t pin, uint8_t* mutex_pin);
    bool _isAdcPin(uint8_t pin, uint8_t* adc_channel);
    bool _isPwmPin(uint8_t pin, uint8_t* pwm_channel);
    bool _isNeopixelPin(uint8_t pin);
    bool _hasActiveInterrupt(uint8_t pin);
    bool _hasActiveAdc(uint8_t pin);
    bool _hasActivePwm(uint8_t pin);
    bool _hasI2cSleepEnabled();
    
    // Auto-snapshot helper function
    void _autoSnapshot();
    
    // Interrupt handling
    static void IRAM_ATTR _interruptHandler(void* arg);
    void _handleInterrupt();
    static void _interruptTask(void* arg);
    
    // ISR异步处理资源
    QueueHandle_t _intr_queue = nullptr;
    TaskHandle_t _intr_task   = nullptr;
    
    // Internal initialization
    bool _initializeDevice();
    bool _setupInterrupts();
};

#endif // M5_IO_PY32IOEXPANDER_H