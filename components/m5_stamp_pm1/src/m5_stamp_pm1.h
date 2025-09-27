#ifndef __PM1_CONTROL_H__
#define __PM1_CONTROL_H__

#include "esp_err.h"
#include <stdint.h>


#ifdef ARDUINO
#include "Wire.h"
#else
#include "i2c_bus.h"
#endif

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
    #include "esp32-hal-log.h"
#else
    #include "esp_log.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
#define PM1_DEV_VERSION "0.0.2" // PM1驱动版本号

#define PM1_ADDR_UID_L 0x00            // R     [7-0] UID_L
#define PM1_ADDR_UID_H 0x01            // R     [7-0] UID_H
#define PM1_ADDR_HW_REV 0x02           // R     [7-0] 硬件版本号
#define PM1_ADDR_SW_REV 0x03           // R     [7-0] 软件/固件版本号
#define PM1_ADDR_GPIO_MODE 0x04        // R/W   [7-5] Reserved | [4-0] GPIO模式寄存器 1:输出 | 0:输入
#define PM1_ADDR_GPIO_OUT 0x05         // R/W   [7-5] Reserved | [4-0] GPIO输出寄存器 1:高 | 0:低
#define PM1_ADDR_GPIO_IN 0x06          // R     [7-5] Reserved | [4-0] GPIO输入寄存器 实时输入的值
#define PM1_ADDR_GPIO_DRV 0x07         // R/W   [7-5] Reserved | [5] LED_EN_DRV | [4-0] GPIO驱动寄存器 1:开漏 | 0:推挽
#define PM1_ADDR_ADC_RES_L 0x08        // R     [7-0] ADC结果低8位
#define PM1_ADDR_ADC_RES_H 0x09        // R     [7-4] Reserved | [3-0] ADC结果高4位
#define PM1_ADDR_ADC_CTRL 0x0A         // R/W   [7-4] Reserved | [3-1] 通道选择(1[gpio]\2[gpio]\6[temp]) | [0] 开始转换 注：FUNC必须为11
// #define PM1_ADDR_
//   占空比 0-4096
#define PM1_ADDR_PWM0_L 0x0C           // R/W   [7-0] PWM0占空比低8位
#define PM1_ADDR_PWM0_HC 0x0D          // R/W   [7-6] Reserved | [5] 极性 | [4] 使能 | [3-0] 占空比高4位
#define PM1_ADDR_PWM1_L 0x0E           // R/W   [7-0] PWM1占空比低8位
#define PM1_ADDR_PWM1_HC 0x0F          // R/W   [7-6] Reserved | [5] 极性 | [4] 使能 | [3-0] 占空比高4位
//   00:GPIO | 01:IRQ | 10:WAKE | 11:LED/PWM/ADC
#define PM1_ADDR_GPIO_FUNC0 0x10       // R/W   [7-6] GPIO3 | [5-4] GPIO2 | [3-2] GPIO1 | [1-0] GPIO0
#define PM1_ADDR_GPIO_FUNC1 0x11       // R/W   [7-2] Reserved | [1-0] GPIO4
#define PM1_ADDR_GPIO_PUPD0 0x12       // R/W   [7-6] GPIO3 | [5-4] GPIO2 | [3-2] GPIO1 | [1-0] GPIO0 上下拉配置
#define PM1_ADDR_GPIO_PUPD1 0x13       // R/W   [1-0] GPIO4 上下拉配置 | 其余位保留
#define PM1_ADDR_WDT_CNT 0x14          // R/W   [7-0] 看门狗倒计时(秒) 0:不启用看门狗, 1-255s:看门狗时限
#define PM1_ADDR_WDT_KEY 0x15          // W     [7-0] 在该寄存器写0xA5以喂狗
#define PM1_ADDR_GPIO_WAKE_EN 0x16     // R/W   [7-5] Reserved | [4-0] GPIO唤醒使能
#define PM1_ADDR_GPIO_WAKE_CFG 0x17    // R/W   [7-5] Reserved | [4-0] GPIO唤醒边沿配置
//   定时唤醒计数器 寄存器 范围 0-0x7FFFFFFF 单位为秒，约合最长68.1年。
#define PM1_ADDR_TIM_CNT_BYTE_0 0x18   // R/W   [7-0] 定时唤醒计数器 Byte0
#define PM1_ADDR_TIM_CNT_BYTE_1 0x19   // R/W   [7-0] 定时唤醒计数器 Byte1
#define PM1_ADDR_TIM_CNT_BYTE_2 0x1A   // R/W   [7-4] 定时唤醒计数器 Byte2
#define PM1_ADDR_TIM_CNT_BYTE_3 0x1B   // R/W   [7] Reserved | [6-0] 定时唤醒计数器 Byte3
#define PM1_ADDR_PWR_CFG 0x1C          // R/W   [7-5] Reserved [4] LED_CONTROL | [3] 5V_INOUT | [2] LDO_EN | [1] DCDC_EN | [0] CHG_EN
//   ACTION: 000：停止计数器 | 001：仅置位 WAKE 标志 | 010：系统重启 | 011：系统上电 | 100：系统关机
#define PM1_ADDR_TIM_CFG 0x1D          // R/W   [7-4] Reserved | [3] ARM(是否自动清零) | [2-0] ACTION
#define PM1_ADDR_PWM_FREQ_L 0x1E       // R/W   [7-0] PWM频率低8位
#define PM1_ADDR_PWM_FREQ_H 0x1F       // R/W   [7-0] PWM频率高8位
#define PM1_ADDR_BTN_CFG 0x20          // R/W   [7] DL_LOCK | [6-5] DBL | [4-3] LONG | [2-1] SINGLE | [0] SINGLE_RESET_DIS
#define PM1_ADDR_IRQ_STATUS1 0x21      // R/W   [7-5] Reserved | [4-0] GPIO中断状态 [4:GPIO4 3:GPIO3 2:GPIO2 1:GPIO1 0:GPIO0]
// 备注：电池拔插仅在充电使能为打开时有效，5VINOUT插拔仅在设置为输入时有效
#define PM1_ADDR_IRQ_STATUS2 0x22      // R/W   [7-6] Reserved | [5] 电池移除 | [4] 电池插入 | [3] 5VINOUT移除 | [2] 5VINOUT插入 | [1] 5V IN移除 | [0] 5V IN插入
#define PM1_ADDR_IRQ_STATUS3 0x23      // R/W   [7-3] Reserved | [2] Double click | [1] Wakeup | [0] Click
// #define PM1_ADDR_
#define PM1_ADDR_BATT_LVP 0x24         // R/W   [7-0] 低压阈值:2000mV+n×7.81mV
#define PM1_ADDR_TIM_KEY 0x25          // W     [7-0] 写0xA5清零并重载
#define PM1_ADDR_VREF_L 0x26           // R     [7-0] VREF低8位
#define PM1_ADDR_VREF_H 0x27           // R     [7-0] VREF高8位
#define PM1_ADDR_VBAT_L 0x28           // R     [7-0] 电池电压低8位
#define PM1_ADDR_VBAT_H 0x29           // R     [7-4] Reserved | [3-0] 电池电压高4位
#define PM1_ADDR_VIN_L 0x2A            // R     [7-0] VIN电压低8位
#define PM1_ADDR_VIN_H 0x2B            // R     [7-4] Reserved | [3-0] VIN电压高4位
#define PM1_ADDR_5VINOUT_L 0x2C        // R     [7-0] 5VINOUT(5VOUT)电压低8位
#define PM1_ADDR_5VINOUT_H 0x2D        // R     [7-4] Reserved | [3-0] 5VINOUT(5VOUT)电压高4位
#define PM1_ADDR_PWR_SRC 0x2E          // R     [7-3] Reserved | [2-0] VALID 0:5VIN 1:5VINOUT 2:BAT
#define PM1_ADDR_WAKE_SRC 0x2F         // R/W   [7] Reserved | [6] 5V INOUT | [5] EXT_WAKE | [4] CMD_RST | [3] RSTBTN | [2] PWRBTN | [1] VIN | [0] TIM
#define PM1_ADDR_I2C_CFG 0x30          // R/W   [7-5] Reserved | [4] SPD 0->100K 1:400K | [3-0] SLP_TO
#define PM1_ADDR_BTN_CFG_2 0x31        // R/W   [7-1] Reserved | [0] DOUBLE_POWEROFF_DIS
#define PM1_ADDR_SYS_CMD 0x32          // W     [7-4] KEY(0xA) | [3-2] Reserved | [1-0] CMD 00:无 01:关机 10:重启 11:下载
// 注意：当系统进入下载模式或发生复位事件（包括I2C看门狗复位、软件命令复位、定时器复位）时，此寄存器将自动清零至0x00
#define PM1_ADDR_GPIO_POWER_HOLD 0x33  // R/W   [7-6] Reserved | [5] LDO(3.3V) | [4] GPIO4 | [3] GPIO3 | [2] GPIO2 | [1] GPIO1 | [0] GPIO0 电源保持控制
#define PM1_ADDR_NEO_CFG 0x34          // R/W   [7-6] Reserved | [5] REFRESH | [4-0] LED_CNT
// #define PM1_ADDR_
// 例如 rgb1_red:36H[4:0] rgb1_green:36H[5:3]&37H[2:0] rgb1_blue:37H[4:0] ...
#define PM1_ADDR_NEO_PIXn_ADDR_START 0x36 // R/W   NeoPixel像素RGB565数据起始地址
#define PM1_ADDR_NEO_PIXn_ADDR_END 0x75   // R/W   NeoPixel像素RGB565数据结束地址
// #define PM1_ADDR_
#define PM1_ADDR_RTC_MEM_ADDR_START 0x80  // R/W   RTC备份RAM起始地址 32字节
#define PM1_ADDR_RTC_MEM_ADDR_END 0x9F    // R/W   RTC备份RAM结束地址
#define PM1_ADDR_IRQ_STATUS1_MASK 0xA0    // R/W   [7-5] Reserved | [4-0] GPIO中断屏蔽掩码 [4:GPIO4 3:GPIO3 2:GPIO2 1:GPIO1 0:GPIO0] 位=1屏蔽对应中断
#define PM1_ADDR_IRQ_STATUS2_MASK 0xA1    // R/W   [7-6] Reserved | [5] 电池移除 | [4] 电池插入 | [3] 5VINOUT移除 | [2] 5VINOUT插入 | [1] 5V IN移除 | [0] 5V IN插入 位=1屏蔽对应中断
#define PM1_ADDR_IRQ_STATUS3_MASK 0xA2    // R/W   [7-3] Reserved | [2] Double click | [1] Wakeup | [0] Click 位=1屏蔽对应中断

// GPIO Function Types
typedef enum  {
    PM1_GPIO_FUNC_GPIO = 0b00,
    PM1_GPIO_FUNC_IRQ = 0b01,
    PM1_GPIO_FUNC_RES = 0b10,      //保留 reserve
    PM1_GPIO_FUNC_OTHER = 0b11     //LED:频率固定为24Mhz/PWM/ADC
} pm1_gpio_func_t;

// GPIO Number Types
typedef enum  {
    PM1_GPIO_NUM_0 = 0,
    PM1_GPIO_NUM_1 = 1,
    PM1_GPIO_NUM_2 = 2,
    PM1_GPIO_NUM_3 = 3,
    PM1_GPIO_NUM_4 = 4,
    PM1_GPIO_NUM_NC = 255
} pm1_gpio_num_t;

// GPIO Mode Types
typedef enum {
    PM1_GPIO_MODE_INPUT = 0b00,   // 输入模式
    PM1_GPIO_MODE_OUTPUT = 0b01,  // 输出模式
} pm1_gpio_mode_t;

// GPIO State Types
typedef enum {
    PM1_GPIO_OUTPUT_LOW = 0b0,     // 低电平
    PM1_GPIO_OUTPUT_HIGH = 0b1,    // 高电平
    PM1_GPIO_INPUT_NC = 0b0,    // 输入模式下不关心状态
} pm1_gpio_state_t;

// GPIO Pull-up/Pull-down Types
typedef enum {
    PM1_GPIO_PUPD_NC = 0b00,       // 不配置上下拉
    PM1_GPIO_PUPD_PULLUP = 0b01,   // 上拉
    PM1_GPIO_PUPD_PULLDOWN = 0b10, // 下拉
} pm1_gpio_pupd_t;

// GPIO Input State Types
typedef enum {
    PM1_GPIO_IN_STATE_LOW = 0b0,   // 输入状态低
    PM1_GPIO_IN_STATE_HIGH = 0b1,  // 输入状态高
} pm1_gpio_in_state_t;

// GPIO Drive Mode Types
typedef enum {
    PM1_GPIO_DRV_PUSH_PULL = 0b0,   // 推挽输出
    PM1_GPIO_DRV_OPEN_DRAIN = 0b1,  // 开漏输出
} pm1_gpio_drv_t;

// ADC Channel Types
typedef enum {
    PM1_ADC_CHANNEL_0 = 0,      // ADC通道0  0-4095 [本项目不支持]
    PM1_ADC_CHANNEL_1 = 1,      // ADC通道1  0-4095
    PM1_ADC_CHANNEL_2 = 2,      // ADC通道2  0-4095
    PM1_ADC_CHANNEL_3 = 3,      // ADC通道3  0-4095 [本项目不支持]
    PM1_ADC_CHANNEL_4 = 4,      // ADC通道4  0-4095 [本项目不支持]
    PM1_ADC_CHANNEL_5 = 5,      // ADC通道5  0-4095 [本项目不支持]
    PM1_ADC_CHANNEL_TEMP = 6,   // ADC通道6  MCU内部温度探针 单位：摄氏度
} pm1_adc_channel_t;

// ADC Control Types
typedef enum {
    PM1_ADC_CTRL_DISABLE = 0b0, // ADC禁用
    PM1_ADC_CTRL_ENABLE = 0b1,  // ADC使能
} pm1_adc_ctrl_t;

// PWM Channel Types
typedef enum {
    PM1_PWM_CHANNEL_0 = 0,      // PWM通道0
    PM1_PWM_CHANNEL_1 = 1,      // PWM通道1
} pm1_pwm_channel_t;

// PWM Control Types
typedef enum {
    PM1_PWM_CTRL_DISABLE = 0b0, // PWM禁用
    PM1_PWM_CTRL_ENABLE = 0b1,  // PWM使能
} pm1_pwm_ctrl_t;

// PWM Polarity Types
typedef enum {
    PM1_PWM_POLARITY_NORMAL = 0b0, // 正常极性
    PM1_PWM_POLARITY_INVERTED = 0b1, // 反向极性
} pm1_pwm_polarity_t;

// WDT Control Types
typedef enum {
    PM1_WDT_CTRL_DISABLE = 0b0, // WDT禁用
    PM1_WDT_CTRL_ENABLE = 0b1,  // WDT使能
} pm1_wdt_ctrl_t;

// Timer Control Types
typedef enum {
    PM1_ADDR_TIM_DISABLE = 0b0, // 定时器禁用
    PM1_ADDR_TIM_ENABLE = 0b1,  // 定时器使能
} pm1_tim_ctrl_t;

// Timer Action Types
typedef enum {
    PM1_TIM_ACTION_000 = 0b000, // 停止计数器
    PM1_TIM_ACTION_001 = 0b001, // 仅置位 WAKE 标志
    PM1_TIM_ACTION_010 = 0b010, // 系统重启
    PM1_TIM_ACTION_011 = 0b011, // 系统上电
    PM1_TIM_ACTION_100 = 0b100, // 系统关机
} pm1_tim_action_t;

// I2C Clock Speed Types
typedef enum {
    PM1_CLK_SPEED_100KHZ = 0, // 100KHz MCU_CLK:3MHz
    PM1_CLK_SPEED_400KHZ = 1, // 400KHz MCU_CLK:12MHz （功耗增加）
} pm1_clk_speed_t;

// I2C ACK Check Types
// [0:直到唤醒为止，第二个参数为超时参数，单位为毫秒，0表示不超时]
// [1:尝试唤醒一定次数，第二个参数为唤醒次数，每次唤醒间隔为500ms]
typedef enum {
    PM1_I2C_ACK_CHECK_UNTIL_WAKE = 0, // I2C尝试唤醒直到成功
    PM1_I2C_ACK_CHECK_TRY_TIMES = 1,   // I2C尝试唤醒指定次数
} pm1_i2c_ack_check_t;


// GPIO Pull-up/Pull-down Types
typedef enum {
    PM1_GPIO_PULL_NO = 0b00,   // 无上拉下拉
    PM1_GPIO_PULL_UP = 0b01,   // 上拉
    PM1_GPIO_PULL_DOWN = 0b11, // 下拉
} pm1_gpio_pull_t;

// GPIO Wake Types
typedef enum {
    PM1_GPIO_WAKE_DISABLE = 0b0, // 禁用唤醒
    PM1_GPIO_WAKE_ENABLE = 0b1,  // 使能唤醒
} pm1_gpio_wake_t;

// GPIO Wake Edge Types
typedef enum {
    PM1_GPIO_WAKE_FALLING = 0b0, // 下降沿唤醒
    PM1_GPIO_WAKE_RISING = 0b1,  // 上升沿唤醒
} pm1_gpio_wake_edge_t;

// Download Enable Types
typedef enum {
    PM1_ADDR_DOWNLOAD_DISABLE = 0x00, // 禁用下载模式
    PM1_ADDR_DOWNLOAD_ENABLE = 0x01   // 启用下载模式
} pm1_download_enable_t;

// Button Types
typedef enum {
    PM1_ADDR_BTN_TYPE_CLICK = 0x00,        // 点击
    PM1_ADDR_BTN_TYPE_DOUBLE_CLICK = 0x01, // 双击
    PM1_ADDR_BTN_TYPE_LONG_PRESS = 0x02    // 长按
} pm1_btn_type_t;

// Button Delay Types
typedef enum {
    PM1_ADDR_BTN_CLICK_DELAY_125MS = 0x00,          // 125毫秒
    PM1_ADDR_BTN_CLICK_DELAY_250MS = 0x01,          // 250毫秒
    PM1_ADDR_BTN_CLICK_DELAY_500MS = 0x02,          // 500毫秒
    PM1_ADDR_BTN_CLICK_DELAY_1000MS = 0x03,         // 1000毫秒
    PM1_ADDR_BTN_DOUBLE_CLICK_DELAY_125MS = 0x00,   // 双击125毫秒
    PM1_ADDR_BTN_DOUBLE_CLICK_DELAY_250MS = 0x01,   // 双击250毫秒
    PM1_ADDR_BTN_DOUBLE_CLICK_DELAY_500MS = 0x02,   // 双击500毫秒
    PM1_ADDR_BTN_DOUBLE_CLICK_DELAY_1000MS = 0x03,  // 双击1000毫秒
    PM1_ADDR_BTN_LONG_PRESS_DELAY_1000MS = 0x00,    // 长按1000毫秒
    PM1_ADDR_BTN_LONG_PRESS_DELAY_2000MS = 0x01,    // 长按2000毫秒
    PM1_ADDR_BTN_LONG_PRESS_DELAY_3000MS = 0x02,    // 长按3000毫秒
    PM1_ADDR_BTN_LONG_PRESS_DELAY_4000MS = 0x03     // 长按4000毫秒
} pm1_btn_delay_t;

// GPIO IRQ Types
typedef enum {
    PM1_ADDR_IRQ_GPIO0 = 0x00,
    PM1_ADDR_IRQ_GPIO1 = 0x01,
    PM1_ADDR_IRQ_GPIO2 = 0x02,
    PM1_ADDR_IRQ_GPIO3 = 0x03,
    PM1_ADDR_IRQ_GPIO4 = 0x04,
    PM1_ADDR_IRQ_GPIO_ALL = 0x1F,
    PM1_ADDR_IRQ_NULL  = 0xFF,
} pm1_irq_gpio_t;

// GPIO IRQ Clean Types
typedef enum {
    PM1_ADDR_IRQ_GPIO_NOT_CLEAN = 0x00, // 不清除中断
    PM1_ADDR_IRQ_GPIO_ONCE_CLEAN = 0x01,    // 清除一位中断
    PM1_ADDR_IRQ_GPIO_ALL_CLEAN = 0x02      // 清除所有中断
} pm1_irq_gpio_clean_type_t;

// System IRQ Types
typedef enum {
    PM1_ADDR_IRQ_SYS_BAT_REMOVE = 0x05,    // 电池移除
    PM1_ADDR_IRQ_SYS_BAT_INSERT = 0x04,    // 电池插入
    PM1_ADDR_IRQ_SYS_5VINOUT_REMOVE = 0x03,  // 5VINOUT移除
    PM1_ADDR_IRQ_SYS_5VINOUT_INSERT = 0x02,  // 5VINOUT插入
    PM1_ADDR_IRQ_SYS_5VIN_REMOVE = 0x01,   // 5VIN移除
    PM1_ADDR_IRQ_SYS_5VIN_INSERT = 0x00,   // 5VIN插入
    PM1_ADDR_IRQ_SYS_ALL = 0x3F,           // 所有系统中断
    PM1_ADDR_IRQ_SYS_NULL = 0xFF
} pm1_irq_sys_t;

// System IRQ Clean Types
typedef enum {
    PM1_ADDR_IRQ_SYS_NOT_CLEAN = 0x00, // 不清除中断
    PM1_ADDR_IRQ_SYS_ONCE_CLEAN = 0x01,    // 清除一位中断
    PM1_ADDR_IRQ_SYS_ALL_CLEAN = 0x02      // 清除所有中断
} pm1_irq_sys_clean_type_t;

// Button IRQ Types
typedef enum {
    PM1_ADDR_IRQ_BTN_CLICK = 0x00,         // 单击中断
    PM1_ADDR_IRQ_BTN_WAKEUP = 0x01,        // 唤醒中断
    PM1_ADDR_IRQ_BTN_DOUBLE_CLICK = 0x02,  // 双击中断
    PM1_ADDR_IRQ_BTN_ALL = 0x07,           // 所有按钮中断
    PM1_ADDR_IRQ_BTN_NULL = 0xFF,
} pm1_irq_btn_t;

// Button IRQ Clean Types
typedef enum {
    PM1_ADDR_IRQ_BTN_NOT_CLEAN = 0x00, // 不清除中断
    PM1_ADDR_IRQ_BTN_ONCE_CLEAN = 0x01,    // 清除一次中断
    PM1_ADDR_IRQ_BTN_ALL_CLEAN = 0x02      // 清除所有中断
} pm1_irq_btn_clean_type_t;

// IRQ Mask Control Types
typedef enum {
    PM1_IRQ_MASK_DISABLE = 0x00, // 不屏蔽中断（允许中断）
    PM1_IRQ_MASK_ENABLE = 0x01   // 屏蔽中断（禁止中断）
} pm1_irq_mask_ctrl_t;

// Power Source Types
typedef enum {
    PM1_PWR_SRC_5VIN = 0x00,    // 5VIN供电
    PM1_PWR_SRC_5VINOUT = 0x01,   // 5VINOUT供电
    PM1_PWR_SRC_BAT = 0x02,     // 电池供电
    PM1_PWR_SRC_UNKNOWN = 0x03  // 未知电源
} pm1_pwr_src_t;

// Wake Source Types
typedef enum {
    PM1_WAKE_SRC_TIM = 0x01,         // [0] 定时器唤醒
    PM1_WAKE_SRC_VIN = 0x02,         // [1] 5VIN插入唤醒
    PM1_WAKE_SRC_PWRBTN = 0x04,      // [2] 电源按钮唤醒
    PM1_WAKE_SRC_RSTBTN = 0x08,      // [3] 按钮复位唤醒
    PM1_WAKE_SRC_CMD_RST = 0x10,     // [4] 复位命令唤醒
    PM1_WAKE_SRC_EXT_WAKE = 0x20,    // [5] GPIO WAKE唤醒
    PM1_WAKE_SRC_5VINOUT = 0x40,     // [6] 5V INOUT插入唤醒（仅当5V升压关闭时）
    PM1_WAKE_SRC_NULL = 0x00,      // 默认空值
    PM1_WAKE_SRC_UNKNOWN = 0xFF      // 未知唤醒源
} pm1_wake_src_t;

// WAKE Flag Clean Types
typedef enum {
    PM1_ADDR_WAKE_FLAG_NOT_CLEAN = 0x00, // 不清除中断
    PM1_ADDR_WAKE_FLAG_ONCE_CLEAN = 0x01,    // 清除一位中断
    PM1_ADDR_WAKE_FLAG_ALL_CLEAN = 0x02      // 清除所有中断
} pm1_wake_flag_clean_type_t;

// System Command Types
typedef enum {
    PM1_SYS_CMD_NULL = 0x00,     // 无命令
    PM1_SYS_CMD_SHUTDOWN = 0x01, // 关机
    PM1_SYS_CMD_REBOOT = 0x02,   // 重启
    PM1_SYS_CMD_JTAG = 0x03,     // JTAG下载
} pm1_sys_cmd_t;

// PWR CFG Types
typedef enum {
    PM1_PWR_CFG_CHG_EN      = 0x01, // [0] CHG_EN 充电使能
    PM1_PWR_CFG_DCDC_EN     = 0x02, // [1] DCDC_EN 升压使能
    PM1_PWR_CFG_LDO_EN      = 0x04, // [2] LDO_EN LDO保持
    PM1_PWR_CFG_5V_INOUT    = 0x08, // [3] 5V_INOUT 5V输入输出
    PM1_PWR_CFG_LED_CONTROL = 0x10  // [4] LED_CONTROL LED控制
} pm1_pwr_cfg_t;

// GPIO Power Hold Types
typedef enum {
    PM1_GPIO_POWER_HOLD_DISABLE = 0b0, // 禁用GPIO电源保持
    PM1_GPIO_POWER_HOLD_ENABLE = 0b1,  // 使能GPIO电源保持
} pm1_gpio_power_hold_t;

// Single Reset Disable Types
typedef enum {
    PM1_SINGLE_RESET_ENABLE = 0,   // 启用单击复位
    PM1_SINGLE_RESET_DISABLE = 1   // 禁止单击复位
} pm1_single_reset_dis_t;

// Double PowerOff Disable Types
typedef enum {
    PM1_DOUBLE_POWEROFF_ENABLE = 0,   // 启用双击关机
    PM1_DOUBLE_POWEROFF_DISABLE = 1   // 禁止双击关机
} pm1_double_poweroff_dis_t;

// Function Declarations
#ifdef ARDUINO
esp_err_t pm1_init(TwoWire *wire);
esp_err_t pm1_deinit(void);
#else
esp_err_t pm1_init(i2c_bus_handle_t i2c_bus, i2c_bus_device_handle_t *i2c_device, uint32_t i2c_freq_hz);
esp_err_t pm1_deinit(void);
#endif
esp_err_t pm1_set_clk_speed(pm1_clk_speed_t clk_speed_flag);
esp_err_t pm1_set_i2c_sleep_time(uint8_t sleep_time_sec);
esp_err_t pm1_i2c_try_wake(pm1_i2c_ack_check_t ack_check_type, uint32_t timeout_or_try_times);

// Device Information Functions
esp_err_t pm1_get_hw_version(uint8_t *hw_version);
esp_err_t pm1_get_sw_version(uint8_t *sw_version);
esp_err_t pm1_get_chip_id(uint16_t *chip_id);

// Pin Status Functions
esp_err_t pm1_read_and_check_pin_status(bool enable_print);

// GPIO Functions
esp_err_t pm1_gpio_set_func(pm1_gpio_num_t gpio_num, pm1_gpio_func_t func);
esp_err_t pm1_gpio_set(pm1_gpio_num_t gpio_num, pm1_gpio_mode_t mode, pm1_gpio_state_t state, pm1_gpio_pupd_t pupd, pm1_gpio_drv_t drv_mode);
esp_err_t pm1_gpio_set_mode(pm1_gpio_num_t gpio_num, pm1_gpio_mode_t mode);
esp_err_t pm1_gpio_set_state(pm1_gpio_num_t gpio_num, pm1_gpio_state_t state);
esp_err_t pm1_gpio_set_pupd(pm1_gpio_num_t gpio_num, pm1_gpio_pupd_t pupd);
esp_err_t pm1_gpio_set_drv(pm1_gpio_num_t gpio_num, pm1_gpio_drv_t drv_mode);
esp_err_t pm1_led_en_set_drv(pm1_gpio_drv_t drv_mode);
esp_err_t pm1_gpio_set_wake_en(pm1_gpio_num_t gpio_num, pm1_gpio_wake_t wake_en);
esp_err_t pm1_gpio_set_wake_cfg(pm1_gpio_num_t gpio_num, pm1_gpio_wake_edge_t wake_cfg);
esp_err_t pm1_gpio_get_in_state(pm1_gpio_num_t gpio_num, pm1_gpio_in_state_t *state);

// GPIO Power Hold Functions
esp_err_t pm1_gpio_set_power_hold(pm1_gpio_num_t gpio_num, pm1_gpio_power_hold_t power_hold);
esp_err_t pm1_gpio_get_power_hold(pm1_gpio_num_t gpio_num, pm1_gpio_power_hold_t *power_hold);

// LDO Power Hold Functions
esp_err_t pm1_ldo_set_power_hold(bool enable);
esp_err_t pm1_ldo_get_power_hold(bool *enable);

// ADC Functions
esp_err_t pm1_adc_read(pm1_adc_channel_t channel, uint16_t *adc_value);

// PWM Functions
esp_err_t pm1_pwm_set(pm1_pwm_channel_t channel, pm1_pwm_ctrl_t ctrl, pm1_pwm_polarity_t polarity, uint32_t pwm_freq_16bit_value, uint32_t duty_cycle_12bit_value);

// WDT Functions
esp_err_t pm1_wdt_set(pm1_wdt_ctrl_t ctrl, uint8_t timeout_s);
esp_err_t pm1_wdt_feed(uint8_t key);
esp_err_t pm1_wdt_get_wdt_cnt(uint8_t *wdt_cnt);

// Timer Functions
esp_err_t pm1_tim_set(pm1_tim_ctrl_t ctrl, pm1_tim_action_t action, uint32_t tim_ct_31bit_value);
esp_err_t pm1_tim_clear(uint8_t key);

// Download and Button Control Functions
esp_err_t pm1_download_enable_set(pm1_download_enable_t enable);
esp_err_t pm1_download_enable_get(pm1_download_enable_t *enable);
esp_err_t pm1_single_reset_dis_set(pm1_single_reset_dis_t enable);
esp_err_t pm1_single_reset_dis_get(pm1_single_reset_dis_t *enable);
esp_err_t pm1_double_poweroff_dis_set(pm1_double_poweroff_dis_t enable);
esp_err_t pm1_double_poweroff_dis_get(pm1_double_poweroff_dis_t *enable);

// Button Functions
esp_err_t pm1_btn_set_cfg(pm1_btn_type_t btn_type, pm1_btn_delay_t delay);

// IRQ Functions
esp_err_t pm1_irq_get_status(pm1_irq_gpio_t *gpio_num, pm1_irq_gpio_clean_type_t clean_type);
esp_err_t pm1_irq_clear_gpio_flag(pm1_irq_gpio_t gpio_num);
esp_err_t pm1_irq_get_sys_status(pm1_irq_sys_t *sys_irq, pm1_irq_sys_clean_type_t clean_type);
esp_err_t pm1_irq_clear_sys_status(pm1_irq_sys_t sys_irq);
esp_err_t pm1_irq_get_btn_status(pm1_irq_btn_t *btn_irq, pm1_irq_btn_clean_type_t clean_type);
esp_err_t pm1_irq_clear_btn_status(pm1_irq_btn_t btn_irq);

// IRQ Mask Functions
esp_err_t pm1_irq_set_gpio_mask(pm1_gpio_num_t gpio_num, pm1_irq_mask_ctrl_t mask_ctrl);
esp_err_t pm1_irq_get_gpio_mask(pm1_gpio_num_t gpio_num, pm1_irq_mask_ctrl_t *mask_ctrl);
esp_err_t pm1_irq_set_gpio_mask_all(uint8_t mask_value);
esp_err_t pm1_irq_get_gpio_mask_all(uint8_t *mask_value);

esp_err_t pm1_irq_set_sys_mask(pm1_irq_sys_t sys_irq, pm1_irq_mask_ctrl_t mask_ctrl);
esp_err_t pm1_irq_get_sys_mask(pm1_irq_sys_t sys_irq, pm1_irq_mask_ctrl_t *mask_ctrl);
esp_err_t pm1_irq_set_sys_mask_all(uint8_t mask_value);
esp_err_t pm1_irq_get_sys_mask_all(uint8_t *mask_value);

esp_err_t pm1_irq_set_btn_mask(pm1_irq_btn_t btn_irq, pm1_irq_mask_ctrl_t mask_ctrl);
esp_err_t pm1_irq_get_btn_mask(pm1_irq_btn_t btn_irq, pm1_irq_mask_ctrl_t *mask_ctrl);
esp_err_t pm1_irq_set_btn_mask_all(uint8_t mask_value);
esp_err_t pm1_irq_get_btn_mask_all(uint8_t *mask_value);

// Battery and Voltage Functions
esp_err_t pm1_batt_set_lvp(uint16_t lvp_mv);
esp_err_t pm1_vref_read(uint16_t *vref_mv);
esp_err_t pm1_vbat_read(uint16_t *vbat_mv);
esp_err_t pm1_vin_read(uint16_t *vin_mv);
esp_err_t pm1_5vinout_read(uint16_t *vinout_mv);

// Power and Wake Functions
esp_err_t pm1_pwr_src_read(pm1_pwr_src_t *pwr_src);
esp_err_t pm1_pwr_set_cfg(uint8_t mask, uint8_t value, uint8_t *final_cfg);
esp_err_t pm1_pwr_clear_cfg(uint8_t cfg, uint8_t *final_cfg);
esp_err_t pm1_pwr_get_cfg(uint8_t *current_cfg);
esp_err_t pm1_wake_src_read(pm1_wake_src_t *wake_src, pm1_wake_flag_clean_type_t clean_type);

// System Command Functions
esp_err_t pm1_sys_cmd(pm1_sys_cmd_t cmd);

// NeoPixel Functions
esp_err_t pm1_neo_refresh(void);
esp_err_t pm1_neo_set_cfg(uint8_t neo_num, uint16_t *neo_data, uint8_t refresh_flag);

// RTC RAM Functions
esp_err_t pm1_rtc_ram_write(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data);
esp_err_t pm1_rtc_ram_read(uint8_t rtc_ram_addr_start, uint8_t len, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // __PM1_CONTROL_H__