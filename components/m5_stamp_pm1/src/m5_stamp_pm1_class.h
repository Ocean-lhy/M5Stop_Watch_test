#ifndef __M5_STAMP_PM1_OBJECT_SUPPORT_H__
#define __M5_STAMP_PM1_OBJECT_SUPPORT_H__

#include "m5_stamp_pm1.h"
#include <utility>

#ifdef __cplusplus

// 定义一个宏来自动创建转发函数（支持静态和对象调用）
#define DEFINE_PM1_FORWARD_FUNCTION(func_name) \
    template<typename... Args> \
    static auto func_name(Args&&... args) -> decltype(::func_name(std::forward<Args>(args)...)) { \
        return ::func_name(std::forward<Args>(args)...); \
    }

class m5_stamp_pm1 {
private:
    // 私有构造函数，防止直接实例化
    m5_stamp_pm1() = default;
    ~m5_stamp_pm1() = default;
    
    // 禁用拷贝构造和赋值（单例模式必须）
    m5_stamp_pm1(const m5_stamp_pm1&) = delete;
    m5_stamp_pm1& operator=(const m5_stamp_pm1&) = delete;
    
public:
    // 获取单例实例的静态方法
    static m5_stamp_pm1& getInstance() {
        static m5_stamp_pm1 instance;
        return instance;
    }
    
    // 基础初始化和配置函数
    DEFINE_PM1_FORWARD_FUNCTION(pm1_init)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_deinit)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_set_clk_speed)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_set_i2c_sleep_time)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_i2c_try_wake)
    
    // Device Information Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_get_hw_version)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_get_sw_version)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_get_chip_id)
    
    // Pin Status Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_read_and_check_pin_status)
    
    // GPIO Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_func)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_mode)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_state)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_pupd)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_drv)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_led_en_set_drv)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_wake_en)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_wake_cfg)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_get_in_state)
    
    // GPIO Power Hold Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_set_power_hold)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_gpio_get_power_hold)
    
    // LDO Power Hold Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_ldo_set_power_hold)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_ldo_get_power_hold)
    
    // ADC Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_adc_read)
    
    // PWM Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_pwm_set)
    
    // WDT Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_wdt_set)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_wdt_feed)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_wdt_get_wdt_cnt)
    
    // Timer Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_tim_set)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_tim_clear)
    
    // Download and Button Control Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_download_enable_set)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_download_enable_get)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_single_reset_dis_set)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_single_reset_dis_get)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_double_poweroff_dis_set)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_double_poweroff_dis_get)
    
    // Button Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_btn_set_cfg)
    
    // IRQ Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_status)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_clear_gpio_flag)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_sys_status)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_clear_sys_status)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_btn_status)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_clear_btn_status)
    
    // IRQ Mask Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_set_gpio_mask)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_gpio_mask)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_set_gpio_mask_all)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_gpio_mask_all)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_set_sys_mask)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_sys_mask)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_set_sys_mask_all)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_sys_mask_all)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_set_btn_mask)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_btn_mask)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_set_btn_mask_all)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_irq_get_btn_mask_all)
    
    // Battery and Voltage Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_batt_set_lvp)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_vref_read)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_vbat_read)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_vin_read)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_5vinout_read)
    
    // Power and Wake Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_pwr_src_read)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_pwr_set_cfg)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_pwr_clear_cfg)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_pwr_get_cfg)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_wake_src_read)
    
    // System Command Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_sys_cmd)
    
    // NeoPixel Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_neo_refresh)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_neo_set_cfg)
    
    // RTC RAM Functions
    DEFINE_PM1_FORWARD_FUNCTION(pm1_rtc_ram_write)
    DEFINE_PM1_FORWARD_FUNCTION(pm1_rtc_ram_read)
};

#endif // __cplusplus

#endif // __M5_STAMP_PM1_OBJECT_SUPPORT_H__


