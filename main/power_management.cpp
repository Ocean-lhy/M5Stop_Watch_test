#include "power_management.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "power_management";

// PMIC
m5_stamp_pm1& pmic                        = m5_stamp_pm1::getInstance();
i2c_bus_device_handle_t pm1_device_handle = NULL;

void pmic_init()
{
    if (pm1_device_handle != NULL) {
        return;
    }
    pm1_device_handle = i2c_bus_device_create(
        g_i2c_bus, PMIC_ADDR, 100000);  // default 100KHz, if you want to use 400KHz, you need to configure
    pmic.pm1_init(g_i2c_bus, &pm1_device_handle, 100000);

    pm1_set_clk_speed(PM1_CLK_SPEED_400KHZ);

    // set button delay click 1s
    pm1_btn_set_cfg(PM1_ADDR_BTN_TYPE_CLICK, PM1_ADDR_BTN_CLICK_DELAY_1000MS);
    // disable WDT, default is open
    pm1_wdt_set(PM1_WDT_CTRL_DISABLE, 0);
    //  hold LDO power close when power off, default is close
    pm1_ldo_set_power_hold(false);

    // set 5V output enable or input enable
    // pm1_pwr_set_cfg(PM1_PWR_CFG_5V_INOUT, PM1_PWR_CFG_5V_INOUT, NULL);
    // pm1_pwr_set_cfg(PM1_PWR_CFG_5V_INOUT, 0, NULL);

    // set LED control enable
    pm1_pwr_set_cfg(PM1_PWR_CFG_LED_CONTROL, 0, NULL);

    // set charge enable or disable, this setting will keep working after power off
    pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, PM1_PWR_CFG_CHG_EN, NULL);
    // pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, 0, NULL);

    // quick charge
    pm1_gpio_set(PMG3_CHG_PROG, PM1_GPIO_MODE_OUTPUT, PM1_GPIO_OUTPUT_LOW, PM1_GPIO_PUPD_NC, PM1_GPIO_DRV_PUSH_PULL);
    // normal charge
    // pm1_gpio_set(PMG3_CHG_PROG, PM1_GPIO_MODE_OUTPUT, PM1_GPIO_OUTPUT_HIGH, PM1_GPIO_PUPD_NC,
    // PM1_GPIO_DRV_PUSH_PULL);

    // set charge detect pin to input
    pm1_gpio_set_mode(PMG2_CHG_STAT, PM1_GPIO_MODE_INPUT);

    // get charge state
    pm1_gpio_in_state_t gpio_state;
    pm1_gpio_get_in_state(PMG2_CHG_STAT, &gpio_state);

    // get reference voltage
    uint16_t reference_voltage;
    pm1_vref_read(&reference_voltage);

    // get battery voltage
    uint16_t battery_voltage;
    pm1_vbat_read(&battery_voltage);

    // get input voltage
    uint16_t input_voltage;
    pm1_vin_read(&input_voltage);

    // get 5VINOUT voltage
    uint16_t _5vinout_voltage;
    pm1_5vinout_read(&_5vinout_voltage);
}

void clean_irq_flags()
{
    pm1_irq_clear_gpio_flag(PM1_ADDR_IRQ_GPIO_ALL);
    pm1_irq_clear_sys_status(PM1_ADDR_IRQ_SYS_ALL);

    pm1_gpio_set_wake_en(PM1_GPIO_NUM_0, PM1_GPIO_WAKE_DISABLE);
    pm1_gpio_set_wake_en(PM1_GPIO_NUM_4, PM1_GPIO_WAKE_DISABLE);

    pm1_irq_gpio_t irq_gpio_num = PM1_ADDR_IRQ_GPIO_ALL;
    pm1_irq_btn_t irq_btn_num   = PM1_ADDR_IRQ_BTN_ALL;
    pm1_irq_sys_t irq_sys_num   = PM1_ADDR_IRQ_SYS_ALL;
    pm1_irq_get_status(&irq_gpio_num, PM1_ADDR_IRQ_GPIO_ALL_CLEAN);
    pm1_irq_get_btn_status(&irq_btn_num, PM1_ADDR_IRQ_BTN_ALL_CLEAN);
    pm1_irq_get_sys_status(&irq_sys_num, PM1_ADDR_IRQ_SYS_ALL_CLEAN);
}

// PY32 IO Expander
m5_io_py32ioexpander io_expander;
i2c_bus_device_handle_t py32_device_handle = NULL;

void py32_io_expander_init()
{
    if (py32_device_handle == NULL) {
        py32_device_handle =
            i2c_bus_device_create(g_i2c_bus, PY32_IO_EXPANDER_ADDR,
                                  100000);  // default 100KHz, if you want to use 400KHz, you need to configure
        io_expander = m5_io_py32ioexpander(g_i2c_bus, py32_device_handle);
        uint8_t fw_version;
        io_expander.readVersion(&fw_version);
        ESP_LOGI(TAG, "PY32 IO Expander-> FW Version: %d", fw_version);
        uint16_t uid;
        io_expander.readDeviceUID(&uid);
        ESP_LOGI(TAG, "PY32 IO Expander-> UID: 0x%04X", uid);
        io_expander.init(PY32_IO_EXPANDER_ADDR);
    }

    // sleep close, 400KHz, wake mode, inter pull off
    io_expander.setI2cConfig(0, true, true, true);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint16_t ref_voltage;
    io_expander.getRefVoltage(&ref_voltage);
    ESP_LOGI(TAG, "PY32 IO Expander-> Ref Voltage: %d", ref_voltage);

    uint16_t temperature;
    io_expander.readTemperature(&temperature);
    ESP_LOGI(TAG, "PY32 IO Expander-> Temperature: %d", temperature);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    io_expander.pinMode(PY32_MOTOR_EN_PIN, OUTPUT);
    io_expander.pinMode(PY32_L3B_EN_PIN, OUTPUT);
    io_expander.pinMode(PY32_SPK_PA_PIN, OUTPUT);
    io_expander.pinMode(PY32_TP_RST_PIN, OUTPUT);
    io_expander.pinMode(PY32_OLED_RST_PIN, OUTPUT);
    io_expander.pinMode(PY32_MUX_CTR_PIN, OUTPUT);
    io_expander.pinMode(PY32_AU_EN_PIN, OUTPUT);

    io_expander.digitalWrite(PY32_L3B_EN_PIN, 1);
    io_expander.digitalWrite(PY32_TP_RST_PIN, 1);
    io_expander.digitalWrite(PY32_SPK_PA_PIN, 0);
    io_expander.digitalWrite(PY32_OLED_RST_PIN, 1);
    io_expander.digitalWrite(PY32_MUX_CTR_PIN, 0);
    io_expander.digitalWrite(PY32_AU_EN_PIN, 1);

    io_expander.setPwmFrequency(5000);

    gpio_set_direction(SPK_PA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SPK_PA_PIN, 0);
}

void py32_io_expander_sleep()
{
    if (py32_device_handle == NULL) {
        // default 100KHz, if you want to use 400KHz, you need to configure
        py32_device_handle = i2c_bus_device_create(g_i2c_bus, PY32_IO_EXPANDER_ADDR, 100000);
        io_expander        = m5_io_py32ioexpander(g_i2c_bus, py32_device_handle);
        uint8_t fw_version;
        io_expander.readVersion(&fw_version);
        ESP_LOGI(TAG, "PY32 IO Expander-> FW Version: %d", fw_version);
        uint16_t uid;
        io_expander.readDeviceUID(&uid);
        ESP_LOGI(TAG, "PY32 IO Expander-> UID: 0x%04X", uid);
        io_expander.init(PY32_IO_EXPANDER_ADDR);
    }
    io_expander.pinMode(PY32_MOTOR_EN_PIN, INPUT);
    io_expander.pinMode(PY32_L3B_EN_PIN, INPUT);
    io_expander.pinMode(PY32_SPK_PA_PIN, INPUT);
    io_expander.pinMode(PY32_TP_RST_PIN, INPUT);
    io_expander.pinMode(PY32_OLED_RST_PIN, INPUT);
    io_expander.pinMode(PY32_MUX_CTR_PIN, INPUT);
    io_expander.pinMode(PY32_AU_EN_PIN, INPUT);
    io_expander.setDriveMode(PY32_MOTOR_EN_PIN, 1);
    io_expander.setDriveMode(PY32_L3B_EN_PIN, 1);
    io_expander.setDriveMode(PY32_SPK_PA_PIN, 1);
    io_expander.setDriveMode(PY32_TP_RST_PIN, 1);
    io_expander.setDriveMode(PY32_OLED_RST_PIN, 1);
    io_expander.setDriveMode(PY32_MUX_CTR_PIN, 1);
    io_expander.setDriveMode(PY32_AU_EN_PIN, 1);
    io_expander.setI2cConfig(3, true, true, true);
}

// power management
// shipping mode, close LDO3V3 and shutdown, only PMIC and RTC keep on
void stop_watch_power_mode_L0()
{
    pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, 0, NULL);  // disable charge
    pm1_ldo_set_power_hold(false);                 // hold LDO3V3(PM_3V3_L1_EN) power close
    pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN);
}

// standby mode, open LDO3V3 and shutdown, only PMIC, RTC, IMU&MAG keep on
void stop_watch_power_mode_L1()
{
    pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, 0, NULL);  // disable charge
    pm1_ldo_set_power_hold(true);                  // hold LDO3V3(PM_3V3_L1_EN) power open
    pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN);
}

// deepsleep mode, put all devices into sleep mode on the basis of L3A
void stop_watch_power_mode_L2()
{
    if (pm1_device_handle == NULL) {
        pm1_device_handle = i2c_bus_device_create(
            g_i2c_bus, PMIC_ADDR, 100000);  // default 100KHz, if you want to use 400KHz, you need to configure
        pmic.pm1_init(g_i2c_bus, &pm1_device_handle, 100000);
    }
    pm1_btn_set_cfg(PM1_ADDR_BTN_TYPE_CLICK, PM1_ADDR_BTN_CLICK_DELAY_1000MS);  // 单击延迟1秒
    pm1_wdt_set(PM1_WDT_CTRL_DISABLE, 0);                                       // 禁用WDT

    pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, 0, NULL);  // disable charge
    pm1_pwr_set_cfg(PM1_PWR_CFG_5V_INOUT, 0, NULL);
    pm1_pwr_set_cfg(PM1_PWR_CFG_LED_CONTROL, 0, NULL);  // disable LED

    pm1_gpio_set(PM1_GPIO_NUM_0, PM1_GPIO_MODE_INPUT, PM1_GPIO_INPUT_NC, PM1_GPIO_PUPD_NC, PM1_GPIO_DRV_OPEN_DRAIN);
    pm1_gpio_set(PM1_GPIO_NUM_1, PM1_GPIO_MODE_INPUT, PM1_GPIO_INPUT_NC, PM1_GPIO_PUPD_NC, PM1_GPIO_DRV_OPEN_DRAIN);
    pm1_gpio_set(PM1_GPIO_NUM_2, PM1_GPIO_MODE_INPUT, PM1_GPIO_INPUT_NC, PM1_GPIO_PUPD_NC, PM1_GPIO_DRV_OPEN_DRAIN);
    pm1_gpio_set(PM1_GPIO_NUM_3, PM1_GPIO_MODE_INPUT, PM1_GPIO_INPUT_NC, PM1_GPIO_PUPD_NC, PM1_GPIO_DRV_OPEN_DRAIN);
    pm1_gpio_set(PM1_GPIO_NUM_4, PM1_GPIO_MODE_INPUT, PM1_GPIO_INPUT_NC, PM1_GPIO_PUPD_NC, PM1_GPIO_DRV_OPEN_DRAIN);

    // IMU sleep
    i2c_bus_device_handle_t i2c_dev_handle_bmi270 = i2c_bus_device_create(g_i2c_bus, BMI270_ADDR, 100000);
    if (i2c_dev_handle_bmi270 == NULL) {
        ESP_LOGE(TAG, "i2c_dev_handle_bmi270 create failed");
    }
    uint8_t reg_data = 0x01;
    i2c_bus_write_byte(i2c_dev_handle_bmi270, 0x7C, reg_data);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    reg_data = 0x00;
    i2c_bus_write_byte(i2c_dev_handle_bmi270, 0x7D, reg_data);

    // cst820 sleep
    i2c_bus_device_handle_t cst820_dev = i2c_bus_device_create(g_i2c_bus, CST820_ADDR, 100000);
    if (cst820_dev == NULL) {
        ESP_LOGE(TAG, "cst820_dev create failed");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t i2c_buf = 0x03;
    i2c_bus_write_byte(cst820_dev, 0xE5, i2c_buf);

    // py32 sleep
    py32_io_expander_sleep();

    // pmic sleep
    pm1_set_i2c_sleep_time(3);

    rtc_gpio_isolate((gpio_num_t)GPIO_NUM_1);   // KEY1
    rtc_gpio_isolate((gpio_num_t)GPIO_NUM_2);   // KEY2
    rtc_gpio_isolate((gpio_num_t)GPIO_NUM_13);  // TP_INT
    rtc_gpio_isolate((gpio_num_t)GPIO_NUM_12);  // PY_INT

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_deep_sleep_hold_en();
    esp_deep_sleep_start();
}

// core active mode, only close OLED, MIC, SPK, MOTOR, EXT.PORT.
void stop_watch_power_mode_L3A()
{
    io_expander.digitalWrite(PY32_L3B_EN_PIN, 0);
}

// all active mode, all power on
void stop_watch_power_mode_L3B()
{
    io_expander.digitalWrite(PY32_L3B_EN_PIN, 1);
}

void stop_watch_speaker_set(bool enable)
{
    if (enable) {
        io_expander.digitalWrite(PY32_SPK_PA_PIN, 1);
        gpio_set_level(SPK_PA_PIN, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    } else {
        io_expander.digitalWrite(PY32_SPK_PA_PIN, 0);
        gpio_set_level(SPK_PA_PIN, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void stop_watch_set_i2c_speed(bool is_400khz)
{
    if (is_400khz) {
        pm1_set_clk_speed(PM1_CLK_SPEED_400KHZ);
        i2c_bus_device_delete(&pm1_device_handle);
        pm1_device_handle = i2c_bus_device_create(
            g_i2c_bus, PMIC_ADDR, 400000);  // default 100KHz, if you want to use 400KHz, you need to configure
        pmic.pm1_init(g_i2c_bus, &pm1_device_handle, 400000);

        io_expander.setI2cConfig(0, true, true, true);  // 400k
        i2c_bus_device_delete(&py32_device_handle);
        py32_device_handle =
            i2c_bus_device_create(g_i2c_bus, PY32_IO_EXPANDER_ADDR,
                                  400000);  // default 100KHz, if you want to use 400KHz, you need to configure
        io_expander = m5_io_py32ioexpander(g_i2c_bus, py32_device_handle);
        io_expander.init(PY32_IO_EXPANDER_ADDR);
    } else {
        pm1_set_clk_speed(PM1_CLK_SPEED_100KHZ);
        i2c_bus_device_delete(&pm1_device_handle);
        pm1_device_handle = i2c_bus_device_create(
            g_i2c_bus, PMIC_ADDR, 100000);  // default 100KHz, if you want to use 400KHz, you need to configure
        pmic.pm1_init(g_i2c_bus, &pm1_device_handle, 100000);

        io_expander.setI2cConfig(0, false, true, true);  // 100k
        i2c_bus_device_delete(&py32_device_handle);
        py32_device_handle =
            i2c_bus_device_create(g_i2c_bus, PY32_IO_EXPANDER_ADDR,
                                  100000);  // default 100KHz, if you want to use 400KHz, you need to configure
        io_expander = m5_io_py32ioexpander(g_i2c_bus, py32_device_handle);
        io_expander.init(PY32_IO_EXPANDER_ADDR);
    }
}