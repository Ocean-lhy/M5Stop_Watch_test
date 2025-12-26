#include "power_management.h"
#include "rx8130.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/rtc_io.h"
#include "bmi270.h"

static const char *TAG = "wakeup_test";

/*  移植已经实现的BMI270功能  */

#define ACCEL UINT8_C(0x00)
#define GYRO  UINT8_C(0x01)
#define AUX   UINT8_C(0x02)

static i2c_bus_device_handle_t bmi270_dev = NULL;
static struct bmi2_dev aux_bmi2_dev;
static struct bmi2_sens_config config[3];
static struct bmi2_sens_data bmi_sensor_data = {{0}};
static uint8_t sensor_list[3]                = {BMI2_ACCEL, BMI2_GYRO,
                                                BMI2_WRIST_WEAR_WAKE_UP}; /* Select features and their pins to be mapped to */
static struct bmi2_sens_int_config sens_int  = {.type = BMI2_WRIST_WEAR_WAKE_UP, .hw_int_pin = BMI2_INT1};
/* Initialize status of wrist wear wakeup interrupt */
struct bmi2_int_pin_config pin_config = {0};
uint16_t int_status                   = 0;

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32)) {
        return -1;
    }
    i2c_bus_read_bytes(bmi270_dev, reg_addr, len, reg_data);
    return 0;
}

int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32)) {
        return -1;
    }
    i2c_bus_write_bytes(bmi270_dev, reg_addr, len, reg_data);
    return 0;
}

void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    uint64_t m = (uint64_t)esp_timer_get_time();
    if (period) {
        uint64_t e = (m + period);
        if (m > e) {  // overflow
            while ((uint64_t)esp_timer_get_time() > e) {
                asm volatile("nop");
            }
        }
        while ((uint64_t)esp_timer_get_time() < e) {
            asm volatile("nop");
        }
    }
}

void bmi270_dev_init(i2c_bus_handle_t i2c_bus)
{
    bmi270_dev = i2c_bus_device_create(i2c_bus, BMI270_ADDR, 100000);
    if (bmi270_dev == NULL) {
        ESP_LOGE(TAG, "bmi270_dev create failed");
    } else {
        ESP_LOGI(TAG, "bmi270_dev create success");
    }

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type  = BMI2_GYRO;
    config[2].type     = BMI2_WRIST_WEAR_WAKE_UP;

    /* To enable the i2c interface settings for bmi270. */
    aux_bmi2_dev.intf            = BMI2_I2C_INTF;
    aux_bmi2_dev.read            = bmi2_i2c_read;
    aux_bmi2_dev.write           = bmi2_i2c_write;
    aux_bmi2_dev.delay_us        = bmi2_delay_us;
    aux_bmi2_dev.read_write_len  = 30;
    aux_bmi2_dev.config_file_ptr = NULL;
    // aux_bmi2_dev.intf_ptr        = (void *)BMI270_ADDR;

    /* Initialize bmi270. */
    bmi270_init(&aux_bmi2_dev);

    /* Get default configurations for the type of feature selected. */
    bmi270_get_sensor_config(config, 2, &aux_bmi2_dev);

    /* Configurations for accel. */
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[ACCEL].cfg.acc.bwp         = BMI2_ACC_OSR2_AVG2;
    config[ACCEL].cfg.acc.odr         = BMI2_ACC_ODR_100HZ;
    config[ACCEL].cfg.acc.range       = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.noise_perf  = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.bwp         = BMI2_GYR_OSR2_MODE;
    config[GYRO].cfg.gyr.odr         = BMI2_GYR_ODR_100HZ;
    config[GYRO].cfg.gyr.range       = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.ois_range   = BMI2_GYR_OIS_2000;

    /* Set new configurations for accel, gyro and aux. */
    bmi270_set_sensor_config(config, 2, &aux_bmi2_dev);

    /* NOTE:
     * Accel and gyro enable must be done after setting configurations
     */
    bmi270_sensor_enable(sensor_list, 3, &aux_bmi2_dev);

    bmi2_get_int_pin_config(&pin_config, &aux_bmi2_dev);
    ESP_LOGI(
        TAG,
        "Pin config: %d, %d,lvl: %d, od: %d, output_en: %d, input_en: %d, lvl: %d, od: %d, output_en: %d, input_en: %d",
        pin_config.pin_type, pin_config.int_latch, pin_config.pin_cfg[0].lvl, pin_config.pin_cfg[0].od,
        pin_config.pin_cfg[0].output_en, pin_config.pin_cfg[0].input_en, pin_config.pin_cfg[1].lvl,
        pin_config.pin_cfg[1].od, pin_config.pin_cfg[1].output_en, pin_config.pin_cfg[1].input_en);

    bmi270_get_sensor_config(config, 2, &aux_bmi2_dev);

    /* Set the aux configurations. */
    bmi270_set_sensor_config(config, 2, &aux_bmi2_dev);

    pin_config.pin_type             = BMI2_INT1;
    pin_config.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;
    pin_config.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_HIGH;
    pin_config.pin_cfg[0].od        = BMI2_INT_PUSH_PULL;
    pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_config.int_latch            = BMI2_INT_LATCH;

    /* Map data ready interrupt to interrupt pin. */
    // bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &aux_bmi2_dev);

    // bmi2_set_int_pin_config(&pin_config, &aux_bmi2_dev);
    // bmi270_map_feat_int(&sens_int, 1, &aux_bmi2_dev);
}

void bmi270_wakeup_enable(uint8_t enable)
{
    if (enable) {
        bmi270_sensor_enable(sensor_list, 3, &aux_bmi2_dev);
    } else {
        bmi270_sensor_disable(sensor_list, 3, &aux_bmi2_dev);
    }
}

void bmi270_dev_update()
{
    bmi2_get_sensor_data(&bmi_sensor_data, &aux_bmi2_dev);
    ESP_LOGI(TAG, "Accel: x: %d, y: %d, z: %d", bmi_sensor_data.acc.x, bmi_sensor_data.acc.y, bmi_sensor_data.acc.z);
    ESP_LOGI(TAG, "Gyro: x: %d, y: %d, z: %d", bmi_sensor_data.gyr.x, bmi_sensor_data.gyr.y, bmi_sensor_data.gyr.z);
    bmi2_get_int_status(&int_status, &aux_bmi2_dev);
    ESP_LOGI(TAG, "Int status: %d", int_status);
}

void bmi270_get_data(int *ax, int *ay, int *az, int *gx, int *gy, int *gz)
{
    if (ax) *ax = bmi_sensor_data.acc.x;
    if (ay) *ay = bmi_sensor_data.acc.y;
    if (az) *az = bmi_sensor_data.acc.z;
    if (gx) *gx = bmi_sensor_data.gyr.x;
    if (gy) *gy = bmi_sensor_data.gyr.y;
    if (gz) *gz = bmi_sensor_data.gyr.z;
}

void bmi270_dev_sleep()
{
    uint8_t data = 0x04;
    bmi2_i2c_write(0x7D, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    data = 0x17;
    bmi2_i2c_write(0x40, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    data = 0x03;
    bmi2_i2c_write(0x7C, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void bmi270_dev_wakeup()
{
    uint8_t data = 0x0E;
    bmi2_i2c_write(0x7D, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    data = 0xA8;
    bmi2_i2c_write(0x40, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    data = 0xA9;
    bmi2_i2c_write(0x42, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    data = 0x02;
    bmi2_i2c_write(0x7C, &data, 1, &aux_bmi2_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bmi270_sensor_enable(sensor_list, 3, &aux_bmi2_dev);
}

void bmi270_INT_wakeup_deepsleep_test()
{
    int8_t rslt;

    ESP_LOGI(TAG, "配置BMI270 INT1中断唤醒深度睡眠测试");

    /* 配置INT2引脚 */
    struct bmi2_int_pin_config int2_pin_cfg = {0};
    int2_pin_cfg.pin_type                   = BMI2_INT1;
    int2_pin_cfg.pin_cfg[0].lvl             = BMI2_INT_ACTIVE_HIGH;
    int2_pin_cfg.pin_cfg[0].od              = BMI2_INT_PUSH_PULL;
    int2_pin_cfg.pin_cfg[0].output_en       = BMI2_INT_OUTPUT_ENABLE;
    int2_pin_cfg.pin_cfg[0].input_en        = BMI2_INT_INPUT_DISABLE;
    int2_pin_cfg.int_latch                  = BMI2_INT_NON_LATCH;

    rslt = bmi2_set_int_pin_config(&int2_pin_cfg, &aux_bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "INT1引脚配置失败: %d", rslt);
        return;
    }

    /* 配置任意运动检测 */
    struct bmi2_sens_config any_motion_cfg = {0};
    any_motion_cfg.type                    = BMI2_ANY_MOTION;

    /* 获取默认配置 */
    rslt = bmi270_get_sensor_config(&any_motion_cfg, 1, &aux_bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "获取任意运动默认配置失败: %d", rslt);
        return;
    }

    /* 设置任意运动检测参数 */
    any_motion_cfg.cfg.any_motion.threshold = 30;           // 阈值 (约30mg)
    any_motion_cfg.cfg.any_motion.duration  = 10;           // 持续时间 (10 * 20ms = 200ms)
    any_motion_cfg.cfg.any_motion.select_x  = BMI2_ENABLE;  // 启用X轴
    any_motion_cfg.cfg.any_motion.select_y  = BMI2_ENABLE;  // 启用Y轴
    any_motion_cfg.cfg.any_motion.select_z  = BMI2_ENABLE;  // 启用Z轴

    /* 应用配置 */
    rslt = bmi270_set_sensor_config(&any_motion_cfg, 1, &aux_bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "设置任意运动配置失败: %d", rslt);
        return;
    }

    /* 启用传感器 */
    uint8_t sensor_list_motion[] = {BMI2_ACCEL, BMI2_ANY_MOTION};
    rslt                         = bmi270_sensor_enable(sensor_list_motion, 2, &aux_bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "启用传感器失败: %d", rslt);
        return;
    }

    /* 映射任意运动中断到INT1引脚 */
    rslt = bmi2_map_feat_int(BMI2_ANY_MOTION, BMI2_INT1, &aux_bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "映射中断到INT1失败: %d", rslt);
        return;
    }

    ESP_LOGI(TAG, "BMI270 INT1配置完成");
    ESP_LOGI(TAG, "任意运动检测阈值: %d mg", any_motion_cfg.cfg.any_motion.threshold);
    ESP_LOGI(TAG, "持续时间: %d ms", any_motion_cfg.cfg.any_motion.duration * 20);
    ESP_LOGI(TAG, "当设备发生运动时，INT1引脚将产生中断信号");

    /* 等待一段时间确保配置生效 */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    bmi2_get_int_status(&int_status, &aux_bmi2_dev);
    ESP_LOGI(TAG, "INT1状态: %d", int_status);

    ESP_LOGI(TAG, "请晃动设备以触发任意运动检测");

    // while (1)
    // {
    //     bmi2_get_int_status(&int_status, &aux_bmi2_dev);
    //     ESP_LOGI(TAG, "INT1状态: %d", int_status);
    //     uint8_t level = gpio_get_level(IRQ_PIN);
    //     ESP_LOGI(TAG, "唤醒引脚状态: %d", level);
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }

    /* 添加延时确保日志输出完成 */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* 进入深度睡眠 */
    // esp_deep_sleep_start();
}

/* ---------------------------------------------------------------------*/

RX8130_Class rx8130;
void wakeup_test(WakeupDevice wakeup_device, WakeupMode wakeup_mode)
{
    rx8130.begin(g_i2c_bus, RX8130_ADDR);
    // recovery
    rx8130.clearIrqFlags();
    rx8130.disableIrq();
    switch (wakeup_device) {
        case WakeupDevice::RTC_WAKEUP: {
            struct tm time;
            rx8130.getTime(&time);
            ESP_LOGI(TAG, "READ RX8130-> Time: %04d-%02d-%02d %02d:%02d:%02d", time.tm_year + 1900, time.tm_mon + 1,
                     time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
            // Set time 2025 09 27 12:00:00
            time.tm_year = 2025 - 1900;
            time.tm_mon  = 9 - 1;
            time.tm_mday = 27;
            time.tm_hour = 12;
            time.tm_min  = 0;
            time.tm_sec  = 0;
            ESP_LOGI(TAG, "SET RX8130-> Time: %04d-%02d-%02d %02d:%02d:%02d", time.tm_year + 1900, time.tm_mon + 1,
                     time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
            rx8130.setTime(&time);
        } break;
        case WakeupDevice::IMU_WAKEUP: {
            ESP_LOGI(TAG, "Configuring BMI270 for motion wake-up");
            bmi270_dev_init(g_i2c_bus);
            pm1_ldo_set_power_hold(true);  // hold LDO3V3 power open to keep IMU power on
        } break;
        case WakeupDevice::PORT_WAKEUP: {
            ESP_LOGI(TAG, "Configuring PORT for wake-up");
        } break;
    }

    // clear all GPIO and system irq flags
    pm1_irq_clear_gpio_flag(PM1_ADDR_IRQ_GPIO_ALL);
    pm1_irq_clear_sys_status(PM1_ADDR_IRQ_SYS_ALL);

    // set G1 to IRQ function, for sending irq signal to ESP32S3
    if (wakeup_mode == WakeupMode::WAKEUP_DEEPSLEEP) {
        pm1_gpio_set_func(PMG1_G12_PY_IRQ, PM1_GPIO_FUNC_IRQ);
    }

    // G0 RTC & IMU and G4 PORT irq MASK DISABLE
    pm1_irq_set_gpio_mask(PMG0_RTC_IMU_INT, PM1_IRQ_MASK_DISABLE);
    pm1_irq_set_gpio_mask(PMG4_PORT_INT, PM1_IRQ_MASK_DISABLE);

    pm1_irq_set_gpio_mask(PMG2_CHG_STAT, PM1_IRQ_MASK_ENABLE);
    pm1_irq_set_gpio_mask(PMG3_CHG_PROG, PM1_IRQ_MASK_ENABLE);

    pm1_gpio_num_t wakeup_gpio_num = wakeup_device == WakeupDevice::PORT_WAKEUP ? PMG4_PORT_INT : PMG0_RTC_IMU_INT;
    // set G0 to wakeup pin, for wakeup trigger
    pm1_gpio_set_mode(PMG0_RTC_IMU_INT, PM1_GPIO_MODE_INPUT);
    pm1_gpio_set_pupd(PMG0_RTC_IMU_INT, PM1_GPIO_PUPD_PULLUP);
    // pm1_gpio_set_drv(PMG0_RTC_IMU_INT, PM1_GPIO_DRV_PUSH_PULL);
    pm1_gpio_set_wake_en(PMG0_RTC_IMU_INT, PM1_GPIO_WAKE_ENABLE);
    pm1_gpio_set_wake_cfg(PMG0_RTC_IMU_INT, PM1_GPIO_WAKE_FALLING);

    pm1_gpio_set_mode(PMG4_PORT_INT, PM1_GPIO_MODE_INPUT);
    pm1_gpio_set_pupd(PMG4_PORT_INT, PM1_GPIO_PUPD_PULLUP);
    pm1_gpio_set_wake_en(PMG4_PORT_INT, PM1_GPIO_WAKE_ENABLE);
    pm1_gpio_set_wake_cfg(PMG4_PORT_INT, PM1_GPIO_WAKE_FALLING);

    // clear all wake flags
    pm1_wake_src_t wake_src;
    pm1_irq_gpio_t irq_gpio_num = PM1_ADDR_IRQ_GPIO_ALL;
    pm1_irq_btn_t irq_btn_num   = PM1_ADDR_IRQ_BTN_ALL;
    pm1_irq_sys_t irq_sys_num   = PM1_ADDR_IRQ_SYS_ALL;
    // must clen wake flags first
    pm1_wake_src_read(&wake_src, PM1_ADDR_WAKE_FLAG_ALL_CLEAN);
    // clean other irq flags
    pm1_irq_get_status(&irq_gpio_num, PM1_ADDR_IRQ_GPIO_ALL_CLEAN);
    pm1_irq_get_btn_status(&irq_btn_num, PM1_ADDR_IRQ_BTN_ALL_CLEAN);
    pm1_irq_get_sys_status(&irq_sys_num, PM1_ADDR_IRQ_SYS_ALL_CLEAN);

    if (wakeup_device == WakeupDevice::RTC_WAKEUP) {
        rx8130.setTimerIrq(10);
        if (wakeup_mode == WakeupMode::WAKEUP_SHUTDOWN) {
            pm1_ldo_set_power_hold(true);  // hold LDO3V3 power open to keep pullup on IRQ_PIN
        }
        ESP_LOGI(TAG, "system will deep sleep or shutdown, and it will automatically wake up within 20 seconds.");
#if 0
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << IRQ_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        esp_sleep_enable_ext0_wakeup(IRQ_PIN, 0);
        while (1)
        {
            uint8_t level = gpio_get_level(IRQ_PIN);
            ESP_LOGI(TAG, "唤醒引脚状态: %d", level);
            struct tm time;
            rx8130.getTime(&time);
            ESP_LOGI(TAG, "RX8130-> Time: %04d-%02d-%02d %02d:%02d:%02d", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
            pm1_gpio_in_state_t gpio_state;
            pm1_gpio_get_in_state(PMG0_RTC_IMU_INT, &gpio_state);
            ESP_LOGI(TAG, "G0 state: %s", gpio_state == PM1_GPIO_IN_STATE_HIGH ? "HIGH" : "LOW");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
#endif
    } else if (wakeup_device == WakeupDevice::IMU_WAKEUP) {
        bmi270_INT_wakeup_deepsleep_test();
        ESP_LOGI(TAG, "system will deep sleep or shutdown, and it will automatically wake up when motion is detected.");
#if 0
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << IRQ_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        esp_sleep_enable_ext0_wakeup(IRQ_PIN, 0);
        while (1)
        {
            pm1_gpio_in_state_t gpio_state;
            pm1_gpio_get_in_state(PMG0_RTC_IMU_INT, &gpio_state);
            ESP_LOGI(TAG, "G0 state: %s", gpio_state == PM1_GPIO_IN_STATE_HIGH ? "HIGH" : "LOW");
            pm1_gpio_get_in_state(PMG4_PORT_INT, &gpio_state);
            ESP_LOGI(TAG, "G4 state: %s", gpio_state == PM1_GPIO_IN_STATE_HIGH ? "HIGH" : "LOW");
            uint8_t level = gpio_get_level(IRQ_PIN);
            ESP_LOGI(TAG, "唤醒引脚状态: %d", level);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
#endif
    } else if (wakeup_device == WakeupDevice::PORT_WAKEUP) {
        ESP_LOGI(TAG, "system will deep sleep or shutdown, and it will automatically wake up when PORT is detected.");
    }

    if (wakeup_mode == WakeupMode::WAKEUP_DEEPSLEEP) {
        py32_io_expander_sleep();
        pm1_pwr_set_cfg(PM1_PWR_CFG_LED_CONTROL, 0, NULL);  // disable LED
        pm1_pwr_set_cfg(PM1_PWR_CFG_5V_INOUT, 0, NULL);

        gpio_reset_pin(IRQ_PIN);
        rtc_gpio_pullup_en(IRQ_PIN);
        esp_sleep_enable_ext0_wakeup(IRQ_PIN, 0);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_deep_sleep_start();
    } else {
        pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN);
    }
}