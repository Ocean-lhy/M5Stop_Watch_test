#include "tools.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include <dirent.h>
#include <string.h>
#include "lwip/inet.h"
#include "rx8130.h"
#include "accel_gyro_bmi270.h"
#include <lwip/tcpip.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "freertos/event_groups.h"
#include <M5GFX.h>
#include <lgfx/v1/panel/Panel_AMOLED.hpp>

static const char *TAG = "tools";

i2c_bus_handle_t g_i2c_bus = NULL; // I2C bus handle
i2c_bus_device_handle_t i2c_device_all[0xFF] = {NULL};

// display
static constexpr gpio_num_t cfg_pin_sclk = QSPI_SCLK_PIN;
static constexpr gpio_num_t cfg_pin_io0  = QSPI_D0_PIN;
static constexpr gpio_num_t cfg_pin_io1  = QSPI_D1_PIN;
static constexpr gpio_num_t cfg_pin_io2  = QSPI_D2_PIN;
static constexpr gpio_num_t cfg_pin_io3  = QSPI_D3_PIN;
static constexpr gpio_num_t cfg_pin_cs   = QSPI_CS_PIN;
static constexpr gpio_num_t cfg_pin_te   = QSPI_TE_PIN;
static constexpr gpio_num_t cfg_pin_rst  = QSPI_RST_PIN;    // NC

class Panel_CO5300 : public lgfx::Panel_AMOLED {
    public:
        Panel_CO5300(void)
        {
            _cfg.memory_width = _cfg.panel_width = 480;
            _cfg.memory_height = _cfg.panel_height = 480;
            _write_depth                           = lgfx::color_depth_t::rgb565_2Byte;
            _read_depth                            = lgfx::color_depth_t::rgb565_2Byte;
        }
    
        const uint8_t* getInitCommands(uint8_t listno) const override
        {
            static constexpr uint8_t list0[] = {
                0x11, 0 + CMD_INIT_DELAY,
                150,  // Sleep out
                0xC4, 1,
                0x80, 0x35,
                1,    0x80,
                0x44, 2,
                0x01, 0xD2,  // Tear Effect Line = 0x1D2 == 466
                0x53, 1,
                0x20, 0x20,
                0,    0x36,
                1,    0,
                0x51, 1,
                0xA0, 0x29,
                0,    0xff,
                0xff  // end
            };
            switch (listno) {
                case 0:
                    return list0;
                default:
                    return nullptr;
            }
        }
    };

class M5StopWatch : public M5GFX {
    lgfx::Bus_SPI _bus_instance;
    Panel_CO5300 _panel_instance;

public:
    M5StopWatch(void)
    {
    }

    static constexpr int in_i2c_port                   = 1;  // I2C_NUM_1

    bool init_impl(bool use_reset, bool use_clear) override
    {
        {
            auto cfg = _bus_instance.config();

            cfg.freq_write = 80000000;
            cfg.freq_read  = 10000000;  // irrelevant

            cfg.pin_sclk = cfg_pin_sclk;
            cfg.pin_io0  = cfg_pin_io0;
            cfg.pin_io1  = cfg_pin_io1;
            cfg.pin_io2  = cfg_pin_io2;
            cfg.pin_io3  = cfg_pin_io3;

            cfg.spi_host    = LCD_HOST;
            cfg.spi_mode    = 0;  // SPI_MODE0;
            cfg.spi_3wire   = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg         = _panel_instance.config();
            cfg.pin_rst      = cfg_pin_rst;
            cfg.pin_cs       = cfg_pin_cs;
            cfg.panel_width  = 468;
            cfg.panel_height = 466;
            cfg.offset_x     = 6;
            cfg.offset_y     = 0;

            cfg.readable = false;

            _panel_instance.config(cfg);
        }

        setPanel(&_panel_instance);

        // M5GFX use I2C master, but we use I2C bus, so we don't need to init it
        // lgfx::pinMode(cfg_pin_te, lgfx::pin_mode_t::input_pullup);
        // lgfx::i2c::init(in_i2c_port, TOUCH_SDA_PIN, TOUCH_SCL_PIN);  // SDA, SCL, freq

        if (!LGFX_Device::init_impl(use_reset, use_clear)) return false;

        enableFrameBuffer(true);

        _panel_instance.setBrightness(128);

        return true;
    }

    bool enableFrameBuffer(bool auto_display = false)
    {
        if (_panel_instance.initPanelFb()) {
            auto fbPanel = _panel_instance.getPanelFb();
            if (fbPanel) {
                fbPanel->setBus(&_bus_instance);
                fbPanel->setAutoDisplay(auto_display);
                setPanel(fbPanel);
                return true;
            }
        }
        return false;
    }

    void disableFrameBuffer()
    {
        auto fbPanel = _panel_instance.getPanelFb();
        if (fbPanel) {
            _panel_instance.deinitPanelFb();
            setPanel(&_panel_instance);
        }
    }
};
    
M5StopWatch gfx;

void display_init()
{
    gfx.init();
    gfx.fillScreen(TFT_BLUE);
    gfx.setTextColor(TFT_WHITE);
    gfx.setTextSize(2);
}

void display_gfx_loop()
{
    static uint32_t prev_sec;
    static uint32_t count;
    count++;
    uint32_t t   = lgfx::v1::millis();
    uint32_t sec = t / 1000;
    if (prev_sec != sec) {
        prev_sec = sec;
        printf("count:%d\n", (int)count);
        fflush(stdout);
        lgfx::v1::delay(1);
        count = 0;
        gfx.fillScreen(rand());
        gfx.setRotation(gfx.getRotation() + 1);
        gfx.setCursor(112, 48);
        gfx.println("Hello! Stop Watch!");
    }

    // printf("M5StopWatch loop\n");fflush(stdout);
    gfx.fillCircle(105 + (rand() & 255), 105 + (rand() & 255), 16, rand());
}


// i2c_bus
void register_i2c_bus_device()
{
    // Create I2C bus configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        },
        .clk_flags = 0,
    };
    // Create the I2C bus
    g_i2c_bus = i2c_bus_create(I2C_NUM_0, &conf);
}

void scan_i2c_bus_device()
{
    uint8_t i2c_device_addr[256] = {0};
    uint8_t i2c_device_num = 0;
    i2c_device_num = i2c_bus_scan(g_i2c_bus,i2c_device_addr,0xFF);
    ESP_LOGI(TAG, "I2C devices found:");
    for (int i = 0; i < i2c_device_num; i++) {
        ESP_LOGI(TAG, "I2C device address: 0x%02X", i2c_device_addr[i]);
        i2c_device_all[i2c_device_addr[i]] = i2c_bus_device_create(g_i2c_bus, i2c_device_addr[i], 100000); //创建I2C设备
    }
    ESP_LOGI(TAG, "Total I2C devices found: %d", i2c_device_num);
}

// bmi270
void bmi270_test()
{
    accel_gyro_bmi270_init(g_i2c_bus);
    if (accel_gyro_bmi270_check_irq()) {
        accel_gyro_bmi270_clear_irq_int();
        ESP_LOGI(TAG, "imu irq detected! clear it!");
    }
    accel_gyro_bmi270_enable_sensor();

    static struct bmi2_sens_data bmi_sensor_data;
    uint8_t ct = 10;
    while(ct--)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
        accel_gyro_bmi270_get_data(&bmi_sensor_data);
        ESP_LOGI(TAG, "BMI270-> Accel: x: %d, y: %d, z: %d", bmi_sensor_data.acc.x, bmi_sensor_data.acc.y, bmi_sensor_data.acc.z);
        ESP_LOGI(TAG, "BMI270-> Gyro: x: %d, y: %d, z: %d", bmi_sensor_data.gyr.x, bmi_sensor_data.gyr.y, bmi_sensor_data.gyr.z);
    }
}

// rx8130
#include "power_management.h"
RX8130_Class rx8130;
const bool esp32s3_sleep_flag = false; // if true, use esp32s3 sleep, otherwise use pm1 shutdown
void rx8130_wakeup_test()
{
    // i2c_master_bus_handle_t i2c0_bus_hdl = i2c_bus_get_internal_bus_handle(g_i2c_bus);
    // rx8130.begin(i2c0_bus_hdl, RX8130_ADDR);
    struct tm time;
    rx8130.getTime(&time);
    //
    ESP_LOGI(TAG, "READ RX8130-> Time: %04d-%02d-%02d %02d:%02d:%02d", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    // Set time 2025 09 27 12:00:00
    time.tm_year = 2025 - 1900;
    time.tm_mon = 9 - 1;
    time.tm_mday = 27;
    time.tm_hour = 12;
    time.tm_min = 0;
    time.tm_sec = 0;
    ESP_LOGI(TAG, "SET RX8130-> Time: %04d-%02d-%02d %02d:%02d:%02d", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    rx8130.setTime(&time);

    // clear all GPIO and system irq flags
    pm1_irq_clear_gpio_flag(PM1_ADDR_IRQ_GPIO_ALL);
    pm1_irq_clear_sys_status(PM1_ADDR_IRQ_SYS_ALL);

    // set G2 to IRQ function, for sending irq signal to ESP32S3
    if (esp32s3_sleep_flag) 
    {
        pm1_gpio_set_func(PM1_GPIO_NUM_2, PM1_GPIO_FUNC_IRQ);
    }

    // G0 RTC and G4 IMU irq MASK DISABLE
    pm1_irq_set_gpio_mask(PM1_GPIO_NUM_0, PM1_IRQ_MASK_DISABLE);
    pm1_irq_set_gpio_mask(PM1_GPIO_NUM_4, PM1_IRQ_MASK_DISABLE);

    pm1_irq_set_gpio_mask(PM1_GPIO_NUM_1, PM1_IRQ_MASK_ENABLE);
    pm1_irq_set_gpio_mask(PM1_GPIO_NUM_3, PM1_IRQ_MASK_ENABLE);

    // set G0 to wakeup pin, for wakeup trigger
    pm1_gpio_set_mode(PM1_GPIO_NUM_0, PM1_GPIO_MODE_INPUT);
    pm1_gpio_set_pupd(PM1_GPIO_NUM_0, PM1_GPIO_PUPD_PULLUP);
    // pm1_gpio_set_drv(PM1_GPIO_NUM_0, PM1_GPIO_DRV_PUSH_PULL);
    pm1_gpio_set_wake_en(PM1_GPIO_NUM_0, PM1_GPIO_WAKE_ENABLE);
    pm1_gpio_set_wake_cfg(PM1_GPIO_NUM_0, PM1_GPIO_WAKE_FALLING);

    // clear all wake flags
    pm1_wake_src_t wake_src;
    pm1_irq_gpio_t irq_gpio_num = PM1_ADDR_IRQ_GPIO_ALL;
    pm1_irq_btn_t irq_btn_num = PM1_ADDR_IRQ_BTN_ALL;
    pm1_irq_sys_t irq_sys_num = PM1_ADDR_IRQ_SYS_ALL;
    // must clen wake flags first
    pm1_wake_src_read(&wake_src, PM1_ADDR_WAKE_FLAG_ALL_CLEAN);
    // clean other irq flags
    pm1_irq_get_status(&irq_gpio_num, PM1_ADDR_IRQ_GPIO_ALL_CLEAN);
    pm1_irq_get_btn_status(&irq_btn_num, PM1_ADDR_IRQ_BTN_ALL_CLEAN);
    pm1_irq_get_sys_status(&irq_sys_num, PM1_ADDR_IRQ_SYS_ALL_CLEAN);

    rx8130.setTimerIrq(10);

    ESP_LOGI(TAG, "system will shutdown, and it will automatically wake up within 10 seconds.");
    
    if (esp32s3_sleep_flag) 
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << IRQ_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        esp_sleep_enable_ext0_wakeup(IRQ_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        esp_deep_sleep_start();
    } 
    else 
    {
        pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN);
    }
}

void rx8130_recovery()
{
    rx8130.begin(g_i2c_bus, RX8130_ADDR);
    rx8130.clearIrqFlags();
    rx8130.disableIrq();
}
