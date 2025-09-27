# M5 IO PI4IOE5V6408 DRIVER

M5STACK IO扩展芯片 PI4IOE5V6408 提供驱动支持

| 外设     | 芯片           | 协议  | 电压   |
| ------ | ------------ | --- | ---- |
| IO扩展芯片 | PI4IOE5V6408 | I2C | 3.3V |

## 特性支持

- [x] 寄存器函数化控制
- [x] 支持I2C\_BUS
- [x] 支持类Arduino方式调用
- [x] 支持简单的中断函数实现

## 使用示例

```c 
/*
你的引用代码
*/
#include "m5_io_pi4ioe5v6408.hpp"  //引用本驱动库

#define PI4IO_M_ADDR 0x43   //IO扩展芯片

m5_io_pi4ioe5v6408 io_expander(I2C_NUM_0, GPIO_NUM_10, GPIO_NUM_8, 400000, GPIO_NUM_3); // I2C port, SDA pin, SCL pin, frequency, INT pin
// m5_io_pi4ioe5v6408 io_expander(I2C_NUM_0, GPIO_NUM_10, GPIO_NUM_8, 400000, GPIO_NUM_NC); // I2C port, SDA pin, SCL pin, frequency, not INT pin

void test_m5_io_callback(void);
void test_m5_io_callback_with_arg(void *arg);

extern "C" void app_main(void)
{

    // io4 输出并拉高
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_4),  // 选择要配置的 GPIO
        .mode = GPIO_MODE_OUTPUT,               // 设为输入模式
        .pull_up_en = GPIO_PULLUP_DISABLE,      // 启用上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // 禁用下拉电阻
        .intr_type = GPIO_INTR_DISABLE      // 低电平触发中断
    };
    // Configure the GPIO pin
    gpio_config(&io_conf); 
    gpio_set_level(GPIO_NUM_4, 0); // Set GPIO 4 to high

    //
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Initialize the I2C bus
    io_expander.init(PI4IO_M_ADDR);
    // Set the log level for the I2C bus
    io_expander.set_log_level(ESP_LOG_INFO);

    io_expander.set_interrupt_map_mask(0x00); // Disable all interrupts

    // 通过寄存器函数配置引脚输出示例
    // io_expander.set_pin_direction(2, GPIO_MODE_OUTPUT);
    // io_expander.set_pin_direction(3, GPIO_MODE_OUTPUT);
    // io_expander.set_pin_direction(4, GPIO_MODE_OUTPUT);
    // io_expander.set_high_impedance(2, M5_IO_LOW);
    // io_expander.set_high_impedance(3, M5_IO_LOW);
    // io_expander.set_high_impedance(4, M5_IO_LOW);
    // io_expander.write_pin_value(2, M5_IO_HIGH);
    // io_expander.write_pin_value(3, M5_IO_LOW);
    // io_expander.write_pin_value(4, M5_IO_HIGH);

    // 通过寄存器函数配置中断引脚示例
    // io_expander.set_pin_direction(M5_IO_NUM_1, GPIO_MODE_INPUT);  //配置引脚输出模式
    // io_expander.set_high_impedance(M5_IO_NUM_1, 1);               //配置引脚高阻态
    // io_expander.set_pull_enable(M5_IO_NUM_1, 1);                  //配置引脚上拉下拉使能
    // io_expander.set_pull_select(M5_IO_NUM_1, 0);                  //配置引脚上拉下拉状态
    // io_expander.set_input_default_state(M5_IO_NUM_1, 0);          //配置引脚默认状态（用于中断参考）
    // io_expander.set_interrupt_mask(M5_IO_NUM_1, 1);               //配置引脚中断使能

    // 仿照arduino的方式配置引脚输出
    io_expander.pinMode(M5_IO_NUM_2, OUTPUT);
    io_expander.digitalWrite(M5_IO_NUM_2, HIGH);
    io_expander.pinMode(M5_IO_NUM_3, OUTPUT);
    io_expander.digitalWrite(M5_IO_NUM_3, LOW);
    io_expander.pinMode(M5_IO_NUM_4, OUTPUT);
    io_expander.digitalWrite(M5_IO_NUM_4, HIGH);

    // 仿照arduino的方式配置引脚中断
    // 注意函数运行期间，会占用中断监测任务，中断不会响应，建议只做标志位处理或线程创建，不要运行耗时任务或死循环任务
    //  也支持 io_expander.pinMode(M5_IO_NUM_1, M5_IO_INPUT_PULLDOWN);
    io_expander.pinMode(M5_IO_NUM_0, INPUT_PULLUP);
    //  也支持 io_expander.attachInterrupt(M5_IO_NUM_1, test_m5_io_callback, M5_IO_RISING);
    io_expander.attachInterrupt(M5_IO_NUM_0, test_m5_io_callback, FALLING);

    const char *str = "this is m5_io_callback_with_arg!";
    io_expander.pinMode(M5_IO_NUM_1, INPUT_PULLDOWN);
    io_expander.attachInterruptArg(M5_IO_NUM_1, test_m5_io_callback_with_arg, (void *)str, RISING);

    while(1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// PI4IO 回调测试函数
void test_m5_io_callback(void)
{
    ESP_LOGI("test_m5_io_callback", "test_m5_io_callback ok!");
}

// PI4IO 回调测试函数 + 参数
void test_m5_io_callback_with_arg(void *arg)
{
    const char *str = (const char *)arg;
    ESP_LOGI("test_m5_io_callback_with_arg", "test_m5_io_callback_with_arg ok, arg: %s", str);
}
```


你可以通过examples文件夹下的示例文件进一步了解驱动库使用示例
