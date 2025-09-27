### **IO Expander  操作手册**

**I2C地址**：0x6F

**硬件/固件版本**：SW:2 / HW:1 (固件主版本: 2, 硬件修订: 1) 

**文档版本**：1.1 (2025-07-30)

### **一、芯片概述**

- **功能**：
  
  - 14路GPIO扩展
  
  - 其中4路可复用为12-bit ADC ，4路可复用为PWM，1路可复用为LED控制(RGB565)
  
  - 可获取内置温度传感器，可获取内部参考电压 
  
  - 有32 字节的RAM保存区域

- **接口**：I²C协议（支持100KHz/400kHz模式，默认100KHz，切换400KHz需要配置） 

- **关键特性**：
  
  - 可编程上下拉电阻、开漏输出、中断极性控制
  - 支持Neopixel RGB LED驱动（最大32灯）
  - 12-bit ADC（4通道）及温度传感器，内部参考电压获取
  - PWM
  - 32字节RTC掉电保持RAM

- ![](C:\Users\qweas\AppData\Roaming\marktext\images\2025-07-15-14-43-23-image.png)

- ### **二、引脚定义 (Pinout)**

| Pin# | Pin Name | 描述                              | 默认功能 | 特殊功能        | 备注                        |
| ---- | -------- | ------------------------------- | ---- | ----------- | ------------------------- |
| PB5  | IO_1     | GPIO 端口 1；可配置为输入/输出             | GPIO | -           | 与 IO6 中断互斥                |
| PB1  | IO_2     | GPIO 端口 2；可配置为输入/输出             | GPIO | ADC1        | 与 IO3 中断互斥；ADC 使能时中断关闭    |
| PA1  | IO_3     | GPIO 端口 3；可配置为输入/输出             | GPIO | -           | 与 IO2 中断互斥                |
| PA3  | IO_4     | GPIO 端口 4；可配置为输入/输出             | GPIO | ADC2        | ADC 使能时中断关闭               |
| PA4  | IO_5     | GPIO 端口 5；可配置为输入/输出             | GPIO | ADC3        | I²C 空闲休眠时中断关闭；ADC 使能时中断关闭 |
| PA5  | IO_6     | GPIO 端口 6；可配置为输入/输出             | GPIO | -           | 与 IO1 中断互斥                |
| PA6  | IO_7     | GPIO 端口 7；可配置为输入/输出             | GPIO | ADC4        | 与 IO12 中断互斥；ADC 使能时中断关闭   |
| PB0  | IO_8     | GPIO 端口 8；可配置为输入/输出             | GPIO | PWM2        | 与 IO9 中断互斥；PWM 使能时中断关闭    |
| PA0  | IO_9     | GPIO 端口 9；可配置为输入/输出             | GPIO | PWM1        | 与 IO8 中断互斥；PWM 使能时中断关闭    |
| PA7  | IO_10    | GPIO 端口 10；可配置为输入/输出            | GPIO | PWM4        | 与 IO14 中断互斥；PWM 使能时中断关闭   |
| PB2  | IO_11    | GPIO 端口 11；可配置为输入/输出            | GPIO | PWM3        | 与 IO13 中断互斥；PWM 使能时中断关闭   |
| PB6  | IO_12    | GPIO 端口 12；可配置为输入/输出            | GPIO |             | 与 IO7 中断互斥                |
| PA2  | IO_13    | GPIO 端口 13；可配置为输入/输出            | GPIO |             | 与 IO11 中断互斥               |
| PB7  | IO_14    | GPIO 端口 14；可配置为输入/输出            | GPIO | Neopixel 输出 | 与 IO10 中断互斥               |
| PC1  | INT      | 中断输出；外部中断触发时拉低，清除 `GPIO_IS` 后拉高 | 中断输出 | -           | 需外部上拉电阻                   |
| PB4  | I²C_SDA  | I²C 数据线                         | I²C  | -           | 需外部上拉电阻                   |
| PB3  | I²C_SCL  | I²C 时钟线                         | I²C  | -           | 需外部上拉电阻                   |

### **三、寄存器映射 (Register Map)**

| 地址 (Hex)  | 寄存器名称         | 位 7                           | 位 6            | 位 5       | 位 4   | 位 3       | 位 2       | 位 1       | 位 0       |
| --------- | ------------- | ----------------------------- | -------------- | --------- | ----- | --------- | --------- | --------- | --------- |
| 0x00      | UID_L         | UID[7:0]                      |                |           |       |           |           |           |           |
| 0x01      | UID_H         | UID[15:8]                     |                |           |       |           |           |           |           |
| 0x02      | REV           | HW3                           | HW2            | HW1       | HW0   | SW3       | SW2       | SW1       | SW0       |
| 0x03      | GPIO_M_L      | M7                            | M6             | M5        | M4    | M3        | M2        | M1        | M0        |
| 0x04      | GPIO_M_H      | RES                           | RES            | M13       | M12   | M11       | M10       | M9        | M8        |
| 0x05      | GPIO_O_L      | O7                            | O6             | O5        | O4    | O3        | O2        | O1        | O0        |
| 0x06      | GPIO_O_H      | RES                           | RES            | O13       | O12   | O11       | O10       | O9        | O8        |
| 0x07      | GPIO_I_L      | I7                            | I6             | I5        | I4    | I3        | I2        | I1        | I0        |
| 0x08      | GPIO_I_H      | RES                           | RES            | I13       | I12   | I11       | I10       | I9        | I8        |
| 0x09      | GPIO_PU_L     | PU7                           | PU6            | PU5       | PU4   | PU3       | PU2       | PU1       | PU0       |
| 0x0A      | GPIO_PU_H     | RES                           | RES            | PU13      | PU12  | PU11      | PU10      | PU9       | PU8       |
| 0x0B      | GPIO_PD_L     | PD7                           | PD6            | PD5       | PD4   | PD3       | PD2       | PD1       | PD0       |
| 0x0C      | GPIO_PD_H     | RES                           | RES            | PD13      | PD12  | PD11      | PD10      | PD9       | PD8       |
| 0x0D      | GPIO_IE_L     | IE7                           | IE6            | IE5       | IE4   | IE3       | IE2       | IE1       | IE0       |
| 0x0E      | GPIO_IE_H     | RES                           | RES            | IE13      | IE12  | IE11      | IE10      | IE9       | IE8       |
| 0x0F      | GPIO_IP_L     | IP7                           | IP6            | IP5       | IP4   | IP3       | IP2       | IP1       | IP0       |
| 0x10      | GPIO_IP_H     | RES                           | RES            | IP13      | IP12  | IP11      | IP10      | IP9       | IP8       |
| 0x11      | GPIO_IS_L     | IS7                           | IS6            | IS5       | IS4   | IS3       | IS2       | IS1       | IS0       |
| 0x12      | GPIO_IS_H     | RES                           | RES            | IS13      | IS12  | IS11      | IS10      | IS9       | IS8       |
| 0x13      | GPIO_DRV_L    | DRV7                          | DRV6           | DRV5      | DRV4  | DRV3      | DRV2      | DRV1      | DRV0      |
| 0x14      | GPIO_DRV_H    | RES                           | RES            | DRV13     | DRV12 | DRV11     | DRV10     | DRV9      | DRV8      |
| 0x15      | ADC_CTRL      | BUSY                          | START          | RES       | RES   | RES       | CH2       | CH1       | CH0       |
| 0x16      | ADC_D_L       | ADC Data[7:0]                 |                |           |       |           |           |           |           |
| 0x17      | ADC_D_H       | RES                           | RES            | RES       | RES   | ADC Data  | ADC Data  | ADC Data  | ADC Data  |
| 0x18      | TEMP_CTRL     | TBUSY                         | TSTART         | RES       | RES   | RES       | RES       | RES       | RES       |
| 0x19      | TEMP_D_L      | TEMP Data[7:0]                |                |           |       |           |           |           |           |
| 0x1A      | TEMP_D_H      | RES                           | RES            | RES       | RES   | TEMP Data | TEMP Data | TEMP Data | TEMP Data |
| 0x1B      | PWM1_L        | DUTY[7:0]                     |                |           |       |           |           |           |           |
| 0x1C      | PWM1_H        | EN                            | POL            | RES       | RES   | DUTY      | DUTY      | DUTY      | DUTY      |
| 0x1D      | PWM2_L        | DUTY[7:0]                     |                |           |       |           |           |           |           |
| 0x1E      | PWM2_H        | EN                            | POL            | RES       | RES   | DUTY      | DUTY      | DUTY      | DUTY      |
| 0x1F      | PWM3_L        | DUTY[7:0]                     |                |           |       |           |           |           |           |
| 0x20      | PWM3_H        | EN                            | POL            | RES       | RES   | DUTY      | DUTY      | DUTY      | DUTY      |
| 0x21      | PWM4_L        | DUTY[7:0]                     |                |           |       |           |           |           |           |
| 0x22      | PWM4_H        | EN                            | POL            | RES       | RES   | DUTY      | DUTY      | DUTY      | DUTY      |
| 0x23      | I²C_CFG       | RES                           | INTERNAL_PU/PD | WAKE_TYPE | SPD   | SLEEP3    | SLEEP2    | SLEEP1    | SLEEP0    |
| 0x24      | LED_CFG       | RES                           | REFRESH        | LED5      | LED4  | LED3      | LED2      | LED1      | LED0      |
| 0x25      | PWM_FREQ_L    | FREQ[7:0]                     |                |           |       |           |           |           |           |
| 0x26      | PWM_FREQ_H    | FREQ[15:8]                    |                |           |       |           |           |           |           |
| 0x27      | REF_VOLTAGE_L | REF_V[7:0]                    |                |           |       |           |           |           |           |
| 0x28      | REF_VOLTAGE_H | REF_V[15:8]                   |                |           |       |           |           |           |           |
| 0x29      | RESET         | RESET[7:0]                    |                |           |       |           |           |           |           |
| 0x30-0x6F | LED_RAM       | Neopixel RGB565 数据（32 灯 × 2B） |                |           |       |           |           |           |           |
| 0x70-0x8F | RTC_RAM       | 掉电保持存储区（32B）                  |                |           |       |           |           |           |           |

**注**：

- **RES**：保留位（读为 0，写忽略）。
- **I²C 地址**：`0x6F`（固定）。
- 这里Bit位的Bit0-Bit13，对应封装上IO1-IO14

### **四、关键寄存器详解**

##### **1. 设备信息**

- **UID[15:0]** (`0x00-0x01`)：16 位出厂唯一序号（只读）。
- **REV** (`0x02`)：
  - `HW3:0`：硬件修订版本号（4 位）。
  - `SW3:0`：固件主版本号（4 位）。

##### **2. GPIO 控制（14 个引脚）**

- **GPIO_M_x** (`0x03-0x04`): **方向寄存器**
  
  - `M[n]=0`：输入模式；`M[n]=1`：输出模式。
  - 默认为输入模式

- **GPIO_O_x** (`0x05-0x06`): **输出寄存器**
  
  - 输出电平（仅当 `M[n]=1` 时生效）。

- **GPIO_I_x** (`0x07-0x08`): **输入寄存器**（只读）
  
  - 实时输入电平。

- **GPIO_PU_x / GPIO_PD_x** (`0x09-0x0C`): **上下拉控制**
  
  | `PU[n]` | `PD[n]` | 模式   |
  | ------- | ------- | ---- |
  | 1       | 0       | 上拉使能 |
  | 0       | 1       | 下拉使能 |
  | 0       | 0       | 无上下拉 |
  | 1       | 1       | 无上下拉 |

- **GPIO_IE_x** (`0x0D-0x0E`): **中断使能**
  
  - `IE[n]=1`：使能引脚中断。

- **GPIO_IP_x** (`0x0F-0x10`): **中断极性**
  
  - `IP[n]=1`：上升沿/高电平触发；`IP[n]=0`：下降沿/低电平触发。

- **GPIO_IS_x** (`0x11-0x12`): **中断状态**
  
  - `IS[n]=1`：中断触发（写 0 清除）。

- **GPIO_DRV_x** (`0x13-0x14`): **驱动模式**
  
  - `DRV[n]=0`：推挽输出；`DRV[n]=1`：开漏输出。
  - 默认为开漏输出

> **优先级**：当引脚被 ADC/PWM/I²C/SWD/Neopixel 占用时，GPIO 输入输出模式（GPIO_M_x，GPIO_O_x）失效。

##### **3. ADC 控制** (`0x15-0x17`)

- **ADC_CTRL** (`0x15`):
  - `CH2:0`：通道选择（`000`=禁用；`001`=ADC1(IO2)；`010`=ADC2(IO4)；`011`=ADC3(IO5)；`100`=ADC4(IO7)）。
  - `START`：写 1 启动 12 位转换（硬件自动清零）。
  - `BUSY`：转换中=1；完成=0（只读）。
- **ADC_D_x** (`0x16-0x17`):
  - 12 位转换结果：`ADC_D_H[3:0]`（高 4 位） + `ADC_D_L[7:0]`（低 8 位）。

##### **4. 温度传感器** (`0x18-0x1A`)

- **TEMP_CTRL** (`0x18`):
  - `TSTART`：写 1 启动采样（硬件自动清零）。
  - `TBUSY`：采样中=1（只读）。
- **TEMP_D_x** (`0x19-0x1A`):
  - 12 位温度值：`TEMP_D_H[3:0]`（高 4 位） + `TEMP_D_L[7:0]`（低 8 位）。

##### **5. PWM 控制** (`0x1B-0x22`)

- **PWMx_L**：占空比低字节（`DUTY[7:0]`）。
- **PWMx_H**：
  - `DUTY[11:8]`：占空比高 4 位。
  - `EN`：使能位（1=启动 PWM）。
  - `POL`：极性（1=低电平有效；0=高电平有效）。
- **占空比范围**：`0x000`（0%）至 `0xFFF`（≈100%）。
- **频率控制**：由 `PWM_FREQ` (`0x25-0x26`) 设置（16 位值），单位是Hz。
- **PWM配置:**  其配置的高位与低位在两个字节，需要一次写两个字节，不然会出现在短时间内设置两次不同的duty或频率的情况。

##### **6. I²C 配置** (`0x23`)

- **INTERNAL PU/PD**:是否开启内部上下拉(0=开启;1=关闭)。
- **WAKE TYPE**:唤醒方式(0=下降沿;1=上升沿)。
- **SPD**：I²C 速度（0=100 kHz；1=400 kHz）。
- **SLEEP[3:0]**：I²C 总线空闲休眠时间（秒；0=不休眠）。

##### **7. Neopixel 控制**

- **LED_CFG** (`0x24`):
  - `LED5:0`：Neopixel 灯数量（0-32；0=全部关闭）。
  - `REFRESH`：写 1 立即刷新 LED_RAM 数据（自动清零）。
- **LED_RAM** (`0x30-0x6F`):
  - 64 字节 RGB565 数据（32 灯 × 2B），高位先行。

##### **8. RTC 存储** (`0x70-0x8F`)

- **RTC_RAM**：32 字节掉电保持存储区（可存计数器/校准参数等）。

##### **9. 参考电压** (`0x27-0x28`)

- **REF_VOLTAGE**：16 位参考电压值（用于 ADC/温度校准）。

##### **10. 恢复出厂设置** (`0x29`)

- **RESET**：用于恢复出厂配置，写入0x3A触发

#### **中断互斥关系**

| 引脚组         | 互斥关系说明   |
| ----------- | -------- |
| IO1 & IO6   | 不可同时使能中断 |
| IO2 & IO3   | 不可同时使能中断 |
| IO7 & IO12  | 不可同时使能中断 |
| IO8 & IO9   | 不可同时使能中断 |
| IO10 & IO14 | 不可同时使能中断 |
| IO11 & IO13 | 不可同时使能中断 |

> **注**：
> 
> - **当 ADC/PWM 等功能启用时，对应引脚的中断自动关闭。**
> - **当 I2C空闲休眠功能启用时，对应引脚（IO_5）的中断自动关闭。**

### **五、附加功能说明**

1. **ADC转换流程**：
   - 写入ADC_CTRL选择通道并启动（START=1）→ 等待BUSY=0 → 读取ADC_D_H/L。
2. **PWM输出** ：
   - 占空比 = `(DUTY[11:0] / 0xFFF) × 100%`，频率由PWM_FREQ寄存器设定。
3. **LED控制** ：
   - 设置LED数量 (LED_CFG[5:0]) → 写入LED_RAM (RGB565格式) → 触发REFRESH=1。
4. **I2C空闲休眠:**
   - 配置SLEEP[3:0]可以设置空闲休眠的时间，0=不休眠
     . 注意，PWM功能使能时，I2C空闲休眠失效

### **六、固件修改历史与文档修订历史**

| 版本          | 日期         | 变更描述                                                                                                        |
| ----------- | ---------- | ----------------------------------------------------------------------------------------------------------- |
| HW:1 / SW:1 | 2025-07-11 | 初始版本                                                                                                        |
| HW:1 / SW:2 | 2025-07-30 | 1. 协议添加配置I2C空闲休眠的唤醒方式与是否开启内部上下拉的开关<br>2. 修改PWM占空比更新事件标志位清除顺序，防止可能触发的duty配置问题<br>3. 添加恢复出厂设置的指令<br>4. 参考电压读取 |

| 版本  | 日期         | 变更描述                          |
| --- | ---------- | ----------------------------- |
| 1.0 | 2025-07-15 | 初始版本                          |
| 1.1 | 2025-07-30 | 1. 更新固件版本至V2<br>2. 修正部分技术细节描述 |
