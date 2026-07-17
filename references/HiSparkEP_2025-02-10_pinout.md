# HiSparkEP_2025-02-10 模块功能与引脚速查

来源：`HiSparkEP_2025-02-10.pdf`，原理图 `Hi3863_SmartCar_1218`，版本 `V1.0`，共 4 页。

说明：
- 本表按原理图网络名整理，适合后续写代码、接线和查 I2C 地址时使用。
- `VCC` 为板上 3.3V 电源；`+5V` 主要来自 Type-C/外部 5V；`VBAT` 为电池端；`VBATS` 为经保险丝后的电池检测端；`VBUS` 为电源开关后的输入电源。
- 原理图里 I2C 地址写法类似 `0b0100011_ / 0x46`。这里把 `0x46` 记作 PDF 标注的 8-bit 地址，同时给出常用驱动 API 使用的 7-bit 地址 `0x23`。

## 总线与地址速查

主 I2C：`HI_SCL/HI_SDA` 从 WS63E 引出，经 `TCA9803DGKR` 转为板载 `SCL/SDA` 总线。大部分 I2C 外设挂在 `SCL/SDA`。

| 模块 | 器件 | 功能 | SCL | SDA | 中断/辅助脚 | PDF 地址 | 常用 7-bit 地址 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Expanded IO | STM8S103F3P6TR U11 | 扩展 GPIO/中断汇聚 | SCL | SDA | INT_IO 到 WS63E GPIO_12 | 0x28 | 0x14 |
| PWM Generater | STM8S103F3P6TR U13 | PWM 输出/电机 PWM | SCL | SDA | SWIM2/NRST2 调试 | 0x2A | 0x15 |
| Light Sensor | BH1750FVI-TR U14 | 光照强度 | SCL | SDA | ADDR 接 GND | 0x46 | 0x23 |
| RTC | INS5699S U15 | 实时时钟 | SCL | SDA | INT_RTC | 0x64 | 0x32 |
| Battery Monitor | CW2015CHBD U17 | 电量/电压监测 | SCL | SDA | INT_CW2015 | 0xC4 | 0x62 |
| ADC | SGM58031XMS10G/TR U18 | 4 路 ADC | SCL | SDA | INT_ADC/ALERT | 0x90 | 0x48 |
| Temp & Humidity | AHT20 U19 | 温湿度 | SCL | SDA | 无 | 0x70 | 0x38 |
| BMX055 AC | BMX055 U21 | 加速度计 | SCL | SDA | AC_INT | 0x18 | 0x0C |
| BMX055 GY | BMX055 U21 | 陀螺仪 | SCL | SDA | GY_INT | 0x68 | 0x34 |
| BMX055 MA | BMX055 U21 | 磁力计 | SCL | SDA | 板上固定配置 | 0x10 | 0x08 |
| OLED | OLED 模块 U16 | I2C 显示屏接口 | SCL | SDA | PDF 未标注地址 | 未标注 | 建议扫描 |

## WS63E_1 主控引脚映射

| WS63E 引脚 | 芯片脚名 | 原理图网络 | 连接/用途 |
| --- | --- | --- | --- |
| 1 | GND | GND | 地 |
| 2 | RF0 | RF0 | 射频匹配/天线网络 |
| 3 | GND | GND | 地 |
| 4 | NC | NC | 未连接 |
| 5 | NC | NC | 未连接 |
| 6 | GPIO_01 | ECHO | 超声波回波 CN1-3 |
| 7 | GPIO_05 | LEDS | WS2812 串行灯 LED4 DIN |
| 8 | NC | NC | 未连接 |
| 9 | VCC | VCC | 3.3V |
| 10 | GPIO_04 | BEEP | 蜂鸣器控制，驱动 Q4 |
| 11 | GPIO_00 | TRIG | 超声波触发 CN1-2 |
| 12 | EN | EN | 使能/复位/Boot 电路 |
| 13 | GPIO_02 | SERVO | 舵机 PWM 信号 CN2-2 |
| 14 | GPIO_09 | LED_R | RGB LED 红色控制 |
| 15 | GPIO_07 | LED_B | RGB LED 蓝色控制 |
| 16 | GPIO_11 | LED_G | RGB LED 绿色控制 |
| 17 | GPIO_10 | GPIO_10 | 预留网络 |
| 18 | UART_TX1 | HI_SDA | 主 I2C SDA 复用脚 |
| 19 | UART_RX1 | HI_SCL | 主 I2C SCL 复用脚 |
| 20 | GND | GND | 地 |
| 21 | NC | NC | 未连接 |
| 22 | VCC | VCC | 3.3V |
| 23 | NC | NC | 未连接 |
| 24 | GPIO_06 | MT_C1A | 电机 1 编码/反馈 A，经 TXU0104 |
| 25 | GPIO_03 | MT_C1B | 电机 1 编码/反馈 B，经 TXU0104 |
| 26 | NC | NC | 未连接 |
| 27 | GPIO_13 | MT_C2A | 电机 2 编码/反馈 A，经 TXU0104 |
| 28 | GPIO_14 | MT_C2B | 电机 2 编码/反馈 B，经 TXU0104 |
| 29 | GND | GND | 地 |
| 30 | NC | NC | 未连接 |
| 31 | GND | GND | 地 |
| 32 | NC | NC | 未连接 |
| 33 | NC | NC | 未连接 |
| 34 | NC | NC | 未连接 |
| 35 | NC | NC | 未连接 |
| 36 | GND | GND | 地 |
| 37 | UART_RX0 | UART_RX0 | CH340E TXD 到主控 RX |
| 38 | UART_TX0 | UART_TX0 | CH340E RXD 到主控 TX |
| 39 | GPIO_12 | INT_IO | 扩展 IO 中断 |
| 40 | GPIO_08 | GPIO_08 | 电位器跳线 H2-1 |
| 41 | NC | NC | 未连接 |
| 42 | GND | GND | 地 |
| 43 | RF1 | RF1 | 射频匹配/天线网络 |
| 44 | GND | GND | 地 |

## H5 扩展排针

H5 是 2x18 输出/扩展排针，奇数在左、偶数在右。注意底部 `33/34` 是 GND，`35/36` 是 VCC。

| 引脚 | 网络 | 用途 |
| --- | --- | --- |
| 1 | SDA | I2C SDA |
| 2 | SCL | I2C SCL |
| 3 | AIN1 | ADC 输入 1 |
| 4 | AIN2 | ADC 输入 2 |
| 5 | AIN0 | ADC 输入 0 |
| 6 | AIN3 | ADC 输入 3 |
| 7 | PWM_IO1 | PWM 发生器 U13 扩展脚 |
| 8 | PWM2_2 | PWM 输出 |
| 9 | PWM_IO2 | PWM 发生器 U13 扩展脚 |
| 10 | PWM2_3 | PWM 输出 |
| 11 | PWM_IO3 | PWM 发生器 U13 扩展脚 |
| 12 | PWM1_2 | PWM 输出，电机 2 L9110S 输入 |
| 13 | PWM_IO4 | PWM 发生器 U13 扩展脚，也接 TXU0104 OE 跳线 H6 |
| 14 | PWM1_1 | PWM 输出，电机 2 L9110S 输入 |
| 15 | PWM_IO5 | PWM 发生器 U13 扩展脚 |
| 16 | PWM2_1 | PWM 输出 |
| 17 | PWM_IO6 | PWM 发生器 U13 扩展脚 |
| 18 | PWM1_4 | PWM 输出，电机 1 L9110S 输入 |
| 19 | IO_B06 | 扩展 IO U11 |
| 20 | PWM1_3 | PWM 输出，电机 1 L9110S 输入 |
| 21 | IO_B05 | 扩展 IO U11 |
| 22 | IO_A05 | 扩展 IO U11 / GY_INT |
| 23 | IO_B04 | 扩展 IO U11 / SW7 按键，低有效 |
| 24 | IO_A04 | 扩展 IO U11 / AC_INT |
| 25 | IO_B03 | 扩展 IO U11 / SW6 按键，低有效 |
| 26 | IO_A03 | 扩展 IO U11 / INT_ADC |
| 27 | IO_B02 | 扩展 IO U11 / 编码器按键，低有效 |
| 28 | IO_A02 | 扩展 IO U11 / INT_RTC |
| 29 | IO_B01 | 扩展 IO U11 / 编码器 B 相 |
| 30 | IO_A01 | 扩展 IO U11 / INT_CW2015 |
| 31 | IO_B00 | 扩展 IO U11 / 编码器 A 相 |
| 32 | IO_A00 | 扩展 IO U11 / SWIM1 |
| 33 | GND | 地 |
| 34 | GND | 地 |
| 35 | VCC | 3.3V |
| 36 | VCC | 3.3V |

## 模块清单

### Type-C USB

功能：USB 供电和 USB2.0 数据输入，给 CH340E 串口桥和电源系统使用。

| 连接 | 网络/说明 |
| --- | --- |
| USB1 VBUS | `+5V` |
| USB1 GND/SHELL | GND |
| USB1 DP1/DP2 | `USB_DP` |
| USB1 DN1/DN2 | `USB_DN` |
| USB1 CC1/CC2 | 各经 5.1k 下拉到 GND |
| USB1 SBU1/SBU2 | NC |
| LED13 | `+5V` 电源指示 |

### Power 与 Power Switch

功能：从 `VBAT` 或 `+5V` 选择/OR 后生成 `VBUS`，再由多颗 ME6206A33XG 产生 `VCC`。

| 模块 | 引脚/网络 | 说明 |
| --- | --- | --- |
| D3 SS34 | `VBAT` 到开关公共端 | 电池输入防反/OR |
| D4 SS34 | `+5V` 到开关公共端 | USB/外部 5V 输入防反/OR |
| SW2 | 2 为公共端，3 接 `VBUS`，1 NC | 电源开关 |
| U2-U9 ME6206A33XG | 1 VSS, 2 VOUT, 3 VIN | VIN=`VBUS`，VOUT=`VCC`，VSS=GND |
| LED11 | `VBUS` 指示 | 1k 串联 |
| LED12 | `VCC` 指示 | 1k 串联 |

### Boot/EN

功能：WS63E 使能/复位控制。

| 连接 | 网络/说明 |
| --- | --- |
| WS63E pin12 | `EN` |
| SW1 | 按键复位/拉动 EN |
| H1 | EN 跳线选择 |
| R10 | 20k 上拉到 VCC |
| R13 | 47k 到 GND |

### Potentiometer

功能：板载电位器和 ADC/主控 GPIO 跳线。

| 接口 | 引脚 | 网络 | 说明 |
| --- | --- | --- | --- |
| R14 | 两端 | VCC/GND | 2k 电位器 |
| R14 | 滑动端 | POT | 模拟电压输出 |
| H2 | 1 | GPIO_08 | WS63E GPIO_08 |
| H2 | 2 | POT | 电位器输出 |
| H2 | 3 | AIN0 | ADC 通道 0 |

### USB to UART

功能：Type-C USB 转 UART0，用于调试/下载。

| CH340E U10 引脚 | 网络 | 说明 |
| --- | --- | --- |
| 1 UD+ | USB_DP | USB D+ |
| 2 UD- | USB_DN | USB D- |
| 3 GND | GND | 地 |
| 4 RTS# | NC | 未用 |
| 5 CTS# | NC | 未用 |
| 6 TNOW | NC | 未用 |
| 7 VCC | VCC | 3.3V |
| 8 TXD | UART_RX0 | 到 WS63E UART_RX0 |
| 9 RXD | UART_TX0 | 到 WS63E UART_TX0 |
| 10 V3 | VCC | 3.3V |

### I2C Relay

功能：把 WS63E 侧 `HI_SCL/HI_SDA` 转接到板载 `SCL/SDA`。

| TCA9803DGKR U12 引脚 | 网络 |
| --- | --- |
| 1 VCCA | VCC |
| 2 SCLA | HI_SCL |
| 3 SDAA | HI_SDA |
| 4 GND | GND |
| 5 EN | VCC，经 R20 2.2k |
| 6 SDAB | SDA |
| 7 SCLB | SCL |
| 8 VCCB | VCC |

### Expanded IO

功能：I2C GPIO 扩展和传感器中断汇聚，I2C 地址见总线表。

| STM8S103F3P6TR U11 引脚 | 网络 | 用途 |
| --- | --- | --- |
| 1 PD4/UART1_CK | IO_A03 / INT_ADC | ADC 中断输入 |
| 2 PD5/UART1_TX | IO_A04 / AC_INT | BMX055 加速度中断 |
| 3 PD6/UART1_RX | IO_A05 / GY_INT | BMX055 陀螺仪中断 |
| 4 NRST | NRST1 | 调试/复位 |
| 5 PA1/OSCIN | IO_B00 | 编码器 A 相 |
| 6 PA2/OSCOUT | IO_B01 | 编码器 B 相 |
| 7 VSS | GND | 地 |
| 8 VCAP | VCAP | 去耦 |
| 9 VDD | VCC | 3.3V |
| 10 PA3/SPI_NSS | INT_IO | 到 WS63E GPIO_12 |
| 11 I2C_SDA/PB5 | SDA | I2C SDA |
| 12 I2C_SCL/PB4 | SCL | I2C SCL |
| 13 PC3 | IO_B02 | 编码器按键 |
| 14 PC4 | IO_B03 | SW6 按键 |
| 15 SPI_SCK/PC5 | IO_B04 | SW7 按键 |
| 16 SPI_MOSI/PC6 | IO_B05 | 扩展 IO |
| 17 SPI_MISO/PC7 | IO_B06 | 扩展 IO |
| 18 SWIM/PD1 | IO_A00 / SWIM1 | 调试/扩展 |
| 19 PD2 | IO_A01 / INT_CW2015 | 电量计中断 |
| 20 PD3 | IO_A02 / INT_RTC | RTC 中断 |

H3 调试口：1=`VCC`，2=`SWIM1`，3=`NRST1`，4=`GND`。

### PWM Generater

功能：I2C PWM 控制器，输出给 H5 和电机驱动。I2C 地址见总线表。

| STM8S103F3P6TR U13 引脚 | 网络 | 用途 |
| --- | --- | --- |
| 1 PD4/UART1_CK | PWM_IO4 | 扩展 PWM IO |
| 2 PD5/UART1_TX | PWM_IO5 | 扩展 PWM IO |
| 3 PD6/UART1_RX | PWM_IO6 | 扩展 PWM IO |
| 4 NRST | NRST2 | 调试/复位 |
| 5 PA1/OSCIN | PWM_IO1 | 扩展 PWM IO |
| 6 PA2/OSCOUT | PWM_IO2 | 扩展 PWM IO |
| 7 VSS | GND | 地 |
| 8 VCAP | VCAP | 去耦 |
| 9 VDD | VCC | 3.3V |
| 10 PA3/SPI_NSS | PWM_IO3 | 扩展 PWM IO |
| 11 I2C_SDA/PB5 | SDA | I2C SDA |
| 12 I2C_SCL/PB4 | SCL | I2C SCL |
| 13 PC3 | PWM1_3 | 电机 1 输入 |
| 14 PC4 | PWM1_4 | 电机 1 输入 |
| 15 SPI_SCK/PC5 | PWM2_1 | 扩展 PWM 输出 |
| 16 SPI_MOSI/PC6 | PWM1_1 | 电机 2 输入 |
| 17 SPI_MISO/PC7 | PWM1_2 | 电机 2 输入 |
| 18 SWIM/PD1 | SWIM2 | 调试 |
| 19 PD2 | PWM2_3 | 扩展 PWM 输出 |
| 20 PD3 | PWM2_2 | 扩展 PWM 输出 |

H4 调试口：1=`VCC`，2=`SWIM2`，3=`NRST2`，4=`GND`。

### RGB LED

功能：WS63E 直接控制三色 LED，三路均通过 S8050 三极管驱动，低侧开关。

| 网络 | WS63E | 驱动 | LED |
| --- | --- | --- | --- |
| LED_G | GPIO_11 | Q1 | LED1 |
| LED_B | GPIO_07 | Q2 | LED2 |
| LED_R | GPIO_09 | Q3 | LED3 |

### Series LEDs

功能：6 颗 WS2812C 串联灯。

| 项目 | 网络/引脚 |
| --- | --- |
| 数据输入 | `LEDS`，来自 WS63E GPIO_05，经 R27 100R 到 LED4 DIN |
| 串联顺序 | LED4 -> LED5 -> LED6 -> LED7 -> LED8 -> LED9 |
| 每颗 WS2812C | 1 VDD=`VCC`，2 DOUT，3 VSS=`GND`，4 DIN |
| 末端 | LED9 DOUT 经 R28 100R 后悬空 |

### Light Sensor

功能：BH1750FVI-TR 光照传感器。

| U14 引脚 | 网络 |
| --- | --- |
| 1 VCC | VCC |
| 2 ADDR | GND |
| 3 GND | GND |
| 4 SDA | SDA |
| 5 DVI | 去耦/接地网络 |
| 6 SCL | SCL |
| 7 EP | 通过 R29 10k 到 VCC |

### RTC

功能：INS5699S 实时时钟，带 CR1220 后备电池。

| U15 引脚 | 网络 | 说明 |
| --- | --- | --- |
| 1 FOE | VCC | 板上固定 |
| 2 VDD | VCC | 主供电 |
| 3 VBAT | CR1220-2 | 后备电池 |
| 4 FOUT | NC | 未用 |
| 5 SCL | SCL | I2C 时钟 |
| 6 T1 | NC | 未用 |
| 7 SDA | SDA | I2C 数据 |
| 8 T2 | NC | 未用 |
| 9 GND | GND | 地 |
| 10 INT# | INT_RTC | 10k 上拉到 VCC |

### Extended Device

功能：外接超声波、OLED、舵机。

| 模块 | 接口/引脚 | 网络 | 说明 |
| --- | --- | --- | --- |
| Ultrasonic | CN1-1 | VCC | 电源 |
| Ultrasonic | CN1-2 | TRIG | WS63E GPIO_00 |
| Ultrasonic | CN1-3 | ECHO | WS63E GPIO_01 |
| Ultrasonic | CN1-4 | GND | 地 |
| OLED | U16-1 | GND | 地 |
| OLED | U16-2 | VCC | 电源 |
| OLED | U16-3 | SCL | I2C 时钟 |
| OLED | U16-4 | SDA | I2C 数据 |
| Servo SG90 | CN2-1 | VCC | 电源 |
| Servo SG90 | CN2-2 | SERVO | WS63E GPIO_02 |
| Servo SG90 | CN2-3 | GND | 地 |
| Servo SG90 | CN2-4/5 | GND | 连接器附加地/固定脚 |

### Battery Monitor

功能：CW2015CHBD 电池电量计。

| U17 引脚 | 网络 | 说明 |
| --- | --- | --- |
| 1 CTG | GND/板上固定 | 配置脚 |
| 2 CELL | VBATS 采样 | 经 R31/C54 |
| 3 VDD | VCC | 经 R32/C55 |
| 4 GND | GND | 地 |
| 5 ALRT# | INT_CW2015 | 10k 上拉到 VCC |
| 6 QSTRT | GND/板上固定 | 快速启动配置 |
| 7 SCL | SCL | I2C 时钟 |
| 8 SDA | SDA | I2C 数据 |
| 9 EP | GND | 裸露焊盘 |

### ADC

功能：SGM58031 4 路 I2C ADC。

| U18 引脚 | 网络 | 说明 |
| --- | --- | --- |
| 1 ADDR | GND/板上固定 | 地址配置 |
| 2 ALERT/RDY | INT_ADC | 10k 上拉到 VCC |
| 3 GND | GND | 地 |
| 4 AIN0 | AIN0 | ADC 通道 0 |
| 5 AIN1 | AIN1 | ADC 通道 1 |
| 6 AIN2 | AIN2 | ADC 通道 2 |
| 7 AIN3/VREFIN | AIN3 | ADC 通道 3 |
| 8 VDD | VCC | 3.3V |
| 9 SDA | SDA | I2C 数据 |
| 10 SCL | SCL | I2C 时钟 |

### Buzzer

功能：有源/无源蜂鸣器 MLT-7525，由 WS63E GPIO 控制。

| 连接 | 网络/说明 |
| --- | --- |
| BEEP | WS63E GPIO_04，经 R35 1k 到 Q4 基极 |
| Q4 | S8050 低侧开关 |
| BUZZER1 | VCC 供电，Q4 拉低驱动 |

### Temperature & Wet Sensor

功能：AHT20 温湿度传感器。

| U19 引脚 | 网络 |
| --- | --- |
| 1 NC | NC |
| 2 VDD | VCC |
| 3 SCL | SCL |
| 4 SDA | SDA |
| 5 GND | GND |
| 6 NC | NC |

### Battery Charge

功能：TP4054 单节锂电充电，输入 `+5V`，输出到 `VBAT/VBATS`。

| U20 引脚 | 网络 | 说明 |
| --- | --- | --- |
| 1 STAT | LED10 | 充电状态指示 |
| 2 VSS | GND | 地 |
| 3 VBAT | VBAT | 电池端，接 C65/F1/BT1 |
| 4 VDD | +5V | 充电输入 |
| 5 PROG | R38 3.3k 到 GND | 充电电流设置 |
| F1 | VBAT 到 VBATS | 自恢复保险丝 BSMD1812-200-30V |
| BT1 | VBATS/GND | 18650 电池座 |

### BMX055

功能：9 轴 IMU，I2C 总线，含加速度计/陀螺仪/磁力计三个地址。

| 连接 | 网络 | 说明 |
| --- | --- | --- |
| SCL | SCL | I2C 时钟 |
| SDA | SDA | I2C 数据 |
| VDD/VDDIO | VCC | 3.3V |
| GND/GNDA/GNDIO | GND | 地 |
| INT1 | AC_INT | 加速度中断，进入 Expanded IO 的 IO_A04 |
| INT3 | GY_INT | 陀螺仪中断，进入 Expanded IO 的 IO_A05 |
| 其他地址/模式脚 | 板上固定或 NC | 按 PDF 标注地址使用 |

### TXU0104

功能：4 路信号缓冲/电平转换，用于电机编码/反馈信号到 WS63E。

| U24 引脚 | 网络 | 说明 |
| --- | --- | --- |
| 1 VCCA | VCC | A 侧电源 |
| 2 A1 | MOTOR_C1A | 电机 1 编码/反馈 A |
| 3 A2 | MOTOR_C1B | 电机 1 编码/反馈 B |
| 4 A3 | MOTOR_C2A | 电机 2 编码/反馈 A |
| 5 A4 | MOTOR_C2B | 电机 2 编码/反馈 B |
| 6 NC | NC | 未用 |
| 7 GND | GND | 地 |
| 8 OE | H6-1 | 输出使能 |
| 9 NC | NC | 未用 |
| 10 B4Y | MT_C2B | 到 WS63E GPIO_14 |
| 11 B3Y | MT_C2A | 到 WS63E GPIO_13 |
| 12 B2Y | MT_C1B | 到 WS63E GPIO_03 |
| 13 B1Y | MT_C1A | 到 WS63E GPIO_06 |
| 14 VCCB | VCC | B 侧电源 |

H6：1=`OE`，2=`PWM_IO4`。短接后可用 `PWM_IO4` 控制 TXU0104 使能。

### Motor

功能：双路 L9110S 电机驱动，两个 6 针电机接口同时带编码/反馈信号。

| 电机 | 驱动器 | 输入 PWM | 反馈到主控 | 接口 |
| --- | --- | --- | --- | --- |
| Motor 1 | U22 L9110S | IA=`PWM1_4`，IB=`PWM1_3` | MOTOR_C1A/B -> TXU0104 -> MT_C1A/B | CN3 |
| Motor 2 | U23 L9110S | IA=`PWM1_2`，IB=`PWM1_1` | MOTOR_C2A/B -> TXU0104 -> MT_C2A/B | CN4 |

CN3/CN4 6 针接口顺序相同：

| 引脚 | CN3 Motor 1 | CN4 Motor 2 | 说明 |
| --- | --- | --- | --- |
| 1 | L9110S OB | L9110S OB | 电机端 |
| 2 | VCC | VCC | 编码器/接口电源 |
| 3 | MOTOR_C1A | MOTOR_C2A | 编码/反馈 A |
| 4 | MOTOR_C1B | MOTOR_C2B | 编码/反馈 B |
| 5 | GND | GND | 地 |
| 6 | L9110S OA | L9110S OA | 电机端 |
| 7/8 | GND/固定脚 | GND/固定脚 | 连接器固定/屏蔽地 |

### Switch

功能：EC11L1525G01 旋转编码器/按压开关，全部通过 Expanded IO 读取，低电平有效。

| SW5 引脚/功能 | 网络 | 说明 |
| --- | --- | --- |
| A 相 | IO_B00 | 10k 上拉到 VCC，动作接地 |
| B 相 | IO_B01 | 10k 上拉到 VCC，动作接地 |
| 按键/开关 | IO_B02 | 10k 上拉到 VCC，按下接地 |
| 公共端 | GND | 地 |

### Buttons

功能：两个独立按键，全部通过 Expanded IO 读取，低电平有效。

| 按键 | 网络 | 说明 |
| --- | --- | --- |
| SW6 | IO_B03 | R42 10k 上拉到 VCC，按下接 GND |
| SW7 | IO_B04 | R43 10k 上拉到 VCC，按下接 GND |

## 快速调用提示

| 功能 | 直接使用的网络/地址 |
| --- | --- |
| 超声波 | TRIG=`GPIO_00`，ECHO=`GPIO_01` |
| 舵机 | SERVO=`GPIO_02` |
| 蜂鸣器 | BEEP=`GPIO_04` |
| WS2812 灯带 | LEDS=`GPIO_05`，共 6 颗 |
| RGB LED | R=`GPIO_09`，B=`GPIO_07`，G=`GPIO_11` |
| 主 I2C | HI_SCL=`UART_RX1`，HI_SDA=`UART_TX1`，转接后为 `SCL/SDA` |
| 串口调试 | UART_TX0/UART_RX0 通过 CH340E 到 USB |
| 电机 PWM | Motor1=`PWM1_3/PWM1_4`，Motor2=`PWM1_1/PWM1_2` |
| 电机反馈 | MT_C1A/B=`GPIO_06/GPIO_03`，MT_C2A/B=`GPIO_13/GPIO_14` |
| ADC | `0x48`，通道 `AIN0-AIN3` |
| 温湿度 | AHT20 `0x38` |
| 光照 | BH1750 `0x23` |
| RTC | INS5699S `0x32` |
| 电量计 | CW2015 `0x62` |
| 扩展 IO | STM8 U11 `0x14`，按钮/编码器/中断都在这里 |
| PWM 控制器 | STM8 U13 `0x15` |
