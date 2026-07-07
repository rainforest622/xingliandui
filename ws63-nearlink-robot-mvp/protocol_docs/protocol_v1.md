# Robot Control Protocol v1

## 当前实板通道：COM5 AT 文本命令

WS63 默认 AT 子系统会占用 UART0，因此真实开发板 MVP 先采用 AT 命令接入，避免原始二进制包被 AT 层抢占。

串口参数：

- 端口：COM5
- 波特率：115200
- 数据位：8
- 校验：None
- 停止位：1

命令：

| 上位机键 | AT 命令 | 含义 |
|---|---|---|
| `F` | `AT+ROBOTF\r\n` | 前进 |
| `B` | `AT+ROBOTB\r\n` | 后退 |
| `L` | `AT+ROBOTL\r\n` | 左转 |
| `R` | `AT+ROBOTR\r\n` | 右转 |
| `S` | `AT+ROBOTS\r\n` | 停止 |
| `I` | `AT+ROBOTMI\r\n` | 初始化电机 I2C/PWM 层 |
| `O` | `AT+ROBOTOLED\r\n` | 初始化 OLED 并显示状态页 |
| `E` | `AT+ROBOTENV\r\n` | 读取 AHT20 温湿度环境数据 |
| `D` | `AT+ROBOTOBS\r\n` | 读取前向距离/避障状态 |
| `A` | `AT+ROBOTAVOID\r\n` | 启动板端智能避障自动行驶 |
| `T` | `AT+ROBOTST\r\n` | 查询机器人状态遥测 |

板端 ACK：

```text
+ROBOT:ACK,<seq>,<cmd>,<status>,<moving>
OK
```

字段：

- `seq`：板端递增序号。
- `cmd`：1 前进，2 后退，3 左转，4 右转，5 停止。
- `status`：0 正常，1 障碍停止，2 低电量，3 校验错误，4 非法命令，5 电机未启用/电机错误。
- `moving`：0 静止，1 运动。

电机初始化返回：

```text
+ROBOT:MOTOR,<ready>
OK
```

- `ready=1`：I2C/PWM 初始化成功，后续 F/B/L/R 可驱动电机。
- `ready=0`：初始化失败，后续运动命令会返回 `status=5`。

OLED 初始化返回：

```text
+ROBOT:OLED,<ready>
OK
```

- `ready=1`：I2C 已初始化，SSD1306 初始化流程已执行，并开始在 OLED 上显示状态页。
- `ready=0`：I2C 初始化失败。
- OLED 完整刷新会显著增加命令 RTT；当前实测启用 OLED 后普通命令约 110 ms，首次 OLED 初始化约 330 ms。

环境数据返回：

```text
+ROBOT:ENV,<ok>,<temperature_deci_c>,<humidity_deci_percent>
OK
```

- `ok=1`：AHT20 采样和原始数据换算成功。
- `ok=0`：AHT20 未就绪、I2C 失败或采样忙，温湿度字段无效。
- `temperature_deci_c`：温度，单位 0.1 摄氏度，例如 `253` 表示 25.3 C。
- `humidity_deci_percent`：相对湿度，单位 0.1%，例如 `582` 表示 58.2%RH。
- 当前仓库已补齐可移植 AHT20 原始帧换算和 C 状态包编码；真机 AT 层还需把 `AT+ROBOTENV` 接到 LiteOS I2C 读 AHT20 的回调。

避障/距离返回：

```text
+ROBOT:OBS,<enabled>,<valid>,<blocked>,<distance_mm>,<threshold_mm>[,<reason>]
OK
```

- `enabled=1`：板端避障传感器驱动已启用。
- `valid=1`：本次距离采样有效。
- `blocked=1`：距离低于阈值，前进命令应被拦截或运行中停车。
- `distance_mm`：前向距离，单位毫米；无效时为 `0`。
- `threshold_mm`：当前避障阈值，单位毫米。
- `reason`：可选诊断字段；旧固件没有该字段。新固件定义：`0=ok`，`1=not_ready`，`2=echo_idle_high`，`3=no_echo_rise`，`4=no_echo_fall`，`5=invalid_pulse`。
- 当前板端已启用真实 HC-SR04 GPIO 驱动：TRIG=`GPIO_00`，ECHO=`GPIO_01`，阈值 `250 mm`。
- 当前版本已移除无超声波模拟避障命令；`D` 和 `A` 均读取真实 HC-SR04 外设。

智能避障返回：

```text
+ROBOT:AVOID,<active>,<phase>,<status>,<enabled>,<valid>,<blocked>,<distance_mm>,<threshold_mm>[,<reason>]
OK
```

- `active=1`：自动避障模式已启动；`S` / `AT+ROBOTS` 会立即停车并退出。
- `phase`：0 空闲，1 前进，2 后退，3 右转，4 传感器异常停车。
- `status`：同 ACK 状态码；`0` 正常，`1` 表示传感器/障碍导致未进入自动行驶，`5` 表示电机未初始化或电机错误。
- `reason`：可选诊断字段，含义同 `+ROBOT:OBS`。
- 自动策略：正常前进；距离低于阈值时停车、短暂后退、右转，再重新前进。
- 传感器读数无效时 fail-safe 停车并退出自动模式。

状态查询返回：

```text
+ROBOT:STATE,<uptime_ms>,<ready>,<moving>,<last_cmd>,<last_seq>,<age_ms>
OK
```

- `uptime_ms`：板端启动后的毫秒数，32 位循环。
- `ready`：电机层是否初始化成功。
- `moving`：当前是否处于运动态。
- `last_cmd`：最近一次运动/停止命令，1..5；未收到控制命令时为 0。
- `last_seq`：最近一次控制命令序号。
- `age_ms`：最近一次控制命令距当前的毫秒数；未收到控制命令时板端返回 `0xFFFFFFFF`，上位机 CSV 记录为空值。

当前安全固件为 `motor=SKIPPED`：

- `AT+ROBOTS` 预期 `status=0,moving=0`
- `AT+ROBOTF/B/L/R` 预期 `status=5,moving=0`

## 保留的二进制协议

二进制协议保留在上位机、模拟器和 WS63 代码中，后续如切换到非 AT UART、SLE 或 TCP 透明传输，可直接复用。

SLE/星闪第一版控制 profile 见 `D:\aaa嵌入式比赛\protocol_docs\sle_control_profile.md`。SLE 初期建议先使用 ASCII 兼容模式：central 写入 `F/B/L/R/S/I/O/E/D/A/T`，板端 notify 本文定义的 `+ROBOT:*` 响应；待连接和安全动作稳定后，再切换到下方二进制控制包做 1000 包 RTT 基线测试。

所有多字节整数使用小端序。校验值为从包头到校验字段前一字节的逐字节 XOR。

### 控制包，6 字节

| 偏移 | 字段 | 含义 |
|---:|---|---|
| 0 | `0xAA` | 包头 |
| 1 | `cmd` | 1 前进，2 后退，3 左转，4 右转，5 停止 |
| 2 | `speed_l` | 左轮速度百分比，0..100 |
| 3 | `speed_r` | 右轮速度百分比，0..100 |
| 4 | `seq` | 0..255 循环序号 |
| 5 | `checksum` | 前 5 字节 XOR |

### ACK，4 字节

| 偏移 | 字段 | 含义 |
|---:|---|---|
| 0 | `0x55` | 包头 |
| 1 | `seq` | 对应控制包序号 |
| 2 | `status` | 同 AT ACK 状态码 |
| 3 | `checksum` | 前 3 字节 XOR |

### 状态包，8 字节

| 偏移 | 字段 | 含义 |
|---:|---|---|
| 0 | `0x5A` | 包头 |
| 1 | `seq` | 状态序号 |
| 2 | `battery` | 电量百分比，0..100 |
| 3 | `humidity` | 相对湿度百分比，0..100 |
| 4..5 | `distance_mm` | 超声波距离，uint16 |
| 6 | `motor_state` | 0 停止，1 前进，2 后退，3 左转，4 右转 |
| 7 | `checksum` | 前 7 字节 XOR |

## 安全时序

- 上电默认停止。
- 控制看门狗：500 ms；超时后无条件停车。
- 智能避障是板端自动模式，启动后不依赖上位机持续发送 `F`，但 `S` 仍是最高优先级停车命令。
- 障碍停止不是通信错误，仍返回对应 ACK。
- 实车动作测试前必须让车轮离地。
