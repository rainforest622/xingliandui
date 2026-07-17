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
| `M` | `AT+ROBOTMON\r\n` | 读取工业巡检快照：状态、温湿度、距离和报警位 |
| - | `AT+ROBOTBEEP\r\n` | 蜂鸣器三短声自检 |
| `A` | `AT+ROBOTAVOID\r\n` | 启动板端智能避障自动行驶 |
| `P` | `AT+ROBOTPATROL\r\n` | 启动 40 平米方形路线自主巡检 |
| `G` | `AT+ROBOTPATROLST\r\n` | 查询当前路线巡检阶段，不会重启巡检 |
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
- `reason`：可选诊断字段；旧固件没有该字段。新固件定义：`0=ok`，`1=not_ready`，`2=echo_idle_high`，`3=no_echo_rise`，`4=no_echo_fall`，`5=invalid_pulse`，`6=out_of_range`。
- HC-SR04 在空旷区域可能无回波，启动瞬间也可能出现 ECHO 残留高电平；本版本将这类情况按 `valid=1, blocked=0, distance_mm=4000, reason=6` 处理，避免空地巡检时误报“距离无效”并退出路线。
- 当前板端已启用真实 HC-SR04 GPIO 驱动：TRIG=`GPIO_00`，ECHO=`GPIO_01`，阈值 `250 mm`。
- 当前版本已移除无超声波模拟避障命令；`D` 和 `A` 均读取真实 HC-SR04 外设。

工业巡检快照返回：

```text
+ROBOT:MON,<uptime_ms>,<ready>,<moving>,<env_valid>,<temperature_deci_c>,<humidity_deci_percent>,<obs_enabled>,<obs_valid>,<blocked>,<distance_mm>,<threshold_mm>,<reason>,<alarm_flags>,<sample_count>,<env_age_ms>,<obstacle_age_ms>
OK
```

- `M` 是手机端巡检界面的主轮询命令，用一个响应合并 `T/E/D` 的关键数据，减少星闪 write/notify 次数。
- `env_valid`、`temperature_deci_c`、`humidity_deci_percent`：同 `+ROBOT:ENV`。
- `obs_enabled`、`obs_valid`、`blocked`、`distance_mm`、`threshold_mm`、`reason`：同 `+ROBOT:OBS`。
- `alarm_flags`：位图报警；`bit0=温湿度无效`，`bit1=距离无效`，`bit2=障碍阻挡`，`bit3=温度过高`，`bit4=湿度过高`。
- 温湿度报警阈值：温度 `>=35.0 C` 置 `bit3`，湿度 `>=85.0%RH` 置 `bit4`。
- `sample_count`：板端巡检采样计数；温湿度和距离任一采样成功调度后递增。
- `env_age_ms`、`obstacle_age_ms`：缓存数据年龄；未知时为 `0xFFFFFFFF`。
- 板端后台持续采样：距离约 `500 ms` 一次，温湿度约 `2000 ms` 一次；手机端建议 `1000 ms` 轮询一次 `M`。
- 板载蜂鸣器和红色 LED 跟随 `alarm_flags` 本地告警：`bit2=障碍阻挡` 触发急促三短声/闪烁，`bit3/bit4=温湿度过高` 触发慢速两短声/闪烁。`bit0/bit1` 仅表示传感器数据无效，不触发蜂鸣器，避免瞬时读数失败导致误报。

蜂鸣器自检返回：

```text
+ROBOT:BEEP,<buzzer_ready>,<alarm_led_ready>
OK
```

- `buzzer_ready=1`：`GPIO_04/BEEP` 初始化成功，并已执行直流驱动 + 2.7kHz 方波自检。
- `alarm_led_ready=1`：`GPIO_09/LED_R` 初始化成功，自检时红灯会同步闪烁。
- 若返回 `+ROBOT:BEEP,1,1` 但仍无声音，而红灯能闪烁，优先检查板载蜂鸣器、Q4 驱动、电源或焊接/器件状态。
- 自动报警不需要上位机轮询；板端主循环会持续根据 `alarm_flags` 控制声光报警。

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
- 空旷无回波按远距离安全处理；真正的传感器未就绪/异常仍会 fail-safe 停车并退出自动模式。

路线巡检返回：

```text
+ROBOT:PATROL,<active>,<phase>,<status>,<leg_index>,<loop_count>,<alarm_flags>,<enabled>,<valid>,<blocked>,<distance_mm>,<threshold_mm>,<reason>
OK
```

- `P` 是一键启动板端自动方形巡检路线；启动后路线由 WS63 固件自主执行，不依赖手机持续发送方向键。`G` 是查询巡检轨迹状态，`M` 是读取温湿度/距离快照；手机端开启巡检时先发 `P`，之后只轮询 `G` 和 `M`。
- `active=1`：路线巡检已启动；`S` / `AT+ROBOTS` 会立即停车并退出。
- `leg_index`：当前方形路线边序号，`0..3`。
- `loop_count`：已完成的外圈巡检圈数。
- `phase`：`0=空闲`，`1=按路线直行`，`2=路线右转`，`3=遇障后退`，`4=右转离线`，`5=侧向绕开`，`6=左转平行`，`7=平行前进越过障碍`，`8=左转回线`，`9=回到轨迹`，`10=恢复原方向`，`11=环境告警驻留`，`12=传感器异常停车`。
- 40 平米正方形场地边长约 `6.32 m`。建议小车巡检线离场地边界约 `0.3-0.4 m`；当前固件用时间控制四边方形路线，默认每边直行 `9000 ms`、右转约 `430 ms`。
- 遇到障碍后不是直接改道跑远，而是执行“后退 -> 右转离线 -> 侧向绕开 -> 左转平行 -> 平行前进越过障碍 -> 左转回线 -> 回到轨迹 -> 右转恢复方向”，然后继续当前边剩余巡检时间。`平行前进越过障碍` 默认持续 `2600 ms`，用于确保车身超过障碍物后再回线。
- 巡检避障采用连续两次低于阈值确认，减少超声波单次毛刺导致的误绕行；空旷无回波按远距离安全处理，不再导致巡检退出。
- 温湿度异常时进入短暂停车驻留，声光报警保持触发；驻留后继续巡检。
- 由于当前版本没有编码器/定位，轨迹恢复是时间控制的近似回线；实车校准主要调 `ROBOT_PATROL_FORWARD_MS`、`ROBOT_PATROL_TURN_MS` 和绕行时间参数。

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

SLE/星闪第一版控制 profile 见 `D:\WS63_NearLink_Robot_MVP\protocol_docs\sle_control_profile.md`。SLE 初期建议先使用 ASCII 兼容模式：central 写入 `F/B/L/R/S/I/O/E/D/M/A/T`，板端 notify 本文定义的 `+ROBOT:*` 响应；待连接和安全动作稳定后，再切换到下方二进制控制包做 1000 包 RTT 基线测试。

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
