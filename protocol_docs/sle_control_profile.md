# WS63 Robot SLE 控制 Profile

> 状态：板端最小 SLE server 已烧录并确认 announce 启动；当前真机已验证链路仍是 `COM5` + `AT over UART0`。SLE 还需要 central 扫描、连接、写入和 notify 的实板验收。

## 1. 目标

最终产品不应依赖电脑 USB 线控制小车。USB 当前只承担供电、烧录和调试串口作用；进入产品形态后，控制链路应迁移到 SLE/星闪，供电链路应迁移到电池或独立稳压模块。

SLE 阶段的目标是：

1. 手机、另一块 WS63 或星闪网关作为 SLE central 连接小车；
2. central 写入控制命令；
3. WS63 调用现有 `robot_mvp_control_*` 底盘控制 API；
4. WS63 通过 notify/read 返回与 AT 基本等价的结果；
5. 上位机/实验脚本统计 RTT、丢包和急停响应。

## 2. SDK 参考来源

官方 SLE server 样例位置：

```text
D:\w\src\application\samples\bt\sle\sle_speed_server
```

该样例是吞吐测试，不是机器人控制样例。接入机器人时只借用：

- SLE enable 与 announce callback；
- SSAP server 注册；
- service/property/descriptor 添加流程；
- write request callback；
- notify/indicate 发送流程。

不要直接保留 speed_server 的持续发大包线程，否则会干扰控制 RTT 和电机安全响应。

## 3. 建议服务定义

设备广播名建议：

```text
ws63_robot_mvp
```

UUID 建议：

| 项 | UUID | 权限 | 说明 |
|---|---:|---|---|
| Robot service | `0x7100` | - | 机器人控制服务 |
| Command characteristic | `0x7101` | Write | central 写入控制命令 |
| Response characteristic | `0x7102` | Read + Notify | WS63 返回 ACK/状态/传感器结果 |
| Telemetry characteristic | `0x7103` | Read + Notify | 可选：周期状态、环境和距离 |

如果官方工具或手机调试工具不支持自定义 16-bit UUID，可沿用 SDK 样例的 base UUID，只替换末尾 16-bit 字段。

## 4. 第一版 payload：ASCII 兼容模式

第一版优先使用 ASCII 文本，而不是马上上二进制包。原因是当前 AT 解析、上位机 CSV、测试断言都已经稳定。

central 写入 `Command characteristic`：

| 写入内容 | 等价 AT | 说明 |
|---|---|---|
| `F` | `AT+ROBOTF` | 前进 |
| `B` | `AT+ROBOTB` | 后退 |
| `L` | `AT+ROBOTL` | 左转 |
| `R` | `AT+ROBOTR` | 右转 |
| `S` | `AT+ROBOTS` | 停止/急停 |
| `I` | `AT+ROBOTMI` | 初始化电机 |
| `O` | `AT+ROBOTOLED` | 初始化 OLED |
| `E` | `AT+ROBOTENV` | 读取温湿度 |
| `D` | `AT+ROBOTOBS` | 读取距离/避障 |
| `M` | `AT+ROBOTMON` | 读取工业巡检快照 |
| `A` | `AT+ROBOTAVOID` | 启动真实智能避障 |
| `T` | `AT+ROBOTST` | 查询状态 |

WS63 notify `Response characteristic`：

```text
+ROBOT:ACK,<seq>,<cmd>,<status>,<moving>\r\n
+ROBOT:STATE,<uptime_ms>,<ready>,<moving>,<last_cmd>,<last_seq>,<age_ms>\r\n
+ROBOT:ENV,<ok>,<temperature_deci_c>,<humidity_deci_percent>\r\n
+ROBOT:OBS,<enabled>,<valid>,<blocked>,<distance_mm>,<threshold_mm>[,<reason>]\r\n
+ROBOT:MON,<uptime_ms>,<ready>,<moving>,<env_valid>,<temperature_deci_c>,<humidity_deci_percent>,<obs_enabled>,<obs_valid>,<blocked>,<distance_mm>,<threshold_mm>,<reason>,<alarm_flags>,<sample_count>,<env_age_ms>,<obstacle_age_ms>\r\n
+ROBOT:AVOID,<active>,<phase>,<status>,<enabled>,<valid>,<blocked>,<distance_mm>,<threshold_mm>[,<reason>]\r\n
```

`reason` 为可选诊断字段；旧固件没有该字段。新固件定义：`0=ok`，`1=not_ready`，`2=echo_idle_high`，`3=no_echo_rise`，`4=no_echo_fall`，`5=invalid_pulse`。
`+ROBOT:MON` 是手机巡检界面的主轮询响应；`alarm_flags` 位定义见 `protocol_v1.md`。

说明：

- 响应文本沿用 `protocol_v1.md`，但 SLE notify 不需要额外发送 `OK` 行。
- 每次 write 至少返回一条 notify，便于 central 统计 RTT。
- `S` 必须最高优先级处理；即使 notify 失败，也必须先停车。

## 5. 第二版 payload：二进制兼容模式

当 ASCII 模式跑通后，再引入 `protocol_v1.md` 里保留的二进制控制包：

- 写入 6 字节控制包：`0xAA cmd speed_l speed_r seq checksum`
- notify 4 字节 ACK：`0x55 seq status checksum`
- 可选 notify 8 字节状态包：`0x5A seq battery humidity distance_mm motor_state checksum`

二进制模式适合低 RTT 和 1000 包基线测试；ASCII 模式适合调试和手机工具手动验证。

## 6. 板端接入点

当前已新增的复用入口：

```text
D:\w\src\application\samples\peripheral\robot_mvp\robot_mvp_control.h
```

当前已接入的 SLE server：

```text
D:\w\src\application\samples\peripheral\robot_mvp\robot_sle_server.c
D:\w\src\application\samples\peripheral\robot_mvp\robot_sle_server.h
```

构建开关：

```text
CONFIG_ROBOT_MVP_ENABLE_SLE=y
```

当前实现状态：

- `robot_task()` 启动时调用 `robot_sle_server_start()`；
- 广播名：`ws63_robot_mvp`；
- service UUID：`0x7100`；
- command characteristic：`0x7101`，支持 write / write-no-response；
- response characteristic：`0x7102`，支持 read + notify；
- SLE write callback 只入队，不直接执行电机/I2C/OLED/温湿度操作；
- `RobotSle` 工作线程执行 `F/B/L/R/S/I/O/E/D/M/A/T` 后 notify 对应 `+ROBOT:*` 响应；
- `conn_id=0` 已按合法连接处理，不能再当作未连接。

SLE write callback 不应直接操作 `robot_motor.c`，而应调用：

- `robot_mvp_control_motion()`：处理 `F/B/L/R/S`
- `robot_mvp_control_motor_init()`：处理 `I`
- `robot_mvp_control_oled_init()`：处理 `O`
- `robot_mvp_control_read_env()`：处理 `E`
- `robot_mvp_control_read_obstacle()`：处理 `D`
- `robot_mvp_control_get_monitor()`：处理 `M`
- `robot_mvp_control_start_avoid()`：处理 `A`
- `robot_mvp_control_get_state()`：处理 `T`

这样 AT、SLE、未来 TCP/网关转发可以共享同一套底盘行为和安全约束。

上位机侧命令 profile 已抽到：

```text
D:\WS63_NearLink_Robot_MVP\upper_client\robot_profile.py
```

未来实现 SLE central 时，ASCII 写入命令应复用 `encode_sle_ascii_command()`，不要重新手写一套 `F/B/L/R/S/I/O/E/D/M/A/T` 映射。

当前 SLE bridge 调试包：

```text
D:\WS63_NearLink_Robot_MVP\artifacts\ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg
```

2026-06-30 已确认：

- SDK 标准输出包烧录成功；
- COM5 启动日志出现 `ROBOT SLE enable status=0x0`；
- COM5 启动日志出现 `ROBOT SLE add service ret=0x0`；
- COM5 启动日志出现 `ROBOT SLE adv ret=0x0 name=ws63_robot_mvp`；
- COM5 启动日志出现 `ROBOT_MVP READY protocol=at/sle ...`；
- AT 回归 `T` 正常，RTT 约 `14 ms`。

## 7. 验收顺序

1. 只打开广播：确认能扫描到 `ws63_robot_mvp`；
2. 连接后写入 `T`：应收到状态 notify；
3. 写入 `I,T`：`ready=1`；
4. 车轮离地写入 `F,S,B,S,L,S,R,S`：方向与 AT 验收一致；
5. 写入 `D`：应收到真实 HC-SR04 距离/避障 notify；
6. 写入 `M`：应收到温湿度、距离、报警位和采样年龄组成的巡检快照 notify；
7. 在安全场地写入 `A`，再用 `S` 急停：验证真实超声波避障链路和停车优先级；
8. 统计 1000 次 `T`、`M` 或 `F,S` 的 RTT、P95、P99、最大值、丢包率；
9. 开启树莓派视频后重复第 8 步，形成 SLE 控制与视频拥塞对比材料。

烧录后先用 COM5 日志确认：

```text
ROBOT SLE start requested name=ws63_robot_mvp service=0x7100
ROBOT SLE enable status=0x0
ROBOT SLE add service ret=0x0 ...
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
```

## 8. 注意事项

- 当前手机是否能直接做 SLE central 取决于手机硬件、系统和调试 App；不能默认所有手机都支持星闪。
- 如果手机侧工具不可用，先用另一块 WS63 或官方星闪调试工具做 central。
- 不要把 SLE 吞吐样例的大包发送线程带入机器人固件。
- 首次 SLE 动作测试仍然必须车轮离地。
- USB 可以继续用于供电和日志观察，但不能作为最终控制链路卖点。
