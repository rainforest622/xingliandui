# 硬件接入说明

> 说明：本文件保留时间顺序硬件笔记。当前权威基线、目录真相和标准流程见 `docs/PROJECT_BASELINE_AND_WORKFLOW.md`。

> 2026-07-17 硬件方向更新：当前不再优先焊接 R16/R17，也不使用 H5-1/H5-2 作为 WAVE ROVER UART。物理主线为 WS63 Type-C 接树莓派 USB，树莓派通过 40PIN UART `/dev/serial0` 控制 WAVE ROVER；系统目标升级为工业轨迹闭环巡检。完整决策见 `docs/ARCHITECTURE_DIRECTION.md`，阶段路线见 `docs/FINAL_GOAL_ROADMAP.md`。

## 已确认链路

- 主控为 WS63/WS63E 系列，当前 PC 通信端口为 COM5。
- 串口参数：115200，8N1。
- WS63 默认 AT 子系统占用 UART0；原始二进制包会被 AT 层吞掉，所以实板 MVP 先走 AT 命令。
- `HI_SCL/HI_SDA` 通过 TCA9803 I2C 中继连接到底板 `SCL/SDA`。
- 电机 1 使用 L9110S 的 `PWM1_3/PWM1_4`，电机 2 使用 `PWM1_1/PWM1_2`。
- 这些 PWM 信号来自板载 STM8S103 “PWM Generator”，不是 WS63 直出 PWM。
- AHT20、OLED、电机 PWM 发生器等共享 I2C 总线。
- HC-SR04 前向超声波按 `HiSparkEP_2025-02-10_pinout.md` 使用 TRIG=`GPIO_00`、ECHO=`GPIO_01`。
- 板载蜂鸣器按 `HiSparkEP_2025-02-10_pinout.md` 使用 BEEP=`GPIO_04`，经 Q4 驱动；巡检报警由 WS63 本地 GPIO 直接触发，不依赖手机端。当前固件同时使用 `GPIO_09/LED_R` 做红灯同步告警，便于现场确认告警链路。
- 蜂鸣器排查记录：`AT+ROBOTBEEP` 返回 `+ROBOT:BEEP,<buzzer_ready>,<alarm_led_ready>`。若返回 `+ROBOT:BEEP,1,1`，且 `AT+GETIOMODE=4` 为 `+GETIOMODE:4,0,0,7`、`AT+WTGPIO=4,1/0` 可正常读回高低电平，但现场仍无声音，优先检查 BUZZER1 是否实际焊装、Q4/S8050 与 R35 是否正常、蜂鸣器 VCC 供电和板载器件方向。

## 当前实测记录

2026-06-28：

- COM5 可打开、可收启动日志。
- app-only 固件 `ws63-liteos-app_load_only.fwpkg` 可烧录；不要使用 `all.fwpkg`，之前触发过 image ID/Flash Init 相关异常。
- 板端启动日志出现 `ROBOT_MVP READY protocol=at uart=0 baud=115200 motor=SKIPPED`。
- `AT+ROBOTS` 返回 `+ROBOT:ACK,<seq>,5,0,0`，STOP 通信闭环成功。
- `AT+ROBOTF` 在未初始化电机时返回 `status=5,moving=0`，符合安全预期。
- 新增 `AT+ROBOTMI` 后，实测返回 `+ROBOT:MOTOR,1`，说明 I2C/PWM 初始化与 STOP 写入成功。
- 修正 `AT+ROBOTMI` 重复调用问题后，`I,S,I,S` 连续测试均返回 OK。
- 首次 `F,S,S` 短脉冲控制侧返回 `F status=OK,moving=1`，随后 STOP 返回 `status=OK,moving=0`。物理轮向仍需现场观察确认。
- `I,B,S,L,S,R,S,S` 短脉冲控制侧全部返回 OK：B/L/R 均进入 `moving=1`，每次 STOP 回到 `moving=0`。
- 单独新开一次串口会丢失板端 `g_motor_ready` 运行态，因此每组真实动作测试开头都应先发 `I`。上位机已显式关闭 DTR/RTS，减少打开串口时的复位/扰动。
- 现场观察确认：F 前进、B 后退、L 左转、R 右转均正确。
- 完整演示 `I,F,S,B,S,L,S,R,S,S` 已通过，平均 RTT 约 11 ms，并导出 CSV 延迟记录。
- 新增状态遥测 `AT+ROBOTST`，上位机命令为 `T`。可返回 `ready/moving/uptime_ms/last_cmd/last_seq/age_ms`。
- 带遥测完整演示 `T,I,T,F,S,T,B,S,T,L,S,T,R,S,T,S` 已通过，平均 RTT 约 12 ms，并导出 CSV 延迟与状态记录。
- 新增 OLED 初始化 `AT+ROBOTOLED`，上位机命令为 `O`。实测 `O,T` 返回 OK，OLED 刷新会把 RTT 提升到约 110 ms，首次 OLED 初始化约 330 ms。
- 带 OLED + 遥测 + 四向动作完整演示 `O,T,I,T,F,S,T,B,S,T,L,S,T,R,S,T,S` 已通过，并导出 CSV。
- 环境数据链路已补上上位机命令 `E` / `AT+ROBOTENV`、CSV 字段、WS63 可移植 AHT20 原始帧换算和状态包湿度编码。下一步真机任务是把 LiteOS I2C AHT20 读数接到 `robot_environment_read_fn`，实测返回 `+ROBOT:ENV,1,<temperature_deci_c>,<humidity_deci_percent>`。
- 已参考 HiSpark `fbb_ws63` 官方示例确认 AHT20 接入参数：I2C bus 1、SCL pin 15、SDA pin 16、pin mode 2、AHT20 7-bit 地址 `0x38`、触发测量命令 `0xAC 0x33 0x00`、约 75-80 ms 后读取 6 字节。仓库新增 `ws63_liteos/sensor/aht20_ws63.c` 作为 HiSparkStudio 工程可选硬件适配层。

2026-06-30：

- 启用 HC-SR04 GPIO 驱动，新增 `AT+ROBOTAVOID` 智能避障自动模式。
- 自动避障策略：前进，距离低于 250 mm 时停车、后退约 260 ms、右转约 420 ms，再继续前进。
- 若超声波采样无效，板端会 fail-safe 停车并退出自动避障。
- 实板已确认新固件生效：`A` 命令已注册；未初始化电机时返回 `MOTOR_ERROR`，电机初始化后若无有效 ECHO，则返回 `OBSTACLE_STOP`、`phase=4`、不运动。
- 当前 `D` 距离读取为 `valid=0,distance=0`，下一步需要现场确认 HC-SR04 的 VCC/GND/TRIG/ECHO 接线、前方反射面，以及 ECHO 电平是否已安全转换到 3.3V。
- 历史过渡：超声波模块到货前曾使用 `AT+ROBOTAVOIDTEST` / 上位机 `X` 模拟避障状态机；该模拟入口在真实 HC-SR04 接入后已从当前固件和上位机协议中移除。
- 下一阶段进入 SLE/星闪控制链路。当前已接入 `robot_sle_server.c`，广播名 `ws63_robot_mvp`，service `0x7100`，command `0x7101`，response `0x7102`；仍保留 COM5 AT 作为已验证主链路。
- SLE bridge 已烧录成功，保留包为 `D:\WS63_NearLink_Robot_MVP\artifacts\ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg`。COM5 日志已确认 `ROBOT SLE adv ret=0x0 name=ws63_robot_mvp` 和 `ROBOT_MVP READY protocol=at/sle`；下一步用支持 SLE central 的设备扫描连接。
- 备注：曾尝试 `known_loader...unverified.fwpkg`，但第二阶段 `Ack receiving timeout`，已删除失败包；本次成功烧录的是 SDK 标准输出 `D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg`。

2026-07-04：

- HC-SR04 外设已接入，当前代码路径不再包含模拟避障命令；`D` / `AT+ROBOTOBS` 和 `A` / `AT+ROBOTAVOID` 均读取真实 TRIG/ECHO。
- 板端 HC-SR04 读数增加 ECHO 空闲检测、近/远距离限幅、异常 fail-safe 和 `reason` 诊断字段；若 ECHO 卡高、无有效回波或采样超时，自动避障会停车退出。
- `reason` 映射：`0=ok`，`1=not_ready`，`2=echo_idle_high`，`3=no_echo_rise`，`4=no_echo_fall`，`5=invalid_pulse`。2026-07-06 ECHO 下拉增强版固件已烧录成功，`AT+GETIOMODE=1` 实测为 `+GETIOMODE:1,0,1,0`，当前 `D` 实测为 `reason=3/no_echo_rise`。
- 智能避障 fail-safe 已在旧异常状态验证：`I,A,S,T` 中 `A` 返回 `OBSTACLE_STOP`、`moving=0`、`phase=4`、`reason=2/echo_idle_high`，不会在 ECHO 异常时误动车。当前 `reason=3/no_echo_rise` 下，先修硬件连接，不建议带轮直接跑 `I,A`。
- 2026-07-06 现场照片显示超声波模块为松动直插。若扩展板孔位没有母座弹片或焊接，直插不能保证导通，可能直接造成 ECHO 常高、无上升沿或读数跳变。距离验收前应先用可靠母座/杜邦线/焊接固定，或在临时压紧模块时对比 `D` 的 `reason/dist` 是否变化。
- 2026-07-06 正面照片显示模块丝印顺序为 `VCC / Trig / Echo / Gnd`；购买记录显示该模块工作电压为 `DC3.3-5V`。因此当前优先按 3.3V 兼容模块处理：直接接 CN1；若仍 `no_echo_rise`，优先查模块端 VCC-GND 实测电压、四针接触、CN1 物理 pin1 方向、TRIG/ECHO 是否交叉。

仅 5V HC-SR04 的备用接线方案：

| HC-SR04 | 接到哪里 | 注意 |
|---|---|---|
| VCC | 板上明确的 `+5V` 或外部 5V | 仅适用于不支持 3.3V 的模块 |
| GND | 板子 GND | 必须共地 |
| TRIG | CN1-2 / `GPIO_00` | 3.3V 触发通常可用 |
| ECHO | 经分压后到 CN1-3 / `GPIO_01` | 不可 5V 直连 GPIO |

ECHO 分压：`ECHO -- 10kΩ -- GPIO_01 -- 20kΩ -- GND`，约把 5V 降到 3.3V。

最终产品如果只用 3.7V/18650 电池供电，当前这种 `DC3.3-5V` 模块可继续用板上 3.3V `VCC`；只有换成仅 5V 模块时才需要升压到 5V。

## 电机层当前假设

当前 `robot_motor.c` 使用：

- I2C bus：1
- SCL pin：15
- SDA pin：16
- pin mode：2
- I2C 地址：`0x5A`
- PWM 输出范围：0..999
- 方向通道：
  - 右轮：`0x70/0x80`
  - 左轮：`0x90/0xA0`

这些通道映射来自当前工程假设，首次动作测试必须架空车轮，并由现场观察确认左右轮和前后方向。

## 下一步动作测试

首次真实动作建议命令：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands I,S,F,S --interval 0.2 --timeout 3
```

预期：

- `I` 返回 `status=OK`
- `S` 返回 `status=OK,moving=0`
- `F` 若电机动作成功，返回 `status=OK,moving=1`
- 最后 `S` 停车

如出现左右轮方向相反，只修改 `robot_motor.c` 的 HAL 映射，不改上层协议。

完整离地动作验收：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands O,E,T,I,T,F,S,T,B,S,T,L,S,T,R,S,E,T,S --interval 0.25 --timeout 3
```

真实超声波距离与避障验收：

只读距离连续采样，不启动电机：

```powershell
python scripts\verify_ultrasonic_distance.py --serial-port COM5 --samples 10 --interval 0.5 --timeout 4
```

修好前的当前特征是 `no_echo_rise` 且 `valid_distance_mm=none`；修好后应出现 `valid=1` 和稳定 `distance_mm`。

2026-07-06 改用杜邦线后，COM5 曾短暂掉线，恢复后连续 10 次只读采样仍为 `no_echo_rise`。这说明直插松动不是唯一问题；优先检查模块端 VCC-GND 是否真有约 3.3V、CN1 物理 pin1 方向是否反、TRIG/ECHO 是否交叉。

2026-07-07 安全全功能检测中，通信、OLED、AHT20 温湿度、状态查询和停止命令均正常；超声波曾短暂返回 `reason=4/no_echo_fall`，说明 ECHO 可能被拉高过，但后续只读连续采样又回到 `reason=3/no_echo_rise`。当前判断为硬件连接/供电/反射条件不稳定，不能进入带轮自动避障验收。

2026-07-07 重新摆放反射物/调整接线后，超声波测距已通过：12 次只读采样中 9 次 `reason=0/ok`，有效距离范围 `67..460mm`；二次确认 `D,D,D,D,D` 均为 `obs_valid=1`、`reason=0/ok`，近距离 `dist=39..70mm` 且 `block=1`，250mm 阈值触发正确。下一步是远距离 `block=0` 稳定记录和安全场地/车轮离地的 `A` 智能避障运动验收。

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands D,D,I,A,S,T --interval 0.8 --timeout 3
```

一键演示脚本：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_mvp_demo.ps1 -Port COM5
```

## 安全约束

- 首次动作测试必须车轮离地。
- 手不要接触车轮、齿轮和履带。
- 速度先使用 AT 固件内置安全速度 `20%`。
- 主机发出 `F` 后会立即发 `S`，板端也有 500 ms 看门狗兜底停车。
- 自动避障 `A` 启动后会持续动作，必须保证测试场地安全，随时准备发送 `S` 或断电。
- HC-SR04 的 ECHO 若为 5V，必须确认底板已有电平转换或另加 3.3V 输入保护；不要把 5V ECHO 直接接入 `GPIO_01`。
- 一旦观察到异常抖动、方向反、卡滞或过热，立刻断电。
