# 硬件接入说明

> 说明：本文件保留时间顺序硬件笔记。当前权威基线、目录真相和标准流程见 `docs/PROJECT_BASELINE_AND_WORKFLOW.md`。

## 已确认链路

- 主控为 WS63/WS63E 系列，当前 PC 通信端口为 COM5。
- 串口参数：115200，8N1。
- WS63 默认 AT 子系统占用 UART0；原始二进制包会被 AT 层吞掉，所以实板 MVP 先走 AT 命令。
- `HI_SCL/HI_SDA` 通过 TCA9803 I2C 中继连接到底板 `SCL/SDA`。
- 电机 1 使用 L9110S 的 `PWM1_3/PWM1_4`，电机 2 使用 `PWM1_1/PWM1_2`。
- 这些 PWM 信号来自板载 STM8S103 “PWM Generator”，不是 WS63 直出 PWM。
- AHT20、OLED、电机 PWM 发生器等共享 I2C 总线。
- HC-SR04 前向超声波按 `HiSparkEP_2025-02-10_pinout.md` 使用 TRIG=`GPIO_00`、ECHO=`GPIO_01`。

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
- 等超声波模块到货前，新增 `AT+ROBOTAVOIDTEST` / 上位机 `X`，用软件模拟 120 mm 障碍和 450 mm 安全距离，验证避障状态机；该命令会真实驱动电机，必须车轮离地测试。
- `X` 固件已烧录，未初始化电机时已确认返回 `MOTOR_ERROR`、不运动；`I,T` 已确认 `ready=1`；用户离地实测 `I,X` 动作现象通过。
- 下一阶段进入 SLE/星闪控制链路。当前已接入 `robot_sle_server.c`，广播名 `ws63_robot_mvp`，service `0x7100`，command `0x7101`，response `0x7102`；仍保留 COM5 AT 作为已验证主链路。
- SLE bridge 已烧录成功，保留包为 `D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg`。COM5 日志已确认 `ROBOT SLE adv ret=0x0 name=ws63_robot_mvp` 和 `ROBOT_MVP READY protocol=at/sle`；下一步用支持 SLE central 的设备扫描连接。
- 备注：曾尝试 `known_loader...unverified.fwpkg`，但第二阶段 `Ack receiving timeout`，已删除失败包；本次成功烧录的是 SDK 标准输出 `D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg`。

## 电机层当前假设

当前 `dut_motor.c` 使用：

- I2C bus：1
- SCL pin：15
- SDA pin：16
- pin mode：2
- I2C 地址：`0x5A`
- PWM duty 范围：0..999
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

如出现左右轮方向相反，只修改 `dut_motor.c` 的 HAL 映射，不改上层协议。

完整离地动作验收：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands O,E,T,I,T,F,S,T,B,S,T,L,S,T,R,S,E,T,S --interval 0.25 --timeout 3
```

无超声波模块时的避障状态机验收：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands I,X,T,S,T --interval 0.8 --timeout 3
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
- 模拟避障 `X` 也会真实驱动车轮，不能在车轮接触桌面/地面且无人看守时运行。
- HC-SR04 的 ECHO 若为 5V，必须确认底板已有电平转换或另加 3.3V 输入保护。
- 一旦观察到异常抖动、方向反、卡滞或过热，立刻断电。
