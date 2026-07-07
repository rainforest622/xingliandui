# WS63 MVP 基线与工程化工作流

本文件是当前项目的唯一权威交接文档。

- 日期基线：2026-07-04
- 当前实板通信口：`COM5`
- 当前实板协议层：`AT over UART0`
- 当前最终可烧录源码主线：`D:\w\src\application\samples\peripheral\robot_mvp`

## 1. 当前已实现功能

当前 MVP 已在实板上完成并验证以下能力：

| 功能 | 上位机命令 | 板端 AT | 当前状态 |
|---|---|---|---|
| OLED 初始化与状态页显示 | `O` | `AT+ROBOTOLED` | 已验证 |
| AHT20 温湿度读取 | `E` | `AT+ROBOTENV` | 已验证 |
| 前向距离/避障接口 | `D` | `AT+ROBOTOBS` | HC-SR04 真实外设已接入，待距离实测记录 |
| 智能避障自动行驶 | `A` | `AT+ROBOTAVOID` | 已实现真实超声波链路，待场地避障验收 |
| SLE/星闪控制服务 | SLE 写入 `F/B/L/R/S/I/O/E/D/A/T` | service=`0x7100` | 已烧录并确认广播启动，待 central 扫描/连接实测 |
| 状态遥测 | `T` | `AT+ROBOTST` | 已验证 |
| 电机层初始化 | `I` | `AT+ROBOTMI` | 已验证 |
| 前进 | `F` | `AT+ROBOTF` | 已验证 |
| 后退 | `B` | `AT+ROBOTB` | 已验证 |
| 左转 | `L` | `AT+ROBOTL` | 已验证 |
| 右转 | `R` | `AT+ROBOTR` | 已验证 |
| 停止 | `S` | `AT+ROBOTS` | 已验证 |

2026-06-29 最终实板验收序列：

```text
O,E,T,I,T,F,S,T,B,S,T,L,S,T,R,S,E,T,S
```

最终实板观察结果：

- `E` 已返回真实温湿度，例如 `temp=34.5C hum=45.7%`
- `T` 已正确返回 `ready` 从 `0` 到 `1` 的状态变化
- `F/B/L/R/S` 现场观察方向全部正确
- 板上 `USBSV` 亮，烧录和通信均依赖 `COM5`

最终保留的验证记录：

- `D:\aaa嵌入式比赛\artifacts\mvp_env_full_after_reset.csv`
- `D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_env_load_only.fwpkg`
- `D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_smart_obstacle_known_loader_load_only.fwpkg`
- `D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg`（已烧录，COM5 日志确认 SLE server/announce 启动）
- `D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg`（2026-07-04：真实 HC-SR04，已移除模拟 `X`）

2026-06-30 智能避障开发状态：

- `AT+ROBOTOBS` 已启用 HC-SR04 GPIO 驱动，按 pinout 使用 TRIG=`GPIO_00`、ECHO=`GPIO_01`
- 新增 `AT+ROBOTAVOID` / 上位机 `A`：板端自动前进，遇障后停车、短暂后退、右转，再继续前进
- `S` / `AT+ROBOTS` 会退出自动避障并停车
- 若距离采样无效，自动避障会 fail-safe 停车并退出
- 电机层仍保持当前实测可用的 `0x5A`，不按 pinout 中 U13 `0x15` 修改
- 烧录注意：直接烧新生成的 `ws63-liteos-app_load_only.fwpkg` 曾在 root loader 后第二阶段超时；已使用历史已知可用 loader + 新 app 重新打包为 `ws63-liteos-app_mvp_smart_obstacle_known_loader_load_only.fwpkg` 并烧录成功
- 实板状态：`T` 正常，`A` 已注册；未初始化电机时 `A` 返回 `MOTOR_ERROR`，初始化后若 HC-SR04 无有效回波，`A` 返回 `OBSTACLE_STOP`、`phase=4` 且不运动
- 历史过渡：超声波模块到货前曾使用 `AT+ROBOTAVOIDTEST` / 上位机 `X` 做模拟避障状态机测试；真实 HC-SR04 接入后，当前版本已删除该模拟入口。

2026-07-04 真实超声波接入状态：

- HC-SR04 外设已接入，当前板端代码不再包含模拟避障路径；`D` 和 `A` 均读取真实 TRIG/ECHO。
- `robot_obstacle.c` 已增加 ECHO 空闲检测、近/远距离限幅、64 位距离换算和 `reason` 诊断字段；异常回波继续按 fail-safe 停车处理。
- 2026-07-06 新版诊断固件已烧录成功；旧状态曾为 `reason=2/echo_idle_high`，加入 ECHO 输入下拉并重新烧录后，`AT+GETIOMODE=1` 显示 pull=1，当前 `D` 返回 `reason=3/no_echo_rise`。
- 当前判断：ECHO 不再是单纯空闲常高，模块仍未对 TRIG 产生回波；现场照片显示超声波模块直插较松，优先修复连接可靠性、供电、TRIG/ECHO 顺序和 ECHO 电平转换。
- 已新增只读测距验收脚本 `D:\aaa嵌入式比赛\scripts\verify_ultrasonic_distance.py`，只发送 `AT+ROBOTOBS`，不会启动电机。当前接好模块后连续采样仍为 `no_echo_rise`，说明 TRIG 输出正常但模块未产生 ECHO 脉冲。
- 旧固件/旧状态下 `I,A,S,T` 已验证 fail-safe：`A` 返回 `OBSTACLE_STOP`、`moving=0`、`phase=4`、`reason=2/echo_idle_high`，不会误动车。当前 `reason=3` 下暂不主动跑 `I,A`，避免模块突然接触恢复后小车进入自动运动。
- 当前验收重点从“状态机能跑”转为“真实距离值稳定、障碍触发阈值正确、`A` 模式遇障后能安全后退/右转/继续前进”。

2026-06-30 下一阶段：SLE/星闪控制链路准备状态：

- 当前仍以 `COM5` 的 `AT over UART0` 为实板验收主链路，不破坏已验证功能
- 已新增板端控制抽象接口 `D:\w\src\application\samples\peripheral\robot_mvp\robot_mvp_control.h`
- `F/B/L/R/S/I/O/E/D/A/T` 对应的 AT 处理函数已改为调用 `robot_mvp_control_*`，为未来 SLE write callback 复用同一套底盘逻辑预留入口
- 已新增并接入板端 SLE server：`D:\w\src\application\samples\peripheral\robot_mvp\robot_sle_server.c`
- SLE 启动路径：`robot_task()` 调用 `robot_sle_server_start()`；广播名 `ws63_robot_mvp`，service `0x7100`，command `0x7101`，response/read+notify `0x7102`
- SLE write callback 只入队，不直接做 I2C/OLED/AHT20/电机动作；`RobotSle` 工作线程执行命令后 notify，避免阻塞 SLE service 线程
- 已修复 `conn_id=0` 也可能是合法连接的问题，notify 不再把 `conn_id==0` 当作无效
- 已新增上位机 profile 入口 `D:\aaa嵌入式比赛\upper_client\robot_profile.py`，未来 SLE central 复用 `encode_sle_ascii_command()`
- SDK 官方可参考样例为 `D:\w\src\application\samples\bt\sle\sle_speed_server`；它是吞吐样例，不是机器人控制样例，接入时应只借用 SLE 广播/连接/SSAP 属性框架
- SLE 控制协议草案见 `D:\aaa嵌入式比赛\protocol_docs\sle_control_profile.md`
- SLE bridge 重构时已验证：`python -m unittest discover -s D:\aaa嵌入式比赛\tests -v` 通过；`python build.py ws63-liteos-app` 编译成功。真实超声波版本移除模拟 `X` 并加入 `reason` 诊断字段后，当前软件测试基线为 `22 tests OK`。
- 当前 SLE 调试包：`D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg`
- 2026-06-30 实测：`known_loader...unverified.fwpkg` 第一阶段 loader 下载成功后第二阶段 `Ack receiving timeout`，已删除该失败包；改烧 SDK 标准输出 `D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg` 成功
- 2026-06-30 实测启动日志已确认：`ROBOT SLE enable status=0x0`、`ROBOT SLE add service ret=0x0`、`ROBOT SLE adv ret=0x0 name=ws63_robot_mvp`、`ROBOT_MVP READY protocol=at/sle ...`
- 烧录后 AT 回归：`T` 返回 OK，RTT 约 `14 ms`

## 2. 源码与目录真相

后续开发必须先分清“主线源码”和“工作区参考代码”。

| 路径 | 角色 | 是否主线 | 说明 |
|---|---|---|---|
| `D:\w\src\application\samples\peripheral\robot_mvp` | 实板固件源码 | 是 | 真实烧录、真实调试、真实功能修改都在这里完成；`robot_mvp_control.h` 是后续 SLE/其他传输层复用入口 |
| `D:\w\src\output\ws63\acore\ws63-liteos-app` | 编译输出 | 是 | `elf/bin/map` 等构建产物 |
| `D:\w\src\output\ws63\fwpkg\ws63-liteos-app` | SDK 打包输出 | 是 | 烧录包目录 |
| `D:\aaa嵌入式比赛\upper_client` | 上位机串口/TCP 客户端 | 是 | 命令发送、ACK 解析、CSV 输出 |
| `D:\aaa嵌入式比赛\tests` | 自动化测试 | 是 | Python 单测与 C 核心测试 |
| `D:\aaa嵌入式比赛\scripts` | 演示脚本 | 是 | 一键串口验收脚本 |
| `D:\aaa嵌入式比赛\protocol_docs` | 协议文档 | 是 | AT 协议和保留二进制协议说明 |
| `D:\aaa嵌入式比赛\docs` | 交接/流程/历史说明 | 是 | 先读本文件 |
| `D:\aaa嵌入式比赛\artifacts` | 最终保留产物 | 是 | 仅保留可复现实验结果 |
| `D:\aaa嵌入式比赛\ws63_liteos` | 参考实现镜像 | 否 | 用于说明 AT/AHT20 可移植实现，不是当前烧录主线 |
| `D:\fbb_ws63` | 临时 SDK 工作树 | 否 | 已清理为干净状态，不再作为当前开发主线 |

严格规则：

- 实板行为变更，只改 `D:\w\src\application\samples\peripheral\robot_mvp`
- 上位机行为变更，只改 `upper_client`、`scripts`、`tests`
- 协议字段变更，必须先改 `protocol_docs\protocol_v1.md`
- `ws63_liteos` 只保留为参考，不得把它误当成当前烧录源

## 3. 当前本地结构

工作区根目录保留以下结构：

```text
D:\aaa嵌入式比赛
├─ artifacts        最终保留的固件与验收 CSV
├─ docs             权威交接文档与历史说明
├─ protocol_docs    协议定义
├─ raspberry_pi     树莓派侧预留目录
├─ scripts          本地演示与验收脚本
├─ simulator        无硬件模拟器
├─ tests            单元测试与端到端测试
├─ upper_client     上位机串口/TCP 客户端
└─ ws63_liteos      参考 C 实现，不是主线固件目录
```

## 4. 标准开发流程

### 4.1 先判断改动归属

- 改板端 AT 命令、I2C、OLED、电机、AHT20：进入 `D:\w\src\application\samples\peripheral\robot_mvp`
- 改串口工具、命令格式、CSV、联调脚本：进入工作区 `upper_client` / `scripts` / `tests`
- 改协议语义：先改 `protocol_docs\protocol_v1.md`，再改两端实现

### 4.2 板端开发流程

1. 在 HiSpark Studio 中打开 `D:\w\src`
2. 选择或构建目标 `ws63-liteos-app`
3. 确认 `robot_mvp` 已被纳入样例构建链
   - `D:\w\src\application\samples\peripheral\CMakeLists.txt`
   - `D:\w\src\application\samples\peripheral\Kconfig`
4. 修改 `robot_mvp` 目录中的源码
5. 完成编译后，检查输出目录：
   - `D:\w\src\output\ws63\acore\ws63-liteos-app`
   - `D:\w\src\output\ws63\fwpkg\ws63-liteos-app`

说明：

- SDK 目标配置中，`ws63-liteos-app` 是当前应用构建目标名
- 不把 `D:\fbb_ws63` 当作当前 build 根目录
- 命令行构建前如遇到 `ccache` 找不到，先临时设置：

```powershell
$env:PATH='D:\HiSpark Studio 26.03.1\tools\cfbb\thirdparty\ccache;' + $env:PATH
```

### 4.3 打包流程

当前已验证可用的重打包命令：

```powershell
python D:\w\src\tools\pkg\packet.py ws63 ws63-liteos-app ' '
```

### 4.4 烧录流程

当前已验证可用的烧录命令：

```powershell
D:\HiSpark Studio 26.03.1\tools\BurnToolCmd\BurnToolCmd.exe --burn -n ws63 -m serial COM5 --baudRate 115200 -f D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
```

当前真实 HC-SR04 版本归档包：

```powershell
D:\HiSpark Studio 26.03.1\tools\BurnToolCmd\BurnToolCmd.exe --burn -n ws63 -m serial COM5 --baudRate 115200 -f D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg
```

当前 SLE bridge 调试包（已烧录并确认 SLE announce 启动，待 central 扫描/连接验证）：

```powershell
D:\HiSpark Studio 26.03.1\tools\BurnToolCmd\BurnToolCmd.exe --burn -n ws63 -m serial COM5 --baudRate 115200 -f D:\aaa嵌入式比赛\artifacts\ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg
```

烧录注意事项：

- 只烧录 `load_only.fwpkg`
- 不把 `all.fwpkg` 作为当前常规调试烧录包
- BurnTool 出现 `Please reset the device` 后，按板子 `RESET`
- 这块板没有 `BOOT` 键，不要等待 BOOT 操作
- 2026-06-30 SLE bridge 实测时，SDK 标准输出包烧录成功；不要再使用已删除的 `known_loader...unverified.fwpkg`

### 4.5 联调与验收流程

串口联调命令：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands O,E,T,I,T,F,S,T,B,S,T,L,S,T,R,S,E,T,S --interval 0.25 --timeout 3
```

智能避障联调命令：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands D,D,I,A,S,T --interval 0.8 --timeout 3
```

说明：前两个 `D` 用于观察真实距离是否稳定；`A` 会进入持续自动避障动作，必须在安全场地、车轮离地或低风险环境中执行，并随时准备 `S` 或断电。

SLE bridge 烧录后的串口日志烟雾检查：

```text
ROBOT SLE start requested name=ws63_robot_mvp service=0x7100
ROBOT SLE enable status=0x0
ROBOT SLE add service ret=0x0 ...
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
```

看到以上日志只能说明板端 SLE server 已启动；还需要用支持 SLE central 的手机/网关/另一块 WS63 扫描 `ws63_robot_mvp`，写入 `T` 后收到 `+ROBOT:STATE,...`，才算 SLE 链路实测通过。

一键演示脚本：

```powershell
powershell -ExecutionPolicy Bypass -File D:\aaa嵌入式比赛\scripts\run_mvp_demo.ps1 -Port COM5
```

串口实板规则：

- 每次新开串口做动作测试，先发 `I`
- 建议动作序列前先发 `O`
- 运动命令 `F/B/L/R` 后必须立刻跟 `S`

## 5. 回归规则

每次合入前至少完成以下检查：

### 5.1 纯软件回归

```powershell
python -m unittest discover -s D:\aaa嵌入式比赛\tests -v
```

当前基线结果：真实超声波版本移除模拟 `X` 用例并加入 `reason` 诊断字段后为 `22 tests OK`

### 5.2 实板烟雾测试

最小通过序列：

```text
E,T,I,T,F,S,B,S,L,S,R,S
```

如果改到 OLED 或环境传感器，使用完整序列：

```text
O,E,D,T,I,T,F,S,T,B,S,T,L,S,T,R,S,E,D,T,S
```

### 5.3 产物归档规则

- 只保留最终有价值的 `.csv` 和 `.fwpkg`
- 临时测试日志、旧包、重复 CSV 不长期保留
- `artifacts` 目录中的文件必须能说明“是什么版本、验证了什么”

## 6. 关键注意事项

- 车轮离地后再做首轮动作测试
- 上位机已关闭 `DTR/RTS`，尽量减少串口打开时对板端的扰动
- 当前真实协议是 `AT over UART0`，不要把保留的二进制协议直接拿去打实板串口
- 当前版本已移除 `X` / `AT+ROBOTAVOIDTEST` 模拟避障入口；不要再按旧文档或旧包发送该命令
- AHT20、OLED、PWM 发生器共享 I2C，总线初始化顺序和复用要谨慎
- OLED 初始化和刷新会显著拉高 RTT，属于正常现象
- 温湿度功能已通，不要再把 “AHT20 未接通” 当作当前问题
- `HiSparkEP_2025-02-10_pinout.md` 标注超声波为 TRIG=`GPIO_00`、ECHO=`GPIO_01`；当前 HC-SR04 驱动已启用，若现场模块 ECHO 为 5V，必须确认底板已有电平转换或增加 3.3V 保护
- `HiSparkEP_2025-02-10_pinout.md` 标注 PWM 控制器 U13 地址为 7-bit `0x15`，但当前实车电机层使用 `0x5A` 已验证可驱动；除非用 I2C scan 或官方电机协议确认，不要直接把 `dut_motor.c` 改成 `0x15`

## 7. 后续 AI 接手顺序

后续任何 AI 或开发者进入项目时，必须按以下顺序读取：

1. 本文件：`D:\aaa嵌入式比赛\docs\PROJECT_BASELINE_AND_WORKFLOW.md`
2. 协议：`D:\aaa嵌入式比赛\protocol_docs\protocol_v1.md`
3. SLE 映射草案：`D:\aaa嵌入式比赛\protocol_docs\sle_control_profile.md`
4. 上位机入口：`D:\aaa嵌入式比赛\upper_client\robot_client.py`
5. 自动化测试：`D:\aaa嵌入式比赛\tests`
6. 如需改实板行为，再进入 `D:\w\src\application\samples\peripheral\robot_mvp`

禁止跳过第 1 步直接修改代码。

## 8. 历史文档定位

- `docs\HARDWARE_NOTES.md`：按时间记录的硬件笔记，保留历史价值，但不是当前权威基线
- `docs\THREE_PERSON_MVP_PLAN.md`：最初的三人计划，保留项目背景，但不是当前执行清单
