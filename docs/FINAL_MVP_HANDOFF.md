# WS63 星闪小车 MVP 最终交接与开发流程

更新时间：2026-07-06  
当前实板串口：`COM5`  
当前主通信链路：`AT over UART0`，115200 8N1  
当前固件主线源码：`D:\w\src\application\samples\peripheral\robot_mvp`

> 2026-07-17 更新：本文保留旧 MVP 交接记录。当前比赛实现主线已升级为“WS63 星闪安全控制 + 树莓派 AI Camera 轨迹闭环 + WAVE ROVER 底盘执行”的工业巡检机器人。`WS63 Type-C -> 树莓派 USB -> WAVE ROVER 40PIN UART` 是物理底座，不是最终功能边界。详见 `docs/ARCHITECTURE_DIRECTION.md` 和 `docs/FINAL_GOAL_ROADMAP.md`。后续不要再优先推进 R16/R17 焊接、H5 串口或旧电机板直驱路线。

## 1. 当前结论

MVP 的通信、状态查询、电机控制、OLED、温湿度、SLE 服务、真实超声波避障代码链路都已经接入。当前最新板端固件已经能响应：

```text
ROBOT:STATE,...
+ROBOT:OBS,...
```

2026-07-06 现场复查时，曾出现 `AT+ROBOTST` 返回 `ERROR`。根因不是 Python 客户端，也不是 COM5，而是板子处于 factory/radar 启动模式。执行下面命令后已切回 robot app：

```text
AT+FTM=0
AT+RST
```

切回后，`AT+SYSINFO` 中已经出现：

```text
RobotSle
RobotMvp
```

当前唯一未验收通过的是“真实超声波距离有效值”。旧固件曾读到 `GPIO_01/ECHO` 常高；2026-07-06 更新 ECHO 下拉固件并烧录后，`AT+GETIOMODE=1` 已显示下拉生效，`D` 当前变为 `reason=3/no_echo_rise`。这说明板端命令链路已打通，ECHO 不再被输入脚悬空直接判成常高，但超声波模块仍没有响应 TRIG；优先排查模块直插虚接、脚位顺序、供电和 ECHO 电平转换。

2026-07-06 已新增超声波诊断字段并重新构建归档，且已通过 BurnTool 成功烧录到 COM5 板子。新固件上的 `D` / `A` 会额外返回 `reason`，用于区分 `ECHO` 空闲卡高、无回波、回波不落下等原因。

## 2. 当前功能状态

| 功能 | 上位机命令 | 板端 AT | 状态 |
|---|---|---|---|
| 电机初始化 | `I` | `AT+ROBOTMI` | 已实现，历史实车已验证 |
| 前进 | `F` | `AT+ROBOTF` | 已实现，历史实车已验证 |
| 后退 | `B` | `AT+ROBOTB` | 已实现，历史实车已验证 |
| 左转 | `L` | `AT+ROBOTL` | 已实现，历史实车已验证 |
| 右转 | `R` | `AT+ROBOTR` | 已实现，历史实车已验证 |
| 停止 | `S` | `AT+ROBOTS` | 已实现 |
| OLED | `O` | `AT+ROBOTOLED` | 已实现 |
| AHT20 温湿度 | `E` | `AT+ROBOTENV` | 已实现并实测返回真实温湿度 |
| 状态遥测 | `T` | `AT+ROBOTST` | 当前已恢复可用 |
| 超声波距离 | `D` | `AT+ROBOTOBS` | 固件链路可用；当前 `reason=3/no_echo_rise`，模块未响应 TRIG |
| 智能避障 | `A` | `AT+ROBOTAVOID` | 代码已接真实超声波；需先让 `D` 稳定有效后再做运动验收 |
| SLE/星闪服务 | SLE 写入 `F/B/L/R/S/I/O/E/D/A/T` | service `0x7100` | 板端服务已接入；后续需 central 实测 |

注意：旧模拟避障命令 `X` / `AT+ROBOTAVOIDTEST` 已删除，不要再使用。

## 3. 当前硬件重点：超声波

参考 `D:\WS63_NearLink_Robot_MVP\HiSparkEP_2025-02-10_pinout.md`：

| CN1 | 信号 | 连接 |
|---|---|---|
| CN1-1 | VCC | 电源 |
| CN1-2 | TRIG | WS63E `GPIO_00` |
| CN1-3 | ECHO | WS63E `GPIO_01` |
| CN1-4 | GND | 地 |

当前代码配置：

```c
#define ROBOT_OBSTACLE_TRIG_PIN 0
#define ROBOT_OBSTACLE_ECHO_PIN 1
#define ROBOT_OBSTACLE_THRESHOLD_MM 250U
```

新固件 `reason` 诊断映射：

| reason | 名称 | 含义 |
|---:|---|---|
| 0 | `ok` | 本次测距有效 |
| 1 | `not_ready` | 超声波 GPIO 初始化未就绪 |
| 2 | `echo_idle_high` | 触发前 ECHO 已经常高 |
| 3 | `no_echo_rise` | TRIG 后未等到 ECHO 上升沿 |
| 4 | `no_echo_fall` | ECHO 上升后未等到下降沿 |
| 5 | `invalid_pulse` | 回波时间戳异常 |

2026-07-06 旧固件/旧状态实测 GPIO：

```text
AT+GETIOMODE=0 -> +GETIOMODE:0,0,0,0
AT+GETIOMODE=1 -> +GETIOMODE:1,0,0,0
AT+RDGPIO=0    -> +RDGPIO:0,1,0
AT+RDGPIO=1    -> +RDGPIO:1,0,1
```

2026-07-06 ECHO 下拉增强版固件重新烧录并 reset 后，最新 GPIO/测距结果：

```text
AT+GETIOMODE=1 -> +GETIOMODE:1,0,1,0
AT+ROBOTOBS    -> +ROBOT:OBS,1,0,0,0,250,3
```

含义：ECHO 输入下拉已经生效，当前不再是空闲常高，而是 `no_echo_rise`：TRIG 发出后没有等到 ECHO 上升沿。HC-SR04 正常测距时 ECHO 应该在收到 TRIG 后输出一个脉宽信号，所以现在优先排查：

1. 模块 `VCC/TRIG/ECHO/GND` 是否和 CN1-1/2/3/4 一一对应。
2. 模块是否松动直插导致 VCC/TRIG/ECHO/GND 任一脚虚接。
3. TRIG 是否真的接到模块 TRIG，ECHO 是否真的接到模块 ECHO。
4. 普通 5V HC-SR04 的 ECHO 是否已经做 3.3V 分压/电平转换。
5. 如果模块只支持 5V 供电，而 CN1-1 实际是 3.3V，需换 3.3V 兼容模块或按硬件规范供电并保护 ECHO。
6. 现场照片显示超声波模块为松动直插。该孔位/排针若没有真正母座夹持或焊接，不能当作可靠电气连接；虚接会直接导致 `echo_idle_high`、`no_echo_rise` 或距离跳变。工程验收前应改为：焊接 1x4 母座、使用可靠杜邦线逐针连接，或临时用手压紧后对比 `D` 读数。
7. 2026-07-06 正面照片可见模块丝印顺序为 `VCC / Trig / Echo / Gnd`。购买记录标注该模块工作电压为 `DC3.3-5V`，因此当前不应优先怀疑“3.3V 不兼容”；若仍 `no_echo_rise`，优先确认模块端实际 VCC-GND 电压、四针是否真实接触、CN1 物理 pin1 方向是否与模块方向一致。

供电与电平结论：

- `HiSparkEP_2025-02-10_pinout.md` 明确：`VCC` 是板上 3.3V；`+5V` 主要来自 Type-C/外部 5V；`VBAT/VBATS` 是单节锂电池端，板上再经电源路径生成 `VBUS` 和 3.3V `VCC`。
- 当前购买记录显示模块支持 `DC3.3-5V`，优先按 3.3V 兼容模块处理：直接接 CN1，`VCC/TRIG/ECHO/GND` 对应 `CN1-1/2/3/4`。
- 如果后续改用仅 5V 供电的 HC-SR04：模块 `VCC` 必须接板上明确的 `+5V` 或外部 5V，模块 `GND` 必须和板子 `GND` 共地，模块 `TRIG` 可接 `GPIO_00`，但模块 `ECHO` 不能直连 `GPIO_01`。
- 仅 5V 供电模块的 ECHO 推荐用分压后进 `GPIO_01`：`ECHO -- 10kΩ -- GPIO_01 -- 20kΩ -- GND`，把约 5V 降到约 3.3V。
- 最终脱离电脑 USB、只用 3.7V/18650 电池时，若使用当前 `DC3.3-5V` 模块，可继续由板上 3.3V `VCC` 供电；若换成仅 5V 模块，才需要升压到 5V。

在 `D` 仍为 `obs_valid=0` 之前，不要启动 `I,A` 做带轮运动验收；如果必须测试自动避障状态机，先架空车轮并准备 `S` 或断电。

## 4. 本地工程结构

| 路径 | 角色 | 是否主线 |
|---|---|---|
| `D:\w\src\application\samples\peripheral\robot_mvp` | WS63 板端 robot MVP 固件源码 | 是 |
| `D:\w\src\application\ws63\ws63_liteos_application\main.c` | LiteOS app 启动入口，已直连 robot app 注册/启动 | 是 |
| `D:\w\src\output\ws63\acore\ws63-liteos-app` | 编译产物：elf/bin/map | 是 |
| `D:\w\src\output\ws63\fwpkg\ws63-liteos-app` | SDK 打包产物 | 是 |
| `D:\WS63_NearLink_Robot_MVP\upper_client` | Python 上位机串口/TCP 客户端 | 是 |
| `D:\WS63_NearLink_Robot_MVP\tests` | Python 自动化测试 | 是 |
| `D:\WS63_NearLink_Robot_MVP\protocol_docs` | 协议文档 | 是 |
| `D:\WS63_NearLink_Robot_MVP\docs` | 工程交接、硬件和流程文档 | 是 |
| `D:\WS63_NearLink_Robot_MVP\artifacts` | 最终保留产物 | 是 |
| `D:\WS63_NearLink_Robot_MVP\ws63_liteos` | 早期参考代码镜像 | 否 |
| `D:\WS63_NearLink_Robot_MVP\deliverables` | 历史交付包 | 否，保留追溯 |
| `D:\WS63_NearLink_Robot_MVP\github_repo` | GitHub/Gitee 参考仓库整理 | 否，保留参考 |

后续修改规则：

- 改板端行为，只改 `D:\w\src\application\samples\peripheral\robot_mvp`，必要时改 `main.c` 启动入口。
- 改上位机串口工具，只改 `D:\WS63_NearLink_Robot_MVP\upper_client`、`scripts`、`tests`。
- 改协议，先改 `D:\WS63_NearLink_Robot_MVP\protocol_docs\protocol_v1.md`。
- 不要把 `ws63_liteos` 当作当前烧录主线。

## 5. 构建与烧录流程

构建前如果找不到 `ccache`：

```powershell
$env:PATH='D:\HiSpark Studio 26.03.1\tools\cfbb\thirdparty\ccache;' + $env:PATH
```

构建：

```powershell
cd D:\w\src
python build.py ws63-liteos-app
```

打包：

```powershell
python D:\w\src\tools\pkg\packet.py ws63 ws63-liteos-app ' '
```

推荐烧录包：

```text
D:\WS63_NearLink_Robot_MVP\artifacts\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg
```

该包已由当前 SDK 输出 `D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg` 覆盖归档。

当前归档包信息：`1402260` 字节，LastWriteTime `2026-07-06 10:01:25`，包含 `reason` 诊断字段和 ECHO 输入下拉配置。

PowerShell 烧录命令必须给带空格的 HiSpark Studio 路径加引号和调用符 `&`：

```powershell
& "D:\HiSpark Studio 26.03.1\tools\BurnToolCmd\BurnToolCmd.exe" --burn -n ws63 -m serial COM5 --baudRate 115200 -f "D:\WS63_NearLink_Robot_MVP\artifacts\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg"
```

烧录注意：

- 当前常规只烧 `load_only.fwpkg`。
- 不把 `all.fwpkg` 作为日常调试包。
- BurnTool 提示 reset 时，按板子 `RESET`。
- 这块板没有 BOOT 键，不要等待 BOOT 操作。
- 如果烧录后 `AT+ROBOTST` 返回 `ERROR`，先检查是否进入 factory/radar 模式，按第 6 节恢复。

## 6. factory/radar 模式恢复流程

如果出现：

```text
AT+ROBOTST
ERROR
```

并且：

```text
AT+SYSINFO
SDK Version:1.10.T33
task_info 中有 radar_driver / radar_feature，但没有 RobotMvp
```

说明板子仍在 factory/radar 镜像。恢复命令：

```text
AT+FTM=0
AT+RST
```

恢复成功后应看到：

```text
AT+SYSINFO
RobotSle
RobotMvp
```

并且：

```text
AT+ROBOTST
+ROBOT:STATE,...
OK
```

不要再用裸 `AT+MFGFLAG` 试图切换模式；源码确认它不是退出 factory 的命令。

## 7. 上位机验证命令

先跑软件测试：

```powershell
python -m unittest discover -s D:\WS63_NearLink_Robot_MVP\tests -v
```

当前结果：`22 tests OK`。

2026-07-06 已增强 `upper_client.robot_client`：当烧录/reset 后 robot 自定义 AT 表尚未注册、板端短暂返回裸 `ERROR` 时，会在超时时间内重试；若最终仍失败，会输出可读诊断，而不是直接抛 Python traceback。

检查 robot app 是否在线：

```powershell
cd D:\WS63_NearLink_Robot_MVP
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands T,D --interval 0.8 --timeout 3
```

当前现场输出特征：

```text
cmd=T ... status=OK ... ready=0 ...
cmd=D ... status=OK ... obs_valid=0 block=0 dist=0mm threshold=250mm reason=3/no_echo_rise
```

这表示 robot AT 链路已通，但超声波模块没有对 TRIG 产生有效 ECHO 脉冲。

如果看到 `reason=2/echo_idle_high`，优先排查 ECHO 是否接到 VCC、模块是否插反、ECHO 5V 是否未降压；如果看到当前的 `reason=3/no_echo_rise`，优先排查模块是否未供电、TRIG/ECHO 没接牢或直插虚接。

2026-07-06 ECHO 下拉增强版固件实烧后的当前结果：

```text
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands T,D,D,A,S --interval 0.8 --timeout 4

cmd=T status=OK moving=0 ready=0 ...
cmd=D status=OK moving=0 obs_valid=0 block=0 dist=0mm threshold=250mm reason=3/no_echo_rise
cmd=D status=OK moving=0 obs_valid=0 block=0 dist=0mm threshold=250mm reason=3/no_echo_rise
cmd=A status=MOTOR_ERROR moving=0 obs_valid=0 block=0 dist=0mm threshold=250mm reason=3/no_echo_rise avoid=0 phase=0
cmd=S status=OK moving=0
```

智能避障 fail-safe 验证：

```text
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands I,A,S,T --interval 0.8 --timeout 4

cmd=I status=OK moving=0
cmd=A status=OBSTACLE_STOP moving=0 obs_valid=0 block=0 dist=0mm threshold=250mm reason=2/echo_idle_high avoid=0 phase=4
cmd=S status=OK moving=0
cmd=T status=OK moving=0 ready=1
```

结论：新版固件已烧录并运行；ECHO 下拉已生效。剩余硬件门槛是让模块对 TRIG 产生 ECHO 脉冲，并让 `D` 返回 `obs_valid=1`。

确认超声波修好后的验收顺序：

非运动连续采样脚本：

```powershell
python scripts\verify_ultrasonic_distance.py --serial-port COM5 --samples 10 --interval 0.5 --timeout 4
```

2026-07-06 当前接好模块后的结果：

```text
reasons=no_echo_rise:5
valid_distance_mm=none
```

2026-07-06 改用杜邦线并恢复 COM5 后再次采样：

```text
reasons=no_echo_rise:10
valid_distance_mm=none
```

含义：板端 TRIG 输出已验证可翻转，ECHO 输入下拉也生效，但模块仍没有产生 ECHO 上升沿。直插松动不是唯一原因；当前优先核查模块端 VCC-GND 实测电压、CN1 物理 pin1 方向、TRIG/ECHO 是否交叉。修复后此脚本应出现 `valid=1` 和实际 `distance_mm`。

基础命令验收：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands D,D,D --interval 0.8 --timeout 3
```

预期：

- 空旷或大于 30cm：`obs_valid=1`，`block=0`，`dist` 为实际距离附近。
- 前方 15-20cm 放反射面：`obs_valid=1`，`block=1`，`dist < 250mm`。

只有 `D` 稳定有效后，才进入智能避障运动验收：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands I,D,A,S,T --interval 0.8 --timeout 3
```

首次运动测试必须车轮离地或在安全场地，随时准备 `S` 或断电。

2026-07-07 安全全功能检测结果：

```text
T: status=OK, ready=0, moving=0
O: status=OK, OLED 初始化成功
E: status=OK, temp=30.5C, hum=63.9%
D: status=OK, dist=0mm, reason=4/no_echo_fall 连续 3 次
A: 在 motor_not_ready 状态下做非运动路径检测，status=MOTOR_ERROR, moving=0, reason=4/no_echo_fall
S: status=OK, moving=0
```

随后只读连续采样又回到：

```text
reasons=no_echo_rise:10
valid_distance_mm=none
```

结论：通信、OLED、温湿度、状态、停止命令正常；超声波链路出现过 ECHO 上升但不稳定，随后又退回无上升沿。当前不能做带轮 `I,A` 自动避障验收，优先检查杜邦线接触、模块端 VCC-GND 电压、TRIG/ECHO 是否交叉，并在探头前 20-50cm 放平整反射物后复测。

2026-07-07 重新摆放反射物/调整接线后，超声波测距已通过：

```text
python scripts\verify_ultrasonic_distance.py --serial-port COM5 --samples 12 --interval 0.5 --timeout 4

reasons=no_echo_fall:3, ok:9
valid_distance_mm=min:67 max:460 last:68
```

二次确认：

```text
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands D,D,D,D,D --interval 0.5 --timeout 4

cmd=D status=OBSTACLE_STOP obs_valid=1 block=1 dist=70mm threshold=250mm reason=0/ok
cmd=D status=OBSTACLE_STOP obs_valid=1 block=1 dist=54mm threshold=250mm reason=0/ok
cmd=D status=OBSTACLE_STOP obs_valid=1 block=1 dist=57mm threshold=250mm reason=0/ok
cmd=D status=OBSTACLE_STOP obs_valid=1 block=1 dist=39mm threshold=250mm reason=0/ok
cmd=D status=OBSTACLE_STOP obs_valid=1 block=1 dist=61mm threshold=250mm reason=0/ok
```

结论：真实 HC-SR04 测距链路已打通，250mm 阈值内 `block=1` 正常。剩余验收项是车轮离地或安全场地内的 `I,D,A,S,T` 智能避障运动验收。

## 8. 清理结果

`D:\WS63_NearLink_Robot_MVP\artifacts` 当前只保留：

```text
mvp_env_full_after_reset.csv
ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg
```

已删除过渡旧包：

```text
ws63-liteos-app_mvp_env_load_only.fwpkg
ws63-liteos-app_mvp_obstacle_pinout_load_only.fwpkg
ws63-liteos-app_mvp_smart_obstacle_known_loader_load_only.fwpkg
ws63-liteos-app_mvp_sle_bridge_sdk_load_only_boot_verified.fwpkg
```

没有删除 `deliverables`、`github_repo`、`ws63_liteos`，因为它们是历史追溯/参考材料，不属于当前推荐烧录产物。

## 9. 后续工程目标

下一阶段不要再重做通信/电机/温湿度基础链路，直接推进：

1. 已完成 HC-SR04 真实测距基本验收：`D` 可返回 `obs_valid=1`、`reason=0/ok`，近距离 `block=1`。
2. 补做远距离稳定性记录：保持障碍物大于 30cm，确认连续 `D` 为 `block=0`。
3. 在车轮离地或安全场地完成 `A` 智能避障运动验收。
4. 用支持 SLE central 的设备连接 `ws63_robot_mvp`，写入 `T/D/F/S` 做星闪链路实测。
5. 若智能避障稳定，再做最终产品形态：电池供电、手机/SLE 控制、脱离电脑 USB 运行。
