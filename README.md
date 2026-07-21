星联队参加嵌入式芯片与系统设计竞赛开源代码

# WS63 星闪智能巡检机器人

本仓库用于复现比赛项目：基于 WS63/HiSparkEP 星闪控制、HarmonyOS 端侧路线识别、树莓派边缘视觉和 WAVE ROVER 四驱底盘的可导入路线智能巡检机器人。当前推荐路线已经升级为“手机手绘/选图，手机端识别并压缩路线，星闪分段下发 WS63，本地连续执行”。

图形路线导入的实际参数和协议见 `docs/GRAPHICAL_ROUTE_IMPORT.md`。总体架构见 `docs/ARCHITECTURE_DIRECTION.md`；新队员接手前必须先读 `docs/TEAM_ONBOARDING_MUST_READ.md`。旧文档中的 Wi-Fi 路线 JSON、`D:\w\src`、旧电机板直驱、R16/R17 焊接、H5 串口等内容只作为历史记录或兼容方案，不作为当前路线导入主线。

```text
HarmonyOS 手机/平板
  │ 星闪 SLE 控制 / 急停 / 接管
  │ 手绘 / 选图识别 -> 星闪分段路线
  ▼
WS63 开发板
  │ LiteOS 路线执行 / 安全感知 / Type-C 底盘命令
  ▼
树莓派
  │ AI Camera / 视频服务 / 串口安全桥 / 物联网日志
  │ 40PIN UART JSON，115200
  ▼
WAVE ROVER ESP32 驱动板
  │ JSON 指令
  ▼
四驱底盘电机
```

核心原则是“大数据走 Wi-Fi、确定性控制走星闪 SLE”。视频、AI 结果和日志仍走 Wi-Fi；手动控制、急停以及压缩后的路线段走星闪。手机端完成手绘或路线图识别，WS63 保存并执行最多 12 段路线，同时持续处理温湿度、超声波、蜂鸣器和 OLED 状态；树莓派保留边缘视觉、底盘串口桥和物联网日志能力。

## 当前优先方案：可导入路线智能巡检

最终决策：WS63 不直接焊接 WAVE ROVER，WS63 通过 Type-C USB 串口接树莓派；树莓派再通过 40PIN UART 控制 WAVE ROVER。当前使用树莓派安全仲裁桥作为默认运行时：星闪急停、环境异常、手动接管和超声波避障优先级高于自动路线巡航。

自动巡检只保留一个执行器：手机端 `P` 指令由 WS63 转发为树莓派的路线启动事件，树莓派负责已导入路线、固定左绕避障和回归路线；WS63 不再并行启动旧的本地巡检/右转避障状态机。固定绕障依次执行左、右、右、左四个路线标定的 90 度枢轴转向。

采用该方案的原因：

- 不再焊接 WAVE ROVER 或 WS63 上的 R16/R17 小焊盘，降低比赛前硬件损坏风险。
- 树莓派可以直接安装到 WAVE ROVER 40PIN 接口，当前 Raspberry Pi 5 实测 GPIO14/GPIO15 对应 `/dev/ttyAMA0` 控制底盘。
- WS63 通过 Type-C 数据线连接树莓派 USB，调试、固定、替换都更方便。
- WAVE ROVER 官方支持通过串口/USB/HTTP 下发 JSON 指令，适配成本低。

硬件连接：

```text
WS63 Type-C 数据线 -> 树莓派 USB-A

树莓派 GPIO14 / Pin 8  -> WAVE ROVER ESP32 RX
树莓派 GPIO15 / Pin 10 <- WAVE ROVER ESP32 TX
GND 共地
5V 供电
```

树莓派侧串口：

```text
/dev/ttyUSB0   # WS63 USB 串口，实际也可能是 /dev/ttyACM0
/dev/ttyAMA0   # Raspberry Pi 5 GPIO14/GPIO15 UART0，连接 WAVE ROVER ESP32
baudrate: 115200
```

如果树莓派还没有打开 40PIN 串口：

```bash
sudo raspi-config
# Interface Options -> Serial Port
# Login shell over serial? No
# Enable serial hardware? Yes
sudo reboot
```

安装串口库：

```bash
sudo apt update
sudo apt install -y python3-serial
```

最小桥接程序逻辑如下，后续可放入树莓派运行服务：

```python
import serial
import time

ws63 = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.02)
rover = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.02)

last_cmd_time = time.time()

while True:
    line = ws63.readline().strip()

    if line.startswith(b"{") and line.endswith(b"}"):
        rover.write(line + b"\n")
        last_cmd_time = time.time()

    # 300 ms 没有新控制命令就主动停车
    if time.time() - last_cmd_time > 0.3:
        rover.write(b'{"T":1,"L":0,"R":0}\n')
        last_cmd_time = time.time()
```

注意：WS63 的 USB 串口最好只输出底盘控制 JSON，避免普通日志混入。桥接程序会过滤非 JSON 行，但干净输出更利于现场调试。

## WAVE ROVER 官方 JSON 控制要点

根据 Waveshare WAVE ROVER 官方文档，推荐使用左右轮速度控制：

```json
{"T":1,"L":0.5,"R":0.5}
```

- `T=1`：左右轮速度控制 `CMD_SPEED_CTRL`。
- `L` 为左侧轮速度，`R` 为右侧轮速度。
- 范围为 `-0.5` 到 `0.5`，正值前进，负值后退。
- WAVE ROVER 无编码器，`0.5` 代表该侧电机 100% PWM，`0.25` 代表 50% PWM。

常用调试命令：

```json
{"T":1,"L":0,"R":0}
{"T":1,"L":0.2,"R":0.2}
{"T":1,"L":-0.2,"R":-0.2}
{"T":1,"L":-0.2,"R":0.2}
{"T":1,"L":0.2,"R":-0.2}
{"T":130}
{"T":131,"cmd":0}
{"T":143,"cmd":0}
```

`T=11` 的 PWM 直接控制只建议用于调试：

```json
{"T":11,"L":164,"R":164}
```

PWM 范围为 `-255` 到 `255`，低速时电机可能因为减速电机特性不转；正式运动控制优先使用 `T=1`。

## 分阶段推进步骤

1. 阶段 A：底盘安全闭环。确认 WAVE ROVER、树莓派串口桥、WS63 与手机 SLE 手动控制全通。
2. 阶段 B：固定方形巡航。完成连续四边巡航并把 90° 转向标定为 `0.686s`。
3. 阶段 C：图形路线导入。手机支持手绘和选图识别，路线压缩为最多 12 段，经 `R0/R1/R2` 星闪协议写入 WS63；源码与固件已完成，等待实机烧录验收。
4. 阶段 D：任意路线安全巡航。WS63 按每段独立前进/左转/右转参数连续执行，并复用超声波绕障和温湿度报警闭环。
5. 阶段 E：物联网巡检日志。树莓派记录路线、距离、温湿度、告警、急停、AI 事件，前端展示最近巡检记录。
6. 阶段 F：比赛演示闭环。展示 Wi-Fi 视频、手机端路线识别、星闪低时延下发、智能避障和环境报警。

## 已实现功能

- HarmonyOS 手机/平板端控制界面：星闪连接、手动控制、摄像头、手绘路线、相册选图、手机端路线识别、长度/速度/循环设置。
- WS63/LiteOS 固件：SLE 服务、最多 12 段路线的分包校验与本地执行、左右转、OLED、AHT20、HC-SR04、蜂鸣器/红灯报警和避障闭环。
- 树莓派服务：MJPEG 视频流、状态接口、安全仲裁器、地图大脑巡航、WAVE ROVER 控制；避障采用固定左绕箱形轨迹（左、右、右、左四个 90 度标定转向）并回归原巡检线。
- 比赛文档和关键代码提交包：已在 `artifacts/` 下生成。
- 树莓派路线导入 API 仍作为兼容模式保留；当前主线不依赖 Wi-Fi 即可把路线导入 WS63。

## 待迁移/待验证功能

- WS63 固件已新增 WAVE ROVER JSON 输出模块，运动入口会输出 `{"T":1,"L":...,"R":...}`。
- 已在短路径 SDK 副本 `D:\b\src` 完成 `ws63-liteos-app` 构建，生成新的 WAVE ROVER bridge 烧录包。
- 树莓派已新增透明串口桥脚本，并在实车上识别到 WS63 `/dev/ttyUSB0` 与 WAVE ROVER `/dev/ttyAMA0`。
- 安全仲裁服务 `nearlink-rover-arbiter.service` 已安装运行，旧透明桥保留为回退工具。
- 摄像头服务当前 8080 可访问，`/healthz` 已恢复为 200，手机端可查看完整画面。
- 后续重点转为手机端实机安装验证、超声波智能避障闭环、物联网日志和最终演示闭环。
- 现场录制前固定 WS63，建议用魔术贴、尼龙柱、扎带或上层板，避免 USB 线松动。

## 目录结构

```text
firmware/
  deveco_ws63_overlay/      # 覆盖到 HiSpark/DevEco/SDK 的 WS63 固件源码
  fwpkg/                    # 已验证可烧录固件包
harmony_app/                # HarmonyOS/DevEco 手机端控制 App 源码和 HAP
raspberry_pi/               # 树莓派摄像头、视觉和后续桥接运行目录
upper_client/               # Python 串口/TCP 上位机调试工具
scripts/                    # 演示与验收脚本
simulator/                  # TCP 模拟器和 e2e 测试辅助
tests/                      # Python 协议测试
protocol_docs/              # AT/SLE 协议说明
docs/                       # 工程交接、硬件注意事项、流程文档
references/                 # WS63、树莓派、WAVE ROVER 原理图和参考资料
tools/                      # overlay/构建/烧录辅助脚本
artifacts/                  # 文档、截图、提交包和中间产物
```

## WS63 构建复现

当前建议把 `D:\r\src` 当作相对干净的 SDK 底座保留，复制到短路径副本后再应用本仓库 overlay：

```powershell
robocopy D:\r\src D:\b\src /E /XD D:\r\src\output
.\tools\apply_deveco_overlay.ps1 -SdkRoot D:\b\src
.\tools\build_ws63_liteos_app.ps1 -SdkRoot D:\b\src
```

注意：复制时只排除顶层 `D:\r\src\output`，不要排除 SDK 内部的 `drivers\chips\ws63\rom_config\acore\output`，否则会缺少 ROM 链接脚本。

本次已构建成功并归档：

```text
firmware\fwpkg\ws63-liteos-app_ws63_sle_pi_bridge_load_only.fwpkg
SHA256: A6B78EFB29A5BDFC7BA10009DA6FD3DEA91FC2C9A552CB55DAABC68DFA9C07EC
```

## 固件烧录

当前推荐串口仍按现场环境使用 `COM5`，如 Windows 重新枚举，改成实际端口。

```powershell
.\tools\burn_ws63_app.ps1 -Port COM5
```

手动烧录示例：

```powershell
& "D:\HiSpark Studio 26.03.1\tools\BurnToolCmd\BurnToolCmd.exe" --burn -n ws63 -m serial COM5 --baudRate 115200 -f ".\firmware\fwpkg\ws63-liteos-app_ws63_sle_pi_bridge_load_only.fwpkg"
```

路径中有空格时必须使用 `& "完整路径\BurnToolCmd.exe"`。

## 树莓派摄像头服务

```bash
cd raspberry_pi
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-psutil
python3 run.py --host 0.0.0.0 --port 8080 --camera-backend auto --width 640 --height 480 --fps 15
```

常用地址：

```text
http://<树莓派IP>:8080/
http://<树莓派IP>:8080/driver
http://<树莓派IP>:8080/stream.mjpg
http://<树莓派IP>:8080/status
```

## 树莓派底盘桥接服务

先做只停车的硬件检查：

```bash
cd raspberry_pi
python3 check_rover_bridge.py --rover-port auto
```

小车架空或放在空旷区域后，再做低速短脉冲：

```bash
python3 check_rover_bridge.py --rover-port auto --move-test --speed 0.10 --duration 0.25
```

确认 WS63 USB 串口有 JSON 输出，但暂不转发到底盘：

```bash
python3 check_rover_bridge.py --ws63-listen --ws63-port auto --rover-port auto --listen-seconds 5
```

以上通过后再运行透明桥：

```bash
python3 rover_bridge.py --ws63-port auto --rover-port auto
```

如果自动识别不对，先查设备名：

```bash
ls -l /dev/serial0 /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null
```

指定端口运行：

```bash
python3 rover_bridge.py --ws63-port /dev/ttyUSB0 --rover-port /dev/ttyAMA0
```

验证稳定后安装为开机服务：

```bash
cd raspberry_pi
chmod +x start_rover_bridge.sh install_rover_bridge_service.sh
./install_rover_bridge_service.sh
```

常用服务命令：

```bash
sudo systemctl status nearlink-rover-bridge --no-pager
sudo systemctl restart nearlink-rover-bridge
sudo journalctl -u nearlink-rover-bridge -f
```

## HarmonyOS / DevEco App

```text
harmony_app/nearlink_robot_controller
harmony_app/install_hap/
```

使用 DevEco Studio 打开 `harmony_app/nearlink_robot_controller`。如使用命令行构建，路径尽量使用纯英文目录。

## 关键资料来源

- ChatGPT 分享页最新方案：<https://chatgpt.com/share/6a58a6fd-d9f8-83e8-960d-b387dd26b533>
- Waveshare WAVE ROVER 文档：<https://www.waveshare.net/wiki/WAVE_ROVER>
- Waveshare HTTP 示例：<https://www.waveshare.net/w/upload/2/21/Http_simple_ctrl.zip>
- Waveshare 串口示例：<https://www.waveshare.net/w/upload/c/c9/Serial_simple_ctrl.zip>
- Waveshare 树莓派上位机仓库：<https://github.com/waveshareteam/ugv_rpi>
- WAVE ROVER 驱动板原理图：`references/WaveRover_General_Driver_for_Robots.pdf`
- HiSparkEP 原理图与引脚速查：`references/HiSparkEP_2025-02-10.pdf`、`references/HiSparkEP_2025-02-10_pinout.md`
- 树莓派原理图：`references/Raspberry-Pi-Schematics-R1.0.pdf`

## 重要注意

- 不建议比赛前继续直接焊 R16/R17 或把粗杜邦线焊到贴片焊盘上。
- H5 扩展排针可用于取 GND/3.3V，但不建议把 H5-1/H5-2 当作 WAVE ROVER 稳定 UART。
- 若后续必须 WS63 直连 WAVE ROVER，应使用细漆包线、小转接板和固定胶，避免车辆震动拉掉焊盘。
- 自动巡检测试必须在空旷场地低速开始，随时准备发送停车命令或断电。
