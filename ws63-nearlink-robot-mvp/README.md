# WS63 星闪智能小车 MVP

本仓库用于复现 WS63/HiSparkEP 小车 MVP：串口 AT 控制、OLED、AHT20 温湿度、HC-SR04 超声波测距/避障、基础电机动作、SLE/星闪控制服务。

当前推荐使用 `COM5`，波特率 `115200`。如果 Windows 重新枚举了串口，请把命令里的 `COM5` 改成实际端口。

## 目录结构

```text
firmware/
  deveco_ws63_overlay/      # 覆盖到 HiSpark/DevEco/SDK 的板端代码
  fwpkg/                    # 已验证可烧录固件包
harmony_app/                # HarmonyOS/DevEco 手机端控制 App 源码与 HAP
upper_client/               # Python 串口/TCP 上位机
scripts/                    # 演示与验收脚本
simulator/                  # TCP 模拟器，用于 e2e 测试
tests/                      # Python 协议测试
protocol_docs/              # AT/SLE 协议说明
docs/                       # 工程交接、硬件注意事项、流程文档
references/                 # pinout 与硬件参考
tools/                      # overlay/构建/烧录辅助脚本
```

## 已验证功能状态

已验证：

- `T` / `AT+ROBOTST`：状态查询。
- `O` / `AT+ROBOTOLED`：OLED 初始化。
- `E` / `AT+ROBOTENV`：AHT20 温湿度。
- `D` / `AT+ROBOTOBS`：真实 HC-SR04 测距；已出现 `obs_valid=1`、`reason=0/ok`，近距离小于 `250mm` 时 `block=1`。
- `I/F/S`：电机初始化、前进短时运行、停止；车轮架空时已跑过 2 秒和 5 秒前进。
- Python 协议测试：`22 tests OK`。

待最终场地验收：

- 远距离连续 `D` 稳定 `block=0`。
- 车轮离地或安全场地中的 `I,D,A,S,T` 智能避障运动闭环。
- 手机/SLE central 真实连接 `ws63_robot_mvp` 后写入 `T/D/F/S`。

## HarmonyOS / DevEco 手机端 App

本仓库已经包含本地同款鸿蒙端控制 App：

```text
harmony_app\nearlink_robot_controller
```

已签名 HAP：

```text
harmony_app\install_hap\ws63_robot_controller_harmony_next_signed.hap
```

使用 DevEco Studio 打开：

```text
harmony_app\nearlink_robot_controller
```

更多说明见：

```text
harmony_app\README.md
harmony_app\nearlink_robot_controller\ROBOT_CONNECT_GUIDE.md
```

## 快速使用：直接烧录已打包固件

推荐固件：

```text
firmware\fwpkg\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg
```

PowerShell 烧录：

```powershell
.\tools\burn_ws63_app.ps1 -Port COM5
```

或手动执行：

```powershell
& "D:\HiSpark Studio 26.03.1\tools\BurnToolCmd\BurnToolCmd.exe" --burn -n ws63 -m serial COM5 --baudRate 115200 -f ".\firmware\fwpkg\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg"
```

注意：路径中有空格时，必须使用 `& "完整路径\BurnToolCmd.exe"`。

## 从 SDK/DevEco/HiSpark Studio 复现构建

1. 准备 HiSpark `fbb_bs2x` SDK，例如放在：

```text
D:\w\src
```

参考来源：

```text
https://gitee.com/HiSpark/fbb_bs2x
```

2. 覆盖本仓库的板端代码到 SDK：

```powershell
.\tools\apply_deveco_overlay.ps1 -SdkRoot D:\w\src
```

3. 构建：

```powershell
.\tools\build_ws63_liteos_app.ps1 -SdkRoot D:\w\src
```

4. 构建产物位置：

```text
D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
```

日常调试建议只烧录 `load_only.fwpkg`，不要把 `all.fwpkg` 当作常规调试包。

## 上位机环境

```powershell
python -m pip install -r requirements.txt
python -m unittest discover -s tests
```

## 常用验收命令

状态、OLED、温湿度、测距：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands T,O,E,D,D --interval 0.8 --timeout 4
```

安全全功能检测，不启动电机运动：

```powershell
python scripts\run_safe_full_check.py --serial-port COM5 --avoid-non-motion
```

只读超声波连续采样，不启动电机：

```powershell
python scripts\verify_ultrasonic_distance.py --serial-port COM5 --samples 12 --interval 0.5 --timeout 4
```

车轮架空后短时前进/停止：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands I,F,S --interval 0.4 --timeout 4
```

智能避障运动验收前必须满足：

- `D` 已稳定返回 `obs_valid=1`、`reason=0/ok`。
- 车轮离地或场地安全。
- 随时准备发送 `S` 或断电。

验收命令：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands I,D,A,S,T --interval 0.8 --timeout 4
```

## 硬件要点

超声波 CN1：

```text
CN1-1 VCC  -> 模块 VCC
CN1-2 TRIG -> 模块 Trig / GPIO_00
CN1-3 ECHO -> 模块 Echo / GPIO_01
CN1-4 GND  -> 模块 Gnd
```

当前购买记录显示模块支持 `DC3.3-5V`，可按 3.3V 兼容模块直接接 CN1。若换成仅 5V HC-SR04，ECHO 不能直连 WS63 GPIO，必须分压或电平转换。

更多细节见：

- `docs/FINAL_MVP_HANDOFF.md`
- `docs/HARDWARE_NOTES.md`
- `references/HiSparkEP_2025-02-10_pinout.md`
