# 队员必读：星闪小车接手与环境配置清单

> 更新时间：2026-07-17
> 目标：让新队员能在自己的电脑/手机上构建、安装并使用 HarmonyOS 星闪 App 控制当前小车，同时能判断树莓派、WS63 固件和星闪服务是否处于可用状态。

## 0. 当前系统状态

当前主线不是旧的电机板直驱，也不是只沿黄色线循迹。当前比赛实现主线是：

```text
HarmonyOS 手机/平板
  - 星闪 SLE：手动控制、急停、接管、状态查询
  - Wi-Fi/HTTP：树莓派摄像头、路线导入、路线状态

WS63 开发板
  - 广播名：ws63_robot_mvp
  - SLE service：0x7100
  - command characteristic：0x7101
  - response characteristic：0x7102
  - Type-C USB 串口连接树莓派

树莓派 192.168.43.42
  - 摄像头服务：8080，nearlink-pi-camera
  - 安全仲裁/路线服务：8090，nearlink-rover-arbiter
  - 40PIN UART 控制 WAVE ROVER

WAVE ROVER
  - 通过串口 JSON 接收左右轮速度
```

当前 App 默认地址：

```text
摄像头：http://192.168.43.42:8080
路线服务：http://192.168.43.42:8090
星闪设备名：ws63_robot_mvp
```

## 1. 新队员先做这 8 件事

1. 确认自己的手机/平板支持星闪/NearLink，并在系统设置里打开星闪。
2. 安装 DevEco Studio，并登录项目负责人指定的华为账号。
3. 打开 `harmony_app/nearlink_robot_controller`，刷新调试签名。
4. 使用 HarmonyOS 5.0.1(13) 或更高 SDK 构建 HAP。
5. 安装 HAP 到手机/平板，授权 `ACCESS_NEARLINK` 和网络访问。
6. 确认树莓派 `192.168.43.42` 在线，`8080/status` 和 `8090/status` 返回 HTTP 200。
7. 确认 WS63 已烧录星闪固件，串口日志出现 SLE 广播成功。
8. App 扫描 `ws63_robot_mvp`，连接后先发 `T`/`I`/`S`，不要一上来发前进。

## 2. DevEco / HarmonyOS App 环境

### 2.1 DevEco 与 SDK

本机已验证的 DevEco 路径：

```text
D:\devecostudio-windows-6.1.1.290\DevEco Studio
```

本机已验证的 HarmonyOS SDK 路径：

```text
D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk
```

工程配置里的 SDK 版本：

```text
compatibleSdkVersion = 5.0.1(13)
targetSdkVersion     = 5.0.1(13)
runtimeOS            = HarmonyOS
```

最低要求：DevEco Studio 5.0.1 Release 及以上，HarmonyOS SDK 5.0.1 Release 及以上。为了减少版本差异，队员优先使用上面的 DevEco 6.1.1.290 + SDK 路径。

### 2.2 路径要求

Harmony 工程路径可以在本仓库内：

```text
D:\WS63_NearLink_Robot_MVP\harmony_app\nearlink_robot_controller
```

注意：

- 不要把 Harmony 工程复制到带中文、特殊符号或过长的路径里构建。
- 如果命令行构建报路径或 hvigor 异常，复制到纯英文短路径再试，例如 `D:\harmony_build\nearlink_robot_controller`。
- 不要手动改 `build-profile.json5` 里的签名材料路径来引用别人的 `C:\Users\<其他用户>\.ohos` 文件。

### 2.3 华为账号与签名

队员必须在 DevEco Studio 里登录项目负责人指定的华为账号。原因：

- HarmonyOS 真机安装需要有效调试签名。
- 本仓库里的 `build-profile.json5` 可能记录某台电脑上的本地签名路径，例如 `C:\Users\<当前用户>\.ohos\config\...`，这类文件是机器本地材料，不适合复制给别人。
- 旧调试证书曾在 2026-07-15 14:20:20 CST 过期，已经在本机重新自动生成过；其他电脑仍需要自己刷新。

刷新签名路径：

```text
DevEco Studio -> File/文件 -> Project Structure/项目结构 -> Signing Configs/签名配置
```

建议操作：

1. 勾选或点击“自动生成签名文件”。
2. 如果弹出华为登录/授权页面，由队员本人或负责人登录确认。
3. 确认生成新的 `.cer`、`.p12`、`.p7b` 到当前用户的 `.ohos\config` 目录。
4. 不要把账号密码、`.p12`、`.p7b`、`.cer` 上传到公开仓库。

### 2.4 命令行构建 App

在 PowerShell 中：

```powershell
cd D:\WS63_NearLink_Robot_MVP\harmony_app\nearlink_robot_controller
$env:DEVECO_SDK_HOME = 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk'
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\tools\hvigor\bin\hvigorw.bat' --mode module -p module=entry assembleHap --no-daemon
```

成功输出应包含：

```text
BUILD SUCCESSFUL
entry-default-signed.hap
```

HAP 输出：

```text
harmony_app\nearlink_robot_controller\entry\build\default\outputs\default\entry-default-signed.hap
```

### 2.5 安装到手机/平板

确认设备已开启开发者模式、USB 调试/HDC 调试：

```powershell
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk\default\openharmony\toolchains\hdc.exe' list targets
```

安装：

```powershell
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk\default\openharmony\toolchains\hdc.exe' install -r 'D:\WS63_NearLink_Robot_MVP\harmony_app\nearlink_robot_controller\entry\build\default\outputs\default\entry-default-signed.hap'
```

启动：

```powershell
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk\default\openharmony\toolchains\hdc.exe' shell aa start -b com.example.nearlink -a EntryAbility
```

## 3. 手机/平板侧必须检查

设备要求：

- 必须支持星闪/NearLink。普通蓝牙设备不等价。
- 系统设置中必须开启星闪。
- App 首次运行时要允许星闪权限。
- 手机/平板必须连到小车同一 Wi-Fi/热点网段，当前树莓派 IP 是 `192.168.43.42`。

常见星闪入口，不同系统版本名称可能不同：

```text
设置 -> 星闪和蓝牙 -> 星闪
设置 -> 多设备协同 -> 星闪
设置 -> 更多连接 -> 星闪
```

App 内验收顺序：

1. 进入“星闪小车控制/驾驶台”。
2. 摄像头卡片应显示 `http://192.168.43.42:8080/stream.mjpg`。
3. 摄像头状态应显示 `v4l2`、`FPS 15.0`、帧龄、温度等。
4. 点击扫描，等待 `ws63_robot_mvp` 出现。
5. 连接后先点击 `状态 T`。
6. 再点击 `初始化 I`。
7. 最后点击 `急停 S` 或 `停止 S` 确认停止链路正常。
8. 车轮离地后再测试 `F/B/L/R`。

## 4. WS63 固件 SDK 与路径规则

### 4.1 使用哪个 SDK

当前队员统一使用：

```text
干净 SDK 底座：D:\r\src
构建用短路径副本：D:\b\src
```

历史文档里可能出现 `D:\w\src`、`D:\fbb_ws63\src`、`D:\HiSparkSDK\fbb_ws63\src`。这些是历史路径，不建议新队员继续混用。新队员统一按 `D:\r\src -> D:\b\src` 复现。

### 4.2 为什么必须短路径

Windows 下 WS63 SDK 构建容易遇到命令行过长问题：

```text
ninja: fatal: CreateProcess: The parameter is incorrect.
```

所以：

- SDK 构建路径必须短。
- 路径尽量纯英文。
- 不要放到桌面、下载目录、中文目录、OneDrive 同步目录。
- 不要直接在干净底座 `D:\r\src` 上覆盖 overlay，保留它作为回滚源。

### 4.3 复制 SDK

```powershell
robocopy D:\r\src D:\b\src /E /XD D:\r\src\output
```

只排除顶层 `D:\r\src\output`。不要排除 SDK 内部的：

```text
drivers\chips\ws63\rom_config\acore\output
```

否则会缺少 ROM 链接脚本。

### 4.4 应用本仓库 overlay 并构建

```powershell
cd D:\WS63_NearLink_Robot_MVP
.\tools\apply_deveco_overlay.ps1 -SdkRoot D:\b\src
.\tools\build_ws63_liteos_app.ps1 -SdkRoot D:\b\src
```

构建输出：

```text
D:\b\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
```

当前仓库已归档推荐包：

```text
firmware\fwpkg\ws63-liteos-app_wave_rover_bridge_load_only.fwpkg
```

### 4.5 Python 版本

如果构建报：

```text
ModuleNotFoundError: No module named 'distutils'
```

说明 Python 版本可能是 3.12。该 SDK 的 `build.py` 需要 `distutils`，建议使用 Python 3.11，例如：

```powershell
D:\Python\python.exe D:\b\src\build.py ws63-liteos-app
```

## 5. WS63 烧录与 ST/星闪注册检查

### 5.1 烧录串口

当前现场常用烧录口：

```text
COM5 = USB-SERIAL CH340
```

先检查：

```powershell
Get-PnpDevice -PresentOnly -Class Ports
[System.IO.Ports.SerialPort]::GetPortNames() -join ' '
mode COM5
```

不要误用蓝牙串口。历史上 `COM3/COM4/COM10/COM11/COM12/COM13` 多数不是小车烧录口。

### 5.2 烧录命令

默认使用仓库推荐包：

```powershell
cd D:\WS63_NearLink_Robot_MVP
.\tools\burn_ws63_app.ps1 -Port COM5
```

指定包：

```powershell
.\tools\burn_ws63_app.ps1 -Port COM5 -Package 'D:\WS63_NearLink_Robot_MVP\firmware\fwpkg\ws63-liteos-app_wave_rover_bridge_load_only.fwpkg'
```

如果 BurnTool 提示：

```text
Please reset the device
```

按 WS63 开发板 Reset，或重新上电。

### 5.3 ST 注册是否存在

这里的“ST 注册”请理解为机器人自定义状态命令是否注册成功，重点检查：

```text
AT+ROBOTST
```

快速验证：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands T,I,S --timeout 6
```

期望看到：

```text
cmd=T status=OK
cmd=I status=OK
cmd=S status=OK moving=0
```

如果 `AT+ROBOTST` 返回 `ERROR`，通常不是 App 问题，而是固件 overlay 没真正进 SDK，或板子仍在旧 factory/radar 模式。先检查 SDK 中是否存在：

```powershell
rg -n "CONFIG_SAMPLE_SUPPORT_ROBOT_MVP|ROBOT MAIN|robot_mvp_entry" D:\b\src
```

还要检查启动日志是否有：

```text
ROBOT MAIN app entry
ROBOT MAIN at register
ROBOT_MVP READY protocol=at/sle
```

### 5.4 星闪 SLE 服务注册是否存在

串口启动日志应看到：

```text
ROBOT SLE start requested name=ws63_robot_mvp service=0x7100
ROBOT SLE enable status=0x0
ROBOT SLE add service ret=0x0
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
ROBOT SLE announce enable
```

如果没有 `adv ret=0x0`：

- 按 WS63 Reset，等待自动星闪初始化。
- 不要把 `AT+SLEENABLE` 当作常规启动步骤。
- 如果日志出现 `ERRCODE_AT_CHANNEL_BUSY`，通常是时序错误，先物理复位。

## 6. 树莓派运行环境

当前树莓派：

```text
IP：192.168.43.42
用户：xinglian
密码：不要写进公开仓库，现场向负责人确认
项目路径：/home/xinglian/raspberry_pi
```

不要把弱口令写入公开提交。队员现场需要 SSH 时，由负责人单独提供。

### 6.1 摄像头服务

服务名：

```text
nearlink-pi-camera
```

检查：

```bash
sudo systemctl status nearlink-pi-camera --no-pager
curl http://127.0.0.1:8080/status
curl http://127.0.0.1:8080/healthz
```

PC/手机访问：

```text
http://192.168.43.42:8080/
http://192.168.43.42:8080/driver
http://192.168.43.42:8080/stream.mjpg
http://192.168.43.42:8080/status
```

期望：

```text
camera_backend = v4l2 或 picamera2
frame_ready = true
fps_actual ≈ 15
```

重启：

```bash
sudo systemctl restart nearlink-pi-camera
sudo journalctl -u nearlink-pi-camera -f
```

### 6.2 安全仲裁/路线服务

服务名：

```text
nearlink-rover-arbiter
```

检查：

```bash
sudo systemctl status nearlink-rover-arbiter --no-pager
curl http://127.0.0.1:8090/status
curl http://127.0.0.1:8090/route/current
```

PC/手机访问：

```text
http://192.168.43.42:8090/status
http://192.168.43.42:8090/route/current
```

当前路线基线：

```text
路线名：industrial-square-patrol-v1
直行速度：0.24
右转速度：0.5
右转时间：0.8s
默认只跑一圈
```

急停：

```bash
curl http://127.0.0.1:8090/estop
```

解除急停：

```bash
curl http://127.0.0.1:8090/release
```

停止自动路线：

```bash
curl http://127.0.0.1:8090/auto/stop
```

## 7. App 控制小车的最终验收步骤

### 7.1 上电前

1. 小车架空或放在空旷地面。
2. 树莓派、WS63、WAVE ROVER 供电稳定。
3. 手机和树莓派在同一热点/Wi-Fi。
4. 手机星闪已打开。

### 7.2 树莓派先验收

在电脑上：

```powershell
curl.exe http://192.168.43.42:8080/status
curl.exe http://192.168.43.42:8090/status
```

两个都通，再进入 App。

### 7.3 星闪连接验收

App 中：

1. 点击扫描。
2. 选择 `ws63_robot_mvp`。
3. 连接后先发 `T`。
4. 再发 `I`。
5. 再发 `S`。

能看到响应后，才允许测试：

```text
F 前进
B 后退
L 左转
R 右转
S 停止
```

### 7.4 路线巡检验收

路线导入当前稳定路径是：

```text
App 通过 Wi-Fi/HTTP -> 树莓派 8090 /route/import
App 通过星闪 -> 手动接管 / 急停 / 状态查询
```

先点击“读取路线”，确认路线服务在线。第一次启动只跑一圈，且旁边必须有人准备拿起小车或断电。

## 8. 当前未完成但必须知道的任务

### 8.1 路线通过星闪导入 WS63

你提出的新方向是：不要只通过 Wi-Fi 把路线给树莓派，而是手机通过星闪把路线参数导入 WS63，由 WS63 本地解析并执行。

当前代码进度：

- WS63 固件侧已经有 `R,forward_ms,turn_ms,speed,loops` 形式的路线配置解析入口。
- 响应格式类似：`+ROBOT:ROUTE,OK,...` 或 `+ROBOT:ROUTE,ERR,...`。
- App 的 `writeCommand()` 已支持多字节 ASCII 写入，不再只发 1 字节。
- App 当前可见路线面板仍主要是 Wi-Fi/HTTP 导入树莓派 MapBrain。

队员接手后优先做：

1. 在 App 路线面板增加“导入到 WS63”按钮。
2. 根据图形参数生成短命令，例如：

```text
R,5000,800,100,1
```

3. 调用 `writeCommand(routeCommand)` 发到 SLE `0x7101`。
4. 等待 notify/read 返回：

```text
+ROBOT:ROUTE,OK,5000,800,100,1
```

5. 再发 `P` 启动 WS63 本地巡检。
6. 实车验证边与边之间是否连续，不再出现每条边后停几秒。

### 8.2 自动巡检停顿问题

历史原因：

- 旧路线里有 `inspect/wait/stop` 步骤，会导致每到点位停车。
- 默认方形路线已经按当前思路移除或弱化这类中间停车。
- 如果仍然“跑几秒停几秒”，先查执行的是树莓派 MapBrain 旧路线，还是 WS63 固件旧 `P` 巡检。

排查顺序：

```text
App 当前启动的是 /route/start 还是 SLE P？
树莓派 /route/current 里是否还有 inspect/wait/stop？
WS63 固件是否已烧录支持 R 配置的新版本？
```

## 9. 不能踩的坑

- 不要把旧文档里的 `D:\w\src` 当作当前 SDK 主路径。
- 不要直接覆盖干净 SDK `D:\r\src`。
- 不要把 Harmony 工程放到中文路径构建。
- 不要复制别人的 DevEco 调试签名材料。
- 不要默认所有华为手机都支持星闪，必须实机确认。
- 不要把 `AT+SLEENABLE` 当作常规启动命令。
- 不要在未确认 `S` 停车有效前测试 `F/B/L/R`。
- 不要让自动巡检在桌面、狭窄地面或无人看护时启动。
- 不要把树莓派 SSH 密码、华为账号密码写入提交包。

## 10. 快速故障表

| 现象 | 优先检查 |
|---|---|
| App 装不上 | DevEco 是否登录、签名是否刷新、HDC 是否能看到设备 |
| App 扫不到小车 | 手机是否支持/开启星闪；WS63 日志是否有 `adv ret=0x0` |
| `AT+ROBOTST` 返回 `ERROR` | overlay 是否真正复制到 SDK；是否烧录了旧包 |
| App 能连但控制无响应 | SLE service/characteristic 是否是 `7100/7101/7102`；是否收到 notify |
| 摄像头黑屏 | `curl http://192.168.43.42:8080/status`，看 `frame_ready` |
| 路线服务离线 | `sudo systemctl status nearlink-rover-arbiter` |
| 自动巡检跑一段停一段 | 查 route 是否含 `inspect/wait/stop`；查 App 启动的是 Wi-Fi MapBrain 还是 WS63 `P` |
| 右转角度不准 | 当前右转基线是 `0.8s`；地面/电量变化时微调 `turn_ms` |

## 11. 推荐分工

```text
队员 A：HarmonyOS App
  - DevEco 登录/签名/HAP 构建
  - 星闪扫描连接和 UI
  - 路线参数图形化、导入到 WS63 按钮

队员 B：WS63 固件
  - D:\r\src -> D:\b\src SDK 复现
  - SLE 服务注册、AT+ROBOTST、R 命令解析
  - 烧录和串口日志验收

队员 C：树莓派/WAVE ROVER
  - 8080 摄像头服务
  - 8090 仲裁与路线服务
  - 串口桥、systemd 服务、实车安全测试
```

交接时不要只说“我这边能跑”。必须给出下面四个结果：

```text
1. HAP 能否重新构建并安装
2. 手机能否扫描到 ws63_robot_mvp
3. 8080/status 和 8090/status 是否 HTTP 200
4. T/I/S 是否能通过星闪收到响应
```
