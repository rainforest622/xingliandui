# WS63 小车固件构建与烧录注意事项

本文记录 2026-07-07 这次 WS63 小车固件配置、构建、烧录过程中遇到的问题和处理办法，后续重新烧录前可按此清单排查。

## 1. 确认烧录串口

本次最终可用串口是 `COM5`，设备名为 `USB-SERIAL CH340 (COM5)`。

不要使用蓝牙串口，例如 `COM3`、`COM4`、`COM10`、`COM11`、`COM12`、`COM13`。这些不是小车烧录口。

检查命令：

```powershell
Get-PnpDevice -PresentOnly -Class Ports
[System.IO.Ports.SerialPort]::GetPortNames() -join ' '
mode COM5
```

常见错误：

```text
Open serial port :COM5 failed
Illegal device name - COM5
```

处理办法：

- 确认 CH340 设备状态是 `OK`，不是 `Unknown`。
- 重新插拔 USB 线，或更换 USB 口。
- 确认 HiSpark Studio 串口监视器、Python 客户端、其他串口工具没有占用该端口。
- 如果端口号变化，烧录命令中的 `-Port` 要同步修改。

## 2. BurnTool 等待复位

烧录时如果出现：

```text
Please reset the device, the waiting time is about 30 seconds.
```

需要按一下开发板 Reset，或重新给开发板上电。正常握手后会开始下载：

```text
Start download: root_loaderboot_sign.bin.
Start download: ws63-liteos-app-sign.bin.
```

最终成功标志：

```text
Burned successfully.
```

## 3. 不要直接用长路径 SDK 构建

本次在 `D:\HiSparkSDK\fbb_ws63\src` 构建时遇到过 Windows 命令行过长问题：

```text
ninja: fatal: CreateProcess: The parameter is incorrect.
(is the command line too long?)
```

处理办法：使用短路径 SDK：

```text
D:\fbb_ws63\src
```

最终固件包路径：

```text
D:\fbb_ws63\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
```

## 4. Python 版本要求

短路径 SDK 用系统默认 Python 3.12 构建时出现：

```text
ModuleNotFoundError: No module named 'distutils'
```

原因：该 SDK 的 `build.py` 依赖 `distutils`，Python 3.12 已移除。

处理办法：使用 Python 3.11：

```powershell
D:\Python\python.exe build.py ws63-liteos-app
```

## 5. 避免 ccache 影响构建

本次构建时 `ccache` 参与后出现过：

```text
riscv32-linux-musl-gcc.exe: error: CreateProcess: No such file or directory
```

处理办法：构建前从 `PATH` 中移除 HiSpark Studio 的 `ccache` 路径，并清理旧的 app 构建目录，让 CMake 重新生成不带 `ccache` 的规则。

## 6. Overlay 必须真正复制到 SDK

一开始 `apply_deveco_overlay.ps1` 使用 `-LiteralPath` 搭配通配符，导致 overlay 没有完整复制。现象是：

```text
AT
OK
AT+ROBOTST
ERROR
```

也就是普通 AT 可用，但机器人自定义 AT 命令没有注册。

检查点：

```powershell
rg -n "CONFIG_SAMPLE_SUPPORT_ROBOT_MVP|ROBOT MAIN|robot_mvp_entry" D:\fbb_ws63\src
```

必须看到：

```text
CONFIG_SAMPLE_SUPPORT_ROBOT_MVP=y
ROBOT MAIN app entry
ROBOT MAIN at register
```

## 7. 缺少 OLED SSD1306 依赖

机器人 OLED 模块依赖：

```text
application/samples/peripheral/helloworld_oled/ssd1306.c
application/samples/peripheral/helloworld_oled/ssd1306_fonts.c
```

如果短路径 SDK 没有 `helloworld_oled` 目录，会出现：

```text
Cannot find source file:
D:/fbb_ws63/src/application/samples/peripheral/helloworld_oled/ssd1306.c
```

处理办法：从可用 SDK 复制 `helloworld_oled` 目录到 overlay 和短路径 SDK。

## 8. SDK 版本 API 差异

本次短路径 SDK 的 AT 输出函数是：

```c
uapi_at_printf(...)
```

不是：

```c
uapi_at_print(...)
```

如果使用旧函数名，会出现：

```text
implicit declaration of function 'uapi_at_print'; did you mean 'uapi_at_printf'?
```

处理办法：统一改为 `uapi_at_printf`。

## 9. main.c 不要整文件覆盖新版 SDK 入口

直接使用另一个 SDK 版本的 `main.c` 会导致链接错误：

```text
undefined reference to `sys_mem_show'
undefined reference to `__init_array_end'
undefined reference to `__init_array_start'
```

处理办法：以当前短路径 SDK 原版 `main.c` 为基底，只补机器人相关逻辑：

- `robot_mvp_entry()`
- `robot_at_cmd_register()`
- `CONFIG_SAMPLE_SUPPORT_ROBOT_MVP` 条件块

不要把新版 SDK 的 `timer_patch_init()`、`sys_mem_show()`、`system_init_array()` 整体带入旧链接脚本环境。

## 10. 烧录后验证

先只发安全查询，不要马上让小车运动：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands T,I --timeout 6
```

本次验证成功输出类似：

```text
cmd=T status=OK moving=0 ready=1
cmd=I status=OK moving=0
```

最后发送停止命令确认小车不动：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands S,T --timeout 6
```

成功状态应包含：

```text
cmd=S status=OK moving=0
last_cmd=5
```

## 11. 最高速测试安全提醒

本次速度参数已调到 `100`。测试前建议先把小车轮子架空，确认方向和停止命令正常后再落地测试。

安全测试命令：

```powershell
python -m upper_client.robot_client --transport serial-at --serial-port COM5 --commands F,S --timeout 6
```
