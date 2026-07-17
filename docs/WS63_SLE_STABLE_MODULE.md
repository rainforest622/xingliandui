# WS63 Robot MVP 星闪稳定模块说明

## 结论

当前稳定版本使用“按旧 verified 包的启动时序，等系统 app 任务延迟 1 秒后再启动机器人和 SLE”的方式：

1. `app_main()` 先 `osDelay(100)`，约 1 秒后调用 `robot_mvp_entry()`。
2. `robot_sle_server_start()` 创建 `RobotSle` 工作线程并注册 SSAP、连接、公开回调。
3. 在旧 verified 包相同的时序下调用 `enable_sle()`，避免过早调用卡在 RF/device 初始化前。
4. SLE enable 回调触发后注册服务并启动广播。
5. 手机连接后写入单字节命令，固件直接执行，不再返回 `+ROBOT:SLE,BUSY`。

## 依据

本次参考了 WS63 SDK 自带的华为/海思星闪样例和头文件：

- `D:\fbb_ws63\src\application\samples\bt\sle\sle_uuid_server\src\sle_uuid_server.c`
- `D:\fbb_ws63\src\application\samples\bt\sle\sle_uuid_server\src\sle_server_adv.c`
- `D:\fbb_ws63\src\application\samples\bt\sle\sle_speed_server\src\sle_speed_server.c`
- `D:\fbb_ws63\src\include\middleware\services\bts\sle\sle_device_discovery.h`
- `D:\fbb_ws63\src\include\middleware\services\bts\sle\sle_ssap_server.h`

`sle_device_discovery.h` 对 `sle_enable_cb` 的说明要求该回调运行在 SLE service 线程中，不应阻塞或长时间等待。因此本模块不在回调里同步建服务，而是把建服务和启动广播放到 `RobotSle` 工作线程。

## 修改文件

- `firmware/deveco_ws63_overlay/application/ws63/ws63_liteos_application/main.c`
  - 恢复 `app_main()` 延迟 1 秒后调用 `robot_mvp_entry()`。

- `firmware/deveco_ws63_overlay/application/samples/peripheral/robot_mvp/robot_sle_server.c`
  - 恢复旧 verified 包的 SLE 启动链路。
  - 保留速度 100。
  - 保留写命令直接执行，避免 `+ROBOT:SLE,BUSY`。

## 固件包

构建后应使用：

`D:\WS63_NearLink_Robot_MVP\artifacts\ws63_robot_mvp_sle_verified_timing_speed100_direct_write_load_only.fwpkg`

构建命令：

```powershell
.\tools\apply_deveco_overlay.ps1 -SdkRoot 'D:\fbb_ws63\src'
Push-Location 'D:\fbb_ws63\src'
try {
    & 'D:\Python\python.exe' build.py ws63-liteos-app
} finally {
    Pop-Location
}
```

## 运行流程

烧录：

```powershell
.\tools\burn_ws63_app.ps1 -Port COM5 -Package 'D:\WS63_NearLink_Robot_MVP\artifacts\ws63_robot_mvp_sle_verified_timing_speed100_direct_write_load_only.fwpkg'
```

启动后串口应看到：

```text
ROBOT entry create ok
ROBOT SLE start begin name=ws63_robot_mvp service=0x7100
ROBOT SLE start requested name=ws63_robot_mvp service=0x7100
ROBOT_MVP READY protocol=at/sle uart=0 baud=115200 motor=SKIPPED
```

如果星闪启用成功，应继续看到：

```text
ROBOT SLE enable status=0x0
ROBOT SLE add service ret=0x0
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
ROBOT SLE announce enable
```

此时手机端扫描设备名 `ws63_robot_mvp`，连接后可发送：

- `I` 初始化电机
- `F` 前进
- `B` 后退
- `L` 左转
- `R` 右转
- `S` 停止
- `T` 状态

当前速度常量为 100，即代码限制范围内的最高值。

## 注意事项

不要再把 `AT+SLEENABLE` 作为常规启动步骤。实测该 AT 命令在错误时序下可能长期占住 AT 通道，后续命令会出现 `ERRCODE_AT_CHANNEL_BUSY`，日志表现为：

```text
at_uart_rx_callback fail:0x80003020
```

如果看到这个错误，按小车物理 Reset 重新启动，等待自动星闪初始化线程处理。
