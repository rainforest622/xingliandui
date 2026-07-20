# Firmware

`deveco_ws63_overlay/` 是覆盖到 HiSpark/DevEco/SDK 根目录的板端源码 overlay。

`fwpkg/` 保存当前推荐烧录包：

```text
ws63-liteos-app_ws63_sle_pi_bridge_load_only.fwpkg
```

历史小车电机板直驱包仍保留在 `fwpkg/` 中，比赛当前大方向推荐使用 WAVE ROVER 串口桥方案。

SDK 复现流程：

```powershell
.\tools\build_ws63_liteos_app.ps1 -SdkRoot D:\r\src
```

`D:\r\src` 可作为相对干净的 SDK 底座保留；不要直接覆盖它。复制时只排除顶层 `output`，不要排除 `drivers\chips\ws63\rom_config\acore\output`，否则会缺少 `function.lds`、`rom_callback_wrap.cmake` 等 ROM 配置文件。

构建后 SDK 输出：

```text
D:\r\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
```

当前已归档包：

```text
firmware\fwpkg\ws63-liteos-app_ws63_sle_pi_bridge_load_only.fwpkg
SHA256: 5BF9C4A495A2988D2E5B893F7D9125F5BACE16F09FF81A0444A3432390CD72D6
```
