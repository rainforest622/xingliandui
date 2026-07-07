# Firmware

`deveco_ws63_overlay/` 是覆盖到 HiSpark/DevEco/SDK 根目录的板端源码 overlay。

`fwpkg/` 保存当前推荐烧录包：

```text
ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg
```

SDK 复现流程：

```powershell
.\tools\apply_deveco_overlay.ps1 -SdkRoot D:\w\src
.\tools\build_ws63_liteos_app.ps1 -SdkRoot D:\w\src
```

构建后 SDK 输出：

```text
D:\w\src\output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
```
