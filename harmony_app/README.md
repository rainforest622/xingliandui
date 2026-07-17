# HarmonyOS / DevEco 控制 App

本目录包含本地同款鸿蒙端控制 App：

```text
nearlink_robot_controller/      # DevEco Studio 可打开的 HarmonyOS/ArkTS 工程源码
install_hap/                    # 已签名 HAP 安装包
```

已签名安装包：

```text
install_hap/ws63_robot_controller_harmony_next_signed.hap
```

源码工程入口：

```text
nearlink_robot_controller/oh-package.json5
nearlink_robot_controller/build-profile.json5
nearlink_robot_controller/entry/src/main/ets/pages/
```

关键页面/文件：

```text
entry/src/main/ets/pages/MainPage.ets
entry/src/main/ets/pages/SsapClientPage.ets
entry/src/main/ets/pages/SsapPage.ets
entry/src/main/ets/pages/RobotSafetyBus.ets
ROBOT_CONNECT_GUIDE.md
```

使用说明：

1. 用 DevEco Studio 打开 `nearlink_robot_controller/`。
2. 按 DevEco 提示同步依赖和工程配置。
3. 若使用 HAP，安装 `install_hap/ws63_robot_controller_harmony_next_signed.hap`。
4. 小车板端应已烧录本仓库 `firmware/fwpkg/` 中的固件，并广播 `ws63_robot_mvp`。

命令行构建注意：

- Hvigor 不接受项目路径中包含中文字符。若当前仓库路径含中文，请先把
  `nearlink_robot_controller/` 复制到纯英文路径再构建，例如
  `D:\codex_harmony_build\nearlink_robot_controller`。
- 如果 DevEco Studio 是解压版，需要在当前 PowerShell 会话临时设置 SDK 路径：

```powershell
$env:DEVECO_SDK_HOME = 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk'
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\tools\hvigor\bin\hvigorw.bat' --mode module -p module=entry@default -p product=default assembleHap
```

构建成功后，签名 HAP 位于：

```text
entry\build\default\outputs\default\entry-default-signed.hap
```

连接鸿蒙设备后可用 DevEco 自带 `hdc.exe` 安装：

```powershell
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk\default\openharmony\toolchains\hdc.exe' list targets
& 'D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk\default\openharmony\toolchains\hdc.exe' install .\entry\build\default\outputs\default\entry-default-signed.hap
```

注意：

- 该 App 工程来自本地历史交付包，已纳入本次 GitHub 上传包。
- 当前板端 SLE server 源码位于 `firmware/deveco_ws63_overlay/application/samples/peripheral/robot_mvp/robot_sle_server.c`。
- 具体 SLE profile 见 `../protocol_docs/sle_control_profile.md`。
