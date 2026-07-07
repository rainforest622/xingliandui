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

注意：

- 该 App 工程来自本地历史交付包，已纳入本次 GitHub 上传包。
- 当前板端 SLE server 源码位于 `firmware/deveco_ws63_overlay/application/samples/peripheral/robot_mvp/robot_sle_server.c`。
- 具体 SLE profile 见 `../protocol_docs/sle_control_profile.md`。
