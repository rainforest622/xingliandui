# DevEco / HiSpark SDK Overlay

将本目录内容复制到 `fbb_bs2x` SDK 根目录即可启用 robot MVP。

关键路径：

```text
application/samples/peripheral/robot_mvp/
application/samples/peripheral/CMakeLists.txt
application/samples/peripheral/Kconfig
application/ws63/ws63_liteos_application/main.c
build/config/target_config/ws63/menuconfig/acore/ws63_liteos_app.config
```

集成点：

- `CONFIG_SAMPLE_SUPPORT_ROBOT_MVP=y`
- `CONFIG_ROBOT_MVP_ENABLE_SLE=y`
- `main.c` 注册 robot AT 命令并启动 robot app task。
- `robot_mvp` 实现 AT/SLE 控制、电机、OLED、AHT20、HC-SR04。
