# Release Manifest

生成时间：2026-07-16

## 必要内容

- `firmware/deveco_ws63_overlay/`：覆盖到 HiSpark `fbb_bs2x` SDK 的板端源码与配置。
- `firmware/fwpkg/ws63-liteos-app_wave_rover_bridge_load_only.fwpkg`：当前推荐烧录包，WS63 通过树莓派透明串口桥控制 WAVE ROVER。
- `raspberry_pi/rover_bridge.py`：WS63 USB 串口到 WAVE ROVER `/dev/serial0` 的 JSON 透明桥。
- `docs/ARCHITECTURE_DIRECTION.md`：当前大方向决策，定义“工业轨迹闭环巡检机器人”架构。
- `docs/FINAL_GOAL_ROADMAP.md`：从已联通到最终比赛目标的阶段推进路线图。
- `harmony_app/nearlink_robot_controller/`：HarmonyOS/DevEco 手机端控制 App 源码。
- `harmony_app/install_hap/ws63_robot_controller_harmony_next_signed.hap`：本地已签名 HAP 安装包。
- `upper_client/`：Python 上位机，支持 TCP 模拟和 `serial-at` 实板控制。
- `scripts/verify_ultrasonic_distance.py`：只读超声波距离验收。
- `scripts/run_safe_full_check.py`：状态、OLED、温湿度、超声波、停止的安全全功能检查。
- `simulator/`：TCP 模拟器，供 `tests/test_e2e.py` 使用。
- `tests/`：协议/客户端自动化测试。
- `docs/`：最终交接、硬件注意、工程流程。
- `protocol_docs/`：AT 与 SLE 控制协议。
- `references/`：pinout 与原始硬件参考。
- `references/source_materials/`：赛题、初步方案、前置工作文档等原始资料归档。

## 当前关键验证证据

- Python 测试：`python -m unittest discover -s tests` -> `25 tests OK`。
- WS63 构建：`D:\r\src` 作为 SDK 底座，复制到 `D:\b\src` 后应用 overlay，`ws63-liteos-app` 构建成功。
- 当前烧录包 SHA256：`8FC38BF38BD6B3A0819A5EF519FCE1BD7A73819DE31756D8C0DF86753C809ED6`。
- 树莓派桥接：`192.168.43.42` 可 SSH 登录，`/dev/serial0` 可打开，WS63 已识别为 `/dev/ttyUSB0`，`nearlink-rover-bridge.service` 已运行。
- 摄像头当前状态：8080 服务可访问，但 `camera_backend=error`，需按路线图阶段 3 修复。
- 超声波：已测得 `obs_valid=1`、`reason=0/ok`；近距离 `39..70mm` 时 `block=1`。
- 电机：车轮架空，`I` 初始化 OK，`F` 连续刷新前进 2 秒/5 秒 OK，最后 `S` 停止 OK。
- 鸿蒙端：本地历史交付包中的 DevEco 工程和已签名 HAP 已纳入本包；SLE 实机连接仍按 `protocol_docs/sle_control_profile.md` 做最终验收。
- 安全边界：当前固件没有闭环轮速采样，左右轮“命令占空比相同”，实际转速一致性需肉眼/测速仪/后续编码器功能确认。

## 不包含

- 不包含完整 `fbb_bs2x` SDK。请从 HiSpark/Gitee 获取 SDK 后应用 overlay。
- 不包含历史中间固件包、失败烧录包、临时目录。
