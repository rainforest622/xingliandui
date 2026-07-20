# 声学与语音模块实施说明

## 已落地的设计

声音模块被定义为树莓派上的边缘感知入口，而不是直接电机遥控器：

```text
串口语音模块 / USB 麦克风
        -> 离线语音识别与声学事件解析
        -> voice_service.py
        -> rover_arbiter.py 安全仲裁
        -> WAVE ROVER 底盘
```

语音服务只会向 `rover_arbiter.py` 的 `POST /voice/intent` 发送允许列表内的意图；它绝不写入底盘 JSON。WS63/SLE 手动控制仍在自动巡检和语音之上，SLE 急停也不能由语音解除。

当前允许的语音意图：

- `开始巡检`：启动当前已导入的地图路线。
- `暂停巡检`、`停止巡检`：停止地图任务并切回手动待命。
- `继续巡检`：从路线第一段重新启动当前地图任务。
- `报告状态`、`报告环境`：更新语音状态，不改变运动状态。
- `解除报警`：明确拒绝，必须由星闪端确认，不能绕过急停权限。

当前关键事件：`救命/帮忙/着火`、`报警声`、`撞击/碰撞`。这些事件会停止自动巡检，但保留星闪手动接管；若 WS63 已处于急停锁定，则语音不能解锁。

## 模型选择

ASRPRO 是当前小车的第一层离线命令识别器：命令词和唤醒词编译进板端模型，识别成功后只产生一个确定的事件字节，延迟和树莓派负载都更低。树莓派侧保留 **sherpa-onnx 的量化 SenseVoice** 与 **Silero VAD**，用于后续 USB 麦克风的开放语音交互或声音事件扩展。选择原因：

- `sherpa-onnx` 官方支持 Linux ARM、Raspberry Pi、离线 ASR、VAD、KWS 和 TTS，可在无外网时运行。[官方仓库](https://github.com/k2-fsa/sherpa-onnx)
- 官方模型列表中的 `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17` 支持普通话、粤语和中英日韩混合语音，适合中文比赛现场。
- VAD 持续消耗很低，只有检测到 0.3 至 5 秒的人声段才调用 SenseVoice，避免摄像头与自动巡检同时运行时持续占用 CPU。
- 未采用 openWakeWord 作为中文默认唤醒，因为其官方说明当前预训练语言主要为英语；后续若需要“小星小星”自定义唤醒，应使用 sherpa-onnx 中文 KWS 模型或针对实测声音训练专用模型。[openWakeWord 官方说明](https://github.com/dscripka/openWakeWord)

模型安装脚本会下载：

```text
sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/model.int8.onnx
sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/tokens.txt
silero_vad.onnx
```

## ASRPRO 串口协议与当前 COM14 检测结果

资料确认语音板是 **ASRPRO**：CH340K/Type-C 是下载和 UART0 调试通道，默认串口速率为 `9600 bps`。它在未烧录应用时不会输出识别结果，因此此前被动监听和 `AT` 探测为空是正常现象。

可直接导入天问 Block 的工程位于 `raspberry_pi/asrpro/robot_voice_control.hd`；`robot_voice_control.ino` 是完全相同协议的字符编程源码副本。为保证比赛现场识别稳定，板端模型只保留两条唤醒词和八条核心命令，不把同义词堆入小模型。它将唤醒和语音命令转换为固定的一字节帧：

```text
0xA0  唤醒确认
0xA1  开始巡检       0xA2  暂停巡检
0xA3  继续巡检       0xA4  停止巡检
0xA5  报告状态       0xA6  求助/着火安全事件
0xA7  请求解除报警（树莓派会要求星闪端确认）
```

树莓派在收到命令后发送 `0xF1` 至 `0xF6` 回执，ASRPRO 会播报“开始执行路线”“巡检暂停”“需星闪确认”等本地语音。协议实现位于 `raspberry_pi/voice/serial_module.py`。

### 专业模式推荐工程

若配置模式内置播放音提示维护，优先打开 `raspberry_pi/asrpro/robot_voice_control_baidu.hd`。这是可视化的专业编程模式工程，使用 `播报音设置（百度TTS）`，指定“小鹿-甜美女声”、音量 `16`、语速 `8`，并在图形块中保留每条识别指令、原始串口控制字节与树莓派 `0xF1` 至 `0xF6` 执行回执。`raspberry_pi/asrpro/robot_voice_control_pro.ino` 为同功能的字符编程备份。专业模式文档要求识别 ID 唯一，并在 `ASR_CODE` 回调中处理识别事件；本工程的 ID 为 `1000` 至 `1009`，均唯一。[ASRPRO 专业模式手册](https://haohaodada.com/jpeguploadfile/twen/ASRPRO/asr_pro.pdf)

专业模式烧录顺序：天问 Block 选择 **ASRPRO -> 编程模式**，打开该 `.hd`，先“生成模型”，再在系统当前枚举的 `CH340K` 端口上执行“2M 编译下载”，下载后按一次 `RESET`。不要沿用历史端口号；本机最近一次检测为 `COM15`。

## 树莓派部署

先在 Windows 使用天问 Block 的 **设备：ASRPRO / 配置模式** 打开 `raspberry_pi/asrpro/robot_voice_control.hd`。确认右上角为 `COM14-CH340K` 后，依次执行“生成模型”和“2M 编译下载”。下载完成会出现板端欢迎/识别播报。不要将本工程导入字符编程模式；那会导致配置积木无法渲染。

随后将声音模块的 USB 串口接到树莓派，记下 Linux 端口，例如 `/dev/ttyUSB1`。不要与 WS63 的 `/dev/ttyUSB0` 混用；运行时 Type-C 连接可使用，但更稳妥的长期接线是 ASRPRO 的 UART1/UART2 接入独立 USB-TTL。

```bash
cd /home/xinglian/raspberry_pi
chmod +x install_voice_runtime.sh install_voice_service.sh
./install_voice_runtime.sh

# 串口语音板：它需要输出识别文本或 JSON。
VOICE_MODE=serial VOICE_SERIAL_PORT=/dev/nearlink-asr VOICE_BAUDRATE=9600 \
  VOICE_SERIAL_PROTOCOL=asrpro-byte \
  ./install_voice_service.sh

sudo systemctl status nearlink-voice --no-pager
sudo journalctl -u nearlink-voice -f
```

若实际麦克风作为 Linux 音频设备出现，而不是串口识别板，则改用离线模型模式：

```bash
VOICE_MODE=microphone ./install_voice_service.sh
```

安全的无硬件联调：

```bash
python3 voice_service.py --text "开始巡检"
printf "救命\n" | python3 voice_service.py --stdin
curl http://127.0.0.1:8090/status
```

## 验收顺序

1. 先确认 `nearlink-rover-arbiter` 正常，并让小车处于空旷区域。
2. 运行 `journalctl -u nearlink-voice -f`，说“开始巡检”，检查日志出现 `map patrol started`。
3. 巡检中说“暂停巡检”或制造 `help` 事件，检查底盘停止、`/status` 中 `voice.last_action` 更新。
4. 通过手机 SLE 急停后说“继续巡检”或“解除报警”，应被拒绝；只有 SLE 端解除急停后才能继续。
