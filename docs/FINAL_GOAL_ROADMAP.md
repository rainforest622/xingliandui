# 最终目标路线图：手机导入路线的智能巡检机器人

更新时间：2026-07-17

最终目标：

> HarmonyOS 手机端导入巡检路线，树莓派根据路线自动巡航，WS63 通过星闪负责手动控制、急停、环境感知和超声波安全避障，形成“数据走 Wi-Fi、控制走星闪 SLE”的工业巡检闭环。

## 当前阶段评估

已经完成：

- WS63 SLE 服务可用，手机端可以连接并发送控制命令。
- WS63 通过 USB 接树莓派，树莓派通过 `/dev/ttyAMA0` 控制 WAVE ROVER。
- `nearlink-rover-arbiter.service` 已作为当前默认运行时，支持手动控制、急停、无命令停车和自动地图巡航。
- 摄像头服务 `8080` 已恢复，`/healthz` 返回 200，前端可查看视频画面。
- 地图大脑 `patrol_map.json` 已能执行完整方形巡航。
- 当前实车校准参数：直行 `L=0.24,R=0.24`，右转 `L=0.5,R=-0.5`，右转时间 `0.8s`。

当前所处阶段：

```text
阶段 B 已完成：底盘链路、安全仲裁、摄像头服务、文件式地图巡航
阶段 C1 已完成：树莓派路线导入 API + 动态路线加载
阶段 C2 下一步：HarmonyOS App 路线导入/编辑页面
阶段 D 随后推进：超声波智能避障闭环
```

不再作为主线：

- 不再做二维码、ArUco、AprilTag 巡检点识别。
- 不再把黄色循迹作为当前优先目标，只保留为后续可选纠偏方案。
- 不宣称当前系统已有 SLAM；WAVE ROVER 无编码器，当前地图巡航是可校准的时间轨迹。

## 总体通信分层

```text
HarmonyOS App
  ├─ Wi-Fi/HTTP -> 树莓派：路线 JSON、视频、状态、日志
  └─ 星闪 SLE  -> WS63：开始/停止/急停/手动接管/恢复

WS63
  ├─ SLE 控制入口
  ├─ AHT20 温湿度
  ├─ HC-SR04 超声波距离
  └─ USB 串口 -> 树莓派：手动命令、安全事件、传感器数据

树莓派
  ├─ 路线导入 API
  ├─ MapBrain 路线执行器
  ├─ ObstacleBrain 避障状态机
  ├─ Camera/AI/日志服务
  └─ UART -> WAVE ROVER：左右轮速度 JSON
```

## 阶段 C：手机端路线导入

目标：手机上的星闪控制 App 新增“路线轨迹导入”能力，树莓派根据导入路线自动巡航。

建议实现方式：

- 手机端仍然是一个 App，但通信分两类：
  - 路线 JSON、地图预览、巡航日志走 Wi-Fi/HTTP。
  - 开始、停止、急停、手动控制走星闪 SLE。
- 树莓派新增 HTTP 接口：
  - `POST /route/import`：导入路线 JSON。
  - `GET /route/current`：查看当前路线。
  - `POST /route/start` 或现有 `/auto/map`：启动导入路线。
  - `POST /route/stop`：停止路线。
- App 侧提供路线编辑器：
  - 添加直行段、右转/左转、等待段。
  - 调整每段速度和持续时间。
  - 一键导出/导入 JSON。

第一版路线格式：

```json
{
  "name": "factory-patrol-demo",
  "default_speed": 0.24,
  "default_turn_speed": 0.5,
  "steps": [
    { "id": "s1", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "t1", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "s2", "action": "move", "speed": 0.24, "duration_s": 5.0 }
  ]
}
```

完成标准：

- 手机端能导入或编辑路线。
- 树莓派能保存并加载导入路线。
- `/status` 能显示当前路线名、当前步骤、剩余时间。
- 星闪急停和手动接管始终能覆盖路线执行。

## 阶段 D：超声波智能避障

目标：让避障不再只是停车，而是在自动巡航时遇到障碍能够暂停路线、绕开障碍、尝试回到原轨迹。

数据来源：

- WS63 持续读取 HC-SR04。
- 建议上报频率：10 Hz 左右。
- 建议上报格式：

```json
{"type":"sensor","dist_mm":320,"obs_valid":1,"block":0}
{"type":"obstacle","dist_mm":180,"level":"warn"}
{"type":"obstacle","dist_mm":100,"level":"stop"}
```

树莓派避障状态机：

```text
AUTO_ROUTE
  ↓ 距离 < warn 阈值
SLOW_DOWN
  ↓ 距离 < stop 阈值
OBSTACLE_STOP
  ↓
BACK_OFF
  ↓
BYPASS_TURN
  ↓
BYPASS_FORWARD
  ↓
RETURN_TURN
  ↓
REJOIN_ROUTE
  ↓
AUTO_ROUTE
```

建议阈值：

```text
warn: 350 mm
stop: 220 mm
clear: 450 mm
invalid: 超声波无效时停车，不继续自动巡航
```

第一版避障动作：

```text
停车 0.2s
后退 0.35s，L=-0.20,R=-0.20
右转 0.45s，L=0.5,R=-0.5
前进 0.8s，L=0.20,R=0.20
左转 0.45s，L=-0.5,R=0.5
前进 0.8s，L=0.20,R=0.20
恢复当前路线剩余步骤
```

关键点：

- 避障期间必须暂停 MapBrain 的路线计时，不能让路线时间继续流逝。
- 避障失败或距离持续过近时进入 `OBSTACLE_WAIT`，等待手机端手动接管。
- 星闪急停优先级高于避障。

完成标准：

- 路线巡航中放置障碍，小车能先停车。
- 障碍移除后能继续路线。
- 固定障碍物前能执行后退/绕行/回归动作。
- 超声波数据无效时 fail-safe 停车。

## 阶段 E：物联网巡检日志

目标：体现工业物联网属性，而不是只展示车会动。

树莓派记录：

- 路线名、步骤名、当前模式。
- 温度、湿度、距离、告警级别。
- 急停、手动接管、避障、路线完成事件。
- 摄像头帧状态和 AI 检测结果。

建议存储：

- 第一版：`logs/patrol_events.jsonl`。
- 后续可升级 SQLite。

Web/手机端展示：

- 当前路线状态。
- 最近 20 条巡检事件。
- 当前温湿度/距离/告警状态。
- 视频画面。

## 阶段 F：AI 与演示增强

AI 部分优先做“安全感知”，不要为了模型复杂度牺牲稳定性。

建议顺序：

1. 摄像头人员靠近检测：检测到人进入前方区域时减速或停车。
2. 巡航截图留档：异常时保存图片。
3. 视频页面叠加当前状态：AUTO_ROUTE、OBSTACLE、ESTOP、DONE。

比赛演示主线：

```text
手机导入路线
-> 星闪启动巡航
-> Wi-Fi 视频正常显示
-> 超声波遇障停车/绕行
-> 温湿度异常报警
-> 星闪急停立即覆盖
-> 日志记录完整过程
```

## 下一步执行顺序

1. 在 HarmonyOS App 增加路线导入/编辑页面，先用 Wi-Fi HTTP 调树莓派 `/route/import`、`/route/current`、`/route/start`、`/route/stop`。
2. 扩展 WS63 -> 树莓派串口协议，上报超声波距离和环境告警。
3. 在树莓派仲裁器中实现 ObstacleBrain，并让它暂停/恢复 MapBrain。
4. 增加 `patrol_events.jsonl` 日志和前端状态展示。
