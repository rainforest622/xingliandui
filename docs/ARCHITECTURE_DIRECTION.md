# 当前大方向决策：可导入路线的星闪智能巡检机器人

更新时间：2026-07-17

本文是当前阶段的架构方向定稿。旧文档中关于旧电机板直驱、R16/R17 焊接、H5 串口、巡检点识别、黄色循迹优先路线等内容只作为历史记录；后续实现、烧录、调试和比赛演示按本文执行。

## 1. 最终定位

项目重新定义为：

> 基于 WS63 星闪安全控制、树莓派路线大脑和 WAVE ROVER 底盘的可导入路线智能巡检机器人。

核心逻辑：

> 手机端导入巡检路线，树莓派按路线自动巡航；WS63 负责星闪控制、急停锁存、温湿度与超声波安全感知；树莓派根据超声波事件实现真正智能避障，并记录物联网巡检日志。

这条路线保留比赛亮点：

```text
数据走 Wi-Fi：视频、路线文件、日志、状态页面
控制走星闪：启动、停止、急停、手动接管、恢复
安全由 WS63 托底：急停、温湿度、超声波障碍
```

## 2. 总体架构

```text
┌────────────────────────────────┐
│ HarmonyOS 手机/平板             │
│ SLE 手动控制 / 急停 / 恢复        │
│ Wi-Fi 路线导入 / 视频 / 日志      │
└───────────────┬────────────────┘
        SLE     │                     Wi-Fi/HTTP
                │                         │
                ▼                         ▼
┌────────────────────────────────┐   ┌────────────────────────────────┐
│ WS63 / HiSparkEP                │   │ 树莓派                         │
│ SLE 服务、急停锁存               │   │ 路线导入 API / MapBrain         │
│ AHT20 温湿度                    │   │ ObstacleBrain 智能避障          │
│ HC-SR04 前向距离                │   │ Camera/AI/日志/监控页面         │
│ OLED / 蜂鸣器 / 红灯             │   │ 控制优先级仲裁                  │
└───────────────┬────────────────┘   └───────────────┬────────────────┘
                │ USB 串口：控制/传感器/安全事件       │ 40PIN UART JSON
                └────────────────────────────────────►│
                                                      ▼
                                      ┌────────────────────────────────┐
                                      │ WAVE ROVER ESP32 底盘           │
                                      │ 左右轮速度控制                  │
                                      │ 停车 / 前进 / 转向               │
                                      └────────────────────────────────┘
```

物理主线：

```text
WS63 Type-C USB 串口 -> 树莓派 USB
树莓派 40PIN UART   -> WAVE ROVER
USB 摄像头           -> 树莓派
HC-SR04              -> WS63
AHT20                -> WS63
```

## 3. 模块职责

| 模块 | 当前职责 | 下一步增强 |
|---|---|---|
| HarmonyOS 手机/平板 | SLE 连接、手动控制、急停、摄像头页面 | 路线编辑/导入、路线状态、巡检日志页面 |
| WS63 + LiteOS | SLE 服务、手动控制、急停、AHT20、HC-SR04、OLED/报警 | 周期性上报距离/温湿度、安全事件格式化 |
| 树莓派 | 摄像头服务、安全仲裁、地图大脑、WAVE ROVER 控制 | 路线导入 API、ObstacleBrain、巡检日志、AI 安全检测 |
| WAVE ROVER ESP32 | 接收官方 JSON 并执行左右轮速度 | 保持原厂协议，不改底盘固件 |

## 4. 四个核心闭环

### 闭环 1：手机路线导入闭环

手机端在 App 内编辑或导入路线：

```text
路线编辑器
  -> route.json
  -> Wi-Fi/HTTP 上传到树莓派
  -> 树莓派校验并保存为 active_route.json
  -> MapBrain 加载执行
```

路线数据是“巡航配置”，属于数据通道，走 Wi-Fi；启动、停止、急停仍属于控制通道，走星闪 SLE。

### 闭环 2：路线巡航闭环

树莓派加载路线 JSON，把每个动作转换为 WAVE ROVER 左右轮速度：

```text
move    -> {"T":1,"L":0.24,"R":0.24}
turn R  -> {"T":1,"L":0.5,"R":-0.5}
turn L  -> {"T":1,"L":-0.5,"R":0.5}
wait    -> {"T":1,"L":0.0,"R":0.0}
```

当前实车校准参数：

```text
直行：speed=0.24
右转：speed=0.5, duration_s=0.8
```

注意：WAVE ROVER 当前无编码器，路线巡航是可校准时间轨迹，不宣称为 SLAM。

### 闭环 3：超声波智能避障闭环

WS63 持续采集 HC-SR04 前方距离，并把数据发给树莓派仲裁器：

```text
距离正常       -> 路线继续
距离 < warn   -> 降速
距离 < stop   -> 停车，暂停路线计时
障碍持续存在   -> 后退/绕行/等待手动接管
距离恢复 clear -> 回归路线
```

避障不是简单停车，而是状态机：

```text
AUTO_ROUTE
  -> SLOW_DOWN
  -> OBSTACLE_STOP
  -> BACK_OFF
  -> BYPASS_TURN
  -> BYPASS_FORWARD
  -> RETURN_TURN
  -> REJOIN_ROUTE
  -> AUTO_ROUTE
```

关键要求：

- 避障期间暂停 MapBrain 的当前步骤计时。
- 超声波无效时 fail-safe 停车。
- 急停和手动接管优先级高于避障。

### 闭环 4：物联网安全日志闭环

树莓派记录每次巡航事件：

```text
route_loaded
route_started
step_changed
sensor_update
obstacle_warn
obstacle_stop
avoidance_started
avoidance_finished
manual_override
estop
route_done
```

日志用于手机端/网页端展示，体现工业物联网属性。

## 5. 控制优先级

底盘命令由树莓派最终发给 WAVE ROVER，但必须服从以下优先级：

```text
P0：星闪急停
P1：WS63 环境严重异常 / 超声波无效故障
P2：星闪手动遥控
P3：超声波障碍避让
P4：AI 视觉严重事件
P5：自动路线巡航
P6：无有效命令，停车
```

伪代码：

```python
while True:
    ws63_msg = read_ws63()
    route_cmd = map_brain.next()
    obstacle_cmd = obstacle_brain.next(ws63_msg.distance)
    ai_event = read_ai_camera()

    if ws63_msg.estop:
        cmd = stop("ESTOP_LOCK")
    elif ws63_msg.env_alarm or ws63_msg.distance_invalid:
        cmd = stop("SENSOR_ALARM")
    elif ws63_msg.manual_valid:
        cmd = ws63_msg.manual_cmd
    elif obstacle_brain.active:
        map_brain.pause()
        cmd = obstacle_cmd
    elif ai_event.severe:
        cmd = stop("AI_ALARM")
    elif route_enabled:
        map_brain.resume()
        cmd = route_cmd
    else:
        cmd = stop("IDLE")

    send_to_waverover(cmd)
    write_event_log()
```

## 6. 运行状态机

```text
BOOT
  ↓
IDLE
  ↓ 手机导入路线
ROUTE_READY
  ↓ 星闪启动巡航
AUTO_ROUTE
  ↓ 超声波检测到障碍
OBSTACLE_AVOID
  ↓ 绕开/清障
AUTO_ROUTE
  ↓ 路线完成
TASK_DONE

任意状态下：
  星闪急停       -> ESTOP_LOCK
  星闪手动接管   -> MANUAL_OVERRIDE
  环境/传感器异常 -> SENSOR_ALARM
```

| 状态 | 小车行为 | 手机/网页显示 |
|---|---|---|
| IDLE | 原地等待 | 系统待命 |
| ROUTE_READY | 等待启动 | 路线已导入 |
| AUTO_ROUTE | 按路线巡航 | 当前步骤/剩余时间 |
| OBSTACLE_AVOID | 停车/后退/绕行 | 避障中 |
| MANUAL_OVERRIDE | 星闪手动控制 | 手动接管 |
| SENSOR_ALARM | 停车 | 传感器/环境异常 |
| ESTOP_LOCK | 锁定停车 | 急停 |
| TASK_DONE | 停车 | 巡航完成 |

## 7. 当前实物状态

截至 2026-07-17：

- WS63 已烧录当前 WAVE ROVER bridge 固件，SLE 手动控制可用。
- 手机端可通过星闪连接并控制小车。
- 树莓派 IP 当前为 `192.168.43.42`。
- 树莓派安全仲裁服务 `nearlink-rover-arbiter.service` 已运行。
- 摄像头服务 `8080` 已恢复，健康检查为 200。
- `patrol_map.json` 已完成实车方形巡航测试。
- 当前稳定转角参数暂定：`turn speed=0.5`，`duration_s=0.8`。

## 8. 下一阶段目标

1. **阶段 C1：树莓派路线导入 API（已完成）**
   `rover_arbiter.py` 已增加 `POST /route/import`、`GET /route/current`、`POST /route/start`、`POST /route/stop`，导入路线会保存为 `active_route.json`。

2. **阶段 C2：HarmonyOS 路线导入页面**
   App 增加路线编辑器，支持添加 move/turn/wait 步骤、预览路线、上传路线、启动巡航。

3. **阶段 D1：WS63 传感器上报协议**
   WS63 周期性上报距离、温湿度和障碍状态，树莓派解析后写入状态。

4. **阶段 D2：ObstacleBrain 避障状态机**
   树莓派在路线巡航中遇障后暂停路线计时，执行后退/绕行/回归路线。

5. **阶段 E：物联网日志和展示**
   建立 `patrol_events.jsonl`，手机/网页端展示路线、传感器、告警和避障记录。

## 9. 文档口径

比赛或答辩中建议这样描述：

> 我们没有把路线固定写死在树莓派里，而是在 HarmonyOS 控制端构建路线导入能力。路线数据通过 Wi-Fi 导入树莓派，实时控制和安全接管走星闪 SLE。巡航过程中 WS63 持续采集超声波与温湿度数据，树莓派根据障碍事件暂停路线并执行绕行动作，从而形成可配置、可接管、可记录的工业巡检闭环。
