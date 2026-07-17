# 下一阶段执行方案：路线导入与超声波智能避障

更新时间：2026-07-17

本文用于指导接下来的实现，不再推进二维码、ArUco、AprilTag 巡检点识别；黄色视觉循迹只作为后续可选纠偏方案。当前主线是：

```text
手机端编辑/导入路线
-> 树莓派保存路线并执行 MapBrain
-> WS63 持续上报超声波/温湿度
-> 树莓派 ObstacleBrain 暂停路线、避障、回归路线
-> 手机/网页端展示状态和物联网日志
```

## 1. 当前阶段判断

已完成：

- WS63 星闪手动控制链路可用。
- 树莓派安全仲裁器 `nearlink-rover-arbiter.service` 可用。
- WAVE ROVER 串口控制链路可用。
- 摄像头服务 `8080` 可用。
- `patrol_map.json` 已完成实车方形巡航。
- 当前稳定路线参数：直行 `L=0.24,R=0.24`，右转 `L=0.5,R=-0.5`，右转时间 `0.8s`。
- 树莓派已具备路线导入 API：

```text
POST /route/import
GET  /route/current
POST /route/start
POST /route/stop
```

当前所处阶段：

```text
阶段 A/B/C1 已完成。
阶段 C2 第一版已完成：HarmonyOS 手机端已加入路线 JSON 编辑/导入面板。
阶段 D：超声波智能避障，是 C2 后的下一优先级。
```

## 2. 手机端路线导入功能

目标：在现有星闪 App 内新增“路线轨迹”页面，让用户能配置巡航路线并上传到树莓派。

页面功能：

- 路线 JSON 编辑。
- 一键恢复默认 40 m2 方形路线。
- 上传路线到树莓派。
- 查看当前路线状态。
- 启动/停止路线。
- 当前默认树莓派路线服务地址：`http://192.168.43.42:8090`。

已修改文件：

```text
harmony_app/nearlink_robot_controller/entry/src/main/ets/pages/SsapClientPage.ets
```

通信分工：

```text
Wi-Fi/HTTP：路线 JSON、当前路线、日志、视频
星闪 SLE：手动控制、急停、恢复、关键安全控制
```

第一版路线 JSON：

```json
{
  "name": "factory-patrol-demo",
  "default_speed": 0.24,
  "default_turn_speed": 0.5,
  "steps": [
    { "id": "s1", "name": "A-B", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "t1", "name": "B turn", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "s2", "name": "B-C", "action": "move", "speed": 0.24, "duration_s": 5.0 }
  ]
}
```

验收标准：

- 手机端能编辑路线 JSON。
- 能上传到 `http://树莓派IP:8090/route/import`。
- `/route/current` 能返回刚导入的路线。
- `/route/start` 能启动路线。
- 星闪急停和手动控制仍能覆盖自动巡航。
- 电脑端当前访问 `192.168.43.42:8090` 超时，下一次联调前需要确认树莓派热点/IP/`nearlink-rover-arbiter.service`。

## 3. 超声波智能避障设计

目标：避障不是只停下，而是在巡航中暂停路线、处理障碍、再尝试回归路线。

WS63 上报建议：

```json
{"type":"sensor","dist_mm":420,"obs_valid":1,"block":0,"temp_c":31.2,"hum_pct":58.0}
{"type":"obstacle","dist_mm":180,"level":"stop"}
{"type":"sensor","obs_valid":0,"reason":"no_echo"}
```

阈值建议：

```text
warn：350 mm，路线降速
stop：220 mm，停车并进入避障
clear：450 mm，认为前方恢复安全
invalid：超声波无效，fail-safe 停车
```

ObstacleBrain 第一版状态机：

```text
AUTO_ROUTE
  -> SLOW_DOWN
  -> OBSTACLE_STOP
  -> BACK_OFF
  -> BYPASS_TURN_RIGHT
  -> BYPASS_FORWARD
  -> RETURN_TURN_LEFT
  -> REJOIN_FORWARD
  -> AUTO_ROUTE
```

第一版动作参数：

```text
stop：0.2s, L=0.0,R=0.0
back_off：0.35s, L=-0.20,R=-0.20
bypass_turn_right：0.45s, L=0.5,R=-0.5
bypass_forward：0.8s, L=0.20,R=0.20
return_turn_left：0.45s, L=-0.5,R=0.5
rejoin_forward：0.8s, L=0.20,R=0.20
```

关键约束：

- 避障时暂停 MapBrain 当前步骤计时。
- 避障失败进入 `OBSTACLE_WAIT`，等待手机端手动接管。
- 急停优先级最高。
- 超声波无效时不能继续自动巡航。

## 4. 物联网日志

树莓派记录 `logs/patrol_events.jsonl`：

```json
{"ts": 0, "event": "route_imported", "route": "factory-patrol-demo"}
{"ts": 0, "event": "route_started"}
{"ts": 0, "event": "obstacle_stop", "dist_mm": 180}
{"ts": 0, "event": "avoidance_finished"}
{"ts": 0, "event": "route_done"}
```

后续手机端/网页端展示：

- 当前路线名。
- 当前步骤。
- 当前传感器距离、温湿度。
- 最近 20 条事件。
- 急停/避障/路线完成状态。

## 5. 下一步编码顺序

1. 在 DevEco Studio 中重新构建并安装 HarmonyOS App，确认“路线轨迹导入”面板可见。
2. 确认树莓派 `8090` 服务从手机热点可访问，再用手机端导入默认路线。
3. 树莓派端补充 `/status` 中的路线导入状态展示。
4. WS63 固件增加周期性传感器上报到树莓派。
5. 树莓派实现 ObstacleBrain 状态机，并与 MapBrain 暂停/恢复联动。
6. 增加 `patrol_events.jsonl` 日志和手机/网页端展示。
