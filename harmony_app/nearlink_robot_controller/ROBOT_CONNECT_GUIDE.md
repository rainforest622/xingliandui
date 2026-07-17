# WS63 Robot SLE Debugger 使用说明

本工程基于华为 NearLink Kit 示例改造，用于连接 WS63 小车的 SLE server。

## 目标小车

- 广播名：`ws63_robot_mvp`
- Service：`0x7100`
- Command property：`0x7101`
- Response property：`0x7102`
- 小车返回格式：`+ROBOT:...`

板端已在 COM5 日志确认广播成功：

```text
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
ROBOT SLE announce enable id=1 status=0x0 name=ws63_robot_mvp
ROBOT_MVP READY protocol=at/sle
```

## 手机要求

- 手机支持星闪/NearLink，例如 HUAWEI Pura 70 Pro+。
- HarmonyOS 5.0.1 Release 或以上。
- DevEco Studio 5.0.1 Release 或以上。
- 手机设置里打开星闪。

常见设置路径：

```text
设置 -> 星闪和蓝牙 -> 星闪
设置 -> 多设备协同 -> 星闪
设置 -> 更多连接 -> 星闪
```

不同系统版本入口名称可能不同。

## 工程打开

用 DevEco Studio 打开：

```text
D:\nearlink-kit_-sample-code-master
```

首次打开后等待依赖同步完成，然后连接 Pura 70 Pro+，选择真机运行。

应用权限：

```text
ohos.permission.ACCESS_NEARLINK
```

首次启动应用时会弹权限申请，需要允许。

## 操作步骤

1. 给小车上电，并确认板端运行 SLE 固件。
2. 确认 COM5 日志里有：

```text
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
```

3. 在手机打开本应用。
4. 首页点击 `Ssap Service`。
5. 默认进入 `Client` 页。
6. 点击 `Scan Robot`。
7. 扫描到 `ws63_robot_mvp` 后点列表里的设备。
8. 点击 `Connect`。
9. 连接成功后，页面会显示：

```text
state: connected
robot service ok
notify enabled
```

10. 先点击 `T` 查询状态。

正常返回类似：

```text
+ROBOT:STATE,<seq>,0,0,0,0,4294967295
```

11. 再按需要测试：

| 按钮 | 含义 | 注意 |
| --- | --- | --- |
| `T` | 查询状态 | 安全 |
| `D` | 查询避障距离 | 安全 |
| `I` | 初始化电机 | 车轮先离地 |
| `S` | 停车/退出自动避障 | 最高优先级 |
| `A` | 自动避障 | 确认场地安全后再用 |
| `O` | 初始化 OLED | 安全 |
| `E` | 读取温湿度 | 安全 |
| `F/B/L/R` | 前/后/左/右 | 会持续运动，必须按 `S` 停 |

## 当前改造内容

主要改动文件：

```text
entry/src/main/ets/pages/SsapClientPage.ets
AppScope/resources/base/element/string.json
entry/src/main/resources/base/element/string.json
entry/src/main/module.json5
```

`SsapClientPage.ets` 已改成小车专用页面：

- 默认扫描 `ws63_robot_mvp`
- 连接后自动 `getServices()`
- 自动订阅 response notify
- 支持写入 `T/I/F/B/L/R/S/D/A/O/E`
- 页面显示最后一条 `+ROBOT:...` 返回

## 如果扫不到

先按顺序排查：

1. 小车是否烧录 SLE 版固件。
2. COM5 是否看到：

```text
ROBOT SLE adv ret=0x0 name=ws63_robot_mvp
```

3. 手机星闪是否打开。
4. App 是否获得 NearLink 权限。
5. Pura 70 Pro+ 系统版本是否满足 HarmonyOS 5.0.1+。
6. 重新按小车 `RESET`，等待广播日志后再扫。

## 如果能扫到但连不上

看页面 `Last response` 或 DevEco 日志。

可能原因：

- 手机端 NearLink Kit 版本和示例工程 SDK 版本不匹配。
- 小车端服务 UUID 端序和手机端字符串表示不一致。
- 小车端已经被其他 central 连接。

本页面会在连接后读取 `getServices()`，并优先使用手机实际发现的 service/property UUID；因此只要 `getServices()` 能返回 `0x7100/0x7101/0x7102`，就不依赖硬编码 UUID。
