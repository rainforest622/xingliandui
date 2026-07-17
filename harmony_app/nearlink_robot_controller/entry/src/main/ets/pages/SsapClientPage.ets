/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2025. All rights reserved.
 * Robot adaptation: connect to the WS63 smart car SLE server.
 */
/// <reference path="../types/NearLinkKitCompat.d.ts" />
import { hilog } from '@kit.PerformanceAnalysisKit';
import { BusinessError } from '@kit.BasicServicesKit';
import { scan, ssap, constant } from '@kit.NearLinkKit';
import { PromptAction } from '@kit.ArkUI'
import { webview } from '@kit.ArkWeb';
import { http } from '@kit.NetworkKit';
import { registerRobotSafetyStop, unregisterRobotSafetyStop } from './RobotSafetyBus';

const ROBOT_DEVICE_NAME: string = 'ws63_robot_mvp';
const DEFAULT_PI_BASE_URL: string = 'http://192.168.43.42:8080';
const DEFAULT_ROUTE_SERVICE_URL: string = 'http://192.168.43.42:8090';
const DEFAULT_CAMERA_PATH: string = '/driver';
const DEFAULT_ROUTE_JSON_TEXT: string = `{
  "name": "industrial-square-patrol-v1",
  "description": "Timing-based patrol brain for an approximately 40 m2 square site. Tune forward durations after measuring real floor distance.",
  "area_m2": 40,
  "default_speed": 0.24,
  "default_turn_speed": 0.5,
  "default_loops": 1,
  "notes": [
    "WAVE ROVER has no wheel encoders, so this map is a timed action plan instead of strict SLAM localization.",
    "The 90 degree turns use the measured 0.8s right-turn parameter.",
    "Use speed_scale=0.8 for the first physical test after floor changes."
  ],
  "waypoints": [
    { "id": "A", "name": "Start / charging side", "x_m": 0.0, "y_m": 0.0 },
    { "id": "B", "name": "North inspection point", "x_m": 6.3, "y_m": 0.0 },
    { "id": "C", "name": "East inspection point", "x_m": 6.3, "y_m": 6.3 },
    { "id": "D", "name": "South inspection point", "x_m": 0.0, "y_m": 6.3 }
  ],
  "steps": [
    { "id": "A_to_B", "name": "巡检边 A 到 B", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "inspect_B", "name": "B 点环境采集", "action": "inspect", "duration_s": 0.8 },
    { "id": "turn_B", "name": "B 点右转 90 度", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "B_to_C", "name": "巡检边 B 到 C", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "inspect_C", "name": "C 点环境采集", "action": "inspect", "duration_s": 0.8 },
    { "id": "turn_C", "name": "C 点右转 90 度", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "C_to_D", "name": "巡检边 C 到 D", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "inspect_D", "name": "D 点环境采集", "action": "inspect", "duration_s": 0.8 },
    { "id": "turn_D", "name": "D 点右转 90 度", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "D_to_A", "name": "巡检边 D 回到 A", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "inspect_A", "name": "A 点回站确认", "action": "inspect", "duration_s": 1.0 },
    { "id": "turn_A", "name": "A 点右转恢复起始朝向", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 }
  ]
}`;
const DEFAULT_ROUTE_JSON_TEXT_SAFE: string = `{
  "name": "industrial-square-patrol-v2-continuous",
  "description": "Continuous timing-based square patrol for an approximately 40 m2 site. Sensor monitoring runs in parallel and does not insert stop steps.",
  "area_m2": 40,
  "default_speed": 0.24,
  "default_turn_speed": 0.5,
  "default_loops": 1,
  "notes": [
    "WAVE ROVER has no wheel encoders, so this route is a timed action plan instead of strict SLAM localization.",
    "The 90 degree turns use the measured 0.8s right-turn parameter.",
    "There are no inspect/wait/stop steps, so each edge naturally flows into the next turn."
  ],
  "waypoints": [
    { "id": "A", "name": "Start side", "x_m": 0.0, "y_m": 0.0 },
    { "id": "B", "name": "North side", "x_m": 6.3, "y_m": 0.0 },
    { "id": "C", "name": "East side", "x_m": 6.3, "y_m": 6.3 },
    { "id": "D", "name": "South side", "x_m": 0.0, "y_m": 6.3 }
  ],
  "steps": [
    { "id": "A_to_B", "name": "Patrol edge A to B", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "turn_B", "name": "Turn right 90 degrees at B", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "B_to_C", "name": "Patrol edge B to C", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "turn_C", "name": "Turn right 90 degrees at C", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "C_to_D", "name": "Patrol edge C to D", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "turn_D", "name": "Turn right 90 degrees at D", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "D_to_A", "name": "Patrol edge D back to A", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "turn_A", "name": "Turn right to restore start heading", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 }
  ]
}`;
const ROBOT_SERVICE_SHORT_UUID: string = '7100';
const ROBOT_COMMAND_SHORT_UUID: string = '7101';
const ROBOT_RESPONSE_SHORT_UUID: string = '7102';
const ROBOT_SERVICE_DISCOVERY_RETRY_MAX: number = 10;
const ROBOT_SERVICE_DISCOVERY_RETRY_MS: number = 500;
const ROBOT_MOTION_REPEAT_MS: number = 300;
const CAMERA_STATUS_POLL_MS: number = 3000;
const CAMERA_RELOAD_FAILURES: number = 2;
const INSPECTION_POLL_MS: number = 1000;
const ROBOT_MONITOR_ALARM_ENV_INVALID: number = 1;
const ROBOT_MONITOR_ALARM_OBSTACLE_INVALID: number = 2;
const ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED: number = 4;
const ROBOT_MONITOR_ALARM_TEMP_HIGH: number = 8;
const ROBOT_MONITOR_ALARM_HUMIDITY_HIGH: number = 16;
const ROBOT_MONITOR_TEMP_HIGH_DECI_C: number = 350;
const ROBOT_MONITOR_HUMIDITY_HIGH_DECI_PERCENT: number = 850;
const PHONE_ENV_ALARM_REPEAT_MS: number = 5000;

interface PiCameraStatusPayload {
  camera_backend?: string;
  camera_error?: string;
  frame_ready?: boolean;
  fps_actual?: number;
  frames_total?: number;
  last_frame_age_sec?: number;
  bandwidth_kbps?: number;
  cpu_percent?: number;
  cpu_temp_c?: number;
}

interface CameraAddress {
  baseUrl: string;
  path: string;
}

interface RouteStepDraft {
  id?: string;
  name?: string;
  action?: string;
  type?: string;
  direction?: string;
  speed?: number;
  left?: number;
  right?: number;
  duration_s?: number;
  duration?: number;
}

interface RouteWaypointDraft {
  id?: string;
  name?: string;
  x_m?: number;
  y_m?: number;
}

interface RouteImportPayload {
  name?: string;
  description?: string;
  area_m2?: number;
  default_speed?: number;
  default_turn_speed?: number;
  default_loops?: number;
  notes?: Array<string>;
  waypoints?: Array<RouteWaypointDraft>;
  steps?: Array<RouteStepDraft>;
}

interface RouteStepStatus {
  id?: string;
  name?: string;
  action?: string;
}

interface RouteMapBrainStatus {
  path?: string;
  name?: string;
  description?: string;
  area_m2?: number;
  active?: boolean;
  finished?: boolean;
  speed_scale?: number;
  loop?: number;
  max_loops?: number;
  step_index?: number;
  step_count?: number;
  step_remaining_s?: number;
  current_step?: RouteStepStatus;
  last_error?: string;
}

interface RouteServicePayload {
  ok?: boolean;
  error?: string;
  mode?: string;
  map_brain?: RouteMapBrainStatus;
  route?: RouteImportPayload;
}

// Fallback UUIDs use the same base as the HiSpark SLE sample. The WS63 firmware
// registers 16-bit UUIDs, so getServices() is still the source of truth.
const ROBOT_SERVICE_UUID_FALLBACK: string = '37bea880-fc70-11ea-b720-000000007100';
const ROBOT_COMMAND_UUID_FALLBACK: string = '37bea880-fc70-11ea-b720-000000007101';
const ROBOT_RESPONSE_UUID_FALLBACK: string = '37bea880-fc70-11ea-b720-000000007102';

let robotServiceUuid: string = ROBOT_SERVICE_UUID_FALLBACK;
let robotCommandUuid: string = ROBOT_COMMAND_UUID_FALLBACK;
let robotResponseUuid: string = ROBOT_RESPONSE_UUID_FALLBACK;
let robotCommandProperty: ssap.Property | undefined = undefined;
let robotResponseProperty: ssap.Property | undefined = undefined;

let client: ssap.Client | undefined = undefined;

function makeImmediateCommandProperty(value: ArrayBuffer): ssap.Property {
  let source = robotCommandProperty;
  let property: ssap.Property = {
    serviceUuid: source === undefined ? robotServiceUuid : source.serviceUuid,
    propertyUuid: source === undefined ? robotCommandUuid : source.propertyUuid,
    value: value
  };
  if (source !== undefined && source.descriptors !== undefined) {
    property.descriptors = source.descriptors;
  }
  if (source !== undefined && source.operation !== undefined) {
    property.operation = source.operation;
  } else {
    property.operation = ssap.Operation.WRITE_WITH_RESPONSE | ssap.Operation.WRITE_NO_RESPONSE;
  }
  return property;
}

export function requestRobotImmediateStop(reason: string): void {
  let activeClient = client;
  if (activeClient === undefined || robotCommandProperty === undefined) {
    return;
  }
  let arrayBuffer = new ArrayBuffer(1);
  let value = new Uint8Array(arrayBuffer);
  value[0] = 'S'.charCodeAt(0);
  let property = makeImmediateCommandProperty(arrayBuffer);
  hilog.info(0x0000, 'RobotSleClient', `immediate safety stop: ${reason}`);
  activeClient.writeProperty(property, ssap.PropertyWriteType.WRITE_NO_RESPONSE).then(() => {
    hilog.info(0x0000, 'RobotSleClient', `immediate safety stop sent: ${reason}`);
  }).catch((err: BusinessError) => {
    hilog.warn(0x0000, 'RobotSleClient', `immediate safety stop failed: ${err.code}, ${err.message}`);
  });
}

@Component
export struct SsapClientPage {
  @State scanResults: Array<scan.ScanResults> = [];
  @State chosenDeviceName: string = '';
  @State chosenDeviceAddr: string = '';
  @State connectionState: string = '未连接';
  @State robotStatus: string = `点击“扫描小车”，找到 ${ROBOT_DEVICE_NAME} 后连接。`;
  @State lastResponse: string = '';
  @State responseSummary: string = '暂无响应';
  @State serviceInfo: string = '尚未发现小车服务';
  @State serviceReady: boolean = false;
  @State commandBusy: boolean = false;
  @State serviceDiscoveryStarted: boolean = false;
  @State activeMotionCommand: string = '';
  @State motorAutoInitIssued: boolean = false;
  @State motorStateText: string = '未知';
  @State motionStateText: string = '未运动';
  @State lastCommandText: string = '暂无';
  @State lastCommandTime: string = '--:--:--';
  @State lastResponseTime: string = '--:--:--';
  @State lastSequenceText: string = '暂无';
  @State cameraBaseUrl: string = DEFAULT_PI_BASE_URL;
  @State cameraBaseDraft: string = DEFAULT_PI_BASE_URL;
  @State cameraUrl: string = `${DEFAULT_PI_BASE_URL}${DEFAULT_CAMERA_PATH}`;
  @State routeBaseUrl: string = DEFAULT_ROUTE_SERVICE_URL;
  @State routeBaseDraft: string = DEFAULT_ROUTE_SERVICE_URL;
  @State routeJsonDraft: string = DEFAULT_ROUTE_JSON_TEXT_SAFE;
  @State routeMoveDurationText: string = '5.0';
  @State routeTurnDurationText: string = '0.8';
  @State routeMoveSpeedText: string = '0.24';
  @State routeTurnSpeedText: string = '0.5';
  @State routeLoopText: string = '1';
  @State routeStatusText: string = '等待连接路线服务';
  @State routeDetailText: string = '默认路线：40 m2 方形巡检，右转 0.8s';
  @State routeLastCheckedText: string = '--:--:--';
  @State routeOnline: boolean = false;
  @State routeImportBusy: boolean = false;
  @State routeControlBusy: boolean = false;
  @State cameraStatus: string = '正在检测树莓派摄像头服务';
  @State cameraHealthText: string = '检测中';
  @State cameraDetailText: string = '状态接口：/status';
  @State cameraLastCheckedText: string = '--:--:--';
  @State cameraCheckDone: boolean = false;
  @State cameraOnline: boolean = false;
  @State cameraFrameReady: boolean = false;
  @State inspectionEnabled: boolean = false;
  @State inspectionStatusText: string = '等待启动巡检';
  @State inspectionDistanceText: string = '--';
  @State inspectionTemperatureText: string = '--';
  @State inspectionHumidityText: string = '--';
  @State inspectionAlarmText: string = '未开始';
  @State inspectionAlarmFlags: number = 0;
  @State inspectionDetailText: string = '自动方形巡检：P 一键启动，G 查轨迹，M 查传感器';
  @State inspectionLastUpdateText: string = '--:--:--';
  @State phoneEnvAlarmActive: boolean = false;
  @State phoneEnvAlarmText: string = '手机端环境报警未触发';

  logTag: string = 'RobotSleClient';
  domainId: number = 0x0000;
  promptAction: PromptAction = this.getUIContext().getPromptAction();
  cameraController: webview.WebviewController = new webview.WebviewController('RobotDriverCameraWeb');
  motionRepeatTimer: number = -1;
  autoInitTimer: number = -1;
  cameraStatusTimer: number = -1;
  inspectionTimer: number = -1;
  inspectionPollBusy: boolean = false;
  lastPhoneEnvAlarmMask: number = 0;
  lastPhoneEnvAlarmToastMs: number = 0;
  cameraProbeBusy: boolean = false;
  cameraFailureCount: number = 0;
  safetyStopCallback: (reason: string) => void = (reason: string) => {
    this.safeStop(reason, false);
  };

  aboutToAppear(): void {
    registerRobotSafetyStop(this.safetyStopCallback);
    this.startCameraMonitor();
    this.startInspectionMonitor();
  }

  aboutToDisappear(): void {
    this.stopCameraMonitor();
    this.stopInspectionMonitor();
    unregisterRobotSafetyStop(this.safetyStopCallback);
    this.safeStop('离开小车控制页', true);
  }

  onPageHide(): void {
    this.stopInspectionMonitor();
    this.safeStop('应用切到后台', false);
  }

  onPageShow(): void {
    this.startInspectionMonitor();
  }

  build() {
    Scroll() {
      Column() {
        this.Header();
        this.CameraPanel();
        this.RouteImportPanel();
        this.CommandPanel();
        this.StatusPanel();
        this.DevicePanel();
        this.ScanPanel();
        this.ResponsePanel();
      }
      .width('100%')
      .padding(12)
    }
    .width('100%')
    .height('100%')
    .backgroundColor('#fff8fafc')
  }

  @Builder
  Header() {
    Column() {
      Row() {
        Column() {
          Text('WS63 星闪小车控制台')
            .fontSize(22)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff0f172a')
          Text(`目标设备：${ROBOT_DEVICE_NAME}`)
            .fontSize(13)
            .fontColor('#ff64748b')
            .margin({ top: 4 })
        }
        .alignItems(HorizontalAlign.Start)
        .layoutWeight(1)

        Text(this.serviceReady ? '已就绪' : '未就绪')
          .fontSize(13)
          .fontWeight(FontWeight.Bold)
          .fontColor(this.serviceReady ? '#ff166534' : '#ff991b1b')
          .padding({ left: 12, right: 12, top: 6, bottom: 6 })
          .backgroundColor(this.serviceReady ? '#ffdcfce7' : '#fffee2e2')
          .borderRadius(8)
      }
      .width('100%')

      Text(`当前状态：${this.connectionState}`)
        .fontSize(13)
        .fontColor(this.serviceReady ? '#ff166534' : '#ffb45309')
        .margin({ top: 8 })
      Text(this.robotStatus)
        .fontSize(13)
        .fontColor('#ff334155')
        .margin({ top: 4 })
    }
    .width('100%')
    .alignItems(HorizontalAlign.Start)
    .padding(12)
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  CameraPanel() {
    Column() {
      Row() {
        Column() {
          Text('巡检驾驶舱')
            .fontSize(18)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff0f172a')
          Text('完整画面、实时巡检和遥控状态集中显示')
            .fontSize(11)
            .fontColor('#ff5b6676')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .alignItems(HorizontalAlign.Start)
        .layoutWeight(1)

        Text(this.cameraHealthText)
          .fontSize(12)
          .fontWeight(FontWeight.Bold)
          .fontColor(this.cameraStatusColor())
          .padding({ left: 10, right: 10, top: 6, bottom: 6 })
          .backgroundColor(this.cameraStatusBgColor())
          .borderRadius(6)
          .margin({ right: 8 })

        Button('刷新')
          .height(34)
          .fontSize(13)
          .backgroundColor('#ff256d63')
          .onClick(() => {
            this.reloadCameraStream();
            this.checkCameraStatus(true);
          })
      }
      .width('100%')

      Stack({ alignContent: Alignment.Bottom }) {
        Web({
          src: this.cameraUrl,
          controller: this.cameraController,
        })
          .width('100%')
          .height('100%')
          .javaScriptAccess(true)
          .domStorageAccess(true)
          .onlineImageAccess(true)
          .imageAccess(true)
          .mixedMode(MixedMode.All)
          .zoomAccess(true)
          .onPageBegin(() => {
            this.cameraStatus = `正在打开 ${this.cameraUrl}`;
          })
          .onPageEnd(() => {
            this.cameraStatus = '页面已加载，等待状态确认';
          });

        if (this.cameraCheckDone && !this.cameraOnline) {
          Column() {
            Text('摄像头服务不可达')
              .fontSize(17)
              .fontWeight(FontWeight.Bold)
              .fontColor('#ffffffff')
            Text(this.cameraStatus)
              .fontSize(12)
              .fontColor('#ffcbd5e1')
              .margin({ top: 6 })
              .maxLines(2)
              .textOverflow({ overflow: TextOverflow.Ellipsis })
          }
          .width('100%')
          .height('100%')
          .justifyContent(FlexAlign.Center)
          .alignItems(HorizontalAlign.Center)
          .padding(16)
          .backgroundColor('#cc0f172a')
        }

        Column() {
          Row() {
            Text(this.cameraFrameReady ? 'LIVE' : (this.cameraOnline ? 'WAIT' : 'OFFLINE'))
              .fontSize(11)
              .fontWeight(FontWeight.Bold)
              .fontColor(this.cameraOnline ? '#ffdcfce7' : '#fffee2e2')
              .padding({ left: 8, right: 8, top: 4, bottom: 4 })
              .backgroundColor(this.cameraOnline ? '#cc166534' : '#cc7f1d1d')
              .borderRadius(5)
            Text(this.cameraStatus)
              .fontSize(11)
              .fontColor('#ffe2e8f0')
              .maxLines(1)
              .textOverflow({ overflow: TextOverflow.Ellipsis })
              .layoutWeight(1)
              .margin({ left: 8 })
          }
          .width('100%')

          Text(this.cameraDetailText)
            .fontSize(10)
            .fontColor('#ffcbd5e1')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .width('100%')
            .margin({ top: 4 })
        }
        .width('100%')
        .padding({ left: 10, right: 10, top: 8, bottom: 8 })
        .backgroundColor('#99050607')
      }
      .width('100%')
      .aspectRatio(1.7777778)
      .margin({ top: 10 })
      .backgroundColor('#ff050607')
      .borderRadius(8)
      .clip(true)

      Row() {
        this.InspectionMetric('距离', this.inspectionDistanceText, this.inspectionAlarmFlags === 0 ? '#fff0fdfa' : '#fffff7ed');
        Blank().width('2%')
        this.InspectionMetric('温度', this.inspectionTemperatureText,
          (this.inspectionAlarmFlags & ROBOT_MONITOR_ALARM_TEMP_HIGH) !== 0 ? '#fffee2e2' : '#fff8fafc');
        Blank().width('2%')
        this.InspectionMetric('湿度', this.inspectionHumidityText,
          (this.inspectionAlarmFlags & ROBOT_MONITOR_ALARM_HUMIDITY_HIGH) !== 0 ? '#fffee2e2' : '#fff8fafc');
      }
      .width('100%')
      .margin({ top: 10 })

      if (this.phoneEnvAlarmActive) {
        this.PhoneEnvAlarmBanner();
      }

      Row() {
        Text(this.inspectionAlarmText)
          .fontSize(12)
          .fontWeight(FontWeight.Bold)
          .fontColor(this.inspectionAlarmColor())
          .padding({ left: 10, right: 10, top: 6, bottom: 6 })
          .backgroundColor(this.inspectionAlarmBgColor())
          .borderRadius(6)
        Text(`${this.inspectionStatusText}；${this.inspectionLastUpdateText}`)
          .fontSize(12)
          .fontColor('#ff475569')
          .maxLines(1)
          .textOverflow({ overflow: TextOverflow.Ellipsis })
          .layoutWeight(1)
          .margin({ left: 8 })
      }
      .width('100%')
      .margin({ top: 8 })

      Text(this.inspectionDetailText)
        .fontSize(12)
        .fontColor('#ff475569')
        .maxLines(2)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%')
        .margin({ top: 6 })

      Row({ space: 8 }) {
        this.DriverMotionButton('L', '←');
        this.DriverMotionButton('F', '↑');
        Button('S')
          .height(44)
          .layoutWeight(1)
          .fontSize(16)
          .fontWeight(FontWeight.Bold)
          .backgroundColor('#ffb91c1c')
          .enabled(this.serviceReady)
          .onClick(() => {
            this.sendStopCommand('驾驶舱急停');
          })
        this.DriverMotionButton('B', '↓');
        this.DriverMotionButton('R', '→');
      }
      .width('100%')
      .margin({ top: 8 })

      Row({ space: 8 }) {
        this.CameraLinkButton('适配', DEFAULT_CAMERA_PATH);
        this.CameraLinkButton('原流', '/stream.mjpg');
        this.CameraLinkButton('面板', '/');
      }
      .width('100%')
      .margin({ top: 10 })

      Row({ space: 8 }) {
        Button(this.inspectionEnabled ? '暂停巡检' : '启动自动巡检')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor(this.inspectionEnabled ? '#ff334155' : '#ff0f766e')
          .onClick(() => {
            this.toggleInspectionMonitor();
          })
        Button('立即巡检')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ff2563eb')
          .enabled(this.serviceReady)
          .onClick(() => {
            this.pollInspection(true);
          })
      }
      .width('100%')
      .margin({ top: 8 })

      Row({ space: 8 }) {
        TextInput({ text: this.cameraBaseDraft, placeholder: 'http://192.168.43.42:8080' })
          .height(38)
          .fontSize(13)
          .layoutWeight(1)
          .backgroundColor('#fff8fafc')
          .borderRadius(6)
          .onChange((value: string) => {
            this.cameraBaseDraft = value;
          })
        Button('应用地址')
          .height(38)
          .fontSize(13)
          .backgroundColor('#ff475569')
          .onClick(() => {
            this.applyCameraBaseUrl();
          })
      }
      .width('100%')
      .margin({ top: 8 })

      Text(`当前源：${this.cameraUrl}；检查 ${this.cameraLastCheckedText}`)
        .fontSize(11)
        .fontColor('#ff64748b')
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%')
        .margin({ top: 6 })
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  RouteImportPanel() {
    Column() {
      Row() {
        Column() {
          Text('路线轨迹导入')
            .fontSize(18)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff0f172a')
          Text('Wi-Fi 传路线到树莓派，星闪保留手动控制和急停')
            .fontSize(11)
            .fontColor('#ff5b6676')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .alignItems(HorizontalAlign.Start)
        .layoutWeight(1)

        Text(this.routeOnline ? '路线服务在线' : '路线服务待检')
          .fontSize(12)
          .fontWeight(FontWeight.Bold)
          .fontColor(this.routeStatusColor())
          .padding({ left: 10, right: 10, top: 6, bottom: 6 })
          .backgroundColor(this.routeStatusBgColor())
          .borderRadius(6)
      }
      .width('100%')

      Row({ space: 8 }) {
        TextInput({ text: this.routeBaseDraft, placeholder: 'http://192.168.43.42:8090' })
          .height(38)
          .fontSize(13)
          .layoutWeight(1)
          .backgroundColor('#fff8fafc')
          .borderRadius(6)
          .onChange((value: string) => {
            this.routeBaseDraft = value;
          })
        Button('应用')
          .height(38)
          .fontSize(13)
          .backgroundColor('#ff475569')
          .onClick(() => {
            this.applyRouteBaseUrl();
          })
      }
      .width('100%')
      .margin({ top: 10 })

      this.RouteVisualEditor()

      Row({ space: 8 }) {
        Button('默认方形')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ff475569')
          .onClick(() => {
            this.resetGraphicalRouteDefaults();
          })
        Button(this.routeImportBusy ? '导入中' : '导入路线')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ff0f766e')
          .enabled(!this.routeImportBusy)
          .onClick(() => {
            this.importRouteToPi();
          })
      }
      .width('100%')
      .margin({ top: 8 })

      Row({ space: 8 }) {
        Button('读取路线')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ff2563eb')
          .enabled(!this.routeControlBusy)
          .onClick(() => {
            this.fetchCurrentRoute(true);
          })
        Button('启动路线')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ff0f766e')
          .enabled(!this.routeControlBusy)
          .onClick(() => {
            this.startImportedRoute();
          })
        Button('停止路线')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ffb91c1c')
          .enabled(!this.routeControlBusy)
          .onClick(() => {
            this.stopImportedRoute();
          })
      }
      .width('100%')
      .margin({ top: 8 })

      Text(this.routeStatusText)
        .fontSize(12)
        .fontColor(this.routeStatusColor())
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%')
        .margin({ top: 8 })
      Text(`${this.routeDetailText}；检查 ${this.routeLastCheckedText}`)
        .fontSize(11)
        .fontColor('#ff64748b')
        .maxLines(2)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%')
        .margin({ top: 4 })
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  CameraLinkButton(label: string, path: string) {
    Button(label)
      .height(34)
      .layoutWeight(1)
      .fontSize(13)
      .backgroundColor(this.cameraUrl === `${this.cameraBaseUrl}${path}` ? '#ff0f766e' : '#ff475569')
      .onClick(() => {
        this.loadCameraPath(path);
      })
  }

  @Builder
  DriverMotionButton(command: string, label: string) {
    Button(label)
      .onTouch((event: TouchEvent) => {
        this.handleMotionTouch(command, event);
      })
      .enabled(this.serviceReady)
      .height(44)
      .layoutWeight(1)
      .fontSize(22)
      .fontWeight(FontWeight.Bold)
      .backgroundColor(this.activeMotionCommand === command ? '#ff0f766e' : '#ff2563eb')
  }

  @Builder
  InspectionPanel() {
    Column() {
      Row() {
        Column() {
          Text('自动方形巡检')
            .fontSize(17)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff0f172a')
          Text(`${this.inspectionStatusText}；更新 ${this.inspectionLastUpdateText}`)
            .fontSize(12)
            .fontColor('#ff64748b')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .alignItems(HorizontalAlign.Start)
        .layoutWeight(1)

        Text(this.inspectionAlarmText)
          .fontSize(12)
          .fontWeight(FontWeight.Bold)
          .fontColor(this.inspectionAlarmColor())
          .padding({ left: 10, right: 10, top: 6, bottom: 6 })
          .backgroundColor(this.inspectionAlarmBgColor())
          .borderRadius(8)
      }
      .width('100%')

      Row() {
        this.InspectionMetric('距离', this.inspectionDistanceText, this.inspectionAlarmFlags === 0 ? '#fff1f5f9' : '#fffff7ed');
        Blank().width('2%')
        this.InspectionMetric('温度', this.inspectionTemperatureText,
          (this.inspectionAlarmFlags & ROBOT_MONITOR_ALARM_TEMP_HIGH) !== 0 ? '#fffee2e2' : '#fff1f5f9');
        Blank().width('2%')
        this.InspectionMetric('湿度', this.inspectionHumidityText,
          (this.inspectionAlarmFlags & ROBOT_MONITOR_ALARM_HUMIDITY_HIGH) !== 0 ? '#fffee2e2' : '#fff1f5f9');
      }
      .width('100%')
      .margin({ top: 10 })

      if (this.phoneEnvAlarmActive) {
        this.PhoneEnvAlarmBanner();
      }

      Text(this.inspectionDetailText)
        .fontSize(12)
        .fontColor('#ff475569')
        .maxLines(2)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%')
        .margin({ top: 8 })

      Row({ space: 8 }) {
        Button(this.inspectionEnabled ? '暂停巡检' : '启动自动方形巡检')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor(this.inspectionEnabled ? '#ff334155' : '#ff0f766e')
          .onClick(() => {
            this.toggleInspectionMonitor();
          })
        Button('立即刷新')
          .height(36)
          .layoutWeight(1)
          .fontSize(13)
          .backgroundColor('#ff2563eb')
          .enabled(this.serviceReady)
          .onClick(() => {
            this.pollInspection(true);
          })
      }
      .width('100%')
      .margin({ top: 10 })
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  PhoneEnvAlarmBanner() {
    Row() {
      Text('手机端报警')
        .fontSize(12)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ffffffff')
        .padding({ left: 10, right: 10, top: 5, bottom: 5 })
        .backgroundColor('#ffb91c1c')
        .borderRadius(6)
      Text(this.phoneEnvAlarmText)
        .fontSize(12)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff991b1b')
        .maxLines(2)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .layoutWeight(1)
        .margin({ left: 8 })
    }
    .width('100%')
    .padding({ left: 8, right: 8, top: 8, bottom: 8 })
    .margin({ top: 8 })
    .backgroundColor('#fffee2e2')
    .borderRadius(8)
    .border({ width: 1, color: '#fffca5a5' })
  }

  @Builder
  InspectionMetric(label: string, value: string, bgColor: string) {
    Column() {
      Text(label)
        .fontSize(11)
        .fontColor('#ff64748b')
      Text(value)
        .fontSize(18)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff0f172a')
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .margin({ top: 4 })
    }
    .width('32%')
    .height(70)
    .padding(8)
    .alignItems(HorizontalAlign.Start)
    .backgroundColor(bgColor)
    .borderRadius(8)
  }

  @Builder
  StatusPanel() {
    Column() {
      Text('状态概览')
        .fontSize(16)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff0f172a')
        .width('100%')

      Row() {
        Column() {
          Text('连接')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.connectionState)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor(this.serviceReady ? '#ff166534' : '#ffb45309')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor(this.serviceReady ? '#ffdcfce7' : '#fffff7ed')
        .borderRadius(8)

        Blank().width('2%')

        Column() {
          Text('电机')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.motorStateText)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor(this.motorStateText === '已就绪' ? '#ff166534' : '#ff991b1b')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor(this.motorStateText === '已就绪' ? '#ffdcfce7' : '#fffee2e2')
        .borderRadius(8)

        Blank().width('2%')

        Column() {
          Text('运动')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.motionStateText)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor(this.motionStateText === '已停止' || this.motionStateText === '未运动' ? '#ff475569' : '#ff0369a1')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor(this.motionStateText === '已停止' || this.motionStateText === '未运动' ? '#fff1f5f9' : '#ffe0f2fe')
        .borderRadius(8)
      }
      .width('100%')
      .margin({ top: 8 })

      Row() {
        Column() {
          Text('最近命令')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.lastCommandText)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff334155')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor('#fff1f5f9')
        .borderRadius(8)

        Blank().width('2%')

        Column() {
          Text('发送时间')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.lastCommandTime)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff334155')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor('#fff1f5f9')
        .borderRadius(8)

        Blank().width('2%')

        Column() {
          Text('返回时间')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.lastResponseTime)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff334155')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor('#fff1f5f9')
        .borderRadius(8)
      }
      .width('100%')
      .margin({ top: 8 })

      Row() {
        Column() {
          Text('固件序号')
            .fontSize(11)
            .fontColor('#ff64748b')
          Text(this.lastSequenceText)
            .fontSize(14)
            .fontWeight(FontWeight.Bold)
            .fontColor('#ff334155')
            .maxLines(1)
            .textOverflow({ overflow: TextOverflow.Ellipsis })
            .margin({ top: 2 })
        }
        .width('31%')
        .height(62)
        .padding(8)
        .alignItems(HorizontalAlign.Start)
        .backgroundColor('#fff1f5f9')
        .borderRadius(8)
      }
      .width('100%')
      .margin({ top: 8 })
    }
    .width('100%')
    .alignItems(HorizontalAlign.Start)
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  ScanPanel() {
    Column() {
      Row() {
        Button('扫描小车')
          .onClick(() => {
            this.startRobotScan();
          })
          .width('45%')
        Blank().width('5%')
        Button('停止扫描')
          .onClick(() => {
            this.stopScan();
            this.offDeviceFound();
          })
          .width('45%')
      }
      .width('100%')
      .margin({ top: 10 })

      Row() {
        Text(this.robotStatus)
          .fontSize(13)
          .fontColor('#ff0f766e')
      }
      .width('100%')
      .margin({ top: 6, bottom: 6 })

      if (this.scanResults.length !== 0) {
        List() {
          ForEach(this.scanResults, (item: scan.ScanResults) => {
            ListItem() {
              Column() {
                Text(item.deviceName.length !== 0 ? item.deviceName : '未命名星闪设备')
                  .fontSize(15)
                  .fontWeight(600)
                  .maxLines(1)
                  .textOverflow({ overflow: TextOverflow.Ellipsis })
                Text(item.address)
                  .fontSize(13)
                  .fontColor('#ff64748b')
              }
              .width('100%')
              .padding(8)
              .backgroundColor(item.address === this.chosenDeviceAddr ? '#ffe0f2fe' : '#fff1f5f9')
              .borderRadius(8)
              .onClick(() => {
                this.chosenDeviceName = item.deviceName;
                this.chosenDeviceAddr = item.address;
                this.robotStatus = `已选择：${item.deviceName} [${item.address}]`;
              })
            }
          }, (item: scan.ScanResults) => item.address)
        }
        .height('22%')
        .width('100%')
      } else {
        Text('还没有扫描到小车。确认小车已上电并正在广播后，再点击“扫描小车”。')
          .fontSize(13)
          .height('8%')
          .width('100%')
          .textAlign(TextAlign.Center)
          .fontColor('#ff64748b')
      }
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  DevicePanel() {
    Column() {
      Text('连接与服务')
        .fontSize(16)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff0f172a')
        .width('100%')

      Text(this.chosenDeviceAddr.length !== 0 ?
        `已选设备：${this.chosenDeviceName} [${this.chosenDeviceAddr}]` :
        '已选设备：暂无，请先扫描并点选小车')
        .fontSize(13)
        .maxLines(2)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%')
        .fontColor('#ff334155')
        .margin({ top: 6 })

      Text(this.serviceInfo)
        .fontSize(12)
        .fontColor(this.serviceReady ? '#ff166534' : '#ffb45309')
        .width('100%')
        .margin({ top: 4 })

      Row() {
        Button('连接小车')
          .onClick(() => {
            this.connectDevice(this.chosenDeviceAddr);
          })
          .enabled(this.chosenDeviceAddr.length !== 0 && this.connectionState !== '连接中')
          .width('30%')
        Blank().width('3%')
        Button('读取状态')
          .onClick(() => {
            this.writeCommand('T');
          })
          .enabled(this.serviceReady)
          .width('30%')
        Blank().width('3%')
        Button('断开')
          .onClick(() => {
            this.closeClient();
          })
          .width('30%')
      }
      .width('100%')
      .margin({ top: 8, bottom: 8 })

      Row() {
        Button('Retry 7100')
          .onClick(() => {
            this.retryServiceDiscovery();
          })
          .enabled(client !== undefined)
          .width('45%')
        Blank().width('5%')
        Button('Clear Busy')
          .onClick(() => {
            this.clearBusyState();
          })
          .width('45%')
      }
      .width('100%')
      .margin({ top: 4, bottom: 4 })
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  CommandPanel() {
    Column() {
      Text('遥控控制')
        .fontSize(16)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff0f172a')
        .width('100%')
      Text('服务就绪后自动初始化电机；按住方向键每 300ms 补发一次，松手/切后台/离开页面都会发送 S 停止。')
        .fontSize(12)
        .fontColor('#ffb45309')
        .width('100%')
        .margin({ bottom: 6 })
      Text(this.serviceReady ? '小车服务已就绪，可以遥控。' : '请先完成连接并发现 7100/7101/7102 服务后再操作。')
        .fontSize(12)
        .fontColor(this.serviceReady ? '#ff166534' : '#ff991b1b')
        .width('100%')
        .margin({ bottom: 6 })

      Column() {
        Row() {
          Blank().width('34%')
          this.MotionButton('F', '前进 F')
          Blank().width('34%')
        }
        .width('100%')

        Row() {
          this.MotionButton('L', '左转 L')
          Button('急停 S')
            .onClick(() => {
              this.sendStopCommand('急停');
            })
            .enabled(this.serviceReady)
            .width('32%')
            .height(54)
            .margin({ right: 6, bottom: 6 })
            .backgroundColor('#ffb91c1c')
          this.MotionButton('R', '右转 R')
        }
        .width('100%')

        Row() {
          Blank().width('34%')
          this.MotionButton('B', '后退 B')
          Blank().width('34%')
        }
        .width('100%')
      }
      .width('100%')

      Text('辅助功能')
        .fontSize(15)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff0f172a')
        .width('100%')
        .margin({ top: 10, bottom: 6 })

      Flex({ wrap: FlexWrap.Wrap }) {
        this.ActionButton('T', '状态 T');
        this.ActionButton('I', '电机 I');
        this.ActionButton('D', '测距 D');
        this.ActionButton('A', '避障 A');
        this.ActionButton('P', '自动巡检 P');
        this.ActionButton('G', '轨迹 G');
        this.ActionButton('E', '温湿度 E');
        this.ActionButton('M', '巡检 M');
        this.ActionButton('O', 'OLED O');
      }
      .width('100%')
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  @Builder
  MotionButton(command: string, label: string) {
    Button(label)
      .onTouch((event: TouchEvent) => {
        this.handleMotionTouch(command, event);
      })
      .enabled(this.serviceReady)
      .width('32%')
      .height(54)
      .margin({ right: 6, bottom: 6 })
      .backgroundColor(this.activeMotionCommand === command ? '#ff0f766e' : '#ff2563eb')
  }

  @Builder
  ActionButton(command: string, label: string) {
    Button(label)
      .onClick(() => {
        this.writeCommand(command);
      })
      .enabled(this.serviceReady && !this.commandBusy)
      .width('30%')
      .height(40)
      .margin({ right: 6, bottom: 6 })
      .backgroundColor('#ff475569')
  }

  @Builder
  ResponsePanel() {
    Column() {
      Text('小车返回')
        .fontSize(16)
        .fontWeight(FontWeight.Bold)
        .fontColor('#ff0f172a')
        .width('100%')
      Text(this.responseSummary)
        .fontSize(13)
        .fontColor('#ff334155')
        .width('100%')
        .padding(8)
        .margin({ top: 6, bottom: 6 })
        .backgroundColor('#ffe0f2fe')
        .borderRadius(8)
      Scroll() {
        Text(this.lastResponse.length !== 0 ? this.lastResponse : '暂无原始返回。')
          .fontSize(13)
          .fontFamily('monospace')
          .width('100%')
          .padding(8)
      }
      .width('100%')
      .height('20%')
      .backgroundColor('#fff1f5f9')
      .borderRadius(8)
    }
    .width('100%')
    .padding(12)
    .margin({ top: 10, bottom: 10 })
    .backgroundColor('#ffffffff')
    .borderRadius(8)
  }

  startRobotScan() {
    this.scanResults = [];
    this.chosenDeviceName = '';
    this.chosenDeviceAddr = '';
    this.resetRobotServiceState();
    this.robotStatus = `正在扫描 ${ROBOT_DEVICE_NAME}...`;
    this.onDeviceFound();
    try {
      let scanOptions: scan.ScanOptions = {
        scanMode: 2
      };
      let scanFilter: scan.ScanFilters = {
        deviceName: ROBOT_DEVICE_NAME
      };
      scan.startScan([scanFilter], scanOptions)
        .then(() => {
          hilog.info(this.domainId, this.logTag, 'start robot scan success');
        });
    } catch (err) {
      this.handleError('start scan', err as BusinessError);
    }
  }

  stopScan() {
    try {
      scan.stopScan()
        .then(() => {
          hilog.info(this.domainId, this.logTag, 'stop scan success');
          this.robotStatus = '已停止扫描';
        });
    } catch (err) {
      this.handleError('stop scan', err as BusinessError);
    }
  }

  onDeviceFound() {
    let scanCallback: (data: Array<scan.ScanResults>) => void = (data: Array<scan.ScanResults>) => {
      for (let indexData = 0; indexData < data.length; indexData++) {
        let item = data[indexData];
        let exists = this.scanResults.some((result: scan.ScanResults) => result.address === item.address);
        if (!exists) {
          this.scanResults.push(item);
          hilog.info(this.domainId, this.logTag, `nearlink robot = ${JSON.stringify(item)}`);
          if (item.deviceName === ROBOT_DEVICE_NAME || this.chosenDeviceAddr.length === 0) {
            this.chosenDeviceName = item.deviceName;
            this.chosenDeviceAddr = item.address;
            this.robotStatus = `已发现：${item.deviceName} [${item.address}]`;
          }
        }
      }
    }
    try {
      scan.off('deviceFound');
      scan.on('deviceFound', scanCallback);
    } catch (err) {
      this.handleError('register scan callback', err as BusinessError);
    }
  }

  offDeviceFound() {
    try {
      scan.off('deviceFound');
    } catch (err) {
      this.handleError('unregister scan callback', err as BusinessError);
    }
  }

  connectDevice(address: string) {
    if (address.length === 0) {
      this.robotStatus = '请先扫描并选择小车';
      return;
    }
    try {
      this.stopScanQuiet();
      this.closeActiveClient();
      this.resetRobotServiceState();
      let newClient = ssap.createClient(address);
      client = newClient;
      this.connectionState = '连接中';
      this.serviceInfo = '正在建立星闪链路';
      this.onConnectionStateChange();
      this.onPropertyChange();
      newClient.connect().then(() => {
        if (client !== newClient) {
          return;
        }
        hilog.info(this.domainId, this.logTag, 'connect success');
        this.robotStatus = '连接请求已发出，等待链路状态确认...';
        this.connectionState = 'connected';
        this.robotStatus = 'SLE connected; discovering 7100 service...';
        this.serviceInfo = 'connect() resolved; starting getServices()';
        this.startServiceDiscovery(newClient);
      }).catch((err: BusinessError) => {
        this.handleError('connect', err);
      });
    } catch (err) {
      this.handleError('connect', err as BusinessError);
    }
  }

  startServiceDiscovery(expectedClient: ssap.Client) {
    if (this.serviceDiscoveryStarted || this.serviceReady) {
      return;
    }
    this.serviceDiscoveryStarted = true;
    this.robotStatus = '链路已连接，正在发现小车服务...';
    this.serviceInfo = '正在发现 7100 服务、7101 命令特征、7102 响应特征';
    setTimeout(() => {
      if (client === expectedClient && this.connectionState !== '未连接') {
        this.getServicesAndEnableNotify(1, expectedClient);
      }
    }, ROBOT_SERVICE_DISCOVERY_RETRY_MS);
  }

  getServicesAndEnableNotify(attempt: number, expectedClient: ssap.Client) {
    let activeClient = client;
    if (activeClient === undefined || activeClient !== expectedClient || this.connectionState === '未连接') {
      return;
    }
    try {
      activeClient.getServices().then((result: Array<ssap.Service>) => {
        if (activeClient !== client || activeClient !== expectedClient || this.connectionState === '未连接') {
          return;
        }
        let ready = this.configureRobotProperties(result);
        if (!ready) {
          if (attempt < ROBOT_SERVICE_DISCOVERY_RETRY_MAX) {
            this.robotStatus = `小车服务暂未就绪，重试 ${attempt}/${ROBOT_SERVICE_DISCOVERY_RETRY_MAX}`;
            setTimeout(() => {
              this.getServicesAndEnableNotify(attempt + 1, expectedClient);
            }, ROBOT_SERVICE_DISCOVERY_RETRY_MS);
          } else {
            this.connectionState = '链路已连接';
            this.robotStatus = '链路已连接，但没有发现小车服务';
            this.serviceInfo = '未发现小车服务；建议重启板子后断开重连';
          }
          return;
        }
        this.serviceDiscoveryStarted = false;
        this.setResponseNotification();
        this.readResponseProperty();
      }).catch((err: BusinessError) => {
        if (activeClient !== client || activeClient !== expectedClient || this.connectionState === '未连接') {
          return;
        }
        if (attempt < ROBOT_SERVICE_DISCOVERY_RETRY_MAX) {
          this.robotStatus = `读取服务失败，重试 ${attempt}/${ROBOT_SERVICE_DISCOVERY_RETRY_MAX}`;
          setTimeout(() => {
            this.getServicesAndEnableNotify(attempt + 1, expectedClient);
          }, ROBOT_SERVICE_DISCOVERY_RETRY_MS);
        } else {
          this.serviceDiscoveryStarted = false;
          this.handleError('get services', err);
        }
      });
    } catch (err) {
      this.handleError('get services', err as BusinessError);
    }
  }

  configureRobotProperties(result: Array<ssap.Service>): boolean {
    let serviceFound = false;
    let commandFound = false;
    let responseFound = false;
    let summary = '';

    for (let indexService = 0; indexService < result.length; indexService++) {
      let service = result[indexService];
      summary += `S:${service.serviceUuid}\n`;
      if (this.isRobotUuid(service.serviceUuid, ROBOT_SERVICE_SHORT_UUID)) {
        serviceFound = true;
        robotServiceUuid = service.serviceUuid;
        if (service.properties !== undefined) {
          for (let indexProperty = 0; indexProperty < service.properties.length; indexProperty++) {
            let prop = service.properties[indexProperty];
            summary += `  P:${prop.propertyUuid} op=${prop.operation}\n`;
            if (this.isRobotUuid(prop.propertyUuid, ROBOT_COMMAND_SHORT_UUID)) {
              robotCommandUuid = prop.propertyUuid;
              robotCommandProperty = prop;
              commandFound = true;
            }
            if (this.isRobotUuid(prop.propertyUuid, ROBOT_RESPONSE_SHORT_UUID)) {
              robotResponseUuid = prop.propertyUuid;
              robotResponseProperty = prop;
              responseFound = true;
            }
          }
        }
      }
    }

    this.serviceReady = serviceFound && commandFound && responseFound;
    this.connectionState = this.serviceReady ? '服务就绪' : '链路已连接';
    this.serviceInfo = this.serviceReady ?
      '小车服务就绪：7100 服务正常，7101 命令可写，7102 响应可读/可通知' :
      `小车服务不完整：服务=${serviceFound ? '正常' : '缺失'}，命令=${commandFound ? '正常' : '缺失'}，响应=${responseFound ? '正常' : '缺失'}`;
    this.inspectionStatusText = this.serviceReady ? '自动巡检已就绪' : '等待星闪服务';
    this.lastResponse = `${this.lastResponse}\nservices:\n${summary}`.trim();
    return this.serviceReady;
  }

  enableFallbackRobotService(reason: string) {
    if (client === undefined) {
      this.robotStatus = 'No active SLE client for fallback';
      return;
    }

    robotServiceUuid = ROBOT_SERVICE_UUID_FALLBACK;
    robotCommandUuid = ROBOT_COMMAND_UUID_FALLBACK;
    robotResponseUuid = ROBOT_RESPONSE_UUID_FALLBACK;
    robotCommandProperty = {
      serviceUuid: robotServiceUuid,
      propertyUuid: robotCommandUuid,
      value: new ArrayBuffer(1),
      operation: ssap.Operation.WRITE_WITH_RESPONSE | ssap.Operation.WRITE_NO_RESPONSE
    };
    robotResponseProperty = {
      serviceUuid: robotServiceUuid,
      propertyUuid: robotResponseUuid,
      value: new ArrayBuffer(1),
      operation: ssap.Operation.READABLE | ssap.Operation.NOTIFY
    };

    this.serviceReady = true;
    this.commandBusy = false;
    this.serviceDiscoveryStarted = false;
    this.connectionState = 'fallback ready';
    this.serviceInfo = `Fallback 7100 ready (${reason})`;
    this.robotStatus = 'Fallback ready: send T first';
    this.inspectionStatusText = '自动巡检已就绪';
    this.updateRobotResponse(`fallback service=${robotServiceUuid} command=${robotCommandUuid} response=${robotResponseUuid}`);
  }

  clearBusyState() {
    this.commandBusy = false;
    this.serviceDiscoveryStarted = false;
    this.robotStatus = 'Busy state cleared';
    if (!this.serviceReady) {
      this.serviceInfo = 'Service discovery reset; reconnect or retry 7100';
    }
  }

  retryServiceDiscovery() {
    let activeClient = client;
    if (activeClient === undefined) {
      this.robotStatus = 'No active SLE client; scan and connect first';
      return;
    }
    this.commandBusy = false;
    this.serviceReady = false;
    this.serviceDiscoveryStarted = false;
    this.motorAutoInitIssued = false;
    robotCommandProperty = undefined;
    robotResponseProperty = undefined;
    this.connectionState = 'connected';
    this.robotStatus = 'Retrying 7100 service discovery...';
    this.serviceInfo = 'Manual retry: getServices()';
    this.startServiceDiscovery(activeClient);
  }

  setResponseNotification() {
    let activeClient = client;
    if (activeClient === undefined) {
      return;
    }
    try {
      activeClient.setPropertyNotification(this.makeResponseProperty(), true).then(() => {
        hilog.info(this.domainId, this.logTag, 'set response notification success');
        this.robotStatus = '小车服务已就绪；正在自动初始化电机 I';
        this.motionStateText = '已停止';
      }).catch((err: BusinessError) => {
        this.robotStatus = `通知开启失败，将改用读取响应 (${err.code})`;
        hilog.warn(this.domainId, this.logTag, `enable notify ignored: ${err.code}, ${err.message}`);
        this.motionStateText = '已停止';
      });
    } catch (err) {
      this.handleError('enable notify', err as BusinessError);
    }
  }

  onPropertyChange() {
    let activeClient = client;
    if (activeClient === undefined) {
      return;
    }
    let propertyChangeCallback:(data: ssap.Property) => void = (data: ssap.Property) => {
      if (activeClient !== client) {
        return;
      }
      let text = this.bufferToText(data.value);
      this.updateRobotResponse(text.length !== 0 ? text : JSON.stringify(data));
      hilog.info(this.domainId, this.logTag, `notify: ${this.lastResponse}`);
    }
    try {
      activeClient.on('propertyChange', propertyChangeCallback);
    } catch (err) {
      this.handleError('register notify callback', err as BusinessError);
    }
  }

  onConnectionStateChange() {
    let activeClient = client;
    if (activeClient === undefined) {
      return;
    }
    let expectedClient: ssap.Client = activeClient;
    let connectionStateChangeCallback:(data: ssap.ConnectionChangeState) => void =
      (data: ssap.ConnectionChangeState) => {
        if (expectedClient !== client) {
          return;
        }
        hilog.info(this.domainId, this.logTag, 'state:'+ JSON.stringify(data));
        if (data.state === constant.ConnectionState.STATE_CONNECTED) {
          if (!this.serviceReady) {
            this.connectionState = '链路已连接';
            this.startServiceDiscovery(expectedClient);
          }
        } else if (data.state === constant.ConnectionState.STATE_DISCONNECTED) {
          this.connectionState = '未连接';
          this.clearAutoInitTimer();
          this.stopMotionRepeat();
          this.activeMotionCommand = '';
          this.resetRobotServiceState();
          this.robotStatus = '链路已断开；请断开后重新连接';
        }
    }
    try {
      activeClient.on('connectionStateChange', connectionStateChangeCallback);
    } catch (err) {
      this.handleError('register connection callback', err as BusinessError);
    }
  }

  readResponseProperty() {
    let activeClient = client;
    if (activeClient === undefined) {
      this.robotStatus = '尚未连接小车';
      return;
    }
    if (!this.serviceReady) {
      this.robotStatus = '小车服务尚未就绪';
      this.updateRobotResponse('暂时不能读取：还没有发现小车服务。');
      return;
    }
    try {
      activeClient.readProperty(this.makeResponseProperty()).then((result: ssap.Property) => {
        let text = this.bufferToText(result.value);
        this.updateRobotResponse(text.length !== 0 ? text : JSON.stringify(result));
        hilog.info(this.domainId, this.logTag, `read response: ${this.lastResponse}`);
      }).catch((err: BusinessError) => {
        this.handleError('read response', err);
      });
    } catch (err) {
      this.handleError('read response', err as BusinessError);
    }
  }

  startInspectionMonitor() {
    this.stopInspectionMonitor();
    this.inspectionStatusText = this.serviceReady ? '自动方形巡检待启动' : '等待星闪服务';
    this.inspectionTimer = setInterval(() => {
      this.pollInspection(false);
    }, INSPECTION_POLL_MS);
    this.pollInspection(true);
  }

  stopInspectionMonitor() {
    if (this.inspectionTimer !== -1) {
      clearInterval(this.inspectionTimer);
      this.inspectionTimer = -1;
    }
    this.inspectionPollBusy = false;
  }

  toggleInspectionMonitor() {
    if (!this.inspectionEnabled) {
      if (!this.serviceReady || client === undefined) {
        this.inspectionStatusText = '等待星闪服务';
        return;
      }
      this.inspectionEnabled = true;
      this.inspectionStatusText = '正在启动自动方形巡检';
      this.writeCommand('P', true, false);
      setTimeout(() => {
        this.pollInspection(true);
      }, 800);
    } else {
      this.inspectionEnabled = false;
      this.inspectionStatusText = '自动方形巡检已暂停';
      this.sendStopCommand('暂停巡检');
    }
  }

  pollInspection(force: boolean) {
    if (!this.inspectionEnabled) {
      return;
    }
    if (!this.serviceReady || client === undefined) {
      if (force) {
        this.inspectionStatusText = '等待星闪服务';
      }
      return;
    }
    if (this.inspectionPollBusy && !force) {
      return;
    }
    this.inspectionPollBusy = true;
    this.inspectionStatusText = '正在读取巡检快照';
    if (this.inspectionEnabled) {
      this.writeCommand('G', true, false);
      setTimeout(() => {
        if (this.inspectionEnabled && this.serviceReady) {
          this.writeCommand('M', true, false);
        }
      }, 350);
    } else {
      this.writeCommand('M', true, false);
    }
    setTimeout(() => {
      this.inspectionPollBusy = false;
    }, Math.max(INSPECTION_POLL_MS - 100, 300));
  }

  tryAutoInitMotor() {
    if (!this.serviceReady || this.motorAutoInitIssued) {
      return;
    }
    this.motorAutoInitIssued = true;
    this.clearAutoInitTimer();
    this.autoInitTimer = setTimeout(() => {
      this.autoInitTimer = -1;
      if (!this.serviceReady || client === undefined) {
        return;
      }
      this.robotStatus = '小车服务已就绪，正在自动初始化电机 I...';
      this.motorStateText = '初始化中';
      this.writeCommand('I', true, false);
    }, 150);
  }

  clearAutoInitTimer() {
    if (this.autoInitTimer !== -1) {
      clearTimeout(this.autoInitTimer);
      this.autoInitTimer = -1;
    }
  }

  handleMotionTouch(command: string, event: TouchEvent) {
    if (event.type === TouchType.Down) {
      this.beginMotion(command);
      return;
    }
    if (event.type === TouchType.Up || event.type === TouchType.Cancel) {
      this.endMotion(command);
    }
  }

  beginMotion(command: string) {
    if (!this.serviceReady) {
      this.robotStatus = '小车服务尚未就绪';
      return;
    }
    if (this.activeMotionCommand === command) {
      return;
    }
    this.stopMotionRepeat();
    this.activeMotionCommand = command;
    this.motionStateText = `${this.motionName(command)}中`;
    this.robotStatus = `按住中：${this.motionName(command)}，正在持续发送 ${command}`;
    this.writeCommand(command, false, false);
    this.motionRepeatTimer = setInterval(() => {
      if (this.activeMotionCommand === command && this.serviceReady) {
        this.writeCommand(command, false, false);
      }
    }, ROBOT_MOTION_REPEAT_MS);
  }

  endMotion(command: string) {
    if (this.activeMotionCommand.length === 0) {
      return;
    }
    let endedCommand = this.activeMotionCommand;
    this.stopMotionRepeat();
    this.activeMotionCommand = '';
    this.sendStopCommand(`松开 ${this.motionName(endedCommand)}`);
  }

  sendStopCommand(reason: string) {
    if (!this.serviceReady) {
      this.robotStatus = '小车服务尚未就绪';
      return;
    }
    this.stopMotionRepeat();
    this.activeMotionCommand = '';
    this.motionStateText = '停止中';
    this.robotStatus = `${reason}，已发送停止 S`;
    this.writeCommand('S', true, false);
  }

  stopMotionRepeat() {
    if (this.motionRepeatTimer !== -1) {
      clearInterval(this.motionRepeatTimer);
      this.motionRepeatTimer = -1;
    }
  }

  safeStop(reason: string, closeAfterStop: boolean) {
    this.clearAutoInitTimer();
    this.stopMotionRepeat();
    this.activeMotionCommand = '';
    this.motionStateText = '已停止';

    let canWriteStop = client !== undefined && this.serviceReady && robotCommandProperty !== undefined;
    if (canWriteStop) {
      this.robotStatus = `${reason}，安全停车：已发送停止 S`;
      this.writeCommand('S', false, false);
    } else {
      this.robotStatus = `${reason}，已清除运动定时器`;
    }

    if (closeAfterStop) {
      setTimeout(() => {
        this.closeActiveClient();
        this.connectionState = '未连接';
        this.resetRobotServiceState();
      }, canWriteStop ? 180 : 0);
    }
  }

  recordCommandSent(command: string) {
    this.lastCommandText = `${this.commandName(command)} (${command})`;
    this.lastCommandTime = this.nowText();
    if (this.isMotionCommand(command)) {
      this.motionStateText = `${this.motionName(command)}中`;
    } else if (command === 'S') {
      this.motionStateText = '已停止';
    } else if (command === 'I' && this.motorStateText !== '已就绪') {
      this.motorStateText = '初始化中';
    }
  }

  writeCommand(command: string, readAfterWrite: boolean = true, markBusy: boolean = true) {
    let activeClient = client;
    if (activeClient === undefined) {
      this.robotStatus = '尚未连接小车';
      return;
    }
    if (!this.serviceReady || robotCommandProperty === undefined) {
      this.robotStatus = '小车服务尚未就绪';
      this.updateRobotResponse('暂时不能发送命令：还没有发现小车服务。');
      return;
    }
    if (command.length === 0) {
      return;
    }
    try {
      if (markBusy) {
        this.commandBusy = true;
      }
      let arrayBuffer = new ArrayBuffer(command.length);
      let value = new Uint8Array(arrayBuffer);
      for (let index = 0; index < command.length; index++) {
        value[index] = command.charCodeAt(index) & 0xFF;
      }
      let property = this.makeCommandProperty(arrayBuffer);
      let writeType = this.chooseWriteType(property);
      hilog.info(this.domainId, this.logTag,
        `write command ${command} service=${property.serviceUuid} property=${property.propertyUuid} operation=${property.operation} type=${writeType}`);
      this.writePropertyWithFallback(activeClient, property, command, writeType, readAfterWrite, markBusy);
    } catch (err) {
      if (markBusy) {
        this.commandBusy = false;
      }
      this.handleError(`write ${command}`, err as BusinessError);
    }
  }

  writePropertyWithFallback(
    activeClient: ssap.Client,
    property: ssap.Property,
    command: string,
    writeType: ssap.PropertyWriteType,
    readAfterWrite: boolean,
    markBusy: boolean
  ) {
    activeClient.writeProperty(property, writeType).then(() => {
      if (markBusy) {
        this.commandBusy = false;
      }
      this.recordCommandSent(command);
      if (command === 'S') {
        this.robotStatus = '已发送停止 S';
        this.motionStateText = '已停止';
      } else if (!this.isMotionCommand(command)) {
        this.robotStatus = `已发送 ${this.commandName(command)} (${command})`;
      }
      hilog.info(this.domainId, this.logTag, `write command ${command} success type=${writeType}`);
      if (readAfterWrite) {
        setTimeout(() => {
          this.readResponseProperty();
        }, 300);
      }
    }).catch((err: BusinessError) => {
      if (writeType === ssap.PropertyWriteType.WRITE && err.code === 401) {
        hilog.warn(this.domainId, this.logTag, `write command ${command} retry no response after 401`);
        this.writePropertyWithFallback(activeClient, property, command, ssap.PropertyWriteType.WRITE_NO_RESPONSE,
          readAfterWrite, markBusy);
        return;
      }
      if (markBusy) {
        this.commandBusy = false;
      }
      this.handleError(`write ${command}`, err);
    });
  }

  closeClient() {
    try {
      this.safeStop('手动断开前', true);
      this.robotStatus = '已发送安全停车并断开连接';
    } catch (err) {
      this.handleError('close client', err as BusinessError);
    }
  }

  closeActiveClient() {
    let activeClient = client;
    if (activeClient === undefined) {
      return;
    }
    try {
      activeClient.off('propertyChange');
      activeClient.off('connectionStateChange');
      activeClient.disconnect().then(() => {
        hilog.info(this.domainId, this.logTag, 'disconnect success');
      }).catch((err: BusinessError) => {
        hilog.warn(this.domainId, this.logTag, `disconnect ignored: ${err.code}, ${err.message}`);
      });
      activeClient.close();
    } catch (err) {
      hilog.warn(this.domainId, this.logTag, `close client ignored: ${(err as BusinessError).code}`);
    }
    client = undefined;
  }

  stopScanQuiet() {
    try {
      scan.stopScan().then(() => {
        hilog.info(this.domainId, this.logTag, 'stop scan before connect success');
      }).catch((err: BusinessError) => {
        hilog.warn(this.domainId, this.logTag, `stop scan before connect ignored: ${err.code}, ${err.message}`);
      });
      scan.off('deviceFound');
    } catch (err) {
      hilog.warn(this.domainId, this.logTag, `stop scan before connect ignored: ${(err as BusinessError).code}`);
    }
  }

  makeResponseProperty(): ssap.Property {
    let source = robotResponseProperty;
    let property: ssap.Property = {
      serviceUuid: source === undefined ? robotServiceUuid : source.serviceUuid,
      propertyUuid: source === undefined ? robotResponseUuid : source.propertyUuid,
      value: new ArrayBuffer(1)
    };
    if (source !== undefined && source.descriptors !== undefined) {
      property.descriptors = source.descriptors;
    }
    if (source !== undefined && source.operation !== undefined) {
      property.operation = source.operation;
    } else {
      property.operation = ssap.Operation.READABLE | ssap.Operation.NOTIFY;
    }
    return property;
  }

  makeCommandProperty(value: ArrayBuffer): ssap.Property {
    let source = robotCommandProperty;
    let property: ssap.Property = {
      serviceUuid: source === undefined ? robotServiceUuid : source.serviceUuid,
      propertyUuid: source === undefined ? robotCommandUuid : source.propertyUuid,
      value: value
    };
    if (source !== undefined && source.descriptors !== undefined) {
      property.descriptors = source.descriptors;
    }
    if (source !== undefined && source.operation !== undefined) {
      property.operation = source.operation;
    } else {
      property.operation = ssap.Operation.WRITE_WITH_RESPONSE | ssap.Operation.WRITE_NO_RESPONSE;
    }
    return property;
  }

  chooseWriteType(property: ssap.Property): ssap.PropertyWriteType {
    if (property.operation !== undefined &&
      (property.operation & ssap.Operation.WRITE_NO_RESPONSE) === ssap.Operation.WRITE_NO_RESPONSE) {
      hilog.info(this.domainId, this.logTag, `write no response operation=${property.operation}`);
      return ssap.PropertyWriteType.WRITE_NO_RESPONSE;
    }
    if (property.operation !== undefined &&
      (property.operation & ssap.Operation.WRITE_WITH_RESPONSE) === ssap.Operation.WRITE_WITH_RESPONSE) {
      hilog.info(this.domainId, this.logTag, `write with response operation=${property.operation}`);
      return ssap.PropertyWriteType.WRITE;
    }
    hilog.info(this.domainId, this.logTag, `write fallback with response operation=${property.operation}`);
    return ssap.PropertyWriteType.WRITE;
  }

  resetRobotServiceState() {
    this.clearAutoInitTimer();
    robotServiceUuid = ROBOT_SERVICE_UUID_FALLBACK;
    robotCommandUuid = ROBOT_COMMAND_UUID_FALLBACK;
    robotResponseUuid = ROBOT_RESPONSE_UUID_FALLBACK;
    robotCommandProperty = undefined;
    robotResponseProperty = undefined;
    this.serviceReady = false;
    this.commandBusy = false;
    this.serviceDiscoveryStarted = false;
    this.motorAutoInitIssued = false;
    this.motorStateText = '未知';
    this.motionStateText = '未运动';
    this.lastCommandText = '暂无';
    this.lastCommandTime = '--:--:--';
    this.lastResponseTime = '--:--:--';
    this.lastSequenceText = '暂无';
    this.lastResponse = '';
    this.responseSummary = '暂无响应';
    this.serviceInfo = '尚未发现小车服务';
    this.inspectionStatusText = '等待启动巡检';
    this.inspectionDistanceText = '--';
    this.inspectionTemperatureText = '--';
    this.inspectionHumidityText = '--';
    this.inspectionAlarmText = '未开始';
    this.inspectionAlarmFlags = 0;
    this.inspectionDetailText = '自动方形巡检：P 一键启动，G 查轨迹，M 查传感器';
    this.inspectionLastUpdateText = '--:--:--';
    this.phoneEnvAlarmActive = false;
    this.phoneEnvAlarmText = '手机端环境报警未触发';
    this.lastPhoneEnvAlarmMask = 0;
    this.lastPhoneEnvAlarmToastMs = 0;
  }

  isRobotUuid(uuid: string, shortUuid: string): boolean {
    let normalized = uuid.toLowerCase().replace(/-/g, '');
    let shortLower = shortUuid.toLowerCase();
    let little = shortLower.substring(2, 4) + shortLower.substring(0, 2);
    return normalized.endsWith(shortLower) || normalized.endsWith(little);
  }

  isMotionCommand(command: string): boolean {
    return command === 'F' || command === 'B' || command === 'L' || command === 'R';
  }

  motionName(command: string): string {
    switch (command) {
      case 'F':
        return '前进';
      case 'B':
        return '后退';
      case 'L':
        return '左转';
      case 'R':
        return '右转';
      default:
        return '行驶';
    }
  }

  commandName(command: string): string {
    switch (command) {
      case 'T':
        return '读取状态';
      case 'I':
        return '初始化电机';
      case 'D':
        return '测距';
      case 'A':
        return '启动避障';
      case 'P':
        return '自动方形巡检';
      case 'G':
        return '轨迹状态';
      case 'E':
        return '读取温湿度';
      case 'M':
        return '巡检监测';
      case 'O':
        return '初始化 OLED';
      case 'S':
        return '停止';
      default:
        return command;
    }
  }

  commandCodeName(code: number): string {
    switch (code) {
      case 1:
        return '前进';
      case 2:
        return '后退';
      case 3:
        return '左转';
      case 4:
        return '右转';
      case 5:
        return '停止';
      default:
        return `未知命令 ${code}`;
    }
  }

  statusName(status: number): string {
    switch (status) {
      case 0:
        return '正常';
      case 1:
        return '前方障碍，已保护停车';
      case 2:
        return '电量低';
      case 3:
        return '校验错误';
      case 4:
        return '命令无效';
      case 5:
        return '电机异常';
      default:
        return `未知状态 ${status}`;
    }
  }

  updateInspectionEnv(ok: boolean, temperature: number, humidity: number, notifyPhone: boolean = true) {
    if (ok) {
      this.inspectionTemperatureText = `${this.formatDeciValue(temperature)}°C`;
      this.inspectionHumidityText = `${this.formatDeciValue(humidity)}%`;
      if (notifyPhone) {
        this.handlePhoneEnvironmentAlarm(this.environmentFlagsFromValues(temperature, humidity), temperature, humidity);
      }
    } else {
      this.inspectionTemperatureText = '--';
      this.inspectionHumidityText = '--';
    }
    this.inspectionLastUpdateText = this.nowText();
  }

  updateInspectionObstacle(
    enabled: boolean,
    valid: boolean,
    blocked: boolean,
    distance: number,
    threshold: number,
    reason: number
  ) {
    if (!enabled) {
      this.inspectionDistanceText = '未启用';
      this.inspectionDetailText = '距离传感器未启用';
    } else if (!valid) {
      this.inspectionDistanceText = '无效';
      this.inspectionDetailText = `距离无效：${this.obstacleReasonName(reason)}`;
    } else {
      this.inspectionDistanceText = `${distance}mm`;
      this.inspectionDetailText = `距离阈值 ${threshold}mm，${blocked ? '前方障碍' : '前方安全'}`;
    }
    this.inspectionLastUpdateText = this.nowText();
  }

  updateInspectionMonitor(parts: Array<string>) {
    if (parts.length < 17) {
      this.inspectionStatusText = '巡检快照字段不完整';
      return;
    }
    let uptimeMs = Number(parts[1]);
    let motorReady = Number(parts[2]) === 1;
    let moving = Number(parts[3]) === 1;
    let envOk = Number(parts[4]) === 1;
    let temperature = Number(parts[5]);
    let humidity = Number(parts[6]);
    let obstacleEnabled = Number(parts[7]) === 1;
    let obstacleValid = Number(parts[8]) === 1;
    let obstacleBlocked = Number(parts[9]) === 1;
    let distance = Number(parts[10]);
    let threshold = Number(parts[11]);
    let reason = Number(parts[12]);
    let alarmFlags = Number(parts[13]);
    let sampleCount = Number(parts[14]);
    let envAge = Number(parts[15]);
    let obstacleAge = Number(parts[16]);

    this.motorStateText = motorReady ? '已就绪' : '未就绪';
    this.motionStateText = moving ? '运动中' : '已停止';
    this.updateInspectionEnv(envOk, temperature, humidity, false);
    this.updateInspectionObstacle(obstacleEnabled, obstacleValid, obstacleBlocked, distance, threshold, reason);
    this.inspectionAlarmFlags = alarmFlags;
    this.inspectionAlarmText = this.inspectionAlarmSummary(alarmFlags);
    this.handlePhoneEnvironmentAlarm(alarmFlags, temperature, humidity);
    this.inspectionStatusText = this.environmentAlarmMask(alarmFlags) !== 0 ? '温湿度异常报警' :
      (this.inspectionEnabled ? '自动方形巡检中' : '巡检快照已更新');
    this.inspectionDetailText =
      `运行 ${Math.floor(uptimeMs / 1000)}s，采样 ${sampleCount}，环境龄 ${this.ageText(envAge)}，距离龄 ${this.ageText(obstacleAge)}`;
  }

  obstacleReasonName(reason: number): string {
    switch (reason) {
      case 0:
        return '正常';
      case 1:
        return '未就绪';
      case 2:
        return 'ECHO 空闲高电平';
      case 3:
        return '没有 ECHO 上升沿';
      case 4:
        return '没有 ECHO 下降沿';
      case 5:
        return '脉冲无效';
      case 6:
        return '超出量程/空旷';
      default:
        return '未知诊断';
    }
  }

  inspectionAlarmSummary(flags: number): string {
    if (flags === 0) {
      return '正常';
    }
    let alarms: Array<string> = [];
    if ((flags & ROBOT_MONITOR_ALARM_ENV_INVALID) !== 0) {
      alarms.push('环境无效');
    }
    if ((flags & ROBOT_MONITOR_ALARM_OBSTACLE_INVALID) !== 0) {
      alarms.push('距离无效');
    }
    if ((flags & ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED) !== 0) {
      alarms.push('前方障碍');
    }
    if ((flags & ROBOT_MONITOR_ALARM_TEMP_HIGH) !== 0) {
      alarms.push('高温');
    }
    if ((flags & ROBOT_MONITOR_ALARM_HUMIDITY_HIGH) !== 0) {
      alarms.push('高湿');
    }
    return alarms.join(' / ');
  }

  inspectionAlarmColor(): string {
    if (this.inspectionAlarmFlags === 0) {
      return '#ff166534';
    }
    if ((this.inspectionAlarmFlags & (ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED |
      ROBOT_MONITOR_ALARM_TEMP_HIGH | ROBOT_MONITOR_ALARM_HUMIDITY_HIGH)) !== 0) {
      return '#ff991b1b';
    }
    return '#ffb45309';
  }

  inspectionAlarmBgColor(): string {
    if (this.inspectionAlarmFlags === 0) {
      return '#ffdcfce7';
    }
    if ((this.inspectionAlarmFlags & (ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED |
      ROBOT_MONITOR_ALARM_TEMP_HIGH | ROBOT_MONITOR_ALARM_HUMIDITY_HIGH)) !== 0) {
      return '#fffee2e2';
    }
    return '#fffff7ed';
  }

  environmentAlarmMask(flags: number): number {
    return flags & (ROBOT_MONITOR_ALARM_TEMP_HIGH | ROBOT_MONITOR_ALARM_HUMIDITY_HIGH);
  }

  environmentFlagsFromValues(temperature: number, humidity: number): number {
    let flags = 0;
    if (temperature >= ROBOT_MONITOR_TEMP_HIGH_DECI_C) {
      flags |= ROBOT_MONITOR_ALARM_TEMP_HIGH;
    }
    if (humidity >= ROBOT_MONITOR_HUMIDITY_HIGH_DECI_PERCENT) {
      flags |= ROBOT_MONITOR_ALARM_HUMIDITY_HIGH;
    }
    return flags;
  }

  phoneEnvironmentAlarmText(mask: number, temperature: number | undefined = undefined,
    humidity: number | undefined = undefined): string {
    let parts: Array<string> = [];
    if ((mask & ROBOT_MONITOR_ALARM_TEMP_HIGH) !== 0) {
      let tempText = temperature !== undefined && temperature === temperature ?
        `${this.formatDeciValue(temperature)}°C` : `超过 ${this.formatDeciValue(ROBOT_MONITOR_TEMP_HIGH_DECI_C)}°C`;
      parts.push(`温度异常 ${tempText}`);
    }
    if ((mask & ROBOT_MONITOR_ALARM_HUMIDITY_HIGH) !== 0) {
      let humidityText = humidity !== undefined && humidity === humidity ?
        `${this.formatDeciValue(humidity)}%` : `超过 ${this.formatDeciValue(ROBOT_MONITOR_HUMIDITY_HIGH_DECI_PERCENT)}%`;
      parts.push(`湿度异常 ${humidityText}`);
    }
    return parts.length === 0 ? '手机端环境正常' : parts.join('，');
  }

  showInspectionToast(message: string, duration: number = 3000) {
    try {
      this.promptAction.showToast({
        message: message,
        duration: duration
      });
    } catch (toastErr) {
      hilog.warn(this.domainId, this.logTag, `inspection toast ignored: ${(toastErr as BusinessError).code}`);
    }
  }

  handlePhoneEnvironmentAlarm(flags: number, temperature: number | undefined = undefined,
    humidity: number | undefined = undefined) {
    let mask = this.environmentAlarmMask(flags);
    let nowMs = new Date().getTime();
    if (mask !== 0) {
      this.phoneEnvAlarmActive = true;
      this.phoneEnvAlarmText = this.phoneEnvironmentAlarmText(mask, temperature, humidity);
      let isNewAlarm = mask !== this.lastPhoneEnvAlarmMask;
      let shouldRepeat = nowMs - this.lastPhoneEnvAlarmToastMs >= PHONE_ENV_ALARM_REPEAT_MS;
      if (isNewAlarm || shouldRepeat) {
        this.showInspectionToast(`环境异常报警：${this.phoneEnvAlarmText}`, 3000);
        this.lastPhoneEnvAlarmToastMs = nowMs;
      }
      this.lastPhoneEnvAlarmMask = mask;
      return;
    }
    if (this.lastPhoneEnvAlarmMask !== 0 || this.phoneEnvAlarmActive) {
      this.showInspectionToast('环境报警已解除', 1500);
    }
    this.phoneEnvAlarmActive = false;
    this.phoneEnvAlarmText = '手机端环境正常';
    this.lastPhoneEnvAlarmMask = 0;
  }

  ageText(value: number): string {
    if (value === 4294967295) {
      return '未知';
    }
    return `${value}ms`;
  }

  avoidPhaseName(phase: number): string {
    switch (phase) {
      case 0:
        return '空闲';
      case 1:
        return '前进';
      case 2:
        return '后退';
      case 3:
        return '右转避让';
      case 4:
        return '传感器异常';
      default:
        return `阶段 ${phase}`;
    }
  }

  patrolPhaseName(phase: number): string {
    switch (phase) {
      case 0:
        return '空闲';
      case 1:
        return '按路线直行';
      case 2:
        return '路线右转';
      case 3:
        return '遇障后退';
      case 4:
        return '右转离线';
      case 5:
        return '侧向绕开';
      case 6:
        return '左转平行';
      case 7:
        return '平行前进越障';
      case 8:
        return '左转回线';
      case 9:
        return '回到轨迹';
      case 10:
        return '恢复方向';
      case 11:
        return '环境告警驻留';
      case 12:
        return '传感器异常';
      default:
        return `阶段 ${phase}`;
    }
  }

  updateRobotResponse(text: string) {
    this.lastResponse = text;
    this.lastResponseTime = this.nowText();
    this.updateTelemetryFromResponse(text);
    this.responseSummary = this.describeRobotResponse(text);
  }

  updateTelemetryFromResponse(text: string) {
    let clean = text.trim();
    if (!clean.startsWith('+ROBOT:')) {
      return;
    }
    let body = clean.substring(7);
    let parts = body.split(',');
    let kind = parts[0];

    switch (kind) {
      case 'ACK': {
        let seq = Number(parts[1]);
        let command = Number(parts[2]);
        let status = Number(parts[3]);
        let moving = Number(parts[4]) === 1;
        if (seq === seq) {
          this.lastSequenceText = `${seq}`;
        }
        this.motionStateText = moving ? `${this.commandCodeName(command)}中` : '已停止';
        if (status === 0) {
          this.motorStateText = '已就绪';
        } else if (status === 5) {
          this.motorStateText = '未初始化/异常';
        }
        break;
      }
      case 'STATE': {
        let motorReady = Number(parts[2]) === 1;
        let moving = Number(parts[3]) === 1;
        let lastCommand = Number(parts[4]);
        let lastSeq = Number(parts[5]);
        this.motorStateText = motorReady ? '已就绪' : '未就绪';
        this.motionStateText = moving ? `${this.commandCodeName(lastCommand)}中` : '已停止';
        if (lastSeq === lastSeq) {
          this.lastSequenceText = `${lastSeq}`;
        }
        break;
      }
      case 'MOTOR':
        this.motorStateText = Number(parts[1]) === 1 ? '已就绪' : '初始化失败';
        break;
      case 'ENV': {
        let ok = Number(parts[1]) === 1;
        let temperature = Number(parts[2]);
        let humidity = Number(parts[3]);
        this.updateInspectionEnv(ok, temperature, humidity);
        break;
      }
      case 'OBS': {
        let enabled = Number(parts[1]) === 1;
        let valid = Number(parts[2]) === 1;
        let blocked = Number(parts[3]) === 1;
        let distance = Number(parts[4]);
        let threshold = Number(parts[5]);
        let reason = parts.length > 6 ? Number(parts[6]) : -1;
        this.updateInspectionObstacle(enabled, valid, blocked, distance, threshold, reason);
        break;
      }
      case 'MON':
        this.updateInspectionMonitor(parts);
        break;
      case 'PATROL': {
        let active = Number(parts[1]) === 1;
        let phase = Number(parts[2]);
        let status = Number(parts[3]);
        let leg = Number(parts[4]);
        let loop = Number(parts[5]);
        let alarmFlags = Number(parts[6]);
        let enabled = Number(parts[7]) === 1;
        let valid = Number(parts[8]) === 1;
        let blocked = Number(parts[9]) === 1;
        let distance = Number(parts[10]);
        let threshold = Number(parts[11]);
        let reason = Number(parts[12]);
        this.inspectionEnabled = active;
        this.motionStateText = active ? this.patrolPhaseName(phase) : '已停止';
        this.motorStateText = status === 5 ? '电机异常' : '已就绪';
        this.inspectionAlarmFlags = alarmFlags;
        this.inspectionAlarmText = this.inspectionAlarmSummary(alarmFlags);
        this.handlePhoneEnvironmentAlarm(alarmFlags);
        this.updateInspectionObstacle(enabled, valid, blocked, distance, threshold, reason);
        this.inspectionStatusText = this.environmentAlarmMask(alarmFlags) !== 0 ? '温湿度异常报警' :
          (active ? `自动方形巡检中：第 ${leg + 1}/4 边` : `自动方形巡检未启动：${this.statusName(status)}`);
        this.inspectionDetailText = `阶段：${this.patrolPhaseName(phase)}，已完成 ${loop} 圈，${this.inspectionAlarmSummary(alarmFlags)}`;
        break;
      }
      case 'ERR':
        this.motionStateText = '异常';
        break;
      default:
        break;
    }
  }

  describeRobotResponse(text: string): string {
    let clean = text.trim();
    if (clean.length === 0) {
      return '收到空响应';
    }
    if (!clean.startsWith('+ROBOT:')) {
      return clean;
    }

    let body = clean.substring(7);
    let parts = body.split(',');
    let kind = parts[0];

    switch (kind) {
      case 'SLE':
        return `星闪服务：${parts.length > 1 ? parts[1] : '状态更新'}`;
      case 'ACK': {
        let seq = Number(parts[1]);
        let command = Number(parts[2]);
        let status = Number(parts[3]);
        let moving = Number(parts[4]) === 1;
        return `序号 ${seq}，${this.commandCodeName(command)}，结果：${this.statusName(status)}，${moving ? '运动中' : '已停止'}`;
      }
      case 'STATE': {
        let uptimeMs = Number(parts[1]);
        let motorReady = Number(parts[2]) === 1;
        let moving = Number(parts[3]) === 1;
        let lastCommand = Number(parts[4]);
        let lastSeq = Number(parts[5]);
        let ageMs = Number(parts[6]);
        return `运行 ${Math.floor(uptimeMs / 1000)} 秒，电机${motorReady ? '已就绪' : '未就绪'}，${moving ? '正在运动' : '已停止'}，上次命令 ${this.commandCodeName(lastCommand)}，序号 ${lastSeq}，距今 ${ageMs}ms`;
      }
      case 'MOTOR':
        return Number(parts[1]) === 1 ? '电机初始化成功' : '电机初始化失败';
      case 'OLED':
        return Number(parts[1]) === 1 ? 'OLED 初始化成功' : 'OLED 初始化失败';
      case 'ENV': {
        let ok = Number(parts[1]) === 1;
        let temperature = Number(parts[2]);
        let humidity = Number(parts[3]);
        return ok ?
          `环境读取成功：温度 ${this.formatDeciValue(temperature)}°C，湿度 ${this.formatDeciValue(humidity)}%` :
          '环境读取失败';
      }
      case 'OBS': {
        let enabled = Number(parts[1]) === 1;
        let valid = Number(parts[2]) === 1;
        let blocked = Number(parts[3]) === 1;
        let distance = Number(parts[4]);
        let threshold = Number(parts[5]);
        let reason = parts.length > 6 ? Number(parts[6]) : -1;
        if (!enabled) {
          return '测距模块未启用';
        }
        if (!valid) {
          return `测距结果无效：${this.obstacleReasonName(reason)}`;
        }
        return `测距 ${distance}mm，阈值 ${threshold}mm，${blocked ? '前方有障碍' : '前方安全'}`;
      }
      case 'MON': {
        if (parts.length < 17) {
          return `巡检快照字段不完整：${clean}`;
        }
        let envOk = Number(parts[4]) === 1;
        let temperature = Number(parts[5]);
        let humidity = Number(parts[6]);
        let obsValid = Number(parts[8]) === 1;
        let blocked = Number(parts[9]) === 1;
        let distance = Number(parts[10]);
        let threshold = Number(parts[11]);
        let reason = Number(parts[12]);
        let alarmFlags = Number(parts[13]);
        let tempText = envOk ? `${this.formatDeciValue(temperature)}°C/${this.formatDeciValue(humidity)}%` : '环境无效';
        let distText = obsValid ? `${distance}mm/${threshold}mm` : `距离无效(${this.obstacleReasonName(reason)})`;
        return `巡检快照：${tempText}，${distText}，${blocked ? '前方障碍' : '无障碍'}，${this.inspectionAlarmSummary(alarmFlags)}`;
      }
      case 'AVOID': {
        let active = Number(parts[1]) === 1;
        let phase = Number(parts[2]);
        let status = Number(parts[3]);
        let valid = Number(parts[5]) === 1;
        let blocked = Number(parts[6]) === 1;
        let distance = Number(parts[7]);
        let threshold = Number(parts[8]);
        return `自动避障${active ? '已启动' : '未启动'}，阶段：${this.avoidPhaseName(phase)}，结果：${this.statusName(status)}，测距${valid ? ` ${distance}mm/${threshold}mm` : '无效'}，${blocked ? '检测到障碍' : '未检测到障碍'}`;
      }
      case 'PATROL': {
        let active = Number(parts[1]) === 1;
        let phase = Number(parts[2]);
        let status = Number(parts[3]);
        let leg = Number(parts[4]);
        let loop = Number(parts[5]);
        let alarmFlags = Number(parts[6]);
        let valid = Number(parts[8]) === 1;
        let blocked = Number(parts[9]) === 1;
        let distance = Number(parts[10]);
        let threshold = Number(parts[11]);
        return `自动方形巡检${active ? '已启动' : '未启动'}，第 ${leg + 1}/4 边，${this.patrolPhaseName(phase)}，${this.statusName(status)}，${valid ? `距离 ${distance}/${threshold}mm` : '距离无效'}，${blocked ? '遇到障碍' : '前方安全'}，${this.inspectionAlarmSummary(alarmFlags)}，已完成 ${loop} 圈`;
      }
      case 'ERR':
        return `小车返回错误：${body}`;
      default:
        return `收到小车响应：${clean}`;
    }
  }

  formatDeciValue(value: number): string {
    let sign = value < 0 ? '-' : '';
    let absValue = Math.abs(value);
    return `${sign}${Math.floor(absValue / 10)}.${absValue % 10}`;
  }

  normalizeCameraAddress(value: string): CameraAddress {
    let next = value.trim();
    if (next.length === 0) {
      return { baseUrl: '', path: DEFAULT_CAMERA_PATH };
    }
    if (!next.startsWith('http://') && !next.startsWith('https://')) {
      next = `http://${next}`;
    }

    let schemeEnd = next.indexOf('://');
    let hostStart = schemeEnd >= 0 ? schemeEnd + 3 : 0;
    let pathStart = next.indexOf('/', hostStart);
    let baseUrl = pathStart >= 0 ? next.substring(0, pathStart) : next;
    let path = pathStart >= 0 ? next.substring(pathStart) : DEFAULT_CAMERA_PATH;
    if (path.length === 0) {
      path = DEFAULT_CAMERA_PATH;
    }
    if (path.indexOf('?') >= 0) {
      path = path.substring(0, path.indexOf('?'));
    }
    if (path.indexOf('#') >= 0) {
      path = path.substring(0, path.indexOf('#'));
    }
    if (path.length === 0) {
      path = DEFAULT_CAMERA_PATH;
    }
    return { baseUrl: baseUrl, path: path };
  }

  applyCameraBaseUrl() {
    let next = this.normalizeCameraAddress(this.cameraBaseDraft);
    if (next.baseUrl.length === 0) {
      this.cameraStatus = '树莓派地址不能为空';
      return;
    }
    this.cameraBaseUrl = next.baseUrl;
    this.cameraBaseDraft = `${next.baseUrl}${next.path}`;
    this.cameraCheckDone = false;
    this.cameraOnline = false;
    this.cameraFrameReady = false;
    this.cameraHealthText = '检测中';
    this.loadCameraPath(next.path);
  }

  normalizeRouteBaseUrl(value: string): string {
    let next = value.trim();
    if (next.length === 0) {
      return '';
    }
    if (!next.startsWith('http://') && !next.startsWith('https://')) {
      next = `http://${next}`;
    }

    let schemeEnd = next.indexOf('://');
    let hostStart = schemeEnd >= 0 ? schemeEnd + 3 : 0;
    let pathStart = next.indexOf('/', hostStart);
    let baseUrl = pathStart >= 0 ? next.substring(0, pathStart) : next;
    if (baseUrl.length >= 5 && baseUrl.substring(baseUrl.length - 5) === ':8080') {
      baseUrl = `${baseUrl.substring(0, baseUrl.length - 5)}:8090`;
    } else {
      let colonIndex = baseUrl.lastIndexOf(':');
      if (colonIndex < hostStart) {
        baseUrl = `${baseUrl}:8090`;
      }
    }
    return baseUrl;
  }

  applyRouteBaseUrl() {
    let next = this.normalizeRouteBaseUrl(this.routeBaseDraft);
    if (next.length === 0) {
      this.routeStatusText = '路线服务地址不能为空';
      this.routeOnline = false;
      return;
    }
    this.routeBaseUrl = next;
    this.routeBaseDraft = next;
    this.routeStatusText = '路线服务地址已更新，正在读取当前路线';
    this.fetchCurrentRoute(false);
  }

  isRouteActionSupported(action: string): boolean {
    let normalized = action.toLowerCase();
    return normalized === 'move' || normalized === 'drive' || normalized === 'forward' ||
      normalized === 'turn' || normalized === 'rotate' ||
      normalized === 'wait' || normalized === 'inspect' ||
      normalized === 'stop' || normalized === 'pause';
  }

  validateRouteDraft(route: RouteImportPayload): string {
    if (route.steps === undefined || route.steps.length === 0) {
      return '路线 steps 不能为空';
    }
    for (let index = 0; index < route.steps.length; index++) {
      let step = route.steps[index];
      let action = step.action === undefined ? (step.type === undefined ? 'move' : step.type) : step.action;
      if (!this.isRouteActionSupported(action)) {
        return `第 ${index + 1} 步 action 不支持：${action}`;
      }
      let duration = step.duration_s === undefined ? step.duration : step.duration_s;
      if (duration === undefined || duration <= 0) {
        return `第 ${index + 1} 步 duration_s 必须大于 0`;
      }
      if (action.toLowerCase() === 'turn' || action.toLowerCase() === 'rotate') {
        let direction = step.direction === undefined ? 'right' : step.direction.toLowerCase();
        if (direction !== 'right' && direction !== 'cw' && direction !== 'clockwise' &&
          direction !== 'left' && direction !== 'ccw' && direction !== 'counterclockwise') {
          return `第 ${index + 1} 步转向不支持：${direction}`;
        }
      }
    }
    return '';
  }

  importRouteToPi() {
    let route: RouteImportPayload;
    try {
      route = JSON.parse(this.routeJsonDraft) as RouteImportPayload;
    } catch (parseErr) {
      this.routeStatusText = `路线 JSON 解析失败：${(parseErr as Error).message}`;
      this.routeOnline = false;
      return;
    }

    let validation = this.validateRouteDraft(route);
    if (validation.length > 0) {
      this.routeStatusText = validation;
      this.routeOnline = false;
      return;
    }

    this.routeImportBusy = true;
    this.postRouteRequest('/route/import', this.routeJsonDraft, '路线导入', (payload: RouteServicePayload) => {
      this.updateRouteStatusFromPayload(payload);
      this.routeStatusText = '路线已导入树莓派';
      this.showInspectionToast('路线导入成功', 1600);
      this.fetchCurrentRoute(false);
    }, () => {
      this.routeImportBusy = false;
    });
  }

  fetchCurrentRoute(showToast: boolean) {
    if (this.routeControlBusy) {
      return;
    }
    this.routeControlBusy = true;
    let request = http.createHttp();
    request.request(`${this.routeBaseUrl}/route/current`, {
      method: http.RequestMethod.GET,
      connectTimeout: 3000,
      readTimeout: 3000,
      expectDataType: http.HttpDataType.STRING
    }, (err: BusinessError, data: http.HttpResponse) => {
      this.routeControlBusy = false;
      this.routeLastCheckedText = this.nowText();
      try {
        request.destroy();
      } catch (destroyErr) {
        hilog.warn(this.domainId, this.logTag, `route current request destroy ignored: ${(destroyErr as BusinessError).code}`);
      }

      if (err !== undefined && err !== null) {
        this.handleRouteFailure(`路线读取失败：${err.code}, ${err.message}`);
        return;
      }
      if (data.responseCode < 200 || data.responseCode >= 300) {
        this.handleRouteFailure(`路线服务返回 HTTP ${data.responseCode}`);
        return;
      }

      let resultText = typeof data.result === 'string' ? data.result : JSON.stringify(data.result);
      try {
        let payload = JSON.parse(resultText) as RouteServicePayload;
        this.updateRouteStatusFromPayload(payload);
        if (showToast) {
          this.showInspectionToast('路线状态已刷新', 1200);
        }
      } catch (parseErr) {
        this.handleRouteFailure(`路线状态解析失败：${(parseErr as Error).message}`);
      }
    });
  }

  startImportedRoute() {
    if (this.routeControlBusy) {
      return;
    }
    this.routeControlBusy = true;
    this.postRouteRequest('/route/start', '{"loops":1,"speed_scale":1.0}', '启动路线', (payload: RouteServicePayload) => {
      this.updateRouteStatusFromPayload(payload);
      this.routeStatusText = '路线巡航已启动';
      this.showInspectionToast('路线巡航已启动', 1600);
    }, () => {
      this.routeControlBusy = false;
    });
  }

  stopImportedRoute() {
    if (this.routeControlBusy) {
      return;
    }
    this.routeControlBusy = true;
    this.postRouteRequest('/route/stop', '{}', '停止路线', (payload: RouteServicePayload) => {
      this.updateRouteStatusFromPayload(payload);
      this.routeStatusText = '路线巡航已停止';
      this.showInspectionToast('路线巡航已停止', 1400);
    }, () => {
      this.routeControlBusy = false;
    });
  }

  postRouteRequest(path: string, body: string, action: string,
    onSuccess: (payload: RouteServicePayload) => void, onFinish: () => void) {
    let request = http.createHttp();
    request.request(`${this.routeBaseUrl}${path}`, {
      method: http.RequestMethod.POST,
      header: {
        'Content-Type': 'application/json'
      },
      extraData: body,
      connectTimeout: 3000,
      readTimeout: 3000,
      expectDataType: http.HttpDataType.STRING
    }, (err: BusinessError, data: http.HttpResponse) => {
      onFinish();
      this.routeLastCheckedText = this.nowText();
      try {
        request.destroy();
      } catch (destroyErr) {
        hilog.warn(this.domainId, this.logTag, `route post request destroy ignored: ${(destroyErr as BusinessError).code}`);
      }

      if (err !== undefined && err !== null) {
        this.handleRouteFailure(`${action}失败：${err.code}, ${err.message}`);
        return;
      }
      if (data.responseCode < 200 || data.responseCode >= 300) {
        let resultText = typeof data.result === 'string' ? data.result : JSON.stringify(data.result);
        this.handleRouteFailure(`${action}失败：HTTP ${data.responseCode} ${resultText}`);
        return;
      }

      let resultText = typeof data.result === 'string' ? data.result : JSON.stringify(data.result);
      try {
        let payload = JSON.parse(resultText) as RouteServicePayload;
        onSuccess(payload);
      } catch (parseErr) {
        this.handleRouteFailure(`${action}响应解析失败：${(parseErr as Error).message}`);
      }
    });
  }

  updateRouteStatusFromPayload(payload: RouteServicePayload) {
    this.routeOnline = true;
    if (payload.error !== undefined && payload.error.length > 0) {
      this.routeStatusText = `路线服务错误：${payload.error}`;
      this.routeDetailText = '请检查树莓派 MapBrain 服务状态';
      return;
    }
    if (payload.map_brain === undefined) {
      this.routeStatusText = '路线服务在线';
      this.routeDetailText = '当前响应没有 map_brain 字段';
      return;
    }

    let brain = payload.map_brain;
    let name = this.textOrDefault(brain.name, '未命名路线');
    let stepCount = brain.step_count === undefined ? 0 : brain.step_count;
    let stepIndex = brain.step_index === undefined ? 0 : brain.step_index;
    let loop = brain.loop === undefined ? 0 : brain.loop;
    let maxLoops = brain.max_loops === undefined ? 1 : brain.max_loops;
    let remaining = this.formatOptionalNumber(brain.step_remaining_s, 1);
    let currentName = '--';
    if (brain.current_step !== undefined) {
      currentName = this.textOrDefault(brain.current_step.name,
        this.textOrDefault(brain.current_step.id, this.textOrDefault(brain.current_step.action, '--')));
    }

    if (stepCount <= 0) {
      this.routeStatusText = '路线服务在线，但未加载有效路线';
    } else if (brain.active === true) {
      this.routeStatusText = `执行中：${name}`;
    } else if (brain.finished === true) {
      this.routeStatusText = `路线完成：${name}`;
    } else {
      this.routeStatusText = `路线已就绪：${name}`;
    }

    let loopText = maxLoops === 0 ? `${loop + 1}/∞` : `${loop + 1}/${maxLoops}`;
    this.routeDetailText = `步骤 ${stepIndex + 1}/${stepCount}，当前 ${currentName}，剩余 ${remaining}s，循环 ${loopText}`;
    if (brain.last_error !== undefined && brain.last_error.length > 0) {
      this.routeDetailText = `${this.routeDetailText}，提示：${brain.last_error}`;
    }
  }

  handleRouteFailure(message: string) {
    this.routeOnline = false;
    this.routeStatusText = message;
    this.routeDetailText = `确认树莓派 ${this.routeBaseUrl} 的 nearlink-rover-arbiter 服务正在运行`;
    this.showInspectionToast(message, 2200);
  }

  loadCameraPath(path: string) {
    this.cameraUrl = `${this.cameraBaseUrl}${path}`;
    try {
      this.cameraController.loadUrl(this.cameraUrl);
      this.checkCameraStatus(true);
    } catch (err) {
      this.cameraStatus = `摄像头加载失败：${(err as Error).message}`;
    }
  }

  reloadCameraStream() {
    try {
      this.cameraController.loadUrl(this.cameraUrl);
    } catch (err) {
      this.cameraStatus = `摄像头刷新失败：${(err as Error).message}`;
    }
  }

  startCameraMonitor() {
    this.stopCameraMonitor();
    this.checkCameraStatus(true);
    this.cameraStatusTimer = setInterval(() => {
      this.checkCameraStatus(false);
    }, CAMERA_STATUS_POLL_MS);
  }

  stopCameraMonitor() {
    if (this.cameraStatusTimer !== -1) {
      clearInterval(this.cameraStatusTimer);
      this.cameraStatusTimer = -1;
    }
  }

  checkCameraStatus(force: boolean) {
    if (this.cameraProbeBusy && !force) {
      return;
    }
    this.cameraProbeBusy = true;
    let request = http.createHttp();
    request.request(`${this.cameraBaseUrl}/status`, {
      method: http.RequestMethod.GET,
      connectTimeout: 2000,
      readTimeout: 2000,
      expectDataType: http.HttpDataType.STRING
    }, (err: BusinessError, data: http.HttpResponse) => {
      this.cameraProbeBusy = false;
      this.cameraCheckDone = true;
      this.cameraLastCheckedText = this.nowText();
      try {
        request.destroy();
      } catch (destroyErr) {
        hilog.warn(this.domainId, this.logTag, `camera status request destroy ignored: ${(destroyErr as BusinessError).code}`);
      }

      if (err !== undefined && err !== null) {
        this.handleCameraStatusFailure(`状态检测失败：${err.code}, ${err.message}`);
        return;
      }
      if (data.responseCode < 200 || data.responseCode >= 300) {
        this.handleCameraStatusFailure(`状态接口返回 HTTP ${data.responseCode}`);
        return;
      }

      let resultText = typeof data.result === 'string' ? data.result : JSON.stringify(data.result);
      try {
        let payload = JSON.parse(resultText) as PiCameraStatusPayload;
        this.updateCameraStatus(payload);
      } catch (parseErr) {
        this.handleCameraStatusFailure(`状态解析失败：${(parseErr as Error).message}`);
      }
    });
  }

  updateCameraStatus(payload: PiCameraStatusPayload) {
    this.cameraFailureCount = 0;
    this.cameraOnline = true;
    this.cameraFrameReady = payload.frame_ready === true;
    let backend = this.textOrDefault(payload.camera_backend, 'unknown');
    let frameAge = this.formatOptionalNumber(payload.last_frame_age_sec, 2);
    let fps = this.formatOptionalNumber(payload.fps_actual, 1);
    let frames = payload.frames_total === undefined ? '--' : `${payload.frames_total}`;
    let bandwidth = this.formatOptionalNumber(payload.bandwidth_kbps, 0);
    let cpu = this.formatOptionalNumber(payload.cpu_percent, 0);
    let temp = this.formatOptionalNumber(payload.cpu_temp_c, 1);

    if (payload.camera_error !== undefined && payload.camera_error.length > 0) {
      this.cameraStatus = `摄像头错误：${payload.camera_error}`;
      this.cameraHealthText = '服务异常';
      this.cameraDetailText = `后端 ${backend}，FPS ${fps}，帧龄 ${frameAge}s`;
      return;
    }

    this.cameraHealthText = this.cameraFrameReady ? '画面在线' : '等待画面';
    this.cameraStatus = this.cameraFrameReady ? `摄像头在线：${backend}` : `服务在线，但还没有新画面：${backend}`;
    this.cameraDetailText = `FPS ${fps}，帧 ${frames}，帧龄 ${frameAge}s，带宽 ${bandwidth}kbps，CPU ${cpu}%，温度 ${temp}C`;
  }

  handleCameraStatusFailure(message: string) {
    this.cameraFailureCount += 1;
    this.cameraOnline = false;
    this.cameraFrameReady = false;
    this.cameraHealthText = '摄像头离线';
    this.cameraStatus = `${message}；确认树莓派 ${this.cameraBaseUrl} 正在运行`;
    this.cameraDetailText = '请检查手机和平板是否在同一热点，并确认 run.py 仍在运行';
    if (this.cameraFailureCount >= CAMERA_RELOAD_FAILURES) {
      this.cameraFailureCount = 0;
      this.reloadCameraStream();
    }
  }

  cameraStatusColor(): string {
    if (!this.cameraCheckDone) {
      return '#ff475569';
    }
    if (!this.cameraOnline) {
      return '#ff991b1b';
    }
    return this.cameraFrameReady ? '#ff166534' : '#ffb45309';
  }

  cameraStatusBgColor(): string {
    if (!this.cameraCheckDone) {
      return '#fff1f5f9';
    }
    if (!this.cameraOnline) {
      return '#fffee2e2';
    }
    return this.cameraFrameReady ? '#ffdcfce7' : '#fffff7ed';
  }

  routeStatusColor(): string {
    if (!this.routeOnline || this.routeStatusText.indexOf('错误') >= 0 ||
      this.routeStatusText.indexOf('失败') >= 0 || this.routeStatusText.indexOf('不能为空') >= 0) {
      return '#ff991b1b';
    }
    if (this.routeStatusText.indexOf('执行中') >= 0) {
      return '#ff0369a1';
    }
    return '#ff166534';
  }

  routeStatusBgColor(): string {
    if (!this.routeOnline || this.routeStatusText.indexOf('错误') >= 0 ||
      this.routeStatusText.indexOf('失败') >= 0 || this.routeStatusText.indexOf('不能为空') >= 0) {
      return '#fffee2e2';
    }
    if (this.routeStatusText.indexOf('执行中') >= 0) {
      return '#ffe0f2fe';
    }
    return '#ffdcfce7';
  }

  textOrDefault(value: string | undefined, fallback: string): string {
    if (value === undefined || value.length === 0) {
      return fallback;
    }
    return value;
  }

  formatOptionalNumber(value: number | undefined, fractionDigits: number): string {
    if (value === undefined) {
      return '--';
    }
    return value.toFixed(fractionDigits);
  }

  nowText(): string {
    let now = new Date();
    return `${this.pad2(now.getHours())}:${this.pad2(now.getMinutes())}:${this.pad2(now.getSeconds())}`;
  }

  pad2(value: number): string {
    return value < 10 ? `0${value}` : `${value}`;
  }

  bufferToText(buffer: ArrayBuffer): string {
    let bytes = new Uint8Array(buffer);
    let text = '';
    for (let index = 0; index < bytes.length; index++) {
      let value = bytes[index];
      if (value === 0) {
        continue;
      }
      text += String.fromCharCode(value);
    }
    return text;
  }

  handleError(action: string, err: BusinessError) {
    let message = `${action} 失败：${err.code}, ${err.message}`;
    this.robotStatus = message;
    this.updateRobotResponse(message);
    this.commandBusy = false;
    hilog.error(this.domainId, this.logTag, message);
    try {
      this.promptAction.showToast({
        message: action + ' 失败',
        duration: 2000
      });
    } catch (toastErr) {
      hilog.warn(this.domainId, this.logTag, `toast ignored: ${(toastErr as BusinessError).code}`);
    }
  }
}
