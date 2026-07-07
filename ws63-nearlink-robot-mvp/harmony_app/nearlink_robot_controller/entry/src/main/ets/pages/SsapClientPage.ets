/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2025. All rights reserved.
 * Robot adaptation: connect to the WS63 smart car SLE server.
 */
import { hilog } from '@kit.PerformanceAnalysisKit';
import { BusinessError } from '@kit.BasicServicesKit';
import { scan, ssap, constant } from '@kit.NearLinkKit';
import { PromptAction } from '@kit.ArkUI'
import { registerRobotSafetyStop, unregisterRobotSafetyStop } from './RobotSafetyBus';

const ROBOT_DEVICE_NAME: string = 'ws63_robot_mvp';
const ROBOT_SERVICE_SHORT_UUID: string = '7100';
const ROBOT_COMMAND_SHORT_UUID: string = '7101';
const ROBOT_RESPONSE_SHORT_UUID: string = '7102';
const ROBOT_SERVICE_DISCOVERY_RETRY_MAX: number = 6;
const ROBOT_SERVICE_DISCOVERY_RETRY_MS: number = 500;
const ROBOT_MOTION_REPEAT_MS: number = 300;

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

  logTag: string = 'RobotSleClient';
  domainId: number = 0x0000;
  promptAction: PromptAction = this.getUIContext().getPromptAction();
  motionRepeatTimer: number = -1;
  autoInitTimer: number = -1;
  safetyStopCallback: (reason: string) => void = (reason: string) => {
    this.safeStop(reason, false);
  };

  aboutToAppear(): void {
    registerRobotSafetyStop(this.safetyStopCallback);
  }

  aboutToDisappear(): void {
    unregisterRobotSafetyStop(this.safetyStopCallback);
    this.safeStop('离开小车控制页', true);
  }

  onPageHide(): void {
    this.safeStop('应用切到后台', false);
  }

  build() {
    Scroll() {
      Column() {
        this.Header();
        this.StatusPanel();
        this.ScanPanel();
        this.DevicePanel();
        this.CommandPanel();
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
            this.readResponseProperty();
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
        this.ActionButton('X', '避障测试 X');
        this.ActionButton('E', '温湿度 E');
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
      let scanFilter: scan.ScanFilters = {
        deviceName: ROBOT_DEVICE_NAME
      };
      let scanOptions: scan.ScanOptions = {
        scanMode: 2
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
        hilog.info(this.domainId, this.logTag, 'connect success');
        this.robotStatus = '连接请求已发出，等待链路状态确认...';
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
    this.lastResponse = `${this.lastResponse}\nservices:\n${summary}`.trim();
    return this.serviceReady;
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
        this.tryAutoInitMotor();
      }).catch((err: BusinessError) => {
        this.robotStatus = `通知开启失败，将改用读取响应 (${err.code})`;
        hilog.warn(this.domainId, this.logTag, `enable notify ignored: ${err.code}, ${err.message}`);
        this.motionStateText = '已停止';
        this.tryAutoInitMotor();
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
      let arrayBuffer = new ArrayBuffer(1);
      let value = new Uint8Array(arrayBuffer);
      value[0] = command.charCodeAt(0);
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
    hilog.info(this.domainId, this.logTag, `force write no response operation=${property.operation}`);
    return ssap.PropertyWriteType.WRITE_NO_RESPONSE;
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
      case 'X':
        return '避障测试';
      case 'E':
        return '读取温湿度';
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
        if (!enabled) {
          return '测距模块未启用';
        }
        if (!valid) {
          return '测距结果无效';
        }
        return `测距 ${distance}mm，阈值 ${threshold}mm，${blocked ? '前方有障碍' : '前方安全'}`;
      }
      case 'AVOID':
      case 'AVOIDTEST': {
        let active = Number(parts[1]) === 1;
        let phase = Number(parts[2]);
        let status = Number(parts[3]);
        let valid = Number(parts[5]) === 1;
        let blocked = Number(parts[6]) === 1;
        let distance = Number(parts[7]);
        let threshold = Number(parts[8]);
        let prefix = kind === 'AVOID' ? '自动避障' : '避障测试';
        return `${prefix}${active ? '已启动' : '未启动'}，阶段：${this.avoidPhaseName(phase)}，结果：${this.statusName(status)}，测距${valid ? ` ${distance}mm/${threshold}mm` : '无效'}，${blocked ? '检测到障碍' : '未检测到障碍'}`;
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
