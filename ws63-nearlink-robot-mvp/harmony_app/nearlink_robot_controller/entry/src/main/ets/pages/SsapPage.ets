/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2025. All rights reserved.
 */
import { SsapClientPage, requestRobotImmediateStop } from './SsapClientPage';
import { SsapServerPage } from './SsapServerPage';
import { requestRobotSafetyStop } from './RobotSafetyBus';

@Component
export struct SsapPage {
  @State currentIndex: number = 0;
  logTag: string = 'SsapPage';
  domainId: number = 0x0000;

  fontColor: string = '#ff7fb3cd';
  selectedFontColor: string = '#ff1d54c1';

  build() {
    NavDestination() {
      Column() {
        this.TopTitle();
        Row(){
          this.PageTabs();
        }
        Row(){
          if (this.currentIndex === 0) {
            // 小车控制
            SsapClientPage();
          }
          if (this.currentIndex === 1) {
            // 服务端测试
            SsapServerPage();
          }
        }.padding({bottom: 50})
      }
    }.hideTitleBar(true)
  }

  @Builder
  TopTitle() {
    Row() {
      Text('星闪小车控制')
        .padding(15);
    }.height('6%')
    .width('100%')
    .backgroundColor($r('sys.color.comp_background_gray'));
  }

  @Builder
  PageTabs() {
    Column() {
      List() {
        ListItem() {
          this.TabBuilder(0, '小车控制');
        }

        ListItem() {
          this.TabBuilder(1, '服务端测试');
        };
      }.width('100%')
      .listDirection(Axis.Horizontal)
      .scrollBar(BarState.Off)
    }
    .justifyContent(FlexAlign.Center)
    .height('6%')
    .width('100%')
    .backgroundColor($r('sys.color.ohos_id_scroll_bar_background_color'));
  }

  @Builder
  TabBuilder(index: number, name: Resource | string) {
    Row() {
      Text(name)
        .fontColor(this.currentIndex === index ? this.selectedFontColor : this.fontColor)
        .fontSize(14);
    }
    .width('50%')
    .height('100%')
    .onClick(() => {
      if (this.currentIndex === 0 && index !== 0) {
        requestRobotImmediateStop('切换到服务端测试');
        requestRobotSafetyStop('切换到服务端测试');
      }
      this.currentIndex = index;
    })
    .justifyContent(FlexAlign.Center);
  }
}
