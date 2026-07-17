/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2025. All rights reserved.
 */
import { SsapClientPage, requestRobotImmediateStop } from './SsapClientPage';
import { SsapServerPage } from './SsapServerPage';
import { requestRobotSafetyStop } from './RobotSafetyBus';

@Component
export struct SsapPage {
  @State currentIndex: number = 0;

  private readonly fontColor: string = '#7f8da3';
  private readonly selectedFontColor: string = '#1d54c1';

  build() {
    NavDestination() {
      Column() {
        this.TopTitle();
        this.PageTabs();
        Stack() {
          if (this.currentIndex === 0) {
            SsapClientPage();
          } else {
            SsapServerPage();
          }
        }
        .width('100%')
        .layoutWeight(1);
      }
      .width('100%')
      .height('100%');
    }
    .hideTitleBar(true);
  }

  @Builder
  TopTitle() {
    Row() {
      Text('WS63 Robot Console')
        .fontSize(18)
        .fontWeight(FontWeight.Bold)
        .fontColor('#20242a')
        .padding({ left: 15, right: 15 });
    }
    .height(52)
    .width('100%')
    .backgroundColor($r('sys.color.comp_background_gray'));
  }

  @Builder
  PageTabs() {
    Row() {
      this.TabBuilder(0, '驾驶台');
      this.TabBuilder(1, '调试');
    }
    .height(48)
    .width('100%')
    .backgroundColor('#f4f6fa');
  }

  @Builder
  TabBuilder(index: number, name: string) {
    Row() {
      Text(name)
        .fontColor(this.currentIndex === index ? this.selectedFontColor : this.fontColor)
        .fontSize(14)
        .fontWeight(this.currentIndex === index ? FontWeight.Bold : FontWeight.Regular)
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis });
    }
    .layoutWeight(1)
    .height('100%')
    .justifyContent(FlexAlign.Center)
    .border({
      width: { bottom: this.currentIndex === index ? 2 : 0 },
      color: this.selectedFontColor,
    })
    .onClick(() => {
      if (this.currentIndex === 0 && index !== 0) {
        requestRobotImmediateStop('switch tab');
        requestRobotSafetyStop('switch tab');
      }
      this.currentIndex = index;
    });
  }
}
