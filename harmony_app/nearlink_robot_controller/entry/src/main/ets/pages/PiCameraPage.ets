/*
 * Raspberry Pi camera dashboard for the robot.
 */
import { webview } from '@kit.ArkWeb';

const DEFAULT_PI_BASE_URL: string = 'http://10.17.120.33:8080';

@Component
export struct PiCameraPage {
  @State baseUrl: string = DEFAULT_PI_BASE_URL;
  @State currentUrl: string = `${DEFAULT_PI_BASE_URL}/stream.mjpg`;
  @State pageStatus: string = 'Pi camera stream';

  private controller: webview.WebviewController = new webview.WebviewController('PiCameraWeb');

  build() {
    Column() {
      this.Toolbar();

      Web({
        src: this.currentUrl,
        controller: this.controller,
      })
        .width('100%')
        .layoutWeight(1)
        .javaScriptAccess(true)
        .domStorageAccess(true)
        .onlineImageAccess(true)
        .imageAccess(true)
        .mixedMode(MixedMode.All)
        .zoomAccess(true)
        .onPageBegin(() => {
          this.pageStatus = `Loading ${this.currentUrl}`;
        })
        .onPageEnd(() => {
          this.pageStatus = `Loaded ${this.currentUrl}`;
        });
    }
    .width('100%')
    .height('100%')
    .backgroundColor('#101214');
  }

  @Builder
  Toolbar() {
    Column({ space: 8 }) {
      Row() {
        Text('Pi Camera')
          .fontSize(20)
          .fontWeight(FontWeight.Bold)
          .fontColor('#f4f7fb');
        Blank();
        Button('Refresh')
          .height(34)
          .fontSize(13)
          .onClick(() => {
            this.controller.refresh();
          });
      }
      .width('100%');

      Text(this.currentUrl)
        .fontSize(12)
        .fontColor('#a8b3c2')
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%');

      Row({ space: 8 }) {
        this.LinkButton('Dashboard', '/');
        this.LinkButton('Stream', '/stream.mjpg');
        this.LinkButton('Status', '/status');
      }
      .width('100%');

      Text(this.pageStatus)
        .fontSize(12)
        .fontColor('#7fd08a')
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .width('100%');
    }
    .padding({ left: 12, right: 12, top: 10, bottom: 10 })
    .width('100%')
    .backgroundColor('#171b20');
  }

  @Builder
  LinkButton(label: string, path: string) {
    Button(label)
      .height(34)
      .layoutWeight(1)
      .fontSize(13)
      .onClick(() => {
        this.loadPath(path);
      });
  }

  private loadPath(path: string) {
    this.currentUrl = `${this.baseUrl}${path}`;
    try {
      this.controller.loadUrl(this.currentUrl);
    } catch (err) {
      this.pageStatus = `Load failed: ${(err as Error).message}`;
    }
  }
}
