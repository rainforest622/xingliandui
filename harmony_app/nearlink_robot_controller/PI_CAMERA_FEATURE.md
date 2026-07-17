# Raspberry Pi Camera Page

The app now uses the Raspberry Pi web camera service as the robot camera view.

- The main robot console embeds the camera view above the driving controls.
- Default service URL: `http://192.168.43.42:8080/`
- The tab provides quick buttons for:
  - `Dashboard`: `/`
  - `Stream`: `/stream.mjpg`
  - `Status`: `/status`

The previous local phone camera preview page was removed. The HAP no longer requests `ohos.permission.CAMERA`; it only needs NearLink access for robot control and `ohos.permission.INTERNET` for the Pi camera page.
