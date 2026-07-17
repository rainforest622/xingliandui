# HarmonyOS Route Import Checkpoint - 2026-07-17

## Result

- DevEco automatic signing was refreshed after the old debug certificate expired on 2026-07-15 14:20:20 CST.
- New signing files were generated under the current Windows user's `.ohos\config\default_nearlink_robot_controller_*` directory.
- `harmony_app/nearlink_robot_controller` builds successfully with `assembleHap`.
- Signed HAP output: `harmony_app/nearlink_robot_controller/entry/build/default/outputs/default/entry-default-signed.hap`.
- The signed HAP was installed and launched successfully on HDC target `4CGBB24C13200213`.

## Verified Services

- Camera status: `http://192.168.43.42:8080/status` -> HTTP 200.
- Route arbiter status: `http://192.168.43.42:8090/status` -> HTTP 200.
- Current route: `http://192.168.43.42:8090/route/current` -> HTTP 200.

## Current Route Baseline

- MapBrain route name: `industrial-square-patrol-v1`.
- Step count: 12.
- Route state: loaded, not active.
- Current first step: `A_to_B`.
- Default motion parameters:
  - move speed: `0.24`
  - turn speed: `0.5`
  - right turn duration: `0.8s`

## Next Physical Test

Before starting the route from the phone/tablet app:

1. Put the robot in an open area.
2. Keep the SLE manual stop/emergency stop path available.
3. In the app, use the route panel to read the current route.
4. Start one loop only.
5. Observe whether the route starts and whether manual stop can still override automatic route execution.
