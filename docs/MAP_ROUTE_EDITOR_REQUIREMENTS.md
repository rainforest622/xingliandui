# Map Route Editor Requirements

## Product Position

The mobile application uses an imported image as a map background. The operator annotates the patrol route on top of that map and sends only the resulting motion segments to WS63 over NearLink SLE.

This feature is a calibrated time-based route planner. The current chassis does not include wheel encoders, visual localization, a deployed AI model, or SLAM. The image is not automatically interpreted as a navigable map.

## Route Authoring

- A map image is an optional background layer. It stays visible while the operator draws.
- The editor occupies its own Route workspace. Its width fills the available Route page and its height follows the source image aspect ratio; the background image uses contain-fit rendering and is never stretched to a fixed canvas ratio.
- The default authoring mode is point-by-point. Each tap adds one route vertex and one straight segment, so an operator never needs to complete a route in a single pen stroke. The toolbar also provides straight-line, triangle, rectangle, edit, start-point, and end-point tools.
- The editor renders visible vertex handles. In edit mode, a handle can be dragged to adjust a route corner, while dragging inside a closed route translates the complete route.
- Triangle and rectangle templates are created by dragging across their bounds. Their later corner edits remain constrained as a symmetric triangle or an axis-aligned rectangle instead of degrading into arbitrary polygons.
- During a pen stroke the surrounding page scroll is disabled, so the stroke remains aligned with the pen rather than drifting with the page.
- A closed route returns to its start. The start tool rotates the closed point sequence so that the selected point becomes the physical departure point.
- An open route has a green `S` start point and a red `E` end point. Selecting the start nearest the opposite endpoint reverses the route. The end tool changes the final point only for an open route.
- The selected route direction is the actual initial heading requirement: before launch, place the vehicle at `S` and align its front with the first green segment.

## Route Library

- A single image can have multiple separately named saved routes.
- Each saved item retains its image URI, original route shape type when applicable, points, route length, WS63 speed, loop count, straight-line calibration, and 90-degree turn calibration.
- Saved routes are stored in HarmonyOS Preferences under `robot_route_library` and can be selected, updated, or deleted from the editor.
- Selection restores the complete route configuration. It does not overwrite other routes using the same map image.

## Upload Constraints

- Before upload, the phone reduces every route to at most 12 movement segments.
- The SLE transaction remains `R0` (begin), `R1` (each segment), and `R2` (commit). WS63 rejects incomplete or invalid routes and preserves the previous accepted route.
- Open routes are limited to one loop. Closed routes may use finite loops or continuous looping.
- The calibrated baseline is `0.80 s/m` at speed 100 and `686 ms` for a 90-degree right turn at speed 100. These are physical calibration values, not localization measurements.

## Independent Wheel Calibration

- The mobile slider range is 70% to 120% for each wheel.
- The `W,<left>,<right>` SLE command writes the two scales to WS63. A new calibration stops the vehicle and exits autonomous motion before applying the change.
- The scale is applied in the single WS63-to-WAVE-ROVER motor bridge, so remote driving, route patrol, turns, and obstacle detours use the same correction.
- Changing either wheel scale changes effective linear and angular motion. Re-check the one-metre and 90-degree calibrations after a wheel-scale adjustment.

## Manual NearLink Speed

- Remote driving has its own 30% to 100% speed setting and no longer always sends full motor duty.
- The phone sends `V,<speed>` over SLE before driving. WS63 replies with `+ROBOT:SPEED,OK,<speed>` and uses that value for forward, reverse, left, and right commands.
- The default manual setting after a firmware restart is 70%. Route patrol speed remains an independent route parameter, so planned routes keep their calibrated speed.

## Reproducible Build and Flash

- `tools/build_ws63_liteos_app.ps1` first synchronizes `firmware/deveco_ws63_overlay/application/samples/peripheral/robot_mvp` into the configured `D:\b\src` SDK source before building. This prevents stale SDK files from producing an unchanged firmware package.
- The default package for `tools/burn_ws63_app.ps1` is `firmware/fwpkg/ws63-liteos-app_ws63_route_editor_manual_speed_load_only.fwpkg`.
- The wheel-calibration and manual-speed protocols are available only after that package is flashed and the board has restarted into the new firmware.
