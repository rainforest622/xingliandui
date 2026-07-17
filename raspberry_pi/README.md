# Raspberry Pi Camera Runtime

This folder is the real Raspberry Pi camera service for the robot. It opens the Pi camera, publishes an MJPEG stream, exposes status JSON, and optionally runs person detection with the bundled MobileNetSSD model.

## Install On Raspberry Pi

```bash
cd raspberry_pi
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-psutil
python3 -m pip install -r requirements.txt
```

For a CSI camera, verify the hardware first:

```bash
rpicam-hello --list-cameras
rpicam-hello -t 3000
```

## Self Test

Run this before starting the web page:

```bash
python3 run.py --camera-backend auto --self-test --camera-timeout 8
```

Success means `frame_ready` is `true` and `camera_backend` is usually `picamera2` for a CSI camera or `v4l2` for a USB camera.

## Start The Service

```bash
python3 run.py --host 0.0.0.0 --port 8080 --camera-backend auto --width 640 --height 480 --fps 15
```

Or:

```bash
./start_pi_camera.sh
```

For the current robot setup this is equivalent to:

```bash
python3 run.py --width 640 --height 480 --fps 15 --detect-cooldown 1.0
```

## Run At Boot

Install the camera runtime as a systemd service:

```bash
cd /home/xinglian/raspberry_pi
chmod +x install_pi_camera_service.sh
./install_pi_camera_service.sh
```

Useful service commands:

```bash
sudo systemctl status nearlink-pi-camera --no-pager
sudo systemctl restart nearlink-pi-camera
sudo journalctl -u nearlink-pi-camera -f
```

Open from the tablet or PC:

```text
http://192.168.43.42:8080/
```

Useful endpoints:

```text
/              dashboard
/driver        full-frame driver camera view for the Harmony app
/stream.mjpg   MJPEG camera stream
/status        camera, detection, and system metrics
/healthz       200 when a camera frame is ready, 503 otherwise
/alarms        recent person-detection alarms
```

## Backend Options

```bash
python3 run.py --camera-backend auto
python3 run.py --camera-backend picamera2 --camera 0
python3 run.py --camera-backend v4l2 --camera 0
python3 run.py --camera-backend synthetic --no-detect
```

`auto` is the normal robot mode: it tries Picamera2/CSI first, then falls back to USB/V4L2.

## WAVE ROVER Serial Bridge

The current hardware plan uses the Raspberry Pi as a transparent serial bridge:

```text
WS63 Type-C USB serial -> Raspberry Pi -> 40PIN UART -> WAVE ROVER ESP32
```

Enable the Raspberry Pi 40PIN UART first:

```bash
sudo raspi-config
# Interface Options -> Serial Port
# Login shell over serial? No
# Enable serial hardware? Yes
sudo reboot
```

Manual run:

```bash
cd raspberry_pi
python3 rover_bridge.py --ws63-port auto --rover-port auto
```

On Raspberry Pi 5, `dtparam=uart0=on` exposes the GPIO14/GPIO15 header UART as
`/dev/ttyAMA0`; `/dev/serial0` can still point at `ttyAMA10`. The bridge and
checker therefore default to `--rover-port auto` and prefer `/dev/ttyAMA0` when
`pinctrl get 14` / `pinctrl get 15` show `TXD0` / `RXD0`.

Before running the bridge, do a stop-only hardware check:

```bash
python3 check_rover_bridge.py --rover-port auto
```

This lists serial devices, opens the WAVE ROVER UART, sends only a stop command, and reads any feedback. After the car is lifted or placed in a safe open area, run a short low-speed pulse:

```bash
python3 check_rover_bridge.py --rover-port auto --move-test --speed 0.10 --duration 0.25
```

To confirm the WS63 USB serial is producing JSON lines without forwarding them:

```bash
python3 check_rover_bridge.py --ws63-listen --ws63-port auto --rover-port auto --listen-seconds 5
```

If auto detection chooses the wrong port:

```bash
ls -l /dev/serial0 /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null
pinctrl get 14
pinctrl get 15
python3 rover_bridge.py --ws63-port /dev/ttyUSB0 --rover-port /dev/ttyAMA0
```

Install as a systemd service after the manual test works:

```bash
chmod +x start_rover_bridge.sh install_rover_bridge_service.sh
./install_rover_bridge_service.sh
```

Useful commands:

```bash
sudo systemctl status nearlink-rover-bridge --no-pager
sudo systemctl restart nearlink-rover-bridge
sudo journalctl -u nearlink-rover-bridge -f
```

## Safety Arbiter Service

After the transparent bridge is verified, use the safety arbiter as the normal
runtime. It keeps WS63/SLE manual control as the highest normal priority, sends
a stop when commands time out, and exposes a small HTTP control/status API for
the next auto-patrol stage.

```bash
cd /home/xinglian/raspberry_pi
chmod +x start_rover_arbiter.sh install_rover_arbiter_service.sh
./install_rover_arbiter_service.sh
```

The installer stops and disables `nearlink-rover-bridge.service` but does not
delete it. To roll back:

```bash
sudo systemctl stop nearlink-rover-arbiter
sudo systemctl enable --now nearlink-rover-bridge
```

Useful arbiter commands:

```bash
sudo systemctl status nearlink-rover-arbiter --no-pager
sudo journalctl -u nearlink-rover-arbiter -f
curl http://127.0.0.1:8090/status
curl http://127.0.0.1:8090/estop
curl http://127.0.0.1:8090/release
curl "http://127.0.0.1:8090/auto/start?speed=0.18&forward=3.0&turn_s=0.65"
curl "http://127.0.0.1:8090/auto/map?loops=1&speed_scale=0.8"
curl http://127.0.0.1:8090/auto/stop
```

The arbiter starts in `manual` mode, so it will not begin auto patrol on boot.
Test auto patrol only on an open floor, starting with low speed.

After the camera `/healthz` endpoint is healthy, a first visual line-following
mode is available. It reads the local MJPEG stream, detects yellow track tape
in the lower half of the frame, and stops if the line is lost:

```bash
curl "http://127.0.0.1:8090/auto/vision?speed=0.16&gain=0.14&min_area=600&color=yellow"
curl http://127.0.0.1:8090/status
curl http://127.0.0.1:8090/auto/stop
```

Manual SLE commands still override visual patrol for the manual timeout window,
and `/estop` remains the highest-priority command.
Use `color=black` or `color=both` only after checking that the floor and shadows
are not being detected as the track.

## Map Brain Patrol

`patrol_map.json` is the first "map brain" for the robot. It replaces hard
visual line following with a preloaded patrol route made of timed movement
steps. This is intentionally conservative: WAVE ROVER has no wheel encoders, so
the current map is a calibratable timed route, not full SLAM localization.

The default route is an approximately 40 m2 square patrol:

```text
A -> B -> C -> D -> A
```

Each side has a forward step, each corner has an inspection wait and a right
turn. The current floor-tested turn parameter is `speed=0.5` and
`duration_s=0.8`. Tune `duration_s` values in `patrol_map.json` after a floor
test if battery level or floor friction changes.

Start one low-speed map patrol loop:

```bash
curl "http://127.0.0.1:8090/auto/map?loops=1&speed_scale=0.8"
curl http://127.0.0.1:8090/status
curl http://127.0.0.1:8090/auto/stop
```

Useful query parameters:

```text
loops=1          run the route once; loops=0 or loop=true means repeat forever
speed_scale=0.8  scale every map speed for safer first tests
```

`/manual`, `/auto/stop`, and `/estop` stop the map brain. Manual SLE commands
still override background auto modes during the manual timeout window.

## Route Import API

The HarmonyOS app can import a patrol route over Wi-Fi/HTTP while keeping
start, stop, emergency stop, and manual override on SLE. This keeps large route
data on the data channel and real-time control on NearLink.

Import a route:

```bash
curl -X POST http://127.0.0.1:8090/route/import \
  -H "Content-Type: application/json" \
  --data-binary @patrol_map.json
```

Check the active route:

```bash
curl http://127.0.0.1:8090/route/current
```

Start the imported route:

```bash
curl -X POST http://127.0.0.1:8090/route/start \
  -H "Content-Type: application/json" \
  -d '{"loops":1,"speed_scale":1.0}'
```

Stop the imported route:

```bash
curl -X POST http://127.0.0.1:8090/route/stop
```

Minimum route JSON:

```json
{
  "name": "factory-patrol-demo",
  "default_speed": 0.24,
  "default_turn_speed": 0.5,
  "steps": [
    { "id": "s1", "action": "move", "speed": 0.24, "duration_s": 5.0 },
    { "id": "t1", "action": "turn", "direction": "right", "speed": 0.5, "duration_s": 0.8 },
    { "id": "s2", "action": "move", "speed": 0.24, "duration_s": 5.0 }
  ]
}
```

Supported actions are `move`, `turn`, `wait`, `inspect`, `stop`, and `pause`.
The import API validates the route and saves it as `active_route.json`.
