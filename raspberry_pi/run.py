from __future__ import annotations

import argparse
import json
import os
import threading
import time
from http import server
from pathlib import Path
import urllib.error
import urllib.request
from urllib.parse import unquote

import cv2

from alarm.manager import AlarmManager
from camera.capture import CameraCapture
from detection.fire_smoke_detector import FireSmokeDetector
from detection.person_detector import PersonDetector
from metrics.collector import MetricsCollector

_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WS63 Robot - Raspberry Pi Camera</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#101214; color:#e9eef5; font-family:Arial, sans-serif; }
  header { padding:14px 16px 8px; background:#171b20; border-bottom:1px solid #26303a; }
  h1 { margin:0; font-size:20px; }
  .sub { margin-top:4px; color:#9aa8b8; font-size:13px; }
  #stream { display:block; width:100%; max-height:64vh; object-fit:contain; background:#050607; }
  #panel { display:grid; grid-template-columns:repeat(auto-fit, minmax(112px, 1fr)); gap:8px; padding:10px; }
  .card { background:#171b20; border:1px solid #28323c; border-radius:6px; padding:8px 10px; min-height:54px; }
  .label { color:#94a3b8; font-size:12px; }
  .val { margin-top:3px; font-size:19px; font-weight:700; color:#7fd08a; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .warn .val { color:#ffbf66; }
  .bad .val { color:#ff6b6b; }
  #camera-error { padding:0 12px 8px; color:#ffbf66; font-size:13px; word-break:break-word; }
  #alarms { padding:0 10px 14px; }
  #alarms h3 { margin:6px 0; font-size:15px; color:#cbd5e1; }
  .alarm-row { background:#2a1717; border:1px solid #633; border-radius:4px; padding:6px 8px; margin:4px 0; font-size:13px; }
  a { color:#8bd3ff; }
</style>
</head>
<body>
<header>
  <h1>Raspberry Pi Camera</h1>
  <div class="sub">Real CSI/USB camera stream from the robot Raspberry Pi</div>
</header>
<img id="stream" src="/stream.mjpg" alt="MJPEG stream">
<div id="panel">
  <div class="card"><div class="label">Camera</div><div class="val" id="backend">--</div></div>
  <div class="card"><div class="label">Frame</div><div class="val" id="frame">--</div></div>
  <div class="card"><div class="label">FPS</div><div class="val" id="fps">--</div></div>
  <div class="card"><div class="label">CPU</div><div class="val" id="cpu">--</div></div>
  <div class="card"><div class="label">Temp</div><div class="val" id="temp">--</div></div>
  <div class="card"><div class="label">BW kbps</div><div class="val" id="bw">--</div></div>
  <div class="card"><div class="label">Drop</div><div class="val" id="drop">--</div></div>
  <div class="card" id="det-card"><div class="label">Detection</div><div class="val" id="det">--</div></div>
  <div class="card" id="fire-card"><div class="label">Fire/Smoke</div><div class="val" id="fire">--</div></div>
  <div class="card" id="alarm-card"><div class="label">Alarm</div><div class="val" id="alarm">--</div></div>
</div>
<div id="camera-error"></div>
<div id="alarms"><h3>Recent Alarms</h3></div>
<script>
(async function loop() {
  try {
    const r = await fetch('/status');
    const s = await r.json();
    const backend = document.getElementById('backend');
    backend.textContent = s.camera_backend || '--';
    backend.parentElement.className = s.camera_backend === 'error' ? 'card bad' : 'card';
    document.getElementById('frame').textContent = s.frame_ready ? String(s.frames_total) : 'WAIT';
    document.getElementById('fps').textContent = Number(s.fps_actual || 0).toFixed(1);
    document.getElementById('cpu').textContent = Number(s.cpu_percent || 0).toFixed(1) + '%';
    const temp = document.getElementById('temp');
    temp.textContent = s.cpu_temp_c != null ? Number(s.cpu_temp_c).toFixed(0) + ' C' : '--';
    temp.parentElement.className = s.cpu_temp_c > 75 ? 'card warn' : 'card';
    document.getElementById('bw').textContent = Number(s.bandwidth_kbps || 0).toFixed(0);
    document.getElementById('drop').textContent = (Number(s.drop_rate || 0) * 100).toFixed(1) + '%';
    document.getElementById('det').textContent = s.person_detection_enabled ? (s.person_detected ? 'PERSON' : 'CLEAR') : 'OFF';
    document.getElementById('det').style.color = s.person_detected ? '#ff6b6b' : '#7fd08a';
    const fire = document.getElementById('fire');
    fire.textContent = s.fire_smoke_detection_enabled ? (s.fire_detected ? 'FIRE' : (s.smoke_detected ? 'SMOKE' : 'CLEAR')) : 'OFF';
    fire.style.color = s.fire_smoke_detected ? '#ff6b6b' : '#7fd08a';
    const alarmCard = document.getElementById('alarm-card');
    alarmCard.className = s.alarm_active ? 'card bad' : 'card';
    document.getElementById('alarm').textContent = s.alarm_active ? 'ON' : 'OFF';
    document.getElementById('camera-error').textContent = s.camera_error ? 'Camera: ' + s.camera_error : '';
    const ar = await fetch('/alarms');
    const alarms = await ar.json();
    const div = document.getElementById('alarms');
    div.innerHTML = '<h3>Recent Alarms (' + alarms.length + ')</h3>' +
      alarms.slice(-10).reverse().map(a =>
        '<div class="alarm-row">' + a.time + ' - ' + a.filename +
        ' <a href="/snapshots/' + a.filename + '">view</a></div>'
      ).join('');
  } catch(e) {
    document.getElementById('camera-error').textContent = 'Status request failed. Check Pi service and Wi-Fi.';
  }
  setTimeout(loop, 1500);
})();
</script>
</body>
</html>"""

_DRIVER_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>WS63 Robot Driver Camera</title>
<style>
  :root { color-scheme: dark; }
  html, body { width:100%; height:100%; margin:0; overflow:hidden; background:#050607; }
  body { font-family:Arial, sans-serif; color:#e5eef8; }
  #stage { position:fixed; inset:0; display:flex; align-items:center; justify-content:center; background:#050607; }
  #stream { width:100vw; height:100vh; object-fit:contain; background:#050607; }
  #hud {
    position:fixed; left:10px; right:10px; bottom:10px; display:flex; align-items:center; gap:8px;
    padding:7px 9px; border:1px solid rgba(148,163,184,.26); border-radius:6px;
    background:rgba(5,6,7,.68); backdrop-filter:blur(6px); box-sizing:border-box;
  }
  .pill { flex:0 0 auto; padding:4px 7px; border-radius:5px; font-size:12px; font-weight:700; background:#14532d; color:#dcfce7; }
  .pill.wait { background:#713f12; color:#fef3c7; }
  .pill.bad { background:#7f1d1d; color:#fee2e2; }
  #detail { flex:1 1 auto; min-width:0; color:#cbd5e1; font-size:12px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
</style>
</head>
<body>
<div id="stage"><img id="stream" src="/stream.mjpg" alt="MJPEG stream"></div>
<div id="hud"><div id="state" class="pill wait">WAIT</div><div id="detail">camera starting</div></div>
<script>
(async function loop() {
  try {
    const r = await fetch('/status', { cache: 'no-store' });
    const s = await r.json();
    const state = document.getElementById('state');
    const detail = document.getElementById('detail');
    if (s.camera_error) {
      state.className = 'pill bad';
      state.textContent = 'ERROR';
      detail.textContent = s.camera_error;
    } else if (s.frame_ready) {
      state.className = 'pill';
      state.textContent = 'LIVE';
      detail.textContent = (s.camera_backend || 'camera') + ' | ' +
        Number(s.fps_actual || 0).toFixed(1) + ' fps | frame ' +
        String(s.frames_total || 0);
    } else {
      state.className = 'pill wait';
      state.textContent = 'WAIT';
      detail.textContent = 'service online, waiting for a frame';
    }
  } catch(e) {
    const state = document.getElementById('state');
    state.className = 'pill bad';
    state.textContent = 'OFFLINE';
    document.getElementById('detail').textContent = 'status request failed';
  }
  setTimeout(loop, 1500);
})();
</script>
</body>
</html>"""


class AiRuntimeConfig:
    def __init__(
        self,
        person_detection: bool = False,
        fire_smoke_detection: bool = False,
        fire_smoke_interval_ms: int = 1000,
    ):
        self._lock = threading.Lock()
        self.person_detection = person_detection
        self.fire_smoke_detection = fire_smoke_detection
        self.fire_smoke_interval_ms = fire_smoke_interval_ms

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "person_detection": self.person_detection,
                "fire_smoke_detection": self.fire_smoke_detection,
                "fire_smoke_interval_ms": self.fire_smoke_interval_ms,
            }

    def update(self, payload: dict[str, object]) -> dict[str, object]:
        with self._lock:
            if "person_detection" in payload:
                self.person_detection = bool(payload["person_detection"])
            if "fire_smoke_detection" in payload:
                self.fire_smoke_detection = bool(payload["fire_smoke_detection"])
            if "fire_smoke_interval_ms" in payload:
                try:
                    interval_ms = int(payload["fire_smoke_interval_ms"])
                except (TypeError, ValueError):
                    interval_ms = self.fire_smoke_interval_ms
                self.fire_smoke_interval_ms = max(500, min(5000, interval_ms))
            return {
                "person_detection": self.person_detection,
                "fire_smoke_detection": self.fire_smoke_detection,
                "fire_smoke_interval_ms": self.fire_smoke_interval_ms,
            }


class VoiceAnnouncementClient:
    def __init__(self, arbiter_url: str, cooldown_sec: float = 8.0, timeout_sec: float = 0.35):
        self.arbiter_url = arbiter_url.rstrip("/")
        self.cooldown_sec = cooldown_sec
        self.timeout_sec = timeout_sec
        self._last_sent: dict[str, float] = {}

    def notify(self, kind: str, message: str, telemetry: dict[str, object] | None = None) -> None:
        if not self.arbiter_url:
            return
        now = time.monotonic()
        if now - self._last_sent.get(kind, 0.0) < self.cooldown_sec:
            return
        self._last_sent[kind] = now
        payload = {
            "kind": kind,
            "message": message,
            "source": "pi-camera",
            "telemetry": telemetry or {},
        }
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.arbiter_url}/voice/announcement",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec):
                pass
        except (urllib.error.URLError, TimeoutError, OSError):
            return


class RequestHandler(server.BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        path = self.path.split("?")[0]

        if path == "/":
            self._serve_html()
        elif path == "/driver":
            self._serve_driver_html()
        elif path == "/stream.mjpg":
            self._serve_stream()
        elif path == "/status":
            self._serve_status()
        elif path == "/ai/config":
            self._serve_ai_config()
        elif path == "/healthz":
            self._serve_healthz()
        elif path == "/alarms":
            self._serve_alarms()
        elif path.startswith("/snapshots/"):
            self._serve_snapshot(path)
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        path = self.path.split("?")[0]
        if path != "/ai/config":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else {}
        except json.JSONDecodeError:
            self._send_json(400, {"ok": False, "error": "invalid json"})
            return
        if not isinstance(payload, dict):
            self._send_json(400, {"ok": False, "error": "body must be an object"})
            return
        config = self.server.ai_config.update(payload)
        if config["person_detection"]:
            start = getattr(self.server.detector, "start", None)
            if callable(start):
                start()
        else:
            self.server.detector.clear()
        if config["fire_smoke_detection"]:
            start = getattr(self.server.fire_smoke, "start", None)
            if callable(start):
                start()
        else:
            self.server.fire_smoke.reset()
        self.server.alarm.clear()
        self.server.fire_smoke.cooldown_sec = float(config["fire_smoke_interval_ms"]) / 1000.0
        self._send_json(200, {"ok": True, "ai": config})

    def _send_json(self, status_code: int, payload: object) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_html(self) -> None:
        body = _HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_driver_html(self) -> None:
        body = _DRIVER_HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while True:
            frame, _fps = self.server.capture.read()
            if frame is None or not self.server.capture.frame_is_fresh:
                if self.server.capture.active_backend == "error":
                    return
                time.sleep(0.02)
                continue
            self.server.metrics.tick_captured()

            ai_config = self.server.ai_config.snapshot()
            person_enabled = bool(ai_config["person_detection"])
            fire_smoke_enabled = bool(ai_config["fire_smoke_detection"])

            if person_enabled:
                self.server.detector.feed(frame)
            else:
                self.server.detector.clear()
            person_present = person_enabled and self.server.detector.last_present
            person_count = len(self.server.detector.last_boxes) if person_enabled else 0

            if fire_smoke_enabled:
                self.server.fire_smoke.cooldown_sec = float(ai_config["fire_smoke_interval_ms"]) / 1000.0
                fire_smoke_result = self.server.fire_smoke.feed(frame)
            else:
                self.server.fire_smoke.reset()
                fire_smoke_result = self.server.fire_smoke.last_result

            if person_present:
                self.server.voice.notify("person_alert", "person detected near the patrol route")
            if fire_smoke_result.active:
                self.server.voice.notify(
                    "fire_smoke_alert",
                    "visual fire or smoke abnormality detected; patrol continues",
                    fire_smoke_result.snapshot(),
                )

            alarm_active = bool(person_present or fire_smoke_result.active)
            alarm_kind = "person" if person_present else "fire_smoke"
            if person_present and fire_smoke_result.active:
                alarm_kind = "person_fire_smoke"
            self.server.alarm.feed(
                alarm_active,
                frame,
                person_count,
                kind=alarm_kind,
                detail="visual alert; patrol is not stopped by camera service",
            )

            annotated = frame.copy()
            if person_enabled:
                annotated = self.server.detector.annotate(annotated)
            if fire_smoke_enabled:
                annotated = self.server.fire_smoke.annotate(annotated)
            ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                continue
            data = encoded.tobytes()
            self.server.metrics.tick_encoded(len(data))
            try:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
            time.sleep(1.0 / max(1, self.server.capture.target_fps))

    def _serve_status(self) -> None:
        metrics = self.server.metrics.snapshot()
        ai_config = self.server.ai_config.snapshot()
        fire_smoke = self.server.fire_smoke.last_result.snapshot()
        person_detected = bool(ai_config["person_detection"] and self.server.detector.last_present)
        status = {
            **self.server.capture.status(),
            "ai": ai_config,
            "person_detection_enabled": ai_config["person_detection"],
            "fire_smoke_detection_enabled": ai_config["fire_smoke_detection"],
            "fire_smoke_model_loaded": bool(getattr(self.server.fire_smoke, "model_loaded", False)),
            "fire_smoke_model_error": str(getattr(self.server.fire_smoke, "model_error", "")),
            "person_detected": person_detected,
            **fire_smoke,
            "alarm_active": self.server.alarm.active,
            **metrics,
        }
        body = json.dumps(status).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_healthz(self) -> None:
        frame_ready = self.server.capture.read()[0] is not None
        body = json.dumps({
            "ok": frame_ready and self.server.capture.active_backend != "error",
            "camera_backend": self.server.capture.active_backend,
            "camera_error": self.server.capture.last_error,
        }).encode()
        status_code = 200 if frame_ready else 503
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_alarms(self) -> None:
        alarms = self.server.alarm.get_alarms()
        body = json.dumps(alarms).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_snapshot(self, path: str) -> None:
        filename = unquote(os.path.basename(path))
        safe_dir = str(self.server.snapshot_dir.resolve())
        file_path = os.path.normpath(os.path.join(safe_dir, filename))
        if not file_path.startswith(safe_dir):
            self.send_error(403)
            return
        if not os.path.isfile(file_path):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(os.path.getsize(file_path)))
        self.end_headers()
        with open(file_path, "rb") as f:
            self.wfile.write(f.read())

    def _serve_ai_config(self) -> None:
        self._send_json(200, {"ok": True, "ai": self.server.ai_config.snapshot()})

    def log_message(self, *_: object) -> None:
        return


class AppServer(server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def build_capture(args: argparse.Namespace) -> CameraCapture:
    return CameraCapture(
        args.camera,
        args.width,
        args.height,
        args.fps,
        args.synthetic,
        args.camera_backend,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="WS63 Pi Vision: real camera MJPEG + person detection")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15, help="Target FPS")
    parser.add_argument(
        "--camera-backend",
        choices=["auto", "picamera2", "v4l2", "synthetic"],
        default="auto",
        help="auto tries Picamera2/CSI first, then V4L2 /dev/video*",
    )
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--self-test", action="store_true", help="Open the camera, wait for one frame, print JSON, and exit")
    parser.add_argument("--camera-timeout", type=float, default=8.0)
    parser.add_argument("--detect-cooldown", type=float, default=0.5)
    parser.add_argument("--motion-threshold", type=float, default=0.01)
    parser.add_argument("--person-detect", action="store_true", help="Enable person detection at startup")
    parser.add_argument("--no-detect", action="store_true", help="Legacy alias that keeps person detection disabled")
    parser.add_argument("--fire-smoke-detect", action="store_true", help="Enable lightweight fire/smoke detection at startup")
    parser.add_argument("--fire-smoke-interval-ms", type=int, default=1000)
    parser.add_argument("--voice-arbiter-url", default="http://127.0.0.1:8090")
    parser.add_argument("--voice-alert-cooldown", type=float, default=8.0)
    parser.add_argument("--alarm-cooldown", type=float, default=5.0)
    parser.add_argument("--snapshot-dir", default="snapshots")
    args = parser.parse_args()

    capture = build_capture(args)
    capture.start()

    if args.self_test:
        ok = capture.wait_for_frame(args.camera_timeout)
        print(json.dumps(capture.status(), indent=2))
        capture.stop()
        raise SystemExit(0 if ok else 2)

    detector = PersonDetector(
        cooldown_sec=args.detect_cooldown,
        motion_threshold=args.motion_threshold,
    )
    metrics = MetricsCollector()
    alarm = AlarmManager(snapshot_dir=args.snapshot_dir, cooldown_sec=args.alarm_cooldown)
    fire_smoke = FireSmokeDetector(cooldown_sec=max(0.5, args.fire_smoke_interval_ms / 1000.0))
    ai_config = AiRuntimeConfig(
        person_detection=args.person_detect and not args.no_detect,
        fire_smoke_detection=args.fire_smoke_detect,
        fire_smoke_interval_ms=args.fire_smoke_interval_ms,
    )
    voice = VoiceAnnouncementClient(args.voice_arbiter_url, cooldown_sec=args.voice_alert_cooldown)

    if ai_config.snapshot()["person_detection"]:
        detector.start()
    if ai_config.snapshot()["fire_smoke_detection"]:
        fire_smoke.start()

    srv = AppServer((args.host, args.port), RequestHandler)
    srv.capture = capture
    srv.detector = detector
    srv.fire_smoke = fire_smoke
    srv.ai_config = ai_config
    srv.voice = voice
    srv.metrics = metrics
    srv.alarm = alarm
    srv.snapshot_dir = Path(args.snapshot_dir).resolve()

    url = f"http://{args.host}:{args.port}"
    if args.host == "0.0.0.0":
        url = f"http://localhost:{args.port}"
    print(f"MJPEG stream:  {url}/stream.mjpg")
    print(f"Status:        {url}/status")
    print(f"Health:        {url}/healthz")
    print(f"Alarms:        {url}/alarms")
    print(f"Snapshots:     {srv.snapshot_dir}")
    print(f"FPS target:    {args.fps}")
    print(f"Camera:        requested={capture.backend}, active={capture.active_backend}")
    print(f"Detection:     {'SSD ready' if detector.model_loaded else 'off'}")
    print(f"AI config:     {json.dumps(ai_config.snapshot(), ensure_ascii=False)}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        capture.stop()
        detector.stop()
        srv.server_close()


if __name__ == "__main__":
    main()
