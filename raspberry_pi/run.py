from __future__ import annotations

import argparse
import json
import os
import time
from http import server
from pathlib import Path
from urllib.parse import unquote

import cv2

from camera.capture import CameraCapture
from detection.person_detector import PersonDetector
from metrics.collector import MetricsCollector
from alarm.manager import AlarmManager

_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WS63 Inspection — Pi 5 Vision</title>
<style>
  body { margin:0; background:#111; color:#eee; font-family:monospace; display:flex; flex-direction:column; align-items:center; }
  img { max-width:100%; border:2px solid #333; margin:8px; }
  #panel { display:flex; gap:12px; flex-wrap:wrap; justify-content:center; padding:8px; }
  .card { background:#1a1a2e; border:1px solid #333; border-radius:6px; padding:8px 14px; min-width:100px; text-align:center; }
  .card .val { font-size:20px; font-weight:bold; color:#0f0; }
  .alarm-on .val { color:#f33; }
  .warn .val { color:#fa0; }
  #alarms { max-width:640px; width:100%; padding:8px; }
  #alarms h3 { margin:4px 0; }
  .alarm-row { background:#2a1a1a; border:1px solid #633; border-radius:4px; padding:4px 8px; margin:4px 0; font-size:13px; }
</style>
</head>
<body>
<img id="stream" src="/stream.mjpg" alt="MJPEG stream">
<div id="panel">
  <div class="card"><div>FPS</div><div class="val" id="fps">--</div></div>
  <div class="card"><div>CPU</div><div class="val" id="cpu">--</div></div>
  <div class="card"><div>Temp</div><div class="val" id="temp">--</div></div>
  <div class="card"><div>BW kbps</div><div class="val" id="bw">--</div></div>
  <div class="card"><div>Drop</div><div class="val" id="drop">--</div></div>
  <div class="card" id="det-card"><div>Detection</div><div class="val" id="det">--</div></div>
  <div class="card" id="alarm-card"><div>Alarm</div><div class="val" id="alarm">--</div></div>
</div>
<div id="alarms"><h3>Recent Alarms</h3></div>
<script>
(async function loop() {
  try {
    const r = await fetch('/status');
    const s = await r.json();
    document.getElementById('fps').textContent = s.fps_actual.toFixed(1);
    document.getElementById('cpu').textContent = s.cpu_percent.toFixed(1) + '%';
    var tc = document.getElementById('temp');
    tc.textContent = s.cpu_temp_c != null ? s.cpu_temp_c.toFixed(0) + '°C' : '--';
    if (s.cpu_temp_c > 75) tc.parentElement.className = 'card warn';
    else tc.parentElement.className = 'card';
    document.getElementById('bw').textContent = s.bandwidth_kbps.toFixed(0);
    document.getElementById('drop').textContent = (s.drop_rate * 100).toFixed(1) + '%';
    document.getElementById('det').textContent = s.person_detected ? 'PERSON' : 'CLEAR';
    document.getElementById('det').style.color = s.person_detected ? '#f33' : '#0f0';
    var ac = document.getElementById('alarm-card');
    ac.className = s.alarm_active ? 'card alarm-on' : 'card';
    document.getElementById('alarm').textContent = s.alarm_active ? 'ON' : 'OFF';
    const ar = await fetch('/alarms');
    const alarms = await ar.json();
    const div = document.getElementById('alarms');
    div.innerHTML = '<h3>Recent Alarms (' + alarms.length + ')</h3>' +
      alarms.slice(-10).reverse().map(a =>
        '<div class="alarm-row">' + a.time + ' &mdash; ' + a.filename +
        ' <a href="/snapshots/' + a.filename + '" style="color:#6cf">view</a></div>'
      ).join('');
  } catch(e) {}
  setTimeout(loop, 1500);
})();
</script>
</body>
</html>"""


class RequestHandler(server.BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        path = self.path.split("?")[0]

        if path == "/":
            self._serve_html()
        elif path == "/stream.mjpg":
            self._serve_stream()
        elif path == "/status":
            self._serve_status()
        elif path == "/alarms":
            self._serve_alarms()
        elif path.startswith("/snapshots/"):
            self._serve_snapshot(path)
        else:
            self.send_error(404)

    def _serve_html(self) -> None:
        body = _HTML.encode()
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
            if frame is None:
                time.sleep(0.02)
                continue
            self.server.metrics.tick_captured()

            # Non-blocking: feed frame to async detector
            self.server.detector.feed(frame)
            # Read latest async detection result
            person_present = self.server.detector.last_present
            # Feed alarm manager from cached detection result
            person_count = len(self.server.detector.last_boxes)
            self.server.alarm.feed(person_present, frame, person_count)

            annotated = self.server.detector.annotate(frame)
            ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                continue
            data = encoded.tobytes()
            self.server.metrics.tick_encoded(len(data))
            try:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return
            time.sleep(1.0 / max(1, self.server.capture.target_fps))

    def _serve_status(self) -> None:
        metrics = self.server.metrics.snapshot()
        status = {
            "fps_actual": self.server.capture.read()[1],
            "fps_target": self.server.capture.target_fps,
            "frames_total": self.server.capture.frames_total,
            "person_detected": self.server.detector.last_present,
            "alarm_active": self.server.alarm.active,
            **metrics,
        }
        body = json.dumps(status).encode()
        self.send_response(200)
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

    def log_message(self, *_: object) -> None:
        return


class AppServer(server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser(description="WS63 Pi 5 Vision — MJPEG + async person detection")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15, help="Target FPS (10/15/20)")
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--detect-cooldown", type=float, default=0.5,
                        help="Min seconds between SSD runs (default 0.5)")
    parser.add_argument("--motion-threshold", type=float, default=0.01,
                        help="Motion trigger sensitivity 0..1 (default 0.01)")
    parser.add_argument("--no-detect", action="store_true", help="Disable person detection entirely")
    parser.add_argument("--alarm-cooldown", type=float, default=5.0)
    parser.add_argument("--snapshot-dir", default="snapshots")
    args = parser.parse_args()

    capture = CameraCapture(args.camera, args.width, args.height, args.fps, args.synthetic)
    detector = PersonDetector(
        cooldown_sec=args.detect_cooldown,
        motion_threshold=args.motion_threshold,
    )
    metrics = MetricsCollector()
    alarm = AlarmManager(snapshot_dir=args.snapshot_dir, cooldown_sec=args.alarm_cooldown)

    capture.start()
    if not args.no_detect:
        detector.start()

    srv = AppServer((args.host, args.port), RequestHandler)
    srv.capture = capture
    srv.detector = detector
    srv.metrics = metrics
    srv.alarm = alarm
    srv.snapshot_dir = Path(args.snapshot_dir).resolve()

    url = f"http://{args.host}:{args.port}"
    if args.host == "0.0.0.0":
        url = f"http://localhost:{args.port}"
    print(f"MJPEG stream:  {url}/stream.mjpg")
    print(f"Status:        {url}/status")
    print(f"Alarms:        {url}/alarms")
    print(f"Snapshots:     {srv.snapshot_dir}")
    print(f"FPS target:    {args.fps}")
    print(f"Detection:     {'off' if args.no_detect else ('SSD ready' if detector.model_loaded else 'NO MODEL — run: bash download_models.sh')}")
    print(f"Synthetic:     {args.synthetic}")
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
