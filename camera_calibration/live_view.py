#!/usr/bin/env python3
"""Live side-by-side view of both cameras, with terminal control of the
arm's first two joints, to physically position the stereo cameras so they
keep the arm in frame through its full range of motion.

This is not an OpenCV-style calibration (no chessboard, no saved
intrinsics/extrinsics) — purely a physical positioning aid. Nothing is
recorded to disk.

rpi502 has no desktop / display server, so this can't open a native
window. Instead it serves the combined feed as an MJPEG stream over HTTP:
open the printed URL in a browser (VS Code Remote-SSH usually offers to
forward the port automatically when it sees it opened). The server binds
0.0.0.0, so it's also reachable directly at http://<ip-du-pi>:<port>/ from
any device on the same LAN.

While the stream runs, drive channel 0 (base rotation / azimuth) and
channel 1 (shoulder: vertical <-> horizontal, per calibrate_servo.py) from
the terminal to check both cameras see the arm at its extremes. Reuses
move_to_angle from move_servo.py, so movement is ramped and the reached
position is persisted the same way.

Usage:
    python3 -m camera_calibration.live_view
    python3 -m camera_calibration.live_view --port 8100 --speed 40
    python3 -m camera_calibration.live_view --layout stacked

Commands (in the terminal, while the stream is running):
    0 <angle|c|n|x>   moves channel 0 (base rotation / azimuth)
    1 <angle|c|n|x>   moves channel 1 (shoulder: vertical <-> horizontal)
    q                 quits (stops the stream and the server; the arm
                      keeps its last position, PWM still active)
"""

from __future__ import annotations

import argparse
import io
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional

from PIL import Image
from picamera2 import Picamera2

from servo_calibration.calibration import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_FREQUENCY_HZ,
    DEFAULT_POSITIONS_FILE,
    MAX_SERVO_SPEED_DEG_PER_S,
    MAX_SPEED_PERCENT,
    MIN_SPEED_PERCENT,
    ServoCalibration,
    load_all,
    load_last_angle,
    save_last_angle,
)
from servo_calibration.move_servo import move_to_angle
from servo_calibration.pca9685_driver import Pca9685Driver

CHANNELS = (0, 1)  # base rotation (azimuth), shoulder (vertical <-> horizontal)

DEFAULT_PORT = 8100
DEFAULT_SPEED_PERCENT = 40.0
FRAME_SIZE = (640, 480)
STREAM_FPS = 15
JPEG_QUALITY = 80

HELP_TEXT = __doc__.split("Commands", 1)[1]


class CameraStream:
    """Continuously captures from one camera into a shared latest-frame slot."""

    def __init__(self, camera_num: int, frame_size=FRAME_SIZE):
        self.camera_num = camera_num
        self._picam = Picamera2(camera_num=camera_num)
        config = self._picam.create_preview_configuration(main={"size": frame_size, "format": "RGB888"})
        self._picam.configure(config)
        self._lock = threading.Lock()
        self._frame = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._picam.start()
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            # Despite the "RGB888" config name, picamera2 hands back the
            # array with channels in BGR order — swap them so red/blue
            # aren't inverted once PIL (which expects RGB) encodes it.
            # .copy() forces a contiguous array: the reversed slice is a
            # negative-stride view, which PIL can't encode directly.
            frame = self._picam.capture_array()[:, :, ::-1].copy()
            with self._lock:
                self._frame = frame

    def latest(self):
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._picam.stop()
        self._picam.close()


LAYOUT_SIDE_BY_SIDE = "side-by-side"
LAYOUT_STACKED = "stacked"
LAYOUT_CHOICES = (LAYOUT_SIDE_BY_SIDE, LAYOUT_STACKED)
DEFAULT_LAYOUT = LAYOUT_SIDE_BY_SIDE


def combined_jpeg(cam0: CameraStream, cam1: CameraStream, layout: str = DEFAULT_LAYOUT) -> Optional[bytes]:
    frame0, frame1 = cam0.latest(), cam1.latest()
    if frame0 is None or frame1 is None:
        return None
    img0, img1 = Image.fromarray(frame0), Image.fromarray(frame1)
    if layout == LAYOUT_STACKED:
        combined = Image.new("RGB", (max(img0.width, img1.width), img0.height + img1.height))
        combined.paste(img0, (0, 0))
        combined.paste(img1, (0, img0.height))
    else:
        combined = Image.new("RGB", (img0.width + img1.width, max(img0.height, img1.height)))
        combined.paste(img0, (0, 0))
        combined.paste(img1, (img0.width, 0))
    buf = io.BytesIO()
    combined.save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


INDEX_HTML = b"""<!doctype html>
<html><head><title>Michelangelo - camera view</title>
<style>
  body { margin: 0; background: #111; }
  img { display: block; width: 100%; height: auto; }
</style>
</head><body>
<img src="/stream" alt="camera feed">
</body></html>
"""


def make_handler(cam0: CameraStream, cam1: CameraStream, layout: str = DEFAULT_LAYOUT):
    boundary = "frame"

    class MjpegHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/stream":
                self._serve_stream()
            elif self.path in ("/", ""):
                self._serve_index()
            else:
                self.send_error(404)

        def _serve_index(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)

        def _serve_stream(self) -> None:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.end_headers()
            try:
                while True:
                    jpeg = combined_jpeg(cam0, cam1, layout)
                    if jpeg is not None:
                        self.wfile.write(f"--{boundary}\r\n".encode())
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / STREAM_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format_str: str, *args) -> None:
            pass  # keep the terminal free for arm-control feedback

    return MjpegHandler


def move_channel(
    driver: Pca9685Driver,
    calibs: Dict[int, ServoCalibration],
    channel: int,
    raw: str,
    current: Dict[int, Optional[float]],
    speed_deg_per_s: float,
    positions_file,
) -> None:
    calib = calibs[channel]
    if raw == "c":
        target_deg = 0.0
    elif raw == "n":
        target_deg = calib.angle_min_deg
    elif raw == "x":
        target_deg = calib.angle_max_deg
    else:
        try:
            target_deg = float(raw)
        except ValueError:
            print(f"  Unrecognized input for channel {channel} (expected a number, c, n, or x).")
            return
    new_deg = move_to_angle(driver, calib, target_deg, current[channel], speed_deg_per_s)
    if new_deg != current[channel]:
        current[channel] = new_deg
        save_last_angle(channel, new_deg, positions_file)


def run(
    calibs: Dict[int, ServoCalibration],
    frequency_hz: int,
    speed_deg_per_s: float,
    positions_file,
    port: int,
    layout: str,
) -> None:
    print("Starting both cameras...")
    cams = [CameraStream(0), CameraStream(1)]
    for cam in cams:
        cam.start()

    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(cams[0], cams[1], layout))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"MJPEG stream at http://<pi-ip>:{port}/ (or via the port forwarding VS Code offers).")

    driver = Pca9685Driver(frequency_hz=frequency_hz)
    current: Dict[int, Optional[float]] = {
        channel: load_last_angle(channel, positions_file) for channel in CHANNELS
    }
    print(HELP_TEXT)
    try:
        while True:
            try:
                raw = input("> ").strip()
            except KeyboardInterrupt:
                print()
                break
            if raw == "":
                continue
            if raw == "q":
                break
            parts = raw.split(maxsplit=1)
            if len(parts) != 2 or parts[0] not in ("0", "1"):
                print("  Unrecognized input (expected '0 <angle|c|n|x>' or '1 <angle|c|n|x>').")
                continue
            move_channel(driver, calibs, int(parts[0]), parts[1], current, speed_deg_per_s, positions_file)
    finally:
        print("Shutting down...")
        server.shutdown()
        for cam in cams:
            cam.stop()
        driver.close()
        print("Done (the arm keeps its last position, PWM still active).")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live camera feed + terminal control of the first 2 joints, for positioning the cameras."
    )
    parser.add_argument("--file", default=DEFAULT_CALIBRATION_FILE, help="JSON calibration file")
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT, help=f"HTTP port for the MJPEG stream (default {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--layout",
        choices=LAYOUT_CHOICES,
        default=DEFAULT_LAYOUT,
        help=f"Layout of the two images (default {DEFAULT_LAYOUT})",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED_PERCENT,
        help=(
            f"Movement speed as %% of the servo's assumed max speed "
            f"({MIN_SPEED_PERCENT:.0f}-{MAX_SPEED_PERCENT:.0f}, default {DEFAULT_SPEED_PERCENT:.0f})"
        ),
    )
    args = parser.parse_args()

    if not MIN_SPEED_PERCENT <= args.speed <= MAX_SPEED_PERCENT:
        parser.error(f"--speed must be between {MIN_SPEED_PERCENT:.0f} and {MAX_SPEED_PERCENT:.0f}")

    data = load_all(args.file)
    calibs = {}
    for channel in CHANNELS:
        raw = data.get("servos", {}).get(str(channel))
        if raw is None:
            sys.exit(
                f"Channel {channel} not calibrated in {args.file} — "
                f"run calibrate_servo.py --channel {channel} first."
            )
        calibs[channel] = ServoCalibration(**raw)
    frequency_hz = data.get("pwm_frequency_hz", DEFAULT_FREQUENCY_HZ)
    speed_deg_per_s = MAX_SERVO_SPEED_DEG_PER_S * (args.speed / 100.0)

    run(calibs, frequency_hz, speed_deg_per_s, DEFAULT_POSITIONS_FILE, args.port, args.layout)


if __name__ == "__main__":
    main()
