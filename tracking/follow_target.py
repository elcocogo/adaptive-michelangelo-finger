#!/usr/bin/env python3
"""Live tracking loop: detects a target ArUco tag in both cameras,
triangulates its 3D position, converts it into the arm's frame, solves
inverse kinematics, and drives the arm to point at it — continuously.

Pipeline, each cycle: detect_markers (both cameras) -> triangulate_points
(camera 0's frame) -> apply the camera-to-arm transform (arm frame) ->
solve inverse kinematics -> move_pose (ramped, synchronized across the 4
joints). Everything here is gluing together pieces already built and
validated separately (stereo calibration, camera-to-arm calibration,
kinematics, servo control) — this script itself has very little logic
of its own.

The arm aims at the target but stops DEFAULT_STANDOFF_MM short of it, so
it doesn't actually touch whatever it's tracking. Two ways to do that,
selected with --point-gripper:
  - default (position only): apply_standoff pulls the *fingertip* back
    along the shoulder-target line, then inverse_kinematics_search picks
    whichever wrist angle keeps every joint within its calibrated bounds
    — plain inverse_kinematics alone can demand an elbow bend beyond
    basearm_joint's limit for some targets, even though the same point
    is reachable with a bent wrist.
  - --point-gripper: inverse_kinematics_pointing additionally pins the
    wrist angle so gripper_arm itself stays parallel to the line from
    the wrist through the target — the arm doesn't just get *near* the
    target, its last segment visibly points at it, still stopping
    standoff_mm short.
Adjust DEFAULT_STANDOFF_MM below, or pass --standoff-mm.

Only channels 0-3 (position) are driven; the gripper (channel 4) is left
wherever it was. The arm's own fingertip position is *not* visually
tracked — inverse_kinematics is trusted open-loop, since it's exact given
the commanded angles. (A later iteration may add a second ArUco tag on
the arm itself to visually verify/correct that, since the mechanics
aren't perfectly rigid — not done yet.)

rpi502 has no display, so — same as the other camera tools — this serves
a live preview over HTTP (MJPEG) with the target marker highlighted when
seen, instead of opening a window. Pass --record to also save that same
annotated view to an .mp4 file (recordings/ — the preview server only
draws frames while a browser is actually watching /stream, so recording
runs its own independent capture+annotate loop rather than piggybacking
on it).

Usage:
    python3 -m tracking.follow_target
    python3 -m tracking.follow_target --target-id 0 --speed 40
    python3 -m tracking.follow_target --layout stacked
    python3 -m tracking.follow_target --standoff-mm 50
    python3 -m tracking.follow_target --point-gripper
    python3 -m tracking.follow_target --record

Commands (in the terminal, while the loop runs):
    q   quits (stops the loop, returns the arm to its home position —
        vertical, all angles 0 deg — then leaves PWM active)
"""

from __future__ import annotations

import argparse
import io
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
from PIL import Image

from camera_calibration.aruco_markers import build_marker_detector, detect_markers
from camera_calibration.arm_frame_data import DEFAULT_ARM_FRAME_FILE, load as load_arm_frame
from camera_calibration.live_view import (
    DEFAULT_LAYOUT,
    LAYOUT_CHOICES,
    LAYOUT_STACKED,
    CameraStream,
)
from camera_calibration.stereo_data import DEFAULT_STEREO_CALIBRATION_FILE, load as load_stereo
from camera_calibration.triangulation import triangulate_points
from kinematics.arm_kinematics import apply_standoff, inverse_kinematics_pointing, inverse_kinematics_search
from servo_calibration.arm_show import (
    CH_BASE_ARM,
    CH_BASE_SPIN,
    CH_GRIPPER_ARM,
    CH_MID_ARM,
    load_calibrations,
    move_pose,
)
from servo_calibration.go_home import go_home
from servo_calibration.calibration import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_POSITIONS_FILE,
    MAX_SERVO_SPEED_DEG_PER_S,
    MAX_SPEED_PERCENT,
    MIN_SPEED_PERCENT,
    load_last_angle,
)
from servo_calibration.pca9685_driver import Pca9685Driver

POSITION_CHANNELS = (CH_BASE_SPIN, CH_BASE_ARM, CH_MID_ARM, CH_GRIPPER_ARM)

DEFAULT_TARGET_ID = 0
DEFAULT_PORT = 8100
DEFAULT_SPEED_PERCENT = 50.0

# How far short of the target the arm stops, measured from the target
# back towards the shoulder (see kinematics.arm_kinematics.apply_standoff)
# — so the arm points at the target without actually touching it.
DEFAULT_STANDOFF_MM = 135.0
MIN_LOOP_INTERVAL_S = 0.3  # floor between detection/IK cycles, not a hard cadence
CAPTURE_FRAME_SIZE = (1536, 864)  # same as capture_stereo_images.py / calibrate_camera_to_arm.py
STREAM_FPS = 10
JPEG_QUALITY = 85

RECORDINGS_DIR = Path(__file__).resolve().parent / "recordings"
RECORDING_FPS = STREAM_FPS  # same cadence as the live preview it mirrors
RECORDING_FOURCC = "mp4v"

HELP_TEXT = __doc__.split("Commands", 1)[1]


def annotate(frame: np.ndarray, detector, target_id: int) -> np.ndarray:
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    found = detect_markers(bgr, detector)
    if found:
        ids = np.array(list(found.keys()), dtype=np.int32).reshape(-1, 1)
        corners = [found[i].reshape(1, 4, 2).astype(np.float32) for i in found]
        cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
        if target_id in found:
            center = found[target_id].mean(axis=0).astype(int)
            cv2.circle(bgr, tuple(center), 10, (0, 0, 255), 2)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def combined_frame_size(layout: str = DEFAULT_LAYOUT) -> Tuple[int, int]:
    """The (width, height) of the combined annotated frame for a given
    layout, computable up front from CAPTURE_FRAME_SIZE alone — used to
    open the video writer before any real frame has been captured."""
    w, h = CAPTURE_FRAME_SIZE
    return (w, h * 2) if layout == LAYOUT_STACKED else (w * 2, h)


def build_combined_frame(
    cam0: CameraStream, cam1: CameraStream, detector, target_id: int, layout: str = DEFAULT_LAYOUT
) -> Optional[Image.Image]:
    """Builds the annotated side-by-side/stacked frame — the same image
    the MJPEG preview serves and --record saves to disk."""
    frame0, frame1 = cam0.latest(), cam1.latest()
    if frame0 is None or frame1 is None:
        return None
    img0 = Image.fromarray(annotate(frame0, detector, target_id))
    img1 = Image.fromarray(annotate(frame1, detector, target_id))
    if layout == LAYOUT_STACKED:
        combined = Image.new("RGB", (max(img0.width, img1.width), img0.height + img1.height))
        combined.paste(img0, (0, 0))
        combined.paste(img1, (0, img0.height))
    else:
        combined = Image.new("RGB", (img0.width + img1.width, max(img0.height, img1.height)))
        combined.paste(img0, (0, 0))
        combined.paste(img1, (img0.width, 0))
    return combined


def combined_preview_jpeg(
    cam0: CameraStream, cam1: CameraStream, detector, target_id: int, layout: str = DEFAULT_LAYOUT
) -> Optional[bytes]:
    combined = build_combined_frame(cam0, cam1, detector, target_id, layout)
    if combined is None:
        return None
    buf = io.BytesIO()
    combined.save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def recording_loop(
    cams,
    detector,
    target_id: int,
    layout: str,
    writer: cv2.VideoWriter,
    stop_event: threading.Event,
) -> None:
    """Independently builds and saves the same annotated view the MJPEG
    preview shows, at RECORDING_FPS, regardless of whether anyone is
    actually watching /stream right now."""
    interval_s = 1.0 / RECORDING_FPS
    while not stop_event.is_set():
        loop_start = time.monotonic()
        combined = build_combined_frame(cams[0], cams[1], detector, target_id, layout)
        if combined is not None:
            writer.write(cv2.cvtColor(np.array(combined), cv2.COLOR_RGB2BGR))
        elapsed = time.monotonic() - loop_start
        if elapsed < interval_s:
            time.sleep(interval_s - elapsed)


INDEX_HTML = b"""<!doctype html>
<html><head><title>Michelangelo - tracking</title>
<style>
  body { margin: 0; background: #111; }
  img { display: block; width: 100%; height: auto; }
</style>
</head><body>
<img src="/stream" alt="tracking preview">
</body></html>
"""


def make_handler(cam0: CameraStream, cam1: CameraStream, detector, target_id: int, layout: str = DEFAULT_LAYOUT):
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
                    jpeg = combined_preview_jpeg(cam0, cam1, detector, target_id, layout)
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
            pass  # keep the terminal free for tracking feedback

    return MjpegHandler


def tracking_loop(
    cams,
    detector,
    driver: Pca9685Driver,
    calibs,
    stereo_calib,
    arm_R: np.ndarray,
    arm_T: np.ndarray,
    target_id: int,
    speed_deg_per_s: float,
    standoff_mm: float,
    point_gripper: bool,
    positions_file,
    stop_event: threading.Event,
) -> None:
    current: Dict[int, Optional[float]] = {
        channel: load_last_angle(channel, positions_file) for channel in POSITION_CHANNELS
    }

    while not stop_event.is_set():
        loop_start = time.monotonic()
        frame0, frame1 = cams[0].latest(), cams[1].latest()

        if frame0 is not None and frame1 is not None:
            bgr0 = cv2.cvtColor(frame0, cv2.COLOR_RGB2BGR)
            bgr1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)
            found0 = detect_markers(bgr0, detector)
            found1 = detect_markers(bgr1, detector)

            if target_id in found0 and target_id in found1:
                center0 = found0[target_id].mean(axis=0, keepdims=True)
                center1 = found1[target_id].mean(axis=0, keepdims=True)
                point_cam = triangulate_points(center0, center1, stereo_calib)[0]
                point_arm_mm = (arm_R @ point_cam + arm_T) * 1000.0
                theta1_bounds = (calibs[CH_BASE_ARM].angle_min_deg, calibs[CH_BASE_ARM].angle_max_deg)
                theta2_bounds = (calibs[CH_MID_ARM].angle_min_deg, calibs[CH_MID_ARM].angle_max_deg)
                theta3_bounds = (calibs[CH_GRIPPER_ARM].angle_min_deg, calibs[CH_GRIPPER_ARM].angle_max_deg)

                try:
                    if point_gripper:
                        # inverse_kinematics_pointing works out the standoff
                        # itself (from the *wrist*, so gripper_arm ends up
                        # parallel to the wrist-target line) — it wants the
                        # raw target, not one pre-adjusted by apply_standoff.
                        theta0, theta1, theta2, theta3 = inverse_kinematics_pointing(
                            tuple(point_arm_mm), standoff_mm, theta1_bounds, theta2_bounds, theta3_bounds
                        )
                    else:
                        aim_point_mm = apply_standoff(tuple(point_arm_mm), standoff_mm)
                        theta0, theta1, theta2, theta3 = inverse_kinematics_search(
                            *aim_point_mm,
                            theta1_bounds=theta1_bounds,
                            theta2_bounds=theta2_bounds,
                            theta3_bounds=theta3_bounds,
                        )
                except ValueError as e:
                    print(f"  Target out of reach: {e}")
                else:
                    print(
                        f"  Target (arm frame): x={point_arm_mm[0]:.1f} y={point_arm_mm[1]:.1f} "
                        f"z={point_arm_mm[2]:.1f}mm, {standoff_mm:.0f}mm standoff "
                        f"-> angles {theta0:.1f}/{theta1:.1f}/{theta2:.1f}/{theta3:.1f} deg"
                    )
                    pose = {
                        CH_BASE_SPIN: theta0,
                        CH_BASE_ARM: theta1,
                        CH_MID_ARM: theta2,
                        CH_GRIPPER_ARM: theta3,
                    }
                    try:
                        move_pose(driver, calibs, pose, current, speed_deg_per_s, positions_file)
                    except ValueError as e:
                        print(f"  Cannot move there: {e}")
            else:
                print("  Target not visible in both cameras.")
        else:
            print("  Waiting for camera frames...")

        elapsed = time.monotonic() - loop_start
        if elapsed < MIN_LOOP_INTERVAL_S:
            time.sleep(MIN_LOOP_INTERVAL_S - elapsed)


def run(
    target_id: int,
    speed_deg_per_s: float,
    port: int,
    layout: str = DEFAULT_LAYOUT,
    standoff_mm: float = DEFAULT_STANDOFF_MM,
    point_gripper: bool = False,
    record: bool = False,
) -> None:
    print("Loading calibrations...")
    stereo_calib = load_stereo(DEFAULT_STEREO_CALIBRATION_FILE)
    arm_frame = load_arm_frame(DEFAULT_ARM_FRAME_FILE)
    arm_R = np.array(arm_frame.R)
    arm_T = np.array(arm_frame.T)
    calibs, frequency_hz = load_calibrations(DEFAULT_CALIBRATION_FILE)

    print("Starting both cameras...")
    cams = (CameraStream(0, frame_size=CAPTURE_FRAME_SIZE), CameraStream(1, frame_size=CAPTURE_FRAME_SIZE))
    for cam in cams:
        cam.start()

    detector = build_marker_detector()
    driver = Pca9685Driver(frequency_hz=frequency_hz)

    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(cams[0], cams[1], detector, target_id, layout))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"Preview at http://<pi-ip>:{port}/ (target tag highlighted in red when seen)")

    video_writer = None
    recording_thread = None
    if record:
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        video_path = RECORDINGS_DIR / f"tracking_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FOURCC)
        video_writer = cv2.VideoWriter(str(video_path), fourcc, RECORDING_FPS, combined_frame_size(layout))
        if not video_writer.isOpened():
            sys.exit(f"Could not open {video_path} for recording (codec {RECORDING_FOURCC} unsupported?).")
        print(f"Recording to {video_path}")

    stop_event = threading.Event()
    loop_thread = threading.Thread(
        target=tracking_loop,
        args=(cams, detector, driver, calibs, stereo_calib, arm_R, arm_T, target_id, speed_deg_per_s, standoff_mm, point_gripper, DEFAULT_POSITIONS_FILE, stop_event),
        daemon=True,
    )
    loop_thread.start()

    if record:
        recording_thread = threading.Thread(
            target=recording_loop,
            args=(cams, detector, target_id, layout, video_writer, stop_event),
            daemon=True,
        )
        recording_thread.start()

    print(HELP_TEXT)
    try:
        while True:
            try:
                raw = input("> ").strip()
            except KeyboardInterrupt:
                print()
                break
            if raw == "q":
                break
    finally:
        print("Shutting down...")
        stop_event.set()
        loop_thread.join(timeout=2.0)
        if recording_thread is not None:
            recording_thread.join(timeout=2.0)
        if video_writer is not None:
            video_writer.release()
        server.shutdown()
        for cam in cams:
            cam.stop()
        print("Returning to home position...")
        go_home(driver, calibs, speed_deg_per_s, DEFAULT_POSITIONS_FILE)
        driver.close()
        print("Done (arm back at home position, PWM still active).")


def main() -> None:
    parser = argparse.ArgumentParser(description="Track an ArUco tag and point the arm at it.")
    parser.add_argument("--target-id", type=int, default=DEFAULT_TARGET_ID, help=f"ArUco tag ID to track (default {DEFAULT_TARGET_ID})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP port for the preview (default {DEFAULT_PORT})")
    parser.add_argument(
        "--layout",
        choices=LAYOUT_CHOICES,
        default=DEFAULT_LAYOUT,
        help=f"Layout of the two images in the preview (default {DEFAULT_LAYOUT})",
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
    parser.add_argument(
        "--standoff-mm",
        type=float,
        default=DEFAULT_STANDOFF_MM,
        help=f"How far short of the target the arm stops, in mm (default {DEFAULT_STANDOFF_MM:.0f})",
    )
    parser.add_argument(
        "--point-gripper",
        action="store_true",
        help="Also keep gripper_arm parallel to the wrist-target line, instead of just reaching a nearby position",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help=f"Save the annotated camera view to an .mp4 in {RECORDINGS_DIR.name}/",
    )
    args = parser.parse_args()

    if not MIN_SPEED_PERCENT <= args.speed <= MAX_SPEED_PERCENT:
        parser.error(f"--speed must be between {MIN_SPEED_PERCENT:.0f} and {MAX_SPEED_PERCENT:.0f}")
    if args.standoff_mm < 0:
        parser.error("--standoff-mm must be >= 0")

    speed_deg_per_s = MAX_SERVO_SPEED_DEG_PER_S * (args.speed / 100.0)
    run(args.target_id, speed_deg_per_s, args.port, args.layout, args.standoff_mm, args.point_gripper, args.record)


if __name__ == "__main__":
    main()
