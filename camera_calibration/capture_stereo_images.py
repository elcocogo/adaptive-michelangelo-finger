#!/usr/bin/env python3
"""Captures synchronized image pairs of the ChArUco board from both
cameras, for the stereo calibration computed by calibrate_stereo.py.

Why this needs its own tool rather than reusing live_view.py: stereo
calibration needs many (~15-20) *simultaneous* snapshots of the same
board pose from both cameras, each one checked for a good detection
before it's worth keeping — a live *video* feed doesn't give you that,
you need a "capture this exact instant, from both sensors, and tell me
if it was any good" action.

How a capture is judged "good enough to keep": on each capture, the
ChArUco detector runs on both images independently. If either one finds
fewer than MIN_CHARUCO_CORNERS interpolated chessboard corners, the pair
is discarded (not saved) — a handful of corners isn't enough to properly
constrain the calibration math for that pose, so keeping it would only
add noise. This is exactly the same detector calibrate_stereo.py will use
later, so "captured successfully" here already means "will be usable".

rpi502 has no display, so — same as live_view.py — this serves a live
preview over HTTP (MJPEG) instead of opening a window. The preview draws
the detected markers/corners live, so you can *see* the board being
recognized before you capture, instead of finding out after the fact.

Usage:
    python3 -m camera_calibration.capture_stereo_images
    python3 -m camera_calibration.capture_stereo_images --port 8100

Commands (in the terminal, while the preview is running):
    c   captures a pair (saved only if enough corners are detected
        in BOTH cameras)
    q   quits

Tips for a good calibration:
    - Aim for 15-20 valid pairs minimum.
    - Vary the board's distance, tilt angle, and position in the frame on
      each capture (including near the image's edges and corners, not
      just the center) — that's what lets the computation properly
      estimate the lens distortion.
    - The board must stay still during the capture (motion blur = bad
      detection).
"""

from __future__ import annotations

import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import cv2
from PIL import Image

from camera_calibration.charuco_board import build_detector
from camera_calibration.live_view import (
    DEFAULT_LAYOUT,
    LAYOUT_CHOICES,
    LAYOUT_STACKED,
    CameraStream,
)

CAPTURE_FRAME_SIZE = (1536, 864)  # higher-res than live_view.py's positioning preview
DEFAULT_PORT = 8100
STREAM_FPS = 10  # ChArUco detection on every frame is heavier than a plain passthrough
JPEG_QUALITY = 85
MIN_CHARUCO_CORNERS = 8  # below this, a pose barely constrains the calibration math

CAPTURES_DIR = Path(__file__).resolve().parent / "calibration_captures"

HELP_TEXT = __doc__.split("Commands", 1)[1]


def detect_and_annotate(frame, detector) -> tuple:
    """Runs ChArUco detection on one frame and returns (annotated_frame, num_corners).

    The annotated copy (with markers/corners drawn) is only for the live
    preview — capture() re-detects on the clean frame so nothing saved to
    disk has drawing artifacts baked into it.
    """
    # OpenCV's drawing functions expect BGR memory layout; our frames are
    # RGB (see live_view.CameraStream), so convert for this preview-only copy.
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(bgr)
    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(bgr, marker_corners, marker_ids)
    if charuco_ids is not None and len(charuco_ids) > 0:
        # detectBoard returns corners as a plain (N, 2) array, but this
        # drawing function expects OpenCV's usual "point vector" shape
        # (N, 1, 2) — without the reshape it miscounts the elements
        # internally and raises a size-mismatch assertion against charuco_ids.
        cv2.aruco.drawDetectedCornersCharuco(bgr, charuco_corners.reshape(-1, 1, 2), charuco_ids)
    annotated_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    num_corners = 0 if charuco_ids is None else len(charuco_ids)
    return annotated_rgb, num_corners


def combined_preview_jpeg(
    cam0: CameraStream, cam1: CameraStream, detector, layout: str = DEFAULT_LAYOUT
) -> Optional[bytes]:
    frame0, frame1 = cam0.latest(), cam1.latest()
    if frame0 is None or frame1 is None:
        return None
    annotated0, _ = detect_and_annotate(frame0, detector)
    annotated1, _ = detect_and_annotate(frame1, detector)
    img0, img1 = Image.fromarray(annotated0), Image.fromarray(annotated1)
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
<html><head><title>Michelangelo - stereo capture</title>
<style>
  body { margin: 0; background: #111; }
  img { display: block; width: 100%; height: auto; }
</style>
</head><body>
<img src="/stream" alt="charuco detection preview">
</body></html>
"""


def make_handler(cam0: CameraStream, cam1: CameraStream, detector, layout: str = DEFAULT_LAYOUT):
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
                    jpeg = combined_preview_jpeg(cam0, cam1, detector, layout)
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
            pass  # keep the terminal free for capture feedback

    return MjpegHandler


def next_capture_index(cam_dir: Path) -> int:
    """Resumes numbering after whatever was already captured in a previous run."""
    existing = [int(p.stem) for p in cam_dir.glob("*.jpg") if p.stem.isdigit()]
    return max(existing, default=-1) + 1


def try_capture(cam0: CameraStream, cam1: CameraStream, detector, cam0_dir: Path, cam1_dir: Path) -> bool:
    frame0, frame1 = cam0.latest(), cam1.latest()
    if frame0 is None or frame1 is None:
        print("  No image available yet, try again in a moment.")
        return False

    # Re-detect on the *clean* frames (not the annotated preview copies) —
    # the counts here are what actually decides whether this pair is kept.
    _, corners0 = detect_and_annotate(frame0, detector)
    _, corners1 = detect_and_annotate(frame1, detector)
    print(f"  Corners detected: cam0={corners0}, cam1={corners1} (minimum required: {MIN_CHARUCO_CORNERS})")

    if corners0 < MIN_CHARUCO_CORNERS or corners1 < MIN_CHARUCO_CORNERS:
        print("  -> Not enough corners detected on both sides, pair discarded. Try again.")
        return False

    index = next_capture_index(cam0_dir)
    Image.fromarray(frame0).save(cam0_dir / f"{index:03d}.jpg", quality=95)
    Image.fromarray(frame1).save(cam1_dir / f"{index:03d}.jpg", quality=95)
    print(f"  -> Pair {index:03d} saved.")
    return True


def run(port: int, layout: str = DEFAULT_LAYOUT) -> None:
    cam0_dir = CAPTURES_DIR / "cam0"
    cam1_dir = CAPTURES_DIR / "cam1"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)
    saved_count = next_capture_index(cam0_dir)  # count of pairs already on disk

    print("Starting both cameras...")
    cams = [CameraStream(0, frame_size=CAPTURE_FRAME_SIZE), CameraStream(1, frame_size=CAPTURE_FRAME_SIZE)]
    for cam in cams:
        cam.start()

    detector = build_detector()

    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(cams[0], cams[1], detector, layout))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"Preview (with detection overlay) at http://<pi-ip>:{port}/")
    print(HELP_TEXT)
    print(f"{saved_count} pair(s) already present in {CAPTURES_DIR}/")

    try:
        while True:
            try:
                raw = input("> ").strip()
            except KeyboardInterrupt:
                print()
                break
            if raw == "q":
                break
            if raw == "c":
                if try_capture(cams[0], cams[1], detector, cam0_dir, cam1_dir):
                    saved_count += 1
                    print(f"  Total: {saved_count} valid pair(s).")
            elif raw != "":
                print("  Unrecognized input (expected c or q).")
    finally:
        print("Shutting down...")
        server.shutdown()
        for cam in cams:
            cam.stop()
        print(f"Done. {saved_count} pair(s) in {CAPTURES_DIR}/.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture image pairs for stereo calibration.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP port for the preview (default {DEFAULT_PORT})")
    parser.add_argument(
        "--layout",
        choices=LAYOUT_CHOICES,
        default=DEFAULT_LAYOUT,
        help=f"Layout of the two images in the preview (default {DEFAULT_LAYOUT})",
    )
    args = parser.parse_args()
    run(args.port, args.layout)


if __name__ == "__main__":
    main()
