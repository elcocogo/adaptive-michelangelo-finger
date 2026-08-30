#!/usr/bin/env python3
"""Computes the rigid transform from camera 0's frame to the arm's own
reference frame, using two standalone ArUco markers taped flat on the
floor next to the arm's (non-flat, sometimes-occluded) base.

Arm frame convention: origin at the point on the floor directly below the
base_joint's rotation axis; X axis pointing in the direction the arm
faces when base_joint = 0 deg; Z axis vertical, up. Set base_joint to 0
deg (move_servo.py) before measuring where the markers are, since that's
what fixes which direction "X" actually is.

What each marker contributes:
  - Its *position*: you measure this by hand (ruler/tape) once it's taped
    down, as (x, y) in mm from the arm-frame origin along the X/Y axes
    above. Both markers sit on the floor, so z=0 for both by construction.
  - Its *orientation*: not measured by hand at all. A flat marker's own 4
    corners define a plane, and that plane's normal — computed from the
    corners once they're triangulated into 3D — points straight up out of
    the floor. That's the arm frame's Z axis, for free, regardless of
    which way the marker happens to be rotated on the floor.
Combined, two floor positions plus the (averaged, cross-checked) floor
normal are exactly enough to solve the full rigid transform (3 axes +
origin) with no ambiguity — see build_transform() below for the math.

Usage:
    python3 -m camera_calibration.calibrate_camera_to_arm \\
        --marker1-id 0 --marker1-x-mm 150 --marker1-y-mm 120 \\
        --marker2-id 1 --marker2-x-mm 150 --marker2-y-mm -120
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from camera_calibration.aruco_markers import build_marker_detector, detect_markers
from camera_calibration.arm_frame_data import ArmFrameCalibration, now_iso, save
from camera_calibration.live_view import CameraStream
from camera_calibration.stereo_data import DEFAULT_STEREO_CALIBRATION_FILE, load as load_stereo
from camera_calibration.triangulation import triangulate_points

DEFAULT_SAMPLES = 20
SAMPLE_INTERVAL_S = 0.1
WARMUP_S = 1.5

# Same resolution as capture_stereo_images.py — live_view.py's default
# 640x480 (meant for physically positioning the cameras, not detection) is
# too low-res to reliably decode a 25.6mm ArUco tag's bit pattern once it's
# more than a short distance away.
CAPTURE_FRAME_SIZE = (1536, 864)


@dataclass
class MarkerSpec:
    marker_id: int
    x_mm: float
    y_mm: float


@dataclass
class DetectionCounts:
    cam0_only: int = 0
    cam1_only: int = 0
    both: int = 0
    neither: int = 0


def collect_marker_corners(
    cams: Tuple[CameraStream, CameraStream], marker_ids: List[int], detector, num_samples: int
) -> Tuple[Dict[int, List[np.ndarray]], Dict[int, DetectionCounts]]:
    """Repeatedly detects the given marker ids in both cameras and
    triangulates each sighting, returning {marker_id: [corners_3d, ...]}
    — a list of independent 3D-corner estimates per marker, to be
    averaged by the caller. Averaging many static sightings cancels out
    per-frame detection jitter that a single snapshot wouldn't.

    Also returns per-marker, per-camera detection counts, so a failure
    can be diagnosed (e.g. "cam1 never saw marker 1" points at that
    specific tag/camera, instead of a single opaque "not enough" error).
    """
    stereo_calib = load_stereo(DEFAULT_STEREO_CALIBRATION_FILE)
    by_marker: Dict[int, List[np.ndarray]] = {mid: [] for mid in marker_ids}
    counts: Dict[int, DetectionCounts] = {mid: DetectionCounts() for mid in marker_ids}

    for i in range(num_samples):
        frame0, frame1 = cams[0].latest(), cams[1].latest()
        if frame0 is None or frame1 is None:
            continue
        bgr0 = cv2.cvtColor(frame0, cv2.COLOR_RGB2BGR)
        bgr1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)
        found0 = detect_markers(bgr0, detector)
        found1 = detect_markers(bgr1, detector)

        for marker_id in marker_ids:
            in0, in1 = marker_id in found0, marker_id in found1
            if in0 and in1:
                counts[marker_id].both += 1
                corners_3d = triangulate_points(found0[marker_id], found1[marker_id], stereo_calib)
                by_marker[marker_id].append(corners_3d)
            elif in0:
                counts[marker_id].cam0_only += 1
            elif in1:
                counts[marker_id].cam1_only += 1
            else:
                counts[marker_id].neither += 1

        time.sleep(SAMPLE_INTERVAL_S)

    return by_marker, counts


def marker_center_and_normal(corners_3d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """From one flat marker's 4 triangulated corners (clockwise, as
    detected), returns (center, unit_normal). The normal is oriented to
    point back towards camera 0 (the triangulation's own origin) — the
    only sensible choice, since a marker's *decoded* face is by
    definition the one facing whichever camera read it.
    """
    center = corners_3d.mean(axis=0)
    normal = np.cross(corners_3d[1] - corners_3d[0], corners_3d[3] - corners_3d[0])
    normal = normal / np.linalg.norm(normal)
    toward_camera = -center  # camera 0 sits at the origin of its own frame
    if np.dot(normal, toward_camera) < 0:
        normal = -normal
    return center, normal


def build_transform(
    center1: np.ndarray, normal1: np.ndarray, spec1: MarkerSpec,
    center2: np.ndarray, normal2: np.ndarray, spec2: MarkerSpec,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Solves for (R, T) such that arm_point = R @ cam_point + T, from two
    known floor positions and the (averaged) floor normal.

    The construction: build the same orthonormal basis twice — once from
    quantities we know in arm-frame coordinates (the measured
    displacement between the two markers, and "up"), once from the
    matching quantities in camera-frame coordinates (the triangulated
    displacement, and the detected floor normal). A rotation matrix maps
    one orthonormal basis to another just by matching corresponding
    columns, which is what M_cam @ M_arm.T does below.
    """
    p1_arm = np.array([spec1.x_mm, spec1.y_mm, 0.0]) / 1000.0
    p2_arm = np.array([spec2.x_mm, spec2.y_mm, 0.0]) / 1000.0

    normal_agreement_deg = float(np.degrees(np.arccos(np.clip(np.dot(normal1, normal2), -1.0, 1.0))))
    z_cam = normal1 + normal2
    z_cam = z_cam / np.linalg.norm(z_cam)
    z_arm = np.array([0.0, 0.0, 1.0])

    d_cam = center2 - center1
    d_arm = p2_arm - p1_arm
    ed_cam = d_cam / np.linalg.norm(d_cam)
    ed_arm = d_arm / np.linalg.norm(d_arm)

    eperp_cam = np.cross(z_cam, ed_cam)
    eperp_arm = np.cross(z_arm, ed_arm)

    m_cam = np.column_stack([ed_cam, eperp_cam, z_cam])
    m_arm = np.column_stack([ed_arm, eperp_arm, z_arm])
    r_cam_from_arm = m_cam @ m_arm.T
    r_arm_from_cam = r_cam_from_arm.T

    origin_cam = 0.5 * (
        (center1 - r_cam_from_arm @ p1_arm) + (center2 - r_cam_from_arm @ p2_arm)
    )
    t_arm_from_cam = -r_arm_from_cam @ origin_cam

    return r_arm_from_cam, t_arm_from_cam, normal_agreement_deg


def run(spec1: MarkerSpec, spec2: MarkerSpec, num_samples: int) -> None:
    print("Starting both cameras...")
    cams = (CameraStream(0, frame_size=CAPTURE_FRAME_SIZE), CameraStream(1, frame_size=CAPTURE_FRAME_SIZE))
    for cam in cams:
        cam.start()
    time.sleep(WARMUP_S)

    detector = build_marker_detector()
    print(f"Collecting up to {num_samples} samples of markers {spec1.marker_id} and {spec2.marker_id}...")
    try:
        by_marker, counts = collect_marker_corners(cams, [spec1.marker_id, spec2.marker_id], detector, num_samples)
    finally:
        for cam in cams:
            cam.stop()

    n1, n2 = len(by_marker[spec1.marker_id]), len(by_marker[spec2.marker_id])
    print(f"Usable sightings (seen by both cameras at once): marker {spec1.marker_id}={n1}, marker {spec2.marker_id}={n2} (of {num_samples})")
    for spec in (spec1, spec2):
        c = counts[spec.marker_id]
        print(
            f"  marker {spec.marker_id}: both={c.both} cam0-only={c.cam0_only} "
            f"cam1-only={c.cam1_only} neither={c.neither}"
        )
    if n1 < 5 or n2 < 5:
        sys.exit(
            "Not enough sightings of both markers in both cameras — see the per-camera breakdown above: "
            "'cam0-only'/'cam1-only' means that marker is detected by one camera but not the other (check "
            "focus/angle/distance for the camera stuck at 0), 'neither' on both markers in most samples "
            "usually means the tags are too small/far for this resolution, poorly lit, or not flat."
        )

    corners1 = np.mean(by_marker[spec1.marker_id], axis=0)
    corners2 = np.mean(by_marker[spec2.marker_id], axis=0)
    center1, normal1 = marker_center_and_normal(corners1)
    center2, normal2 = marker_center_and_normal(corners2)

    R, T, normal_agreement_deg = build_transform(center1, normal1, spec1, center2, normal2, spec2)

    measured_distance_mm = float(np.linalg.norm(
        np.array([spec2.x_mm, spec2.y_mm]) - np.array([spec1.x_mm, spec1.y_mm])
    ))
    triangulated_distance_mm = float(np.linalg.norm(center2 - center1) * 1000.0)
    distance_error_pct = 100.0 * abs(triangulated_distance_mm - measured_distance_mm) / measured_distance_mm

    print(f"\nFloor-normal agreement between the two markers: {normal_agreement_deg:.2f} deg (expect a couple degrees or less)")
    print(f"Measured distance between markers:     {measured_distance_mm:.1f}mm")
    print(f"Triangulated distance between markers: {triangulated_distance_mm:.1f}mm (error: {distance_error_pct:.1f}%)")
    if normal_agreement_deg > 5.0 or distance_error_pct > 5.0:
        print(
            "  Warning: one of these checks is off by more than expected — double-check the markers "
            "are flat, the measured positions, and that the stereo calibration is still valid."
        )

    origin_in_cam0 = -R.T @ T
    print(f"Arm origin, expressed in camera 0's frame: {origin_in_cam0 * 1000.0} mm — sanity-check this looks about right.")

    calib = ArmFrameCalibration(
        R=R.tolist(),
        T=T.tolist(),
        marker_ids=[spec1.marker_id, spec2.marker_id],
        measured_distance_mm=measured_distance_mm,
        triangulated_distance_mm=triangulated_distance_mm,
        normal_agreement_deg=normal_agreement_deg,
        calibrated_at=now_iso(),
    )
    save(calib)
    print("\nSaved to calibration_data/camera_to_arm.json.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute the camera-to-arm frame transform from two floor-mounted ArUco markers."
    )
    parser.add_argument("--marker1-id", type=int, required=True)
    parser.add_argument("--marker1-x-mm", type=float, required=True, help="Marker 1 position along the arm frame's X axis (mm)")
    parser.add_argument("--marker1-y-mm", type=float, required=True, help="Marker 1 position along the arm frame's Y axis (mm)")
    parser.add_argument("--marker2-id", type=int, required=True)
    parser.add_argument("--marker2-x-mm", type=float, required=True, help="Marker 2 position along the arm frame's X axis (mm)")
    parser.add_argument("--marker2-y-mm", type=float, required=True, help="Marker 2 position along the arm frame's Y axis (mm)")
    parser.add_argument(
        "--samples", type=int, default=DEFAULT_SAMPLES, help=f"Number of frames to average over (default {DEFAULT_SAMPLES})"
    )
    args = parser.parse_args()

    spec1 = MarkerSpec(args.marker1_id, args.marker1_x_mm, args.marker1_y_mm)
    spec2 = MarkerSpec(args.marker2_id, args.marker2_x_mm, args.marker2_y_mm)
    run(spec1, spec2, args.samples)


if __name__ == "__main__":
    main()
