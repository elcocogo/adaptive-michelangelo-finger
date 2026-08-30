#!/usr/bin/env python3
"""Computes the stereo calibration from the image pairs captured by
capture_stereo_images.py: each camera's intrinsics (focal length, optical
center, lens distortion) and the rigid 3D transform between the two
cameras (rotation + translation). This is the last piece needed before
any 3D reconstruction is possible: intrinsics let you undo a camera's own
lens distortion and know its field of view; the cam0<->cam1 transform is
what turns "this pixel in image 0 and that pixel in image 1" into an
actual 3D point via triangulation, in the next project phase.

Since this is a first pass at OpenCV, this file is commented more heavily
than the rest of the codebase — treat the comments as the explanation,
the code as the implementation of that explanation.

Usage:
    python3 -m camera_calibration.calibrate_stereo
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
from PIL import Image

from camera_calibration.capture_stereo_images import CAPTURES_DIR, MIN_CHARUCO_CORNERS
from camera_calibration.charuco_board import MEASURED_SQUARE_LENGTH_MM, build_board, build_detector
from camera_calibration.stereo_data import StereoCalibration, now_iso, save


def find_pairs(cam0_dir: Path, cam1_dir: Path) -> List[Tuple[Path, Path]]:
    """Matches captures by filename stem (capture_stereo_images.py names
    both sides of a pair identically, e.g. cam0/003.jpg + cam1/003.jpg)."""
    stems0 = {p.stem for p in cam0_dir.glob("*.jpg")}
    stems1 = {p.stem for p in cam1_dir.glob("*.jpg")}
    common_set = stems0 & stems1
    common = sorted(common_set, key=int)
    missing = (stems0 | stems1) - common_set
    if missing:
        print(f"Warning: {len(missing)} file(s) with no match in the other folder, ignored: {sorted(missing)}")
    return [(cam0_dir / f"{stem}.jpg", cam1_dir / f"{stem}.jpg") for stem in common]


def detect(path: Path, detector) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Tuple[int, int]]:
    """Runs ChArUco detection on one saved image.

    Returns (charuco_corners, charuco_ids, (width, height)). Corners/ids
    are None if nothing usable was found (board fully out of frame, too
    blurry, etc).
    """
    # Images were saved as RGB (via PIL, same convention as the rest of
    # the codebase); OpenCV's aruco module expects the OpenCV-native BGR
    # order for its internal grayscale conversion.
    rgb = np.array(Image.open(path))
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    charuco_corners, charuco_ids, _marker_corners, _marker_ids = detector.detectBoard(bgr)
    height, width = bgr.shape[:2]
    return charuco_corners, charuco_ids, (width, height)


def corners_for_ids(all_ids: np.ndarray, all_corners: np.ndarray, wanted_ids: List[int]) -> np.ndarray:
    """Reorders a detection's corners to match an explicit list of ids.

    Needed for the stereo step: cv2.stereoCalibrate requires row i of the
    object-point array and row i of *each* camera's image-point array to
    all refer to the same physical corner. Two cameras looking at the same
    board from different angles won't always detect exactly the same set
    of corners (partial occlusion, a corner too close to the image edge in
    one view...), so we can only use the *intersection* of ids they both
    saw for a given pose, and both cameras' corner lists must be put in
    that same, explicit order before handing them to OpenCV.
    """
    id_to_corner = {int(i): c for i, c in zip(all_ids.flatten(), all_corners)}
    return np.array([id_to_corner[i] for i in wanted_ids], dtype=np.float32)


def build_point_correspondences(pairs: List[Tuple[Path, Path]], detector, board):
    """Turns the raw image pairs into the point arrays cv2.calibrateCamera
    and cv2.stereoCalibrate expect.

    Two separate sets are built:
      - objpoints0/imgpoints0 and objpoints1/imgpoints1: *each camera's
        own* detections, used to estimate that camera's intrinsics alone.
        A frame is useful here even if the other camera's view of it
        wasn't great, so we use everything above MIN_CHARUCO_CORNERS.
      - stereo_objpoints/stereo_imgpoints0/stereo_imgpoints1: only the
        corners *both* cameras detected in a given pair (see
        corners_for_ids above), used for the joint stereo step, which
        needs point-for-point correspondence between the two views.
    """
    objpoints0, imgpoints0 = [], []
    objpoints1, imgpoints1 = [], []
    stereo_objpoints, stereo_imgpoints0, stereo_imgpoints1 = [], [], []
    image_size = None

    for path0, path1 in pairs:
        corners0, ids0, size0 = detect(path0, detector)
        corners1, ids1, size1 = detect(path1, detector)
        image_size = image_size or size0
        n0 = 0 if ids0 is None else len(ids0)
        n1 = 0 if ids1 is None else len(ids1)

        if n0 < MIN_CHARUCO_CORNERS or n1 < MIN_CHARUCO_CORNERS:
            print(f"  {path0.stem}: cam0={n0} cam1={n1} corners -> skipped (minimum {MIN_CHARUCO_CORNERS})")
            continue

        # board.matchImagePoints translates "these detected corner ids"
        # into (3D point on the board, 2D pixel it was seen at) pairs —
        # the 3D side comes purely from the board's known geometry
        # (MEASURED_SQUARE_LENGTH_MM), not from anything in the image.
        op0, ip0 = board.matchImagePoints(corners0, ids0)
        objpoints0.append(op0)
        imgpoints0.append(ip0)
        op1, ip1 = board.matchImagePoints(corners1, ids1)
        objpoints1.append(op1)
        imgpoints1.append(ip1)

        common_ids = sorted(set(ids0.flatten().tolist()) & set(ids1.flatten().tolist()))
        if len(common_ids) < MIN_CHARUCO_CORNERS:
            print(f"  {path0.stem}: only {len(common_ids)} corners shared by both cameras -> excluded from the stereo step")
            continue

        c0 = corners_for_ids(ids0, corners0, common_ids).reshape(-1, 1, 2)
        c1 = corners_for_ids(ids1, corners1, common_ids).reshape(-1, 1, 2)
        common_ids_arr = np.array(common_ids, dtype=np.int32)
        stereo_objp, stereo_ip0 = board.matchImagePoints(c0, common_ids_arr)
        _, stereo_ip1 = board.matchImagePoints(c1, common_ids_arr)
        stereo_objpoints.append(stereo_objp)
        stereo_imgpoints0.append(stereo_ip0)
        stereo_imgpoints1.append(stereo_ip1)
        print(f"  {path0.stem}: cam0={n0} cam1={n1} corners, {len(common_ids)} shared -> kept")

    return (
        (objpoints0, imgpoints0),
        (objpoints1, imgpoints1),
        (stereo_objpoints, stereo_imgpoints0, stereo_imgpoints1),
        image_size,
    )


def describe_rms(rms: float) -> str:
    if rms < 0.5:
        return "excellent"
    if rms < 1.0:
        return "very good"
    if rms < 2.0:
        return "fine for an amateur setup"
    return "high — check the board's rigidity and the variety of captured poses"


def main() -> None:
    cam0_dir, cam1_dir = CAPTURES_DIR / "cam0", CAPTURES_DIR / "cam1"
    pairs = find_pairs(cam0_dir, cam1_dir)
    if len(pairs) < 6:
        sys.exit(
            f"Only {len(pairs)} pair(s) found in {CAPTURES_DIR}/ — "
            "capture at least fifteen or so pairs with capture_stereo_images.py first."
        )
    print(f"{len(pairs)} pair(s) found, detecting...")

    board = build_board()
    detector = build_detector()
    mono0, mono1, stereo, image_size = build_point_correspondences(pairs, detector, board)
    objpoints0, imgpoints0 = mono0
    objpoints1, imgpoints1 = mono1
    stereo_objpoints, stereo_imgpoints0, stereo_imgpoints1 = stereo

    if len(stereo_objpoints) < 6:
        sys.exit(
            f"Only {len(stereo_objpoints)} pair(s) usable for the stereo step "
            "(not enough shared corners) — capture poses where the board is fully visible to both cameras."
        )

    # --- Step 1: each camera's intrinsics, independently ---
    #
    # cv2.calibrateCamera solves a system of equations that explains all
    # detections at once (known 3D points -> observed pixels) with a
    # single set of parameters:
    #   - the camera matrix (3x3): focal length (fx, fy) and optical
    #     center (cx, cy), in pixels — the "pinhole" model that projects a
    #     3D point onto the sensor.
    #   - the distortion coefficients: correction for the deformations
    #     introduced by the lens (a stronger or weaker "fisheye" effect).
    # Each camera is calibrated separately here: these are physical
    # properties of THAT lens/sensor, independent of the other camera.
    print("\nCamera 0 intrinsic calibration...")
    rms_mono0, camera_matrix0, dist_coeffs0, _, _ = cv2.calibrateCamera(
        objpoints0, imgpoints0, image_size, None, None
    )
    print(f"  RMS reprojection error: {rms_mono0:.3f}px ({describe_rms(rms_mono0)})")

    print("Camera 1 intrinsic calibration...")
    rms_mono1, camera_matrix1, dist_coeffs1, _, _ = cv2.calibrateCamera(
        objpoints1, imgpoints1, image_size, None, None
    )
    print(f"  RMS reprojection error: {rms_mono1:.3f}px ({describe_rms(rms_mono1)})")

    # --- Step 2: rigid transform between the two cameras ---
    #
    # cv2.stereoCalibrate reuses the same known 3D points, but this time
    # only the ones seen by BOTH cameras at once for each pose
    # (stereo_objpoints/stereo_imgpoints0/1 built above). It looks for the
    # rotation R and translation T that, applied to camera 0's position,
    # give camera 1's position — exactly the information the 3D
    # triangulation will need later to combine the two views into a
    # position in space.
    #
    # CALIB_FIX_INTRINSIC tells the function to keep the camera matrices
    # and distortions as computed in step 1, and only solve for R and T —
    # more stable than re-optimizing everything at once, especially with a
    # modest number of captured poses.
    print("Stereo calibration (relative position of the 2 cameras)...")
    rms_stereo, camera_matrix0, dist_coeffs0, camera_matrix1, dist_coeffs1, R, T, _E, _F = cv2.stereoCalibrate(
        stereo_objpoints,
        stereo_imgpoints0,
        stereo_imgpoints1,
        camera_matrix0,
        dist_coeffs0,
        camera_matrix1,
        dist_coeffs1,
        image_size,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    baseline_mm = float(np.linalg.norm(T)) * 1000.0
    print(f"  RMS reprojection error: {rms_stereo:.3f}px ({describe_rms(rms_stereo)})")
    print(f"  Measured baseline between the 2 cameras: {baseline_mm:.1f}mm")

    calib = StereoCalibration(
        image_width=image_size[0],
        image_height=image_size[1],
        camera_matrix0=camera_matrix0.tolist(),
        dist_coeffs0=dist_coeffs0.flatten().tolist(),
        camera_matrix1=camera_matrix1.tolist(),
        dist_coeffs1=dist_coeffs1.flatten().tolist(),
        R=R.tolist(),
        T=T.flatten().tolist(),
        rms_mono0=float(rms_mono0),
        rms_mono1=float(rms_mono1),
        rms_stereo=float(rms_stereo),
        num_pairs_used=len(stereo_objpoints),
        square_length_mm=MEASURED_SQUARE_LENGTH_MM,
        calibrated_at=now_iso(),
    )
    save(calib)
    print(f"\nSaved to calibration_data/stereo_calibration.json ({len(stereo_objpoints)} pairs used).")


if __name__ == "__main__":
    main()
