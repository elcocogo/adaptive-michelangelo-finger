"""Turns matching 2D pixel detections from both cameras into 3D points, in
camera 0's coordinate frame, using the calibration from calibrate_stereo.py.

This is the core building block both calibrate_camera_to_arm.py and the
final tracking loop rely on: "this pixel in image 0 and that pixel in
image 1" -> "this point in space".

How it works: cv2.triangulatePoints needs each camera's projection matrix
(intrinsics + pose) and points expressed in *normalized, undistorted*
image coordinates — not raw pixels. cv2.undistortPoints does both jobs at
once (removes lens distortion, then converts from pixels to the
normalized coordinates a pinhole camera model expects) when called
without a target camera matrix. Camera 0 is the reference frame, so its
projection matrix is just [I | 0]; camera 1's is [R | T] from the stereo
calibration, since that's exactly camera 1's pose relative to camera 0.
"""

from __future__ import annotations

import cv2
import numpy as np

from camera_calibration.stereo_data import StereoCalibration


def undistort_normalized(points_px: np.ndarray, camera_matrix, dist_coeffs) -> np.ndarray:
    """Pixel coordinates -> normalized, undistorted coordinates (Nx2)."""
    points_px = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
    dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
    undistorted = cv2.undistortPoints(points_px, camera_matrix, dist_coeffs)
    return undistorted.reshape(-1, 2)


def triangulate_points(points0_px: np.ndarray, points1_px: np.ndarray, calib: StereoCalibration) -> np.ndarray:
    """Triangulates N point correspondences into 3D points in cam0's frame (meters).

    points0_px/points1_px: Nx2 pixel coordinates, points0_px[i] and
    points1_px[i] must be the same physical point seen by each camera.
    """
    norm0 = undistort_normalized(points0_px, calib.camera_matrix0, calib.dist_coeffs0)
    norm1 = undistort_normalized(points1_px, calib.camera_matrix1, calib.dist_coeffs1)

    R = np.asarray(calib.R, dtype=np.float64)
    T = np.asarray(calib.T, dtype=np.float64).reshape(3, 1)
    proj0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    proj1 = np.hstack([R, T])

    points_4d = cv2.triangulatePoints(proj0, proj1, norm0.T, norm1.T)
    points_3d = (points_4d[:3] / points_4d[3]).T
    return points_3d
