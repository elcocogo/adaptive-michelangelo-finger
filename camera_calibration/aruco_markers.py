"""Detection of standalone ArUco markers (as opposed to charuco_board.py's
ChArUco board) — the individual tags from generate_targets.py's
standalone_markers.pdf, used to fix reference points in the world (e.g.
on the floor next to the arm, for calibrate_camera_to_arm.py) rather than
as a checkerboard.

Same measured size and dictionary as the ChArUco board: both PDFs came out
of the same print run and were checked to have scaled by the same factor
(see the conversation/commit history — 25.6mm instead of the nominal
25.0mm for both the board's squares and these tags).
"""

from __future__ import annotations

from typing import Dict

import cv2.aruco as aruco
import numpy as np

from camera_calibration.charuco_board import ARUCO_DICT, MEASURED_SQUARE_LENGTH_MM

MEASURED_MARKER_LENGTH_MM = MEASURED_SQUARE_LENGTH_MM


def build_marker_detector() -> aruco.ArucoDetector:
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    return aruco.ArucoDetector(dictionary)


def detect_markers(image_bgr: np.ndarray, detector: aruco.ArucoDetector) -> Dict[int, np.ndarray]:
    """Detects standalone ArUco markers in one image.

    Returns {marker_id: corners} where corners is a (4, 2) pixel-coordinate
    array, in the detector's clockwise order (same order for every marker,
    which is what makes triangulating "corner i of marker X in image 0"
    against "corner i of marker X in image 1" valid).
    """
    corners, ids, _rejected = detector.detectMarkers(image_bgr)
    if ids is None:
        return {}
    return {int(marker_id): c.reshape(4, 2) for marker_id, c in zip(ids, corners)}
