"""Persisted camera-to-arm calibration: the rigid transform from camera 0's
frame to the arm's own reference frame, as computed by
calibrate_camera_to_arm.py from two floor-mounted ArUco markers.

Arm frame convention (chosen when this was set up, will anchor the
forward/inverse kinematics built later): origin at the point on the floor
directly below the base_joint's rotation axis; X axis pointing in the
direction the arm faces when base_joint = 0 deg; Z axis vertical, up.

Usage of the stored transform: given a 3D point in camera 0's frame (e.g.
from triangulation.py), its coordinates in the arm's frame are
`R @ point_cam + T`.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Union

DEFAULT_ARM_FRAME_FILE = (
    Path(__file__).resolve().parent.parent / "calibration_data" / "camera_to_arm.json"
)

PathLike = Union[str, Path]


@dataclass
class ArmFrameCalibration:
    R: List[List[float]]  # rotation, camera0 frame -> arm frame
    T: List[float]  # translation (meters), camera0 frame -> arm frame
    marker_ids: List[int]  # which two standalone ArUco tags were used
    measured_distance_mm: float  # ruler measurement between the 2 markers
    triangulated_distance_mm: float  # same distance, from stereo triangulation
    normal_agreement_deg: float  # angle between the 2 markers' independently-detected floor normals
    calibrated_at: str


def save(calib: ArmFrameCalibration, path: PathLike = DEFAULT_ARM_FRAME_FILE) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(".tmp")
    with tmp_path.open("w") as f:
        json.dump(asdict(calib), f, indent=2)
    tmp_path.replace(path)  # atomic: a crash mid-write can't corrupt the existing file


def load(path: PathLike = DEFAULT_ARM_FRAME_FILE) -> ArmFrameCalibration:
    with Path(path).open("r") as f:
        data = json.load(f)
    return ArmFrameCalibration(**data)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")
