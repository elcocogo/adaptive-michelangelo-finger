"""Persisted stereo calibration result: each camera's intrinsics/distortion
plus the rigid transform (R, T) between them, as computed by
calibrate_stereo.py from a set of ChArUco image pairs.

Mirrors the save/load pattern used for servo calibration
(servo_calibration/calibration.py) — a plain dataclass, atomic JSON writes.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Union

DEFAULT_STEREO_CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent / "calibration_data" / "stereo_calibration.json"
)

PathLike = Union[str, Path]


@dataclass
class StereoCalibration:
    image_width: int
    image_height: int
    camera_matrix0: List[List[float]]
    dist_coeffs0: List[float]
    camera_matrix1: List[List[float]]
    dist_coeffs1: List[float]
    R: List[List[float]]  # rotation from cam0's frame to cam1's frame
    T: List[float]  # translation from cam0's frame to cam1's frame, in meters
    rms_mono0: float
    rms_mono1: float
    rms_stereo: float
    num_pairs_used: int
    square_length_mm: float
    calibrated_at: str


def save(calib: StereoCalibration, path: PathLike = DEFAULT_STEREO_CALIBRATION_FILE) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(".tmp")
    with tmp_path.open("w") as f:
        json.dump(asdict(calib), f, indent=2)
    tmp_path.replace(path)  # atomic: a crash mid-write can't corrupt the existing file


def load(path: PathLike = DEFAULT_STEREO_CALIBRATION_FILE) -> StereoCalibration:
    with Path(path).open("r") as f:
        data = json.load(f)
    return StereoCalibration(**data)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")
