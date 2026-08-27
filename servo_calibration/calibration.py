"""Persisted servo calibration data: pulse/angle bounds per PCA9685 channel.

Each servo is calibrated independently (see calibrate_servo.py) and its
result is merged into a single shared JSON file, keyed by channel number,
so calibrating a new servo never touches the entries already recorded for
the others.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Union

DEFAULT_CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent / "calibration_data" / "servos.json"
)
DEFAULT_POSITIONS_FILE = (
    Path(__file__).resolve().parent.parent / "calibration_data" / "servo_positions.json"
)
DEFAULT_FREQUENCY_HZ = 50

# Shared speed-ramp constants, used by every tool that moves a servo
# gradually (move_servo.py, arm_show.py, ...) instead of jumping straight
# to a commanded pulse.
#
# MAX_SERVO_SPEED_DEG_PER_S is an assumed full-scale (100%) angular speed
# for a typical hobby servo, used to turn a --speed percentage into an
# actual ramp rate. Real servos vary, but this only needs to be a
# reasonable upper bound for the ramp to be meaningfully gentler at low
# speed settings.
MAX_SERVO_SPEED_DEG_PER_S = 300.0
MIN_SPEED_PERCENT = 10.0
MAX_SPEED_PERCENT = 100.0

# How often intermediate positions are sent to the PCA9685 while ramping.
UPDATE_RATE_HZ = 50.0

PathLike = Union[str, Path]


@dataclass
class ServoCalibration:
    channel: int
    name: str
    pulse_min_us: float
    pulse_center_us: float
    pulse_max_us: float
    angle_min_deg: float
    angle_max_deg: float
    calibrated_at: str

    def pulse_for_angle(self, angle_deg: float) -> float:
        """Linearly interpolate the pulse width for a target angle.

        Two segments (angle_min -> center, center -> angle_max) are used
        instead of a single line through pulse_min/pulse_max, because the
        center pulse found by hand is not guaranteed to be their midpoint.
        The angle is saturated to [angle_min_deg, angle_max_deg] first, so
        this also acts as the safety clamp for arm control commands.
        """
        angle_deg = max(self.angle_min_deg, min(self.angle_max_deg, angle_deg))
        if angle_deg >= 0:
            span_angle, span_pulse = self.angle_max_deg, self.pulse_max_us - self.pulse_center_us
        else:
            span_angle, span_pulse = self.angle_min_deg, self.pulse_min_us - self.pulse_center_us
        if span_angle == 0:
            return self.pulse_center_us
        return self.pulse_center_us + span_pulse * (angle_deg / span_angle)


def load_all(path: PathLike = DEFAULT_CALIBRATION_FILE) -> dict:
    path = Path(path)
    if not path.exists():
        return {"pwm_frequency_hz": DEFAULT_FREQUENCY_HZ, "servos": {}}
    with path.open("r") as f:
        return json.load(f)


def load_servo(channel: int, path: PathLike = DEFAULT_CALIBRATION_FILE) -> Optional[ServoCalibration]:
    data = load_all(path)
    raw = data.get("servos", {}).get(str(channel))
    if raw is None:
        return None
    return ServoCalibration(**raw)


def save_servo(
    calibration: ServoCalibration,
    path: PathLike = DEFAULT_CALIBRATION_FILE,
    frequency_hz: Optional[int] = None,
) -> None:
    """Merge one servo's calibration into the shared file, leaving the rest untouched."""
    path = Path(path)
    data = load_all(path)
    if frequency_hz is not None:
        data["pwm_frequency_hz"] = frequency_hz
    data.setdefault("servos", {})[str(calibration.channel)] = asdict(calibration)

    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(".tmp")
    with tmp_path.open("w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    tmp_path.replace(path)  # atomic: a crash mid-write can't corrupt the existing file


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _write_json_atomic(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(".tmp")
    with tmp_path.open("w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    tmp_path.replace(path)  # atomic: a crash mid-write can't corrupt the existing file


def load_last_angle(channel: int, path: PathLike = DEFAULT_POSITIONS_FILE) -> Optional[float]:
    """Last angle move_servo.py successfully commanded on this channel, if any.

    The PCA9685 keeps driving a channel's last duty cycle even after the
    Python process that set it exits, so as long as nothing released or
    physically moved the servo since, this is still its real position —
    letting a fresh invocation ramp instead of jumping blind.
    """
    path = Path(path)
    if not path.exists():
        return None
    with path.open("r") as f:
        data = json.load(f)
    return data.get(str(channel))


def save_last_angle(channel: int, angle_deg: float, path: PathLike = DEFAULT_POSITIONS_FILE) -> None:
    path = Path(path)
    data = {}
    if path.exists():
        with path.open("r") as f:
            data = json.load(f)
    data[str(channel)] = angle_deg
    _write_json_atomic(data, path)


def clear_last_angle(channel: int, path: PathLike = DEFAULT_POSITIONS_FILE) -> None:
    """Forget the last known angle, e.g. after a release: the real position is no longer known."""
    path = Path(path)
    if not path.exists():
        return
    with path.open("r") as f:
        data = json.load(f)
    if str(channel) in data:
        del data[str(channel)]
        _write_json_atomic(data, path)
