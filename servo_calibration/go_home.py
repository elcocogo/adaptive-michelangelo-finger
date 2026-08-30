#!/usr/bin/env python3
"""Returns the arm to its home position: vertical, all 5 channels at 0 deg
(same HOME pose arm_show.py starts and ends its sequence with).

Usage:
    python3 -m servo_calibration.go_home
    python3 -m servo_calibration.go_home --speed 40
"""

from __future__ import annotations

import argparse
from typing import Dict, Optional

from servo_calibration.arm_show import ALL_CHANNELS, HOME, ServoCalibration, load_calibrations, move_pose
from servo_calibration.calibration import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_POSITIONS_FILE,
    MAX_SERVO_SPEED_DEG_PER_S,
    MAX_SPEED_PERCENT,
    MIN_SPEED_PERCENT,
    load_last_angle,
)
from servo_calibration.pca9685_driver import Pca9685Driver

DEFAULT_SPEED_PERCENT = 50.0


def go_home(
    driver: Pca9685Driver,
    calibs: Dict[int, ServoCalibration],
    speed_deg_per_s: float,
    positions_file,
    current: Optional[Dict[int, Optional[float]]] = None,
) -> None:
    """Moves all 5 channels to HOME (0 deg), synchronized via move_pose.

    `current` lets a caller that already tracks joint positions in memory
    (e.g. tracking.follow_target's loop) pass its own state in, so the
    move ramps from the real last-known angle instead of re-reading
    calibration_data/servo_positions.json from scratch. Left as None,
    it reads that file itself — which is what a standalone call wants.
    """
    if current is None:
        current = {channel: load_last_angle(channel, positions_file) for channel in ALL_CHANNELS}
    move_pose(driver, calibs, HOME, current, speed_deg_per_s, positions_file)


def main() -> None:
    parser = argparse.ArgumentParser(description="Return the arm to its home position (vertical, all angles 0 deg).")
    parser.add_argument("--file", default=DEFAULT_CALIBRATION_FILE, help="JSON calibration file")
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

    calibs, frequency_hz = load_calibrations(args.file)
    speed_deg_per_s = MAX_SERVO_SPEED_DEG_PER_S * (args.speed / 100.0)

    driver = Pca9685Driver(frequency_hz=frequency_hz)
    try:
        go_home(driver, calibs, speed_deg_per_s, DEFAULT_POSITIONS_FILE)
        print("Arm back at home position (PWM still active).")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
