#!/usr/bin/env python3
"""Manual angle-based control for one already-calibrated servo.

Looks up an existing calibration (by channel or by name) in the shared
calibration file and lets you drive that servo by typing a target angle in
degrees. The angle is checked against [angle_min_deg, angle_max_deg] from
the calibration and rejected with an error message if it's out of range,
rather than being silently clamped.

Movement is ramped rather than commanded in one jump, to avoid slamming
the arm from one extreme to the other at the servo's full unrestrained
speed. --speed sets that ramp rate as a percentage of an assumed
full-scale servo speed (MAX_SERVO_SPEED_DEG_PER_S below): 100 is fastest,
10 is slowest, default 50.

This is open-loop control (no angle feedback), so ramping needs a known
starting angle. The PCA9685 keeps driving a channel at its last commanded
duty cycle even after this script exits, so the last angle successfully
commanded is persisted to calibration_data/servo_positions.json and
reloaded on the next run — meaning --speed applies from the very first
command, even in a fresh invocation, as long as nothing released or
manually moved the servo in between. Quitting with 'q' leaves the PWM
signal active (the servo holds its position); only 'r' cuts it, and also
forgets the saved position since it's no longer known afterwards.

Usage:
    python3 -m servo_calibration.move_servo --channel 0
    python3 -m servo_calibration.move_servo --name shoulder --speed 25
    python3 -m servo_calibration.move_servo   # asks for channel or name

Controls:
    <number>   move to that angle in degrees (e.g. 12.5 or -30)
    c          move to center (0 deg)
    n          move to angle_min_deg
    x          move to angle_max_deg
    r          release (stop the PWM signal, servo goes limp)
    q          quit
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from servo_calibration.calibration import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_FREQUENCY_HZ,
    DEFAULT_POSITIONS_FILE,
    MAX_SERVO_SPEED_DEG_PER_S,
    MAX_SPEED_PERCENT,
    MIN_SPEED_PERCENT,
    UPDATE_RATE_HZ,
    ServoCalibration,
    clear_last_angle,
    load_all,
    load_last_angle,
    save_last_angle,
)
from servo_calibration.pca9685_driver import Pca9685Driver

HELP_TEXT = __doc__.split("Controls:", 1)[1]

DEFAULT_SPEED_PERCENT = 70.0


def find_calibration(data: dict, channel: Optional[int], name: Optional[str]) -> Optional[ServoCalibration]:
    servos = data.get("servos", {})
    if channel is not None:
        raw = servos.get(str(channel))
        return ServoCalibration(**raw) if raw else None
    for raw in servos.values():
        if raw.get("name") == name:
            return ServoCalibration(**raw)
    return None


def resolve_calibration(data: dict, channel: Optional[int], name: Optional[str], path) -> ServoCalibration:
    if channel is None and name is None:
        raw = input("Channel or name of the servo to control: ").strip()
        if raw.isdigit():
            channel = int(raw)
        else:
            name = raw

    calib = find_calibration(data, channel, name)
    if calib is None:
        target = f"channel {channel}" if channel is not None else f"name '{name}'"
        sys.exit(f"No calibration found for {target} in {path}.")
    return calib


def move_to_angle(
    driver: Pca9685Driver,
    calib: ServoCalibration,
    target_deg: float,
    current_deg: Optional[float],
    speed_deg_per_s: float,
) -> Optional[float]:
    """Ramp from current_deg to target_deg, sending intermediate pulses along the way.

    Returns the new current_deg (unchanged if the move was rejected as out
    of bounds). current_deg is None when the real starting position isn't
    known (session start, or after a release) — in that case the move
    jumps straight to the target, since there's nothing to ramp from.
    """
    if not calib.angle_min_deg <= target_deg <= calib.angle_max_deg:
        print(
            f"  Error: {target_deg} deg out of bounds "
            f"[{calib.angle_min_deg}, {calib.angle_max_deg}] for '{calib.name}'."
        )
        return current_deg

    if current_deg is not None and current_deg != target_deg:
        step_interval_s = 1.0 / UPDATE_RATE_HZ
        duration_s = abs(target_deg - current_deg) / speed_deg_per_s
        steps = max(1, round(duration_s * UPDATE_RATE_HZ))
        for i in range(1, steps):
            intermediate_deg = current_deg + (target_deg - current_deg) * i / steps
            driver.set_pulse_us(calib.channel, calib.pulse_for_angle(intermediate_deg))
            time.sleep(step_interval_s)

    pulse_us = calib.pulse_for_angle(target_deg)
    driver.set_pulse_us(calib.channel, pulse_us)
    print(f"  -> {calib.name} (channel {calib.channel}) at {target_deg} deg ({pulse_us:.0f}us)")
    return target_deg


def run(calib: ServoCalibration, frequency_hz: int, speed_deg_per_s: float, positions_file) -> None:
    driver = Pca9685Driver(frequency_hz=frequency_hz)
    current_deg = load_last_angle(calib.channel, positions_file)
    print(
        f"Servo '{calib.name}' (channel {calib.channel}), "
        f"bounds [{calib.angle_min_deg}, {calib.angle_max_deg}] deg, "
        f"speed {speed_deg_per_s:.0f} deg/s."
    )
    if current_deg is not None:
        print(f"Last known position: {current_deg} deg (the servo hasn't been released since).")
    print(HELP_TEXT)
    try:
        while True:
            try:
                raw = input("> ").strip()
            except KeyboardInterrupt:
                print()
                break
            if raw == "":
                continue
            elif raw == "q":
                break
            elif raw == "r":
                driver.release(calib.channel)
                current_deg = None
                clear_last_angle(calib.channel, positions_file)
                print("  -> servo released (PWM cut)")
            else:
                if raw == "c":
                    target_deg = 0.0
                elif raw == "n":
                    target_deg = calib.angle_min_deg
                elif raw == "x":
                    target_deg = calib.angle_max_deg
                else:
                    try:
                        target_deg = float(raw)
                    except ValueError:
                        print("  Unrecognized input (expected a number, c, n, x, r, or q).")
                        continue
                new_deg = move_to_angle(driver, calib, target_deg, current_deg, speed_deg_per_s)
                if new_deg != current_deg:
                    current_deg = new_deg
                    save_last_angle(calib.channel, current_deg, positions_file)
    finally:
        driver.close()
        print("Session ended (the servo keeps its position until released with 'r').")


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual angle-based control of an already-calibrated servo.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--channel", type=int, help="PCA9685 channel of the servo (0-15)")
    group.add_argument("--name", help="Servo name (as saved during calibration)")
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

    data = load_all(args.file)
    frequency_hz = data.get("pwm_frequency_hz", DEFAULT_FREQUENCY_HZ)
    calib = resolve_calibration(data, args.channel, args.name, args.file)
    speed_deg_per_s = MAX_SERVO_SPEED_DEG_PER_S * (args.speed / 100.0)

    run(calib, frequency_hz, speed_deg_per_s, DEFAULT_POSITIONS_FILE)


if __name__ == "__main__":
    main()
