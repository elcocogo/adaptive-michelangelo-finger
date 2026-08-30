#!/usr/bin/env python3
"""Interactive PCA9685 servo calibration tool.

Run this once per servo, right after mounting it on the arm, to find by
hand: the neutral pulse width you want to call 0 degrees, and the min/max
pulse widths matching whatever physical/angular limits you choose for it.

Results are merged into the shared calibration file (calibration_data/
servos.json by default) without touching any other channel's entry, so
this is safe to re-run each time a new servo is mounted, or to re-tune a
servo that was already calibrated.

Usage:
    python3 -m servo_calibration.calibrate_servo --channel 0 --name shoulder

Controls (single keypress, no Enter needed):
    h / l    nudge the pulse down / up by the current step
    H / L    nudge the pulse down / up by 10x the current step
    [ / ]    halve / double the step size
    c        mark the current pulse as the 0 degree center
    n        mark the current pulse as the min bound (asks for its angle)
    x        mark the current pulse as the max bound (asks for its angle)
    p        print the current status
    s        save to the calibration file
    r        release the servo (stop the PWM signal, lets it go limp)
    ?        show this help again
    q        quit (asks to save first if there are unsaved changes)
"""

from __future__ import annotations

import argparse
import sys
import termios
import tty
from typing import Optional

from servo_calibration.calibration import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_FREQUENCY_HZ,
    ServoCalibration,
    load_servo,
    now_iso,
    save_servo,
)
from servo_calibration.pca9685_driver import Pca9685Driver

NEUTRAL_PULSE_US = 1500.0
DEFAULT_STEP_US = 10.0
MIN_STEP_US = 1.0
MAX_STEP_US = 200.0

# Hard safety envelope: whatever the user commands, never drive a pulse
# outside this range. Most analog servos are happy within 500-2500us;
# holding one against its mechanical stop outside that can burn the motor.
HARD_PULSE_MIN_US = 400.0
HARD_PULSE_MAX_US = 2600.0

HELP_TEXT = __doc__.split("Controls", 1)[1]


def read_key() -> str:
    """Read a single keypress from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def prompt_angle(label: str, default: float) -> float:
    """Ask (Enter-terminated, unlike the rest of the controls) for the angle of a bound."""
    raw = input(f"  Angle in degrees for the '{label}' position [{default}]: ").strip()
    return float(raw) if raw else default


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CalibrationSession:
    def __init__(self, driver: Pca9685Driver, channel: int, name: str):
        self.driver = driver
        self.channel = channel
        self.name = name
        self.pulse_us = NEUTRAL_PULSE_US
        self.step_us = DEFAULT_STEP_US
        self.center_us: Optional[float] = None
        self.min_us: Optional[float] = None
        self.max_us: Optional[float] = None
        self.angle_min: Optional[float] = None
        self.angle_max: Optional[float] = None
        self.dirty = False

    def load_existing(self, existing: Optional[ServoCalibration]) -> None:
        if existing is None:
            return
        self.center_us = existing.pulse_center_us
        self.min_us = existing.pulse_min_us
        self.max_us = existing.pulse_max_us
        self.angle_min = existing.angle_min_deg
        self.angle_max = existing.angle_max_deg
        self.pulse_us = existing.pulse_center_us
        print(f"Existing calibration loaded for channel {self.channel} ({existing.name}).")

    def apply_pulse(self) -> None:
        self.pulse_us = clamp(self.pulse_us, HARD_PULSE_MIN_US, HARD_PULSE_MAX_US)
        self.driver.set_pulse_us(self.channel, self.pulse_us)

    def status(self) -> str:
        def fmt(value: Optional[float]) -> str:
            return "?" if value is None else f"{value:.0f}us"

        return (
            f"channel={self.channel} name='{self.name}' current_pulse={self.pulse_us:.0f}us step={self.step_us:.0f}us\n"
            f"  min={fmt(self.min_us)} (angle={self.angle_min}) "
            f"center={fmt(self.center_us)} (angle=0) "
            f"max={fmt(self.max_us)} (angle={self.angle_max})"
        )

    def to_calibration(self) -> Optional[ServoCalibration]:
        if self.center_us is None or self.min_us is None or self.max_us is None:
            return None
        return ServoCalibration(
            channel=self.channel,
            name=self.name,
            pulse_min_us=self.min_us,
            pulse_center_us=self.center_us,
            pulse_max_us=self.max_us,
            angle_min_deg=self.angle_min,
            angle_max_deg=self.angle_max,
            calibrated_at=now_iso(),
        )


def run(channel: int, name: Optional[str], calibration_file, frequency_hz: int) -> None:
    existing = load_servo(channel, calibration_file)
    name = name or (existing.name if existing else f"servo_{channel}")

    driver = Pca9685Driver(frequency_hz=frequency_hz)
    session = CalibrationSession(driver, channel, name)
    session.load_existing(existing)
    session.apply_pulse()

    print(HELP_TEXT)
    print(session.status())

    try:
        while True:
            key = read_key()
            if key == "h":
                session.pulse_us -= session.step_us
                session.apply_pulse()
            elif key == "l":
                session.pulse_us += session.step_us
                session.apply_pulse()
            elif key == "H":
                session.pulse_us -= session.step_us * 10
                session.apply_pulse()
            elif key == "L":
                session.pulse_us += session.step_us * 10
                session.apply_pulse()
            elif key == "[":
                session.step_us = clamp(session.step_us / 2, MIN_STEP_US, MAX_STEP_US)
            elif key == "]":
                session.step_us = clamp(session.step_us * 2, MIN_STEP_US, MAX_STEP_US)
            elif key == "c":
                session.center_us = session.pulse_us
                session.dirty = True
                print(f"  -> center (0 deg) = {session.pulse_us:.0f}us")
            elif key == "n":
                session.angle_min = prompt_angle("min", session.angle_min if session.angle_min is not None else -90.0)
                session.min_us = session.pulse_us
                session.dirty = True
                print(f"  -> min ({session.angle_min} deg) = {session.pulse_us:.0f}us")
            elif key == "x":
                session.angle_max = prompt_angle("max", session.angle_max if session.angle_max is not None else 90.0)
                session.max_us = session.pulse_us
                session.dirty = True
                print(f"  -> max ({session.angle_max} deg) = {session.pulse_us:.0f}us")
            elif key == "p":
                print(session.status())
            elif key == "r":
                driver.release(channel)
                print("  -> servo released (PWM cut)")
            elif key == "s":
                calib = session.to_calibration()
                if calib is None:
                    print("  Cannot save: missing min, center, or max.")
                else:
                    save_servo(calib, calibration_file, frequency_hz=frequency_hz)
                    session.dirty = False
                    print(f"  -> saved to {calibration_file}")
            elif key == "?":
                print(HELP_TEXT)
            elif key == "q":
                if session.dirty:
                    confirm = input("Unsaved changes, quit anyway? [y/N] ").strip().lower()
                    if confirm != "y":
                        continue
                break
    finally:
        driver.release(channel)
        driver.close()
        print("Session ended.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive calibration of a servo on the PCA9685.")
    parser.add_argument("--channel", type=int, required=True, help="PCA9685 channel (0-15)")
    parser.add_argument("--name", default=None, help="Servo name (e.g. shoulder, elbow)")
    parser.add_argument("--file", default=DEFAULT_CALIBRATION_FILE, help="JSON calibration file")
    parser.add_argument("--frequency", type=int, default=DEFAULT_FREQUENCY_HZ, help="PWM frequency in Hz (default 50)")
    args = parser.parse_args()

    if not 0 <= args.channel <= 15:
        parser.error("--channel must be between 0 and 15")

    run(args.channel, args.name, args.file, args.frequency)


if __name__ == "__main__":
    main()
