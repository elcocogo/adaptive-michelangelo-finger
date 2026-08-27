#!/usr/bin/env python3
"""Scripted demo choreography for the 5-DOF arm.

Chain of joints (see servo_calibration/README.md and calibrate_servo.py
for how each was calibrated):

    base --[ch0, Z]--> base_spin --[ch1, X]--> base_arm
         --[ch2, X]--> mid_arm --[ch3, X]--> gripper_arm --[ch4]--> finger

Channels 1-3 (base_arm/mid_arm/gripper_arm) share parallel axes and a
*relative* convention: 0 deg means "aligned with the parent segment", not
"vertical in world space" — e.g. channel 2 at 0 deg means mid_arm
continues straight from wherever base_arm is pointing, it doesn't mean
mid_arm itself points up. Only channel 1 (base_arm, attached directly to
base_spin which never tilts) has 0 deg == world-vertical.

Each pose in SEQUENCE lists only the channels it changes; unlisted
channels stay wherever they were. Every pose moves as a single
synchronized step: whichever joint has the largest angle to cover sets
the pace (at --speed), the other joints in that same step glide there
slower so all of them arrive together.

Requires channels 0-4 already calibrated (calibrate_servo.py). Reuses the
open-loop ramp/position-persistence machinery from move_servo.py, so it
can be interrupted (Ctrl+C) safely: the last pose reached stays tracked in
calibration_data/servo_positions.json, and the arm holds it after exit
(no auto-release, same as move_servo.py — use move_servo.py's 'r' if you
want to relax the arm afterwards).

Usage:
    python3 -m servo_calibration.arm_show
    python3 -m servo_calibration.arm_show --speed 40 --pause 1.0
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

from servo_calibration.calibration import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_FREQUENCY_HZ,
    DEFAULT_POSITIONS_FILE,
    MAX_SERVO_SPEED_DEG_PER_S,
    MAX_SPEED_PERCENT,
    MIN_SPEED_PERCENT,
    UPDATE_RATE_HZ,
    ServoCalibration,
    load_all,
    load_last_angle,
    save_last_angle,
)
from servo_calibration.pca9685_driver import Pca9685Driver

# Channels, named after the segment each one moves (see the chain above).
CH_BASE_SPIN = 0    # base -> base_spin, lacet du bras entier autour de Z
CH_BASE_ARM = 1      # base_spin -> base_arm ("epaule"), 0 deg = vertical
CH_MID_ARM = 2        # base_arm -> mid_arm ("coude"), 0 deg = aligne avec base_arm
CH_GRIPPER_ARM = 3    # mid_arm -> gripper_arm ("poignet"), 0 deg = aligne avec mid_arm
CH_GRIPPER = 4         # gripper_arm -> finger, ouverture de la pince

ALL_CHANNELS = (CH_BASE_SPIN, CH_BASE_ARM, CH_MID_ARM, CH_GRIPPER_ARM, CH_GRIPPER)

DEFAULT_SPEED_PERCENT = 60.0
DEFAULT_PAUSE_S = 0.6

Pose = Dict[int, float]

HOME: Pose = {CH_BASE_SPIN: 0.0, CH_BASE_ARM: 0.0, CH_MID_ARM: 0.0, CH_GRIPPER_ARM: 0.0, CH_GRIPPER: 0.0}

SEQUENCE: List[Tuple[str, Pose]] = [
    ("Position de depart (bras vertical, pince entre-ouverte)", HOME),
    ("Extension complete a l'horizontale (bras tendu au loin)",
     {CH_BASE_ARM: 90.0, CH_MID_ARM: 0.0, CH_GRIPPER_ARM: 0.0}),
    ("Repli compact (plie en Z sur lui-meme)",
     {CH_BASE_ARM: 90.0, CH_MID_ARM: -90.0, CH_GRIPPER_ARM: -90.0}),
    ("Retour a la verticale", HOME),
    ("Rotation de la base a +90 deg", {CH_BASE_SPIN: 90.0}),
    ("Rotation de la base a -90 deg (180 deg de trajet)", {CH_BASE_SPIN: -90.0}),
    ("Retour de la base au centre", {CH_BASE_SPIN: 0.0}),
    ("Pince : ouverture complete", {CH_GRIPPER: 15.0}),
    ("Pince : fermeture complete", {CH_GRIPPER: -15.0}),
    ("Pince : entre-ouverte", {CH_GRIPPER: 0.0}),
    ("Mise en position pour plier le coude", {CH_BASE_ARM: 45.0, CH_MID_ARM: 0.0}),
    ("Pliage du coude (mid_arm)", {CH_MID_ARM: -80.0}),
    ("Depliage du coude", {CH_MID_ARM: 80.0}),
    ("Coude au centre", {CH_MID_ARM: 0.0}),
    ("Extension du poignet (gripper_arm)", {CH_GRIPPER_ARM: 80.0}),
    ("Retour du poignet au centre", {CH_GRIPPER_ARM: 0.0}),
    ("Retour a la verticale avant les mouvements combines", HOME),
    ("Combine : extension + rotation + pince ouverte",
     {CH_BASE_SPIN: 45.0, CH_BASE_ARM: 70.0, CH_MID_ARM: -40.0, CH_GRIPPER_ARM: 30.0, CH_GRIPPER: 15.0}),
    ("Combine : repli + rotation opposee + pince fermee (comme une prise)",
     {CH_BASE_SPIN: -45.0, CH_BASE_ARM: 20.0, CH_MID_ARM: 10.0, CH_GRIPPER_ARM: -10.0, CH_GRIPPER: -15.0}),
    ("Combine : petit salut (balayage du poignet)",
     {CH_BASE_SPIN: 0.0, CH_BASE_ARM: 40.0, CH_MID_ARM: -20.0, CH_GRIPPER_ARM: 40.0, CH_GRIPPER: 0.0}),
    ("Retour a la position de depart", HOME),
]


def load_calibrations(path) -> Tuple[Dict[int, ServoCalibration], int]:
    data = load_all(path)
    calibs = {}
    for channel in ALL_CHANNELS:
        raw = data.get("servos", {}).get(str(channel))
        if raw is None:
            sys.exit(
                f"Canal {channel} non calibre dans {path} — "
                f"lance calibrate_servo.py --channel {channel} d'abord."
            )
        calibs[channel] = ServoCalibration(**raw)
    return calibs, data.get("pwm_frequency_hz", DEFAULT_FREQUENCY_HZ)


def move_pose(
    driver: Pca9685Driver,
    calibs: Dict[int, ServoCalibration],
    pose: Pose,
    current: Dict[int, Optional[float]],
    speed_deg_per_s: float,
    positions_file,
) -> None:
    """Move the channels named in `pose` together, synchronized to arrive at once.

    The channel needing the biggest angular change sets the step count (at
    speed_deg_per_s); every other channel in this pose is interpolated
    over that same number of steps, so they all reach their target on the
    same final step instead of finishing at different times.
    """
    for channel, angle_deg in pose.items():
        calib = calibs[channel]
        if not calib.angle_min_deg <= angle_deg <= calib.angle_max_deg:
            raise ValueError(
                f"Angle {angle_deg} hors bornes [{calib.angle_min_deg}, {calib.angle_max_deg}] "
                f"pour le canal {channel} ('{calib.name}')."
            )

    # Position reelle inconnue (tout premier mouvement, ou apres relache) :
    # ce canal saute directement a sa cible, faute de point de depart.
    for channel, target_deg in pose.items():
        if current.get(channel) is None:
            calib = calibs[channel]
            driver.set_pulse_us(channel, calib.pulse_for_angle(target_deg))
            current[channel] = target_deg
            save_last_angle(channel, target_deg, positions_file)

    known = {ch: target for ch, target in pose.items() if current[ch] != target}
    if not known:
        return

    step_interval_s = 1.0 / UPDATE_RATE_HZ
    starts = {ch: current[ch] for ch in known}
    durations_s = [abs(known[ch] - starts[ch]) / speed_deg_per_s for ch in known]
    steps = max(1, round(max(durations_s) * UPDATE_RATE_HZ))

    for i in range(1, steps + 1):
        for channel, target_deg in known.items():
            start_deg = starts[channel]
            intermediate_deg = start_deg + (target_deg - start_deg) * i / steps
            driver.set_pulse_us(channel, calibs[channel].pulse_for_angle(intermediate_deg))
        if i < steps:
            time.sleep(step_interval_s)

    for channel, target_deg in known.items():
        current[channel] = target_deg
        save_last_angle(channel, target_deg, positions_file)


def run_show(
    calibs: Dict[int, ServoCalibration],
    frequency_hz: int,
    speed_deg_per_s: float,
    positions_file,
    pause_s: float,
) -> None:
    driver = Pca9685Driver(frequency_hz=frequency_hz)
    current: Dict[int, Optional[float]] = {
        channel: load_last_angle(channel, positions_file) for channel in ALL_CHANNELS
    }
    try:
        for label, pose in SEQUENCE:
            print(f"-> {label}")
            move_pose(driver, calibs, pose, current, speed_deg_per_s, positions_file)
            time.sleep(pause_s)
        print("Show termine.")
    except KeyboardInterrupt:
        print("\nShow interrompu, le bras reste a sa derniere position.")
    finally:
        driver.close()
        print(
            "Le bras garde sa position (PWM toujours actif) — "
            "utilise move_servo.py avec 'r' si tu veux le relacher."
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Petite choregraphie de demonstration pour le bras 5 DOF.")
    parser.add_argument("--file", default=DEFAULT_CALIBRATION_FILE, help="Fichier de calibration JSON")
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED_PERCENT,
        help=(
            f"Vitesse de deplacement en %% de la vitesse max supposee du servo "
            f"({MIN_SPEED_PERCENT:.0f}-{MAX_SPEED_PERCENT:.0f}, defaut {DEFAULT_SPEED_PERCENT:.0f})"
        ),
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=DEFAULT_PAUSE_S,
        help=f"Pause en secondes entre deux poses (defaut {DEFAULT_PAUSE_S})",
    )
    args = parser.parse_args()

    if not MIN_SPEED_PERCENT <= args.speed <= MAX_SPEED_PERCENT:
        parser.error(f"--speed doit etre entre {MIN_SPEED_PERCENT:.0f} et {MAX_SPEED_PERCENT:.0f}")

    calibs, frequency_hz = load_calibrations(args.file)
    speed_deg_per_s = MAX_SERVO_SPEED_DEG_PER_S * (args.speed / 100.0)

    run_show(calibs, frequency_hz, speed_deg_per_s, DEFAULT_POSITIONS_FILE, args.pause)


if __name__ == "__main__":
    main()
