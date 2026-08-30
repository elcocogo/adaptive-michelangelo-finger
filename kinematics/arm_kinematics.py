"""Forward and inverse kinematics for the 5-DOF arm.

Chain (see servo_calibration/README.md and CLAUDE.md for the physical
description and channel numbering):

    base --[ch0 base_joint, Z]--> base_spin --[ch1 spin_joint, X]--> base_arm
         --[ch2 basearm_joint, X]--> mid_arm --[ch3 midarm_joint, X]--> gripper_arm

Channels 1-3 share parallel (X) axes, so once the base has yawed by ch0,
the rest of the chain moves entirely within one vertical plane that
rotates with it. That reduces the problem to a textbook 2D "planar
N-link arm" plus a base rotation — see below for how each function
uses that.

Angle convention (matches calibrate_servo.py/arm_show.py): channel 1 is
absolute (0 deg = base_arm points straight up); channels 2 and 3 are
*relative to their parent segment* (0 deg = continues straight, no bend).

Units: millimeters and degrees everywhere in this module's public
functions (matching the rest of the project), radians only internally
for trig.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class ArmDimensions:
    shoulder_height_mm: float  # base_joint's floor pivot -> spin_joint (shoulder) pivot
    base_arm_mm: float  # spin_joint -> basearm_joint pivot
    mid_arm_mm: float  # basearm_joint -> midarm_joint pivot
    gripper_arm_mm: float  # midarm_joint -> finger pivot


# Measured on the physical arm (see conversation/commit history) rather
# than the ~5cm estimate from early in the project.
DEFAULT_DIMENSIONS = ArmDimensions(
    shoulder_height_mm=45.0,
    base_arm_mm=54.0,
    mid_arm_mm=50.0,
    gripper_arm_mm=40.0,
)


def forward_kinematics(
    theta0_deg: float,
    theta1_deg: float,
    theta2_deg: float,
    theta3_deg: float,
    dims: ArmDimensions = DEFAULT_DIMENSIONS,
) -> Tuple[float, float, float]:
    """Joint angles -> the finger pivot's (x, y, z) in the arm frame, in mm.

    Each link's angle *from vertical*, in the rotating plane, accumulates
    down the chain (channels 2/3 are relative to their parent):
        link 1 (base_arm):    theta1
        link 2 (mid_arm):     theta1 + theta2
        link 3 (gripper_arm): theta1 + theta2 + theta3
    A link tilted by angle phi from vertical contributes
    (L*sin(phi) outward, L*cos(phi) upward) — at phi=0 that's (0, L), i.e.
    straight up, matching the calibration convention.
    """
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta1_deg + theta2_deg)
    t3 = math.radians(theta1_deg + theta2_deg + theta3_deg)

    r = (
        dims.base_arm_mm * math.sin(t1)
        + dims.mid_arm_mm * math.sin(t2)
        + dims.gripper_arm_mm * math.sin(t3)
    )
    z = (
        dims.shoulder_height_mm
        + dims.base_arm_mm * math.cos(t1)
        + dims.mid_arm_mm * math.cos(t2)
        + dims.gripper_arm_mm * math.cos(t3)
    )

    theta0 = math.radians(theta0_deg)
    x = r * math.cos(theta0)
    y = r * math.sin(theta0)
    return x, y, z


def inverse_kinematics(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    dims: ArmDimensions = DEFAULT_DIMENSIONS,
    elbow_up: bool = True,
) -> Tuple[float, float, float, float]:
    """Target (x, y, z) in the arm frame, in mm -> (theta0, theta1, theta2, theta3) in degrees.

    The chain has 4 position-relevant joints for a 3D target — one more
    than needed, so there isn't a single answer (same as your own arm:
    elbow up or down can reach the same point). This resolves that by
    locking channel 3 to 0 deg (gripper_arm kept straight in line with
    mid_arm), which turns the remaining problem into a classic two-link
    planar arm (link 1 = base_arm, link 2 = mid_arm+gripper_arm combined)
    reaching a point in the vertical plane picked out by theta0 — solved
    with the law of cosines, same as any 2-link arm.

    `elbow_up`/False selects between the two remaining solutions (mid_arm
    bending one way or the other) — try the other one if a target is
    unreachable with the default, or if the chosen side would collide
    with something.

    Raises ValueError if the target is out of reach (too far, or too
    close to fold into).
    """
    theta0 = math.degrees(math.atan2(y_mm, x_mm))

    r = math.hypot(x_mm, y_mm)
    z_rel = z_mm - dims.shoulder_height_mm  # height above the shoulder pivot, not the floor
    distance = math.hypot(r, z_rel)

    l1 = dims.base_arm_mm
    l2 = dims.mid_arm_mm + dims.gripper_arm_mm
    if distance > l1 + l2 or distance < abs(l1 - l2):
        raise ValueError(
            f"Target unreachable: distance from shoulder is {distance:.1f}mm, "
            f"but the arm spans {abs(l1 - l2):.1f}-{l1 + l2:.1f}mm."
        )

    # Law of cosines for the elbow bend (theta2): at theta2=0 (fully
    # extended) distance == l1+l2, matching cos(0)=1 below.
    cos_theta2 = (distance**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # guard tiny float overshoot at the reach limits
    theta2_mag = math.degrees(math.acos(cos_theta2))
    theta2 = theta2_mag if elbow_up else -theta2_mag

    # alpha: direction of the target from the shoulder, from vertical.
    # beta: angle between that direction and link 1, from the triangle
    # (shoulder, elbow, target) via the law of cosines again.
    alpha = math.atan2(r, z_rel)
    cos_beta = (l1**2 + distance**2 - l2**2) / (2 * l1 * distance)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    theta1 = math.degrees(alpha - beta) if elbow_up else math.degrees(alpha + beta)

    theta3 = 0.0
    return theta0, theta1, theta2, theta3
