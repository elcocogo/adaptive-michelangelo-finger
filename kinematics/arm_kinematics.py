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
from typing import Optional, Tuple

import numpy as np


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
    theta3_deg: float = 0.0,
) -> Tuple[float, float, float, float]:
    """Target (x, y, z) in the arm frame, in mm -> (theta0, theta1, theta2, theta3) in degrees.

    The chain has 4 position-relevant joints for a 3D target — one more
    than needed, so there isn't a single answer (same as your own arm:
    elbow up or down can reach the same point). This resolves that by
    fixing the wrist bend (channel 3) to theta3_deg (0 by default:
    gripper_arm kept straight in line with mid_arm) and solving the
    remaining problem as a classic two-link planar arm (link 1 =
    base_arm, link 2 = the straight-line span from elbow to fingertip
    that mid_arm+gripper_arm reach at that fixed wrist bend) — law of
    cosines, same as any 2-link arm. See inverse_kinematics_search below
    if you want theta3 chosen automatically instead of fixed.

    `elbow_up`/False selects between the two remaining solutions (mid_arm
    bending one way or the other) — try the other one if a target is
    unreachable with the default, or if the chosen side would collide
    with something.

    Raises ValueError if the target is out of reach (too far, or too
    close to fold into) *at this particular theta3_deg* — a target can be
    genuinely out of the arm's total reach, or merely unreachable with
    this specific wrist angle while still reachable with another one.
    """
    theta0 = math.degrees(math.atan2(y_mm, x_mm))

    r = math.hypot(x_mm, y_mm)
    z_rel = z_mm - dims.shoulder_height_mm  # height above the shoulder pivot, not the floor
    distance = math.hypot(r, z_rel)

    l1 = dims.base_arm_mm
    l2, l3 = dims.mid_arm_mm, dims.gripper_arm_mm
    theta3_rad = math.radians(theta3_deg)

    # Fixing the wrist bend turns mid_arm+gripper_arm into one virtual
    # segment from elbow to fingertip: work out its length (l2_eff) and
    # the angle (gamma) it makes with mid_arm's own direction, by summing
    # the two links as vectors in mid_arm's local frame (mid_arm itself
    # is (0, l2) there; gripper_arm, bent by theta3_deg off of it, is
    # (l3*sin(theta3), l3*cos(theta3))).
    local_r = l3 * math.sin(theta3_rad)
    local_z = l2 + l3 * math.cos(theta3_rad)
    l2_eff = math.hypot(local_r, local_z)
    gamma = math.degrees(math.atan2(local_r, local_z))

    if distance > l1 + l2_eff or distance < abs(l1 - l2_eff):
        raise ValueError(
            f"Target unreachable at theta3={theta3_deg:.1f} deg: distance from shoulder is "
            f"{distance:.1f}mm, but the arm spans {abs(l1 - l2_eff):.1f}-{l1 + l2_eff:.1f}mm there."
        )

    # Law of cosines for the effective elbow bend (theta2_prime): at
    # theta2_prime=0 (fully extended) distance == l1+l2_eff, matching
    # cos(0)=1 below.
    cos_theta2_prime = (distance**2 - l1**2 - l2_eff**2) / (2 * l1 * l2_eff)
    cos_theta2_prime = max(-1.0, min(1.0, cos_theta2_prime))  # guard tiny float overshoot at the reach limits
    theta2_prime_mag = math.degrees(math.acos(cos_theta2_prime))
    theta2_prime = theta2_prime_mag if elbow_up else -theta2_prime_mag

    # alpha: direction of the target from the shoulder, from vertical.
    # beta: angle between that direction and link 1, from the triangle
    # (shoulder, elbow, target) via the law of cosines again.
    alpha = math.atan2(r, z_rel)
    cos_beta = (l1**2 + distance**2 - l2_eff**2) / (2 * l1 * distance)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    theta1 = math.degrees(alpha - beta) if elbow_up else math.degrees(alpha + beta)

    # theta2_prime is mid_arm's bend if it alone spanned elbow->fingertip
    # (gamma=0 case); recover the real mid_arm bend by removing the
    # offset gripper_arm's fixed angle introduces.
    theta2 = theta2_prime - gamma
    return theta0, theta1, theta2, theta3_deg


def inverse_kinematics_search(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    theta1_bounds: Tuple[float, float] = (-90.0, 90.0),
    theta2_bounds: Tuple[float, float] = (-90.0, 90.0),
    theta3_bounds: Tuple[float, float] = (-90.0, 90.0),
    theta3_step_deg: float = 5.0,
    dims: ArmDimensions = DEFAULT_DIMENSIONS,
    elbow_up: bool = True,
) -> Tuple[float, float, float, float]:
    """Like inverse_kinematics, but searches over the wrist bend (theta3)
    instead of fixing it to 0, looking for a combination where theta1,
    theta2, and theta3 all land within the given bounds.

    Why: locking theta3=0 treats mid_arm+gripper_arm as one rigid 90mm
    segment. Some targets can only be reached that way by bending the
    elbow (theta2) further than basearm_joint is calibrated for — even
    though the *same target* is reachable within every joint's limits by
    also bending the wrist a bit. This tries candidate wrist angles
    (closest to straight first, since a straighter wrist is usually
    preferable) and returns the first one that keeps everything in
    bounds.

    Pass the real calibrated angle_min_deg/angle_max_deg for
    spin_joint/basearm_joint/midarm_joint (in that order) as the bounds —
    this module doesn't know about calibration_data/servos.json itself,
    by design (see the package README).

    Raises ValueError if no candidate wrist angle reaches the target
    within bounds (re-raising the last underlying error from
    inverse_kinematics if every candidate was geometrically unreachable).
    """
    candidates = sorted(
        np.arange(theta3_bounds[0], theta3_bounds[1] + theta3_step_deg / 2.0, theta3_step_deg),
        key=abs,
    )
    last_error: Optional[ValueError] = None
    for theta3_try in candidates:
        try:
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x_mm, y_mm, z_mm, dims=dims, elbow_up=elbow_up, theta3_deg=float(theta3_try)
            )
        except ValueError as e:
            last_error = e
            continue
        if theta1_bounds[0] <= theta1 <= theta1_bounds[1] and theta2_bounds[0] <= theta2 <= theta2_bounds[1]:
            return theta0, theta1, theta2, theta3

    if last_error is not None:
        raise last_error
    raise ValueError(
        f"Target ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f})mm is reachable but no wrist angle "
        f"in [{theta3_bounds[0]:.0f}, {theta3_bounds[1]:.0f}] deg keeps theta1/theta2 within bounds."
    )


def apply_standoff(
    target_mm: Tuple[float, float, float],
    standoff_mm: float,
    dims: ArmDimensions = DEFAULT_DIMENSIONS,
) -> Tuple[float, float, float]:
    """Pulls a target back by standoff_mm along the line from the shoulder
    pivot to that target — so pointing the arm at the result aims it at
    the original target while stopping standoff_mm short, instead of
    reaching all the way to it (e.g. to avoid the arm actually touching
    whatever it's tracking).

    The shoulder pivot (where base_arm attaches, straight above the arm
    frame's origin) is used as the reference point rather than the origin
    itself, since that's the point the arm's own reach is centered on —
    backing off along that line keeps the final segment pointed at the
    target from a closer-to-natural angle than backing off from the floor
    would.

    If standoff_mm is larger than the target's own distance from the
    shoulder, the result is clamped to the shoulder pivot itself (rather
    than overshooting past it to the other side) — inverse_kinematics
    will then correctly reject it as unreachable (too close) if that
    still isn't a valid arm position, instead of silently aiming somewhere
    nonsensical.
    """
    shoulder = np.array([0.0, 0.0, dims.shoulder_height_mm])
    target = np.array(target_mm, dtype=float)
    offset = target - shoulder
    distance = float(np.linalg.norm(offset))
    if distance < 1e-6:
        return tuple(target)  # degenerate: target sits right at the shoulder pivot

    new_distance = max(distance - standoff_mm, 0.0)
    result = shoulder + offset * (new_distance / distance)
    return tuple(result)


def inverse_kinematics_pointing(
    target_mm: Tuple[float, float, float],
    standoff_mm: float,
    theta1_bounds: Tuple[float, float] = (-90.0, 90.0),
    theta2_bounds: Tuple[float, float] = (-90.0, 90.0),
    theta3_bounds: Tuple[float, float] = (-90.0, 90.0),
    dims: ArmDimensions = DEFAULT_DIMENSIONS,
) -> Tuple[float, float, float, float]:
    """Points gripper_arm exactly at the target while stopping standoff_mm
    short of it, instead of just reaching a nearby position.

    Unlike inverse_kinematics_search (which only cares where the
    fingertip ends up and treats theta3 as a free parameter to search
    over), this pins theta3 so gripper_arm — the segment from the wrist
    (midarm_joint) to the fingertip — is parallel to the line from the
    wrist through the target.

    The geometry falls out of gripper_arm's length being fixed
    (dims.gripper_arm_mm): if the wrist ends up exactly
    (gripper_arm_mm + standoff_mm) from the target, *along the straight
    line from the shoulder to the target*, then simply pointing
    gripper_arm at the target from there automatically leaves the
    fingertip standoff_mm short — no search needed:
      1. Place the wrist with apply_standoff(target, gripper_arm_mm +
         standoff_mm) — reusing the same helper as the fingertip-only
         case, just with a bigger pull-back distance.
      2. Solve the ordinary 2-link IK (base_arm, mid_arm alone) to put
         the wrist exactly there.
      3. Because the wrist sits on the shoulder-target line by
         construction, "point gripper_arm at the target" is the same as
         "point it along that same line" — so theta3 is just whatever
         angle closes the gap between mid_arm's own direction
         (theta1+theta2) and that line's direction.

    Tries elbow_up=True first, then elbow_up=False, returning the first
    combination where theta1/theta2/theta3 all fall within the given
    bounds (pass the real calibrated ones — this module doesn't read
    calibration_data/servos.json itself, see the package README).

    Raises ValueError if the target can't be pointed at within bounds
    either way (including if it's out of reach entirely).
    """
    x, y, z = target_mm
    theta0 = math.degrees(math.atan2(y, x))

    r = math.hypot(x, y)
    z_rel = z - dims.shoulder_height_mm
    alpha_deg = math.degrees(math.atan2(r, z_rel))  # direction shoulder->target, from vertical

    # The wrist lands on the shoulder-target line by construction of
    # apply_standoff, so pointing gripper_arm at the target from there is
    # equivalent to pointing it along this same alpha direction.
    wrist_target = apply_standoff(target_mm, dims.gripper_arm_mm + standoff_mm, dims)
    wx, wy, wz = wrist_target
    distance_w = math.hypot(math.hypot(wx, wy), wz - dims.shoulder_height_mm)

    l1 = dims.base_arm_mm
    l2 = dims.mid_arm_mm
    if distance_w > l1 + l2 or distance_w < abs(l1 - l2):
        raise ValueError(
            f"Target unreachable while pointing at it with a {standoff_mm:.0f}mm standoff: "
            f"the wrist would need to be {distance_w:.1f}mm from the shoulder, but "
            f"base_arm+mid_arm only spans {abs(l1 - l2):.1f}-{l1 + l2:.1f}mm."
        )

    cos_theta2 = (distance_w**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2_mag = math.degrees(math.acos(cos_theta2))

    cos_beta = (l1**2 + distance_w**2 - l2**2) / (2 * l1 * distance_w)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta_deg = math.degrees(math.acos(cos_beta))

    last_error: Optional[ValueError] = None
    for elbow_up in (True, False):
        theta2 = theta2_mag if elbow_up else -theta2_mag
        theta1 = (alpha_deg - beta_deg) if elbow_up else (alpha_deg + beta_deg)
        theta3 = alpha_deg - theta1 - theta2
        if (
            theta1_bounds[0] <= theta1 <= theta1_bounds[1]
            and theta2_bounds[0] <= theta2 <= theta2_bounds[1]
            and theta3_bounds[0] <= theta3 <= theta3_bounds[1]
        ):
            return theta0, theta1, theta2, theta3
        last_error = ValueError(
            f"elbow_up={elbow_up}: angles {theta1:.1f}/{theta2:.1f}/{theta3:.1f} deg out of bounds "
            f"[{theta1_bounds}], [{theta2_bounds}], [{theta3_bounds}]."
        )

    raise last_error
