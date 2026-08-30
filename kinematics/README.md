# Arm kinematics

`arm_kinematics.py`: forward and inverse kinematics for the 5-DOF arm,
in the arm frame established by `camera_calibration/calibrate_camera_to_arm.py`
(origin below `base_joint`'s rotation axis, X = the direction the arm
faces at `base_joint = 0`, Z up). Pure geometry — no hardware access, no
dependency on the rest of the project.

```python
from kinematics.arm_kinematics import forward_kinematics, inverse_kinematics

x, y, z = forward_kinematics(theta0, theta1, theta2, theta3)  # degrees -> mm
theta0, theta1, theta2, theta3 = inverse_kinematics(x, y, z)  # mm -> degrees
```

## Why inverse kinematics needs an extra assumption

Channels 1-3 (`spin_joint`/`basearm_joint`/`midarm_joint`) share parallel
axes, so once the base has yawed (`base_joint`), the rest of the chain
moves in a single vertical plane — reducing the problem to a textbook
planar arm plus a base rotation.

That planar sub-chain has 3 joints reaching a 2D point in that plane: one
more degree of freedom than needed, the same redundancy your own elbow
has when your hand is somewhere your shoulder could reach with the elbow
either up or down. `inverse_kinematics` resolves it by fixing the wrist
bend (`midarm_joint`, `theta3_deg`, 0° by default — `gripper_arm` stays
in line with `mid_arm`), which turns the rest into a standard 2-link
planar arm (link 1 = `base_arm`, link 2 = the straight-line span from
elbow to fingertip at that wrist angle), solved with the law of cosines.
`elbow_up=False` selects the other of the two remaining solutions
(`mid_arm` bending the other way), if the default one isn't reachable or
would collide with something.

Raises `ValueError` if the target is out of reach *at that particular
wrist angle* — a target can be genuinely beyond the arm's total reach, or
merely unreachable while the wrist is held at that specific angle.

### Keeping every joint within its calibrated limits: `inverse_kinematics_search`

Fixing the wrist at 0° treats `mid_arm`+`gripper_arm` as one rigid 90mm
segment. For some targets, reaching them that way needs an elbow bend
(`theta2`) beyond what `basearm_joint` is actually calibrated for — even
though the *same target* is reachable within every joint's limits by
also bending the wrist a bit (confirmed on the real arm: a target
requiring a 114° elbow bend at `theta3=0` turned out perfectly reachable
with `theta1=29°, theta2=88°, theta3=45°`, all in bounds).

```python
from kinematics.arm_kinematics import inverse_kinematics_search

theta0, theta1, theta2, theta3 = inverse_kinematics_search(
    x, y, z,
    theta1_bounds=(spin_joint.angle_min_deg, spin_joint.angle_max_deg),
    theta2_bounds=(basearm_joint.angle_min_deg, basearm_joint.angle_max_deg),
    theta3_bounds=(midarm_joint.angle_min_deg, midarm_joint.angle_max_deg),
)
```

Tries wrist-angle candidates across `theta3_bounds` (closest to straight
first) and returns the first one where `theta1` and `theta2` also land in
their given bounds, raising `ValueError` if none do. This is what
`tracking/follow_target.py` actually calls — plain `inverse_kinematics`
is the building block underneath it. The module doesn't read
`calibration_data/servos.json` itself (kept dependency-free, see above);
pass the real calibrated bounds in from the caller.

## Stopping short of a target: `apply_standoff`

```python
from kinematics.arm_kinematics import apply_standoff

aim_point = apply_standoff(target_mm, standoff_mm=100.0)
theta0, theta1, theta2, theta3 = inverse_kinematics_search(*aim_point, ...)
```

Pulls a target back by `standoff_mm` along the line from the shoulder
pivot to that target, so aiming at the result points the arm at the
original target while stopping short of it — used by
`tracking/follow_target.py` so the arm doesn't touch whatever it's
tracking. If the standoff is bigger than the target's own distance from
the shoulder, the result clamps to the shoulder pivot rather than
overshooting past it, and `inverse_kinematics` then naturally rejects it
as unreachable.

## What isn't checked here

`inverse_kinematics` returns a geometrically valid answer, but doesn't
check it against each channel's *calibrated* angle limits
(`calibration_data/servos.json`) — an extreme target can legitimately
call for an angle beyond what a given servo was calibrated to reach.
`move_to_angle` (`servo_calibration/move_servo.py`) already rejects
out-of-bounds angles with a clear error when you actually command the
arm, so this is caught before anything moves — just don't assume an
`inverse_kinematics` result is automatically drivable.

## Dimensions

Measured on the physical arm (`ArmDimensions` in `arm_kinematics.py`,
overridable per call):

| From | To | Length |
|---|---|---|
| `base_joint` floor pivot | `spin_joint` (shoulder) pivot | 45mm (height) |
| `spin_joint` pivot | `basearm_joint` (elbow) pivot | 54mm (`base_arm`) |
| `basearm_joint` pivot | `midarm_joint` (wrist) pivot | 50mm (`mid_arm`) |
| `midarm_joint` pivot | finger pivot | 40mm (`gripper_arm`) |

## Validation

Checked against hand-computed cases (e.g. all-zero angles reaches
`(0, 0, shoulder + base_arm + mid_arm + gripper_arm)` — fully extended
straight up), a 2000-sample randomized FK → IK → FK round-trip at
`theta3=0` (max error `0.000000mm`), and a second 2000-sample round-trip
with `theta3_deg` set to random values instead of 0 (max error
`~1e-10mm`), all with both `elbow_up` branches exercised — confirming the
elbow-to-fingertip geometry (`l2_eff`/`gamma`) generalizes correctly
beyond the `theta3=0` special case. `inverse_kinematics_search` was
separately checked against the real target that first surfaced the
`theta3=0` limitation, confirming it finds an in-bounds solution that FK
confirms reaches the exact same point.
