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
either up or down. `inverse_kinematics` resolves it by locking channel 3
(`midarm_joint`) to 0° — `gripper_arm` stays in line with `mid_arm` —
which turns the rest into a standard 2-link planar arm, solved with the
law of cosines. `elbow_up=False` selects the other of the two remaining
solutions, if the default one isn't reachable or would collide with
something.

Raises `ValueError` if the target is out of reach (too far, or too close
to fold into).

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
straight up) and a 2000-sample randomized FK → IK → FK round-trip: every
sample reached its target to floating-point precision (max error
`0.000000mm`), with both `elbow_up` branches exercised.
