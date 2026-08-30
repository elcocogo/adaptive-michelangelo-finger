# Visual tracking loop

`follow_target.py` ties together everything built so far: detects an
ArUco tag (default ID 0) in both cameras, triangulates its 3D position,
converts it into the arm's frame, solves inverse kinematics
(`inverse_kinematics_search`, which also searches for a wrist angle that
keeps every joint within its calibrated limits — see
`kinematics/README.md`), and drives the arm to point at it —
continuously, until you quit. The arm stops short of the target by a
configurable standoff (100mm by default) rather than reaching all the
way to it, so it points at whatever it's tracking without touching it
(see `kinematics.arm_kinematics.apply_standoff`).

```bash
cd ~/michelangelo && source .venv/bin/activate
python3 -m tracking.follow_target
# track a different tag, a different speed, a tighter/looser standoff:
python3 -m tracking.follow_target --target-id 0 --speed 40 --standoff-mm 50
```

Open `http://<pi-ip>:8100/` for a live preview (same MJPEG-over-HTTP
approach as the other camera tools, `rpi502` has no display) — detected
tags are outlined, the tracked one gets a red dot on its center. In the
terminal, `q` (or `Ctrl+C`) quits, which stops the loop and returns the
arm to its home position (vertical, all angles 0°, via
`servo_calibration/go_home.py`) before leaving PWM active (same
no-auto-release behavior as `move_servo.py`/`arm_show.py`).

| Argument | Description |
|---|---|
| `--target-id` | ArUco tag ID to track (default 0) |
| `--speed` | Movement speed as % of the servo's assumed max speed, 10-100 (default 50) |
| `--port` | HTTP port for the preview (default 8100) |
| `--standoff-mm` | How far short of the target the arm stops, in mm (default 100) |

The standoff is applied along the line from the shoulder pivot
(`spin_joint`) to the target, so the arm keeps pointing at the target
from a point closer to itself rather than drifting off at an angle. If
the standoff is larger than the target's own distance from the shoulder,
the aim point clamps to the shoulder pivot instead of overshooting past
it — `inverse_kinematics` then correctly rejects that as unreachable
(logged as "Target out of reach") rather than doing something
nonsensical.

## What it does *not* do (yet)

- **The arm's own tip position isn't visually verified.** `inverse_kinematics`
  is trusted open-loop: the commanded angles are assumed to put the tip
  exactly where the math says. A planned refinement is to tape a second
  ArUco tag on the arm itself and use it to check/correct for the arm's
  real mechanical precision, since it isn't perfectly rigid — not
  implemented yet.
- **The target is a tag, not a bare finger.** Using a marker was a
  deliberate simplification to validate the full geometric pipeline
  (triangulation -> arm frame -> IK -> movement) against a detector
  that's already proven reliable, before tackling the harder, less
  deterministic problem of detecting an actual fingertip.

## Prerequisites

- `calibration_data/stereo_calibration.json` (`calibrate_stereo.py`) and
  `calibration_data/camera_to_arm.json` (`calibrate_camera_to_arm.py`)
  both already computed.
- All 5 servo channels calibrated (`calibrate_servo.py`).
- The target tag printed at its measured size (see
  `camera_calibration/README.md` — `charuco_board.py`'s
  `MEASURED_SQUARE_LENGTH_MM`, reused for standalone tags too).
