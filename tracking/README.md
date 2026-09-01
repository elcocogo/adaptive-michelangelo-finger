# Visual tracking loop

`follow_target.py` ties together everything built so far: detects an
ArUco tag (default ID 0) in both cameras, triangulates its 3D position,
converts it into the arm's frame, solves inverse kinematics, and drives
the arm to point at it — continuously, until you quit. The arm stops
short of the target by a configurable standoff (135mm by default) rather
than reaching all the way to it, so it points at whatever it's tracking
without touching it.

Two ways to aim, picked with `--point-gripper`:
- **default** (position only): `apply_standoff` pulls the *fingertip*
  back along the shoulder-target line, then `inverse_kinematics_search`
  picks whichever wrist angle keeps every joint within its calibrated
  bounds (plain `inverse_kinematics` alone can demand an elbow bend
  beyond `basearm_joint`'s limit for some targets that are perfectly
  reachable with a bent wrist).
- **`--point-gripper`**: `inverse_kinematics_pointing` additionally pins
  the wrist angle so `gripper_arm` itself stays parallel to the line from
  the wrist through the target — the arm doesn't just get *near* the
  target, its last segment visibly points at it, still stopping
  `standoff_mm` short. Closed-form (no search): fixed to the shoulder,
  the wrist's required distance from the target is exactly
  `gripper_arm_mm + standoff_mm`, which pins the geometry completely (see
  `kinematics/README.md`).

```bash
cd ~/michelangelo && source .venv/bin/activate
python3 -m tracking.follow_target
# track a different tag, a different speed, a tighter/looser standoff:
python3 -m tracking.follow_target --target-id 0 --speed 40 --standoff-mm 50
# keep gripper_arm pointed at the target, not just nearby:
python3 -m tracking.follow_target --point-gripper
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
| `--standoff-mm` | How far short of the target the arm stops, in mm (default 135) |
| `--point-gripper` | Also keep `gripper_arm` parallel to the wrist-target line (see above) |
| `--record` | Save the same annotated view shown at `/stream` to an `.mp4` in `recordings/` |

`--record` runs its own independent capture+annotate loop rather than
piggybacking on the preview — the preview only draws frames while a
browser is actually connected to `/stream`, but recording should keep
going whether or not anyone's watching. Files are named
`tracking_YYYYMMDD_HHMMSS.mp4`, not versioned (see `.gitignore`).

The standoff is applied along the line from the shoulder pivot
(`spin_joint`) to the target, so the arm keeps pointing at the target
from a point closer to itself rather than drifting off at an angle. If
the standoff is larger than the target's own distance from the shoulder,
the aim point clamps to the shoulder pivot instead of overshooting past
it — `inverse_kinematics` then correctly rejects that as unreachable
(logged as "Target out of reach") rather than doing something
nonsensical.

## Known limitations

- **The arm's own tip position isn't visually verified.** The commanded
  angles are trusted open-loop to put the tip exactly where the math
  says. Taping a second ArUco tag on the arm itself, to check/correct
  for the fact it isn't perfectly rigid, was considered and deliberately
  not pursued — accurate enough in practice.
- **The target is tracked via an ArUco tag, not by detecting the printed
  finger itself.** A deliberate simplification to validate the full
  geometric pipeline (triangulation -> arm frame -> IK -> movement)
  against a detector that's already proven reliable, rather than a
  harder, less deterministic vision problem.

## Prerequisites

- `calibration_data/stereo_calibration.json` (`calibrate_stereo.py`) and
  `calibration_data/camera_to_arm.json` (`calibrate_camera_to_arm.py`)
  both already computed.
- All 5 servo channels calibrated (`calibrate_servo.py`).
- The target tag printed at its measured size (see
  `camera_calibration/README.md` — `charuco_board.py`'s
  `MEASURED_SQUARE_LENGTH_MM`, reused for standalone tags too).
