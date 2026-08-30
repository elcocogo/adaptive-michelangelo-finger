# Servo calibration and control (PCA9685)

Three command-line tools for working with the servos wired to the PCA9685:

- **`calibrate_servo.py`**: run once per servo, right after mounting it on
  the arm, to find its center pulse (0°) and min/max bounds by hand.
- **`move_servo.py`**: run afterwards, as often as needed, to drive an
  already-calibrated servo directly to a given angle in degrees.
- **`arm_show.py`**: once all 5 servos are calibrated, a small demo
  choreography that chains poses across the whole arm.

All three rely on the same driver ([pca9685_driver.py](pca9685_driver.py))
and the same shared calibration ([calibration.py](calibration.py)), stored
in `calibration_data/servos.json` at the project root.

## Prerequisites

- PCA9685 wired (I2C on the GPIO header + a separate external supply on
  the V+ block) and detected:
  ```bash
  i2cdetect -y 1   # should show 40 on the 40 row
  ```
- Virtual environment activated:
  ```bash
  cd ~/michelangelo && source .venv/bin/activate
  ```

## Calibration file

`calibration_data/servos.json` is a single JSON file, shared by all
servos, indexed by PCA9685 channel number:

```json
{
  "pwm_frequency_hz": 50,
  "servos": {
    "0": {
      "channel": 0,
      "name": "shoulder",
      "pulse_min_us": 900.0,
      "pulse_center_us": 1500.0,
      "pulse_max_us": 2100.0,
      "angle_min_deg": -90.0,
      "angle_max_deg": 90.0,
      "calibrated_at": "2026-08-26T12:00:00+00:00"
    }
  }
}
```

Each calibration save only touches its own channel's entry: calibrating
or recalibrating a servo never risks overwriting the others.

`move_servo.py` also maintains a second file,
`calibration_data/servo_positions.json`, which tracks the last
*commanded* angle per channel (see the dedicated section below) — no need
to touch it by hand, it's managed automatically.

## 1. `calibrate_servo.py` — interactive calibration

Do this once per servo (or to recalibrate one already mounted).

```bash
python3 -m servo_calibration.calibrate_servo --channel 0 --name shoulder
```

| Argument | Description |
|---|---|
| `--channel` | PCA9685 channel of the servo (0-15), required |
| `--name` | Servo name (e.g. `shoulder`, `elbow`). Optional if the channel already has a calibration — reuses its name |
| `--file` | Calibration file to use (default: `calibration_data/servos.json`) |
| `--frequency` | PWM frequency in Hz (default: 50) |

**Keyboard controls (single keypress, no Enter needed):**

| Key | Action |
|---|---|
| `h` / `l` | decrease / increase the pulse by one step (10µs by default) |
| `H` / `L` | same, step x10 |
| `[` / `]` | halve / double the step size |
| `c` | marks the current position as center (0°) |
| `n` | marks the current position as the min bound (asks for the angle, e.g. `-90`) |
| `x` | marks the current position as the max bound (asks for the angle, e.g. `90`) |
| `p` | prints the current status |
| `r` | cuts the PWM (releases the servo) |
| `s` | saves to the calibration file |
| `?` | shows the help again |
| `q` | quits (asks for confirmation if unsaved) |

**Procedure, for every newly wired servo:**

1. Plug the servo into a free PCA9685 channel, note the channel and which
   joint it represents.
2. Launch the tool with that channel and an explicit name.
3. Gently bring the servo (`h`/`l`, `H`/`L`, adjust the step with `[`/`]`)
   to the position you want for 0°, then `c`.
4. Continue to the mechanical limit you've chosen in one direction —
   **stopping short of the hard stop**, never against it — then `n` and
   the corresponding angle.
5. Same thing in the other direction, then `x`.
6. Check with `p` that the 3 pulses and 2 angles are consistent.
7. Save with `s`.
8. `r` to release the servo (useful for handling the arm afterwards),
   then `q` to quit.

Whatever happens, the pulse sent is always clamped to `[400, 2600]` µs
(`HARD_PULSE_MIN_US`/`HARD_PULSE_MAX_US` in `calibrate_servo.py`) — a
software safeguard that doesn't replace staying alert to the arm's real
mechanical limits during calibration.

## 2. `move_servo.py` — angle-based control

Once a servo is calibrated, use this to position it directly at a given
angle (e.g. after a restart, to test a pose, or to reset a servo to 0°
before continuing to build the arm).

```bash
python3 -m servo_calibration.move_servo --channel 0
# or
python3 -m servo_calibration.move_servo --name shoulder
# or, with no argument, it asks for the channel or name at startup
python3 -m servo_calibration.move_servo
```

| Argument | Description |
|---|---|
| `--channel` | PCA9685 channel of the servo to control |
| `--name` | Servo name (as saved during calibration) |
| `--file` | Calibration file to use (default: `calibration_data/servos.json`) |
| `--speed` | Movement speed as % of the servo's assumed max speed, 10 to 100 (default: 70) |

`--channel` and `--name` are mutually exclusive; if neither is given, the
tool asks interactively at startup.

**Speed limiting (`--speed`)**: the PCA9685 only sets a pulse width, it
has no native control over the servo's speed — a servo commanded
straight from min to max therefore accelerates at the full extent of its
own capabilities, which can be rough on a still-fragile arm.
`move_servo.py` compensates by sending intermediate positions (a ramp),
at 50 updates per second, between the current angle and the requested
one. `--speed 100` corresponds to an assumed full-scale speed
(`MAX_SERVO_SPEED_DEG_PER_S` in the script, 300°/s by default — a generic
estimate for a hobby servo, adjust if needed), `--speed 10` moves 10x
slower.

Since control is open-loop (no real position feedback), the ramp needs to
know the starting angle. The PCA9685 keeps driving a channel at its last
commanded pulse even after the script exits — so `move_servo.py` remembers
the last commanded angle in `calibration_data/servo_positions.json` and
reloads it on the next launch. Result: `--speed` applies from the very
first command of a fresh invocation, as long as the servo hasn't been
released (`r`) or moved by hand in the meantime — which is the most
common use case (one command, then quit).

Quitting with `q` **does not cut the PWM**: the servo keeps its position.
Only `r` releases the servo, and also clears the saved position since
it's no longer reliable afterwards (the arm may have been moved by hand).

**Controls:**

| Input | Action |
|---|---|
| a number (e.g. `12.5`, `-30`) | moves the servo to that angle in degrees |
| `c` | goes to center (0°) |
| `n` | goes to the calibrated min bound |
| `x` | goes to the calibrated max bound |
| `r` | cuts the PWM (releases the servo), forgets the saved position |
| `q` | quits — the servo keeps its position (PWM still active) |

An angle outside `[angle_min_deg, angle_max_deg]` is **rejected with an
error message**, without moving the servo:

```
> 200
  Erreur : 200.0 deg hors bornes [-90.0, 90.0] pour 'shoulder'.
```

The script never moves the servo automatically at startup — the first
command is always explicit.

## 3. `arm_show.py` — demo choreography

Once all 5 channels (0 to 4) are calibrated, automatically runs through a
series of poses across the whole arm: full extension, compact fold, 180°
base rotation, elbow and wrist bending, gripper opening/closing, then a
few combined movements (several joints at once).

```bash
python3 -m servo_calibration.arm_show
# adjustable speed and pause between poses:
python3 -m servo_calibration.arm_show --speed 40 --pause 1.0
```

| Argument | Description |
|---|---|
| `--file` | Calibration file to use (default: `calibration_data/servos.json`) |
| `--speed` | Movement speed as % of the servo's assumed max speed, 10 to 100 (default: 60) |
| `--pause` | Pause in seconds between two poses (default: 0.6) |

Each pose only lists the channels it changes; the others stay wherever
they were. When a pose moves several channels at once, the movement is
**synchronized**: the channel with the largest angle to cover sets the
pace (at `--speed`), the other channels in that same pose are interpolated
over the same duration so they all arrive together instead of finishing
in a staggered sequence.

Channels 1/2/3 (`base_arm`/`mid_arm`/`gripper_arm`) follow an angle
convention *relative to the parent segment* (see the initial
calibration): `mid_arm=0°` doesn't mean it points up, it means it
continues straight in whatever direction `base_arm` is pointing.

Like `move_servo.py`, the script reuses
`calibration_data/servo_positions.json` to know the real position at
startup, and never releases the arm automatically (`Ctrl+C` cleanly
interrupts the show without leaving the arm in an inconsistent state —
it keeps its last pose).

## Safety

- Servo power (PCA9685's V+ block) always kept separate from the Pi's 5V,
  on a dedicated external supply.
- The pulse sent is always hard-clamped in software (`calibrate_servo.py`),
  but that doesn't excuse staying alert to the arm's real mechanical
  limits during a calibration.
- `r` (release / cut the PWM) is available in both interactive tools —
  use it before handling the arm by hand.
- In `move_servo.py` and `arm_show.py`, quitting (`q`, normal end, or
  `Ctrl+C`) **does not release** the servos: they stay actively held at
  their last commanded position. This is intentional (these tools exist
  precisely to set a position and keep it), but it means a servo can
  remain powered/under torque after the script ends — remember `r` if you
  want to release it explicitly.
