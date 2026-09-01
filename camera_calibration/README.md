# Stereo cameras: positioning and calibration

Five tools, in the order you'd use them:

1. **`live_view.py`**: live feed from both cameras to physically position
   them (not an OpenCV calibration — see its own section).
2. **`generate_targets.py`**: generates the printable targets (ChArUco
   board + standalone ArUco tags) for the stereo calibration.
3. **`capture_stereo_images.py`**: captures image pairs of the board from
   both cameras, with live detection to guide the shots.
4. **`calibrate_stereo.py`**: computes the calibration (each camera's
   intrinsics + their relative position) from the captured pairs.
5. **`calibrate_camera_to_arm.py`**: computes the transform between the
   camera pair and the arm's own reference frame, from two floor-mounted
   ArUco markers.

`rpi502` has no desktop — the two tools with a live preview
(`live_view.py`, `capture_stereo_images.py`) serve their feed as MJPEG
over HTTP instead of opening a native window.

## 1. `live_view.py` — physically positioning the cameras

Live feed from both `imx708` sensors, side by side, for positioning the
cameras by hand. Not an OpenCV calibration (no target, nothing is saved
to disk) — only used to check that both cameras keep the arm in frame
through its full range of motion.

```bash
cd ~/michelangelo && source .venv/bin/activate
python3 -m camera_calibration.live_view
```

Then open the printed URL in a browser — `http://<pi-ip>:8100/` (default
port 8100). If you're connected via VS Code Remote-SSH, it usually offers
to forward the port automatically once it detects it open; otherwise use
the Pi's IP directly on the local network (the server listens on all
interfaces).

| Argument | Description |
|---|---|
| `--file` | Calibration file to use (default: `calibration_data/servos.json`) |
| `--port` | HTTP port for the MJPEG stream (default: 8100) |
| `--speed` | Movement speed as % of the servo's assumed max speed, 10 to 100 (default: 40) |
| `--layout` | `side-by-side` (default) or `stacked` |

**Driving the arm while positioning** — the terminal stays available to
move the first two joints (the only ones useful for seeing the arm's
limit positions):

| Input | Action |
|---|---|
| `0 <angle\|c\|n\|x>` | channel 0 — base rotation / azimuth |
| `1 <angle\|c\|n\|x>` | channel 1 — shoulder (vertical at 0°, horizontal at its calibrated max) |
| `q` | quits (stops the stream and the server) |

`c`/`n`/`x` go to center, the calibrated min bound, and the calibrated max
bound respectively. Reuses `move_to_angle` from `move_servo.py`, so the
movement is ramped at `--speed`, and the reached position is persisted to
`calibration_data/servo_positions.json` like the other tools. Quitting
with `q` does not release the arm — same behavior as `move_servo.py` and
`arm_show.py`.

## 2. `generate_targets.py` — printable targets

Generates two PDFs in `print_targets/` (not versioned, regenerable):
- `charuco_board.pdf`: ChArUco board (checkerboard + ArUco tags), for the
  stereo calibration.
- `standalone_markers.pdf`: individual ArUco tags, for later
  (camera-to-arm calibration).

```bash
python3 -m camera_calibration.generate_targets
```

**Important when printing**: print at 100% / actual size, never "fit to
page". Once printed, **measure a square with a ruler** and update
`MEASURED_SQUARE_LENGTH_MM` in `charuco_board.py` with the real value if
it differs from the nominal one (`SQUARE_LENGTH_MM` in
`generate_targets.py`) — that's exactly what happened here: the PDF file
itself is dimensionally correct (verified), but the print pipeline used
rescaled the result by ~2.36% (25.6mm measured instead of 25mm). The
accuracy of the whole calibration depends on this measured value, not the
nominal one — mount the board on a rigid, perfectly flat backing before
measuring.

## 3. `capture_stereo_images.py` — capturing pairs

```bash
python3 -m camera_calibration.capture_stereo_images
# side-by-side (default) or stacked preview:
python3 -m camera_calibration.capture_stereo_images --layout stacked
```

Opens `http://<pi-ip>:8100/` for a live preview with the detected
markers/corners drawn in real time — handy for seeing whether the board
is properly recognized before capturing. In the terminal:

| Input | Action |
|---|---|
| `c` | captures a pair (saved only if both cameras detect at least `MIN_CHARUCO_CORNERS` corners) |
| `q` | quits |

Images are saved to `calibration_captures/cam0/` and `cam1/` (not
versioned), numbered and matched by filename. Restarting the tool resumes
numbering where it left off, so capturing across several sessions is
fine.

Tips for a good calibration: aim for 15-20 valid pairs, vary the board's
distance, tilt, and position in the frame (including near the image's
edges/corners, not just the center), and avoid any motion blur (the board
must be still at the moment of capture).

## 4. `calibrate_stereo.py` — computing the calibration

```bash
python3 -m camera_calibration.calibrate_stereo
```

Detects the board in every captured pair, then computes:
1. each camera's **intrinsics** separately (camera matrix + lens
   distortion) via `cv2.calibrateCamera`;
2. the **relative position** between the two cameras (rotation +
   translation) via `cv2.stereoCalibrate`, keeping the intrinsics fixed
   (`CALIB_FIX_INTRINSIC`).

The script prints the RMS reprojection error (in pixels) at each step —
under ~1px is very good, 1-2px is fine for an amateur setup, beyond that
it's worth recapturing with more varied poses or checking that the board
is properly flat and rigid. The result is saved to
`calibration_data/stereo_calibration.json`.

### What's in `stereo_calibration.json`

| Field | Meaning |
|---|---|
| `image_width`, `image_height` | Resolution the calibration was computed at. Every pixel-based value below only means anything at this exact resolution. |
| `camera_matrix0`, `camera_matrix1` | Each camera's 3x3 intrinsic matrix: `fx`/`fy` (focal length in pixels — derive the field of view with `2·atan(width / (2·fx))`) and `cx`/`cy` (optical center, should sit close to the image center). |
| `dist_coeffs0`, `dist_coeffs1` | Each camera's 5 lens distortion coefficients `[k1, k2, p1, p2, k3]` (radial + tangential, standard OpenCV/Brown-Conrady model). Only meaningful together — a single large-looking coefficient isn't a red flag on its own. |
| `R` | 3x3 rotation matrix from camera 0's frame to camera 1's frame. Close to identity means the two cameras point in nearly the same direction, with little relative rotation. |
| `T` | Translation from camera 0 to camera 1, in **meters**. Its norm is the real baseline between the two cameras — worth sanity-checking against a ruler measurement of the physical rig. |
| `rms_mono0`, `rms_mono1`, `rms_stereo` | The 3 RMS reprojection errors described above, saved for reference. |
| `num_pairs_used` | How many captured pairs actually had enough corners shared by both cameras to contribute to the stereo step. |
| `square_length_mm` | The real, measured square size used for this run (see `generate_targets.py` above) — kept for traceability if the board is ever reprinted or remeasured. |
| `calibrated_at` | Timestamp of the run. |

**What it's for**: `camera_matrix`/`dist_coeffs` undo each camera's lens
distortion for a detected point; `R`/`T` then let you combine the same
point's position in both (undistorted) images into one 3D point via
triangulation (`cv2.triangulatePoints`) — the next step once a target
(the ArUco tag on the tracking wand) can be located in both
views. That 3D point comes out in camera 0's coordinate frame, not the
arm's — converting between the two is the camera-to-arm calibration this
stereo calibration unblocks.

## 5. `calibrate_camera_to_arm.py` — locating the arm in camera space

Computes the rigid transform from camera 0's frame to the arm's own
reference frame: **origin** at the point on the floor directly below the
`base_joint` rotation axis, **X** pointing wherever the arm faces at
`base_joint = 0`, **Z** vertical (up).

Setup: tape two standalone ArUco tags (from `generate_targets.py`) flat
on the floor next to the arm — not on the base itself, since it isn't
flat and can be hidden by the arm depending on its pose. Set
`base_joint` to exactly 0° first (`move_servo.py`), since that's what
fixes which direction counts as X. Then measure each tag's **center**
position with a ruler/tape measure: how far along X (forward) and along
Y (sideways) from the origin. Both tags lie on the floor, so their Z is 0
by construction — you don't need to measure it, and you don't need to
align the tags to any particular rotation either (see below).

```bash
python3 -m camera_calibration.calibrate_camera_to_arm \
    --marker1-id 2 --marker1-x-mm 150 --marker1-y-mm 120 \
    --marker2-id 3 --marker2-x-mm 150 --marker2-y-mm -120 \
    --samples 20
```

| Argument | Description |
|---|---|
| `--marker1-id`, `--marker2-id` | Which two standalone ArUco tag IDs were placed (see `generate_targets.py`) |
| `--marker1-x-mm`/`-y-mm`, `--marker2-x-mm`/`-y-mm` | Each tag's measured center position, in mm, along the arm frame's X/Y axes |
| `--samples` | Frames to average over while both tags sit still (default 20) — averaging cancels out per-frame detection jitter |

Why only two measured positions are enough to fix a full 3D transform
(3 axes + origin, 6 degrees of freedom): each tag's *position* is what
you measure by hand, but its *orientation* doesn't need measuring at all
— a flat tag's own 4 corners define a plane once triangulated, and that
plane's normal is, by construction, straight up out of the floor (the
arm frame's Z axis), independent of how the tag happens to be rotated
where it's taped down. Two floor positions plus that (cross-checked
between both tags) normal fully determine the transform.

The script prints two sanity checks before saving:
- **Floor-normal agreement** between the two tags' independently-detected
  normals — should be a couple of degrees or less; a bigger gap suggests
  a tag isn't flat or the floor isn't level there.
- **Measured vs. triangulated distance** between the two tags — should
  match closely; a large gap suggests a measurement mistake or a tag
  that moved after being measured.

Result saved to `calibration_data/camera_to_arm.json` (rotation,
translation, and both validation numbers, for later reference).

## Prerequisites

- Arm channels 0 and 1 already calibrated (`calibrate_servo.py`), for
  `live_view.py` and `calibrate_camera_to_arm.py` (which sets `base_joint`
  as the X-axis reference).
- ChArUco board printed (see `generate_targets.py` above), mounted on a
  rigid, flat backing, with the real square size measured and recorded in
  `charuco_board.py`.
- `calibration_data/stereo_calibration.json` already computed
  (`calibrate_stereo.py`), for `calibrate_camera_to_arm.py`.
- Dependencies: `picamera2` and `Pillow` come from system packages already
  in place (see `CLAUDE.md`); `opencv-contrib-python` is in
  `requirements.txt` (the `cv2.aruco` module is needed for ChArUco/ArUco).
