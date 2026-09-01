# License — 3D models in this folder

The mechanical design in this folder is derived from **grippy-bot** by
**ROBOTEURS**, originally published on Cults3D:
<https://cults3d.com/en/3d-model/gadget/grippy-bot/comments>

Licensed under **CC BY-NC** (Creative Commons Attribution-NonCommercial —
see <https://creativecommons.org/licenses/by-nc/4.0/> for the full terms).
Non-commercial use only, with attribution to the original author.

## What was changed

Every part was reworked in FreeCAD to fit the dimensions of **MG90S**
servo motors. The
FreeCAD source files (`.FCStd`) are included alongside the exported
`.step` files, so anyone wanting to adapt this design further — for a
different servo, a different scale, whatever — can start from the
editable source instead of reverse-engineering the STEP geometry.

## Files

| File | Part |
|---|---|
| `Base.FCStd` / `.step` | Fixed base |
| `BaseSpin.FCStd` / `.step` | Rotating base (`base_joint`) |
| `BaseArm.FCStd` / `.step` | Base arm / shoulder segment (`spin_joint`) |
| `MidArm.FCStd` / `.step` | Mid arm / elbow segment (`basearm_joint`) |
| `GripperArm.FCStd` / `.step` | Gripper arm / wrist segment (`midarm_joint`) |
| `Finger.FCStd` / `.step` | Gripper finger (`gripper_joint`) |
