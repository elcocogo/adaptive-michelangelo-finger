#!/usr/bin/env python3
"""Generates print-ready calibration targets: a ChArUco board (for stereo
calibration, and later camera<->arm calibration) and a sheet of standalone
ArUco markers (for tracking a single point later, e.g. on the gripper).

Why ChArUco rather than a plain checkerboard: OpenCV can locate the board's
corners even when part of it is out of frame or occluded, because each
square carries a uniquely-identifiable ArUco tag — a plain checkerboard
requires the whole pattern to be visible to detect anything at all. That
robustness matters here because the two cameras don't see identical crops
of the scene.

Output is a PDF with the board's real-world size embedded in the file (via
DPI), not just a pixel image — printed at "100% / actual size" (NOT
"fit to page", which would silently rescale it), the printed squares will
measure exactly SQUARE_LENGTH_MM. That physical accuracy is what the
calibration math relies on: cv2.stereoCalibrate is told the square size in
meters and computes real-world camera positions from it, so if the actual
print is scaled, every distance the whole vision pipeline ever computes is
scaled by the same wrong factor.

Usage:
    python3 -m camera_calibration.generate_targets
"""

from __future__ import annotations

from pathlib import Path

import cv2.aruco as aruco
from PIL import Image, ImageDraw, ImageFont

OUTPUT_DIR = Path(__file__).resolve().parent / "print_targets"

# ChArUco board: physical size chosen for a small desktop arm (~5cm links,
# so a small working volume) — big enough to see clearly, small enough to
# fully fit in frame at typical close range.
SQUARES_X, SQUARES_Y = 7, 5
SQUARE_LENGTH_MM = 25.0
MARKER_LENGTH_MM = 18.0  # must be smaller than the square; ~0.7x is typical
ARUCO_DICT = aruco.DICT_4X4_50  # 50 unique IDs is ample for a 7x5 board (~18 markers used)

# Standalone markers sheet: for tracking a single fixed point later on
# (e.g. glued to the gripper for camera<->arm calibration), not for the
# board itself, so they're printed separately and bigger.
STANDALONE_MARKER_IDS = [0, 1, 2, 3]
STANDALONE_MARKER_MM = 25.0

PRINT_DPI = 300
MARGIN_MM = 15.0


def mm_to_px(mm: float, dpi: int = PRINT_DPI) -> int:
    return round(mm / 25.4 * dpi)


def make_charuco_board_image() -> Image.Image:
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    board = aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM / 1000.0,  # CharucoBoard wants meters, not mm
        MARKER_LENGTH_MM / 1000.0,
        dictionary,
    )
    board_w_px = mm_to_px(SQUARES_X * SQUARE_LENGTH_MM)
    board_h_px = mm_to_px(SQUARES_Y * SQUARE_LENGTH_MM)
    board_img = board.generateImage((board_w_px, board_h_px), marginSize=0)
    return _add_caption(
        Image.fromarray(board_img),
        f"ChArUco {SQUARES_X}x{SQUARES_Y} - square {SQUARE_LENGTH_MM:.0f}mm - "
        f"PRINT AT 100% (not 'fit to page')",
    )


def make_standalone_markers_image() -> Image.Image:
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    marker_px = mm_to_px(STANDALONE_MARKER_MM)
    gap_px = mm_to_px(15.0)
    label_h_px = 40

    n = len(STANDALONE_MARKER_IDS)
    sheet = Image.new("L", (n * marker_px + (n + 1) * gap_px, marker_px + label_h_px + gap_px), 255)
    draw = ImageDraw.Draw(sheet)
    for i, marker_id in enumerate(STANDALONE_MARKER_IDS):
        marker_img = aruco.generateImageMarker(dictionary, marker_id, marker_px)
        x = gap_px + i * (marker_px + gap_px)
        sheet.paste(Image.fromarray(marker_img), (x, gap_px // 2))
        draw.text((x, marker_px + gap_px // 2 + 5), f"ID {marker_id}", fill=0)

    return _add_caption(
        sheet,
        f"{n} standalone ArUco tags - {STANDALONE_MARKER_MM:.0f}mm - PRINT AT 100%",
    )


def _add_caption(img: Image.Image, text: str) -> Image.Image:
    """Adds a margin with a human-readable caption, so the physical size and
    the 'print at 100%' instruction travel with the image to the print shop."""
    margin_px = mm_to_px(MARGIN_MM)
    caption_h_px = 60
    canvas = Image.new("L", (img.width + 2 * margin_px, img.height + margin_px + caption_h_px), 255)
    canvas.paste(img, (margin_px, margin_px // 2))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 28)
    except OSError:
        font = ImageFont.load_default()
    draw.text((margin_px, img.height + margin_px), text, fill=0, font=font)
    return canvas


def save_as_pdf(img: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    img.convert("RGB").save(path, "PDF", resolution=PRINT_DPI)


def main() -> None:
    board_img = make_charuco_board_image()
    save_as_pdf(board_img, OUTPUT_DIR / "charuco_board.pdf")
    board_img.save(OUTPUT_DIR / "charuco_board_preview.png")

    markers_img = make_standalone_markers_image()
    save_as_pdf(markers_img, OUTPUT_DIR / "standalone_markers.pdf")
    markers_img.save(OUTPUT_DIR / "standalone_markers_preview.png")

    print(f"Generated in {OUTPUT_DIR}/:")
    print("  charuco_board.pdf          (print this for the stereo calibration)")
    print("  standalone_markers.pdf     (individual tags, for later)")
    print("  *_preview.png              (quick preview, not for printing)")


if __name__ == "__main__":
    main()
