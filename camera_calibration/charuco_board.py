"""Shared ChArUco board definition, used by both the capture and the
calibration-computation scripts so they always agree on what they're
looking for.

The layout (squares count, dictionary) must match generate_targets.py —
they describe the same physical object, one prints it, the others detect
it. If you ever regenerate the board with different squares/dictionary,
update SQUARES_X/SQUARES_Y/ARUCO_DICT here to match.
"""

from __future__ import annotations

import cv2.aruco as aruco

SQUARES_X, SQUARES_Y = 7, 5
ARUCO_DICT = aruco.DICT_4X4_50

# generate_targets.py's *nominal* design size was 25.0mm/square (18.0mm
# markers). What actually came out of the print shop measures 25.6mm/square
# instead (see conversation history — the print pipeline rescaled it by
# ~2.36%, the PDF itself was verified correct). Calibration accuracy
# depends entirely on this number being the *physically measured* size,
# not the nominal one, so if you ever reprint and remeasure, update this
# single constant — the marker size below is kept proportional to it
# automatically rather than hardcoded separately, so the two can't drift
# out of sync with each other.
MEASURED_SQUARE_LENGTH_MM = 25.6
_NOMINAL_SQUARE_MM = 25.0
_NOMINAL_MARKER_MM = 18.0
MEASURED_MARKER_LENGTH_MM = MEASURED_SQUARE_LENGTH_MM * (_NOMINAL_MARKER_MM / _NOMINAL_SQUARE_MM)


def build_board() -> aruco.CharucoBoard:
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    return aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        MEASURED_SQUARE_LENGTH_MM / 1000.0,  # CharucoBoard wants meters
        MEASURED_MARKER_LENGTH_MM / 1000.0,
        dictionary,
    )


def build_detector() -> aruco.CharucoDetector:
    return aruco.CharucoDetector(build_board())
