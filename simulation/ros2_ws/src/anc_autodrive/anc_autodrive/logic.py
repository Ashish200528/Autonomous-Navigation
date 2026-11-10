"""Pure decision logic shared by simulation and the Raspberry Pi (no ROS imports).

Image inputs are BGR uint8 arrays. Lane convention: right-hand traffic, so the
car's lane is bounded by the yellow centre line on its LEFT and the white edge
line on its RIGHT.
"""
import math

import cv2
import numpy as np

# Thresholds taken from object_detection_new_babu.py.
FLAG_DISTANCE_M = 0.30   # emergency stop
WARN_DISTANCE_M = 1.20   # slow down


def line_masks(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (19, 80, 100), (40, 255, 255))
    white = cv2.inRange(hsv, (0, 0, 175), (180, 45, 255))
    return yellow, white


def white_runs(row):
    """(start, end) pixel spans of consecutive non-zero pixels in a mask row."""
    idx = np.flatnonzero(row)
    if not idx.size:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.r_[idx[0], idx[breaks + 1]]
    ends = np.r_[idx[breaks], idx[-1]]
    return list(zip(starts.tolist(), ends.tolist()))


def is_zebra_row(white_row):
    """3+ separate white bars in the middle 60% of the row. Curve kerbs (also striped)
    sit at the image edge, a crossing spans the road centre."""
    w = white_row.shape[0]
    return len(white_runs(white_row[int(0.2 * w):int(0.8 * w)])) >= 3


def crossing_fraction(bgr, roi=(0.55, 0.92), step=4):
    """Share of lane-band rows that look like a zebra crossing."""
    h = bgr.shape[0]
    _, white = line_masks(bgr)
    rows = range(int(h * roi[0]), int(h * roi[1]), step)
    return sum(is_zebra_row(white[r]) for r in rows) / max(1, len(rows))


def find_lane(bgr, half_lane_px=None, roi=(0.55, 0.92), step=4):
    """Locate the lane centre in the lower band of the image.

    Returns (centre_x, yellow_x, white_x, half_lane_px). Any of the x values may be
    None. half_lane_px is the measured half lane width when both lines are seen,
    otherwise the value passed in (used to infer the centre from one line).
    """
    h, w = bgr.shape[:2]
    yellow, white = line_masks(bgr)
    ys, ws = [], []
    for r in range(int(h * roi[0]), int(h * roi[1]), step):
        if is_zebra_row(white[r]):
            continue  # zebra crossing row: bars, not lane lines
        yx = np.flatnonzero(yellow[r])
        y_x = float(yx.mean()) if yx.size >= 2 else None
        start = int(y_x + 4) if y_x is not None else int(w * 0.45)
        if y_x is not None:
            ys.append(y_x)
        runs = [(a + start, b + start) for a, b in white_runs(white[r, start:])]
        if half_lane_px is not None:
            # A stop line spans most of the lane; an edge line is narrow.
            runs = [(a, b) for a, b in runs if b - a < 1.2 * half_lane_px]
            if y_x is not None:  # and the edge line sits about one lane width from the yellow
                runs = [(a, b) for a, b in runs if abs((a - y_x) - 2 * half_lane_px) < 0.7 * half_lane_px]
        if runs:
            ws.append(float(runs[0][0]))
    y_x = float(np.median(ys)) if len(ys) >= 2 else None
    w_x = float(np.median(ws)) if len(ws) >= 2 else None
    if half_lane_px is None:
        half_lane_px = w * 0.35
    if y_x is not None and w_x is not None and w_x > y_x:
        half_lane_px = (w_x - y_x) / 2.0
        return (y_x + w_x) / 2.0, y_x, w_x, half_lane_px
    if y_x is not None:
        return y_x + half_lane_px, y_x, None, half_lane_px
    if w_x is not None:
        return w_x - half_lane_px, None, w_x, half_lane_px
    return None, None, None, half_lane_px


def lane_error(centre_x, width):
    """Normalised lateral error in [-1, 1]; positive = lane centre is to the right."""
    return max(-1.0, min(1.0, (centre_x - width / 2.0) / (width / 2.0)))


def obstacle_fraction(bgr, roi=(0.35, 1.0)):
    """Fraction of the lower image that looks like a coloured obstacle.

    Road (grey), lane paint (white/yellow), grass (green) and sky are ignored.
    ponytail: colour heuristic, misses grey/green obstacles; YOLO detections are
    OR-ed in by the caller. Replace with a depth/segmentation model if needed.
    """
    h = bgr.shape[0]
    hsv = cv2.cvtColor(bgr[int(h * roi[0]):int(h * roi[1])], cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    coloured = (sat > 90) & (val > 60)
    grass = (hue >= 35) & (hue <= 90)
    paint = (hue >= 19) & (hue < 35)
    return float(np.count_nonzero(coloured & ~grass & ~paint)) / max(1, hue.size)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def overtake_plan(base_yaw, side=1, shift_m=0.5, angle=0.7, pass_m=1.3):
    """Steps to change lane, pass the obstacle and come back.

    side=+1 overtakes on the left. "pass" follows the other lane with the camera
    (odometry heading as fallback). Each step is ("turn", target_yaw), ("pass", ...) or
    ("forward", metres, hold_yaw).
    """
    diag = shift_m / math.sin(angle)
    out_yaw = wrap(base_yaw + side * angle)
    back_yaw = wrap(base_yaw - side * angle)
    return [
        ("turn", out_yaw), ("forward", diag, out_yaw),
        ("turn", base_yaw), ("pass", pass_m, base_yaw),
        ("turn", back_yaw), ("forward", diag, back_yaw),
        ("turn", base_yaw),
    ]
