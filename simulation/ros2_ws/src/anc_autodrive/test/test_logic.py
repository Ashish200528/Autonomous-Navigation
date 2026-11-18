import math

import numpy as np

from anc_autodrive.logic import crossing_fraction, find_lane, lane_error, obstacle_fraction, overtake_plan

ASPHALT, WHITE, YELLOW, ORANGE = (62, 62, 66), (235, 235, 235), (20, 200, 235), (10, 90, 240)


def road(yellow_x=None, white_x=None, w=320, h=240):
    img = np.full((h, w, 3), ASPHALT, np.uint8)
    if yellow_x is not None:
        img[:, yellow_x - 3:yellow_x + 3] = YELLOW
    if white_x is not None:
        img[:, white_x - 3:white_x + 3] = WHITE
    return img


def test_both_lines_give_midpoint():
    c, y, wx, half = find_lane(road(100, 260))
    assert abs(y - 100) < 2 and abs(wx - 258) < 3
    assert abs(c - 179) < 3 and abs(half - 79) < 3


def test_single_line_uses_remembered_width():
    c, _, _, _ = find_lane(road(None, 260), half_lane_px=80)
    assert abs(c - 178) < 3
    c, _, _, _ = find_lane(road(100, None), half_lane_px=80)
    assert abs(c - 180) < 3


def test_zebra_bars_do_not_pull_the_lane():
    img = road(100, 260)
    img[150:220, 120:140] = WHITE  # a crossing bar right next to the yellow line
    c, _, wx, _ = find_lane(img, half_lane_px=80)
    assert abs(wx - 258) < 3 and abs(c - 179) < 3


def test_full_zebra_band_is_a_crossing_not_a_lane():
    img = road(None, None)
    for x in range(20, 320, 50):
        img[130:230, x:x + 25] = WHITE
    assert find_lane(img, half_lane_px=80)[0] is None
    assert crossing_fraction(img) > 0.5 and crossing_fraction(road(100, 260)) == 0


def test_stop_line_is_ignored():
    img = road(100, 260)
    img[170:185, 104:258] = WHITE  # transverse stop line across the lane
    c, _, wx, _ = find_lane(img, half_lane_px=80)
    assert abs(wx - 258) < 3 and abs(c - 179) < 3


def test_no_lines():
    assert find_lane(road())[0] is None


def test_error_sign():
    assert lane_error(240, 320) > 0 > lane_error(80, 320)


def test_obstacle_fraction():
    img = road(100, 260)
    assert obstacle_fraction(img) < 0.01  # paint is not an obstacle
    img[150:240, 120:200] = ORANGE
    assert obstacle_fraction(img) > 0.05


def test_overtake_returns_to_base_heading():
    plan = overtake_plan(0.0)
    assert plan[0] == ("turn", 0.7) and plan[-1] == ("turn", 0.0)
    lateral = sum(s[1] * math.sin(s[2]) for s in plan if s[0] == "forward")
    assert abs(lateral) < 1e-9
