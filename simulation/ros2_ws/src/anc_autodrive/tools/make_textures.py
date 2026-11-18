"""Generate the race-track ground texture and the ArUco station sign.

Run once after changing the track geometry (outputs are committed):
    python tools/make_textures.py

Track: a stadium loop centred on the world origin. Straights run along X,
semicircle centres sit at (+-STRAIGHT_HALF, 0). Two 0.6 m lanes, white edge
lines, dashed yellow centre line. Driving counter-clockwise keeps the car in
the OUTER lane (right-hand traffic), whose centre radius is CENTER_R + LANE/2.
"""
import math
from pathlib import Path

import cv2
import numpy as np

PLANE_W, PLANE_H = 16.0, 10.0  # metres; must match <plane><size> in the model
PX_PER_M = 128
STRAIGHT_HALF = 4.0
CENTER_R = 3.0                 # road centre-line radius
LANE = 0.6
LINE = 0.05
DASH = 0.3

GRASS = (58, 125, 62)   # BGR
ASPHALT = (62, 62, 66)
WHITE = (235, 235, 235)
YELLOW = (20, 200, 235)

HERE = Path(__file__).resolve().parent.parent / "models"


def to_px(x, y):
    return (int(round((x + PLANE_W / 2) * PX_PER_M)), int(round((PLANE_H / 2 - y) * PX_PER_M)))


def fill_stadium(img, r, color):
    r_px = int(round(r * PX_PER_M))
    for cx in (-STRAIGHT_HALF, STRAIGHT_HALF):
        cv2.circle(img, to_px(cx, 0), r_px, color, -1, cv2.LINE_AA)
    cv2.rectangle(img, to_px(-STRAIGHT_HALF, r), to_px(STRAIGHT_HALF, -r), color, -1)


def centerline(step=0.01):
    """Yield points along the road centre line, counter-clockwise."""
    straight = 2 * STRAIGHT_HALF
    arc = math.pi * CENTER_R
    total = 2 * straight + 2 * arc
    for i in range(int(total / step)):
        s = i * step
        if s < straight:
            yield s, -STRAIGHT_HALF + s, -CENTER_R
        elif s < straight + arc:
            a = -math.pi / 2 + (s - straight) / CENTER_R
            yield s, STRAIGHT_HALF + CENTER_R * math.cos(a), CENTER_R * math.sin(a)
        elif s < 2 * straight + arc:
            yield s, STRAIGHT_HALF - (s - straight - arc), CENTER_R
        else:
            a = math.pi / 2 + (s - 2 * straight - arc) / CENTER_R
            yield s, -STRAIGHT_HALF + CENTER_R * math.cos(a), CENTER_R * math.sin(a)


KERB = 0.15            # red/white kerb width outside the curves
KERB_RED = (40, 40, 205)


def kerbs(img, r_in):
    """Alternating 0.2 m red/white blocks on the outside of both curves."""
    r = r_in + KERB / 2
    step = math.degrees(0.2 / r)
    for cx, start in ((STRAIGHT_HALF, -90.0), (-STRAIGHT_HALF, 90.0)):
        for i in range(int(180 / step)):
            a0 = start + i * step
            cv2.ellipse(img, to_px(cx, 0), (int(r * PX_PER_M),) * 2, 0, a0, a0 + step,
                        KERB_RED if i % 2 else WHITE, int(KERB * PX_PER_M), cv2.LINE_AA)


SIDEWALK = (150, 155, 158)   # V < 175 so it never passes the white-line threshold
SIDEWALK_W = 0.45
CROSS_X = -0.3             # zebra crossing on the south straight (car travels +X there)
STOP_X = -0.62             # stop line before it, right-hand lane only
ROAD_IN, ROAD_OUT = CENTER_R - LANE, CENTER_R + LANE


def rect(img, x0, y0, x1, y1, color):
    cv2.rectangle(img, to_px(x0, y1), to_px(x1, y0), color, -1)


def sidewalks(img):
    """Concrete sidewalks with expansion joints on the outside of both straights."""
    for sign in (1, -1):
        y0, y1 = sorted((sign * ROAD_OUT, sign * (ROAD_OUT + SIDEWALK_W)))
        rect(img, -STRAIGHT_HALF, y0, STRAIGHT_HALF, y1, SIDEWALK)
        for x in np.arange(-STRAIGHT_HALF, STRAIGHT_HALF, 0.3):
            rect(img, x, y0, x + 0.008, y1, (128, 132, 135))
        edge = y1 if sign > 0 else y0  # darker curb on the lawn side
        rect(img, -STRAIGHT_HALF, edge - 0.02, STRAIGHT_HALF, edge + 0.02, (110, 112, 115))


def crossing(img):
    for y in np.arange(-ROAD_OUT + 0.06, -ROAD_IN - 0.04, 0.16):
        rect(img, CROSS_X - 0.17, y, CROSS_X + 0.17, y + 0.08, WHITE)
    rect(img, STOP_X - 0.025, -ROAD_OUT + LINE, STOP_X + 0.025, -CENTER_R - 0.03, WHITE)


def asphalt_grain(img, seed=7):
    road = np.all(img == ASPHALT, axis=2)
    noise = np.random.default_rng(seed).normal(0, 5, road.sum())[:, None]
    img[road] = np.clip(img[road] + noise, 0, 255).astype(np.uint8)


def facade(wall, glass=(120, 92, 70), px=512):
    """Building wall with a 4x3 window grid (BGR)."""
    img = np.full((px, px, 3), wall, np.uint8)
    for r in range(3):
        for c in range(4):
            x0, y0 = 40 + c * 116, 50 + r * 150
            cv2.rectangle(img, (x0 - 6, y0 - 6), (x0 + 82, y0 + 106), (235, 235, 235), -1)
            cv2.rectangle(img, (x0, y0), (x0 + 76, y0 + 100), glass, -1)
            cv2.line(img, (x0 + 38, y0), (x0 + 38, y0 + 100), (235, 235, 235), 4)
    cv2.rectangle(img, (0, px - 24), (px, px), tuple(int(v * 0.6) for v in wall), -1)
    return img


def stop_sign_texture(px=512):
    """Red octagon, white rim, STOP lettering; bottom-left corner is grey for the back face."""
    img = np.full((px, px, 3), 255, np.uint8)
    c, r = px / 2, px / 2 - 4
    pts = lambda rr: np.array([(c + rr * math.cos(math.radians(22.5 + 45 * i)),
                                c + rr * math.sin(math.radians(22.5 + 45 * i))) for i in range(8)], np.int32)
    cv2.fillPoly(img, [pts(r)], (255, 255, 255), cv2.LINE_AA)
    cv2.fillPoly(img, [pts(r * 0.9)], (30, 30, 200), cv2.LINE_AA)
    scale, thick = px / 170, int(px / 28)
    (tw, th), _ = cv2.getTextSize("STOP", cv2.FONT_HERSHEY_DUPLEX, scale, thick)
    cv2.putText(img, "STOP", (int(c - tw / 2), int(c + th / 2)), cv2.FONT_HERSHEY_DUPLEX,
                scale, (255, 255, 255), thick, cv2.LINE_AA)
    img[px - 24:, :24] = (150, 150, 150)
    return img


def stop_sign_obj(size=0.15, thick=0.004):
    """Octagon plate facing +X, centred at the origin; UVs map the front to the texture."""
    r = size / 2 / math.cos(math.radians(22.5))
    ring = [(r * math.cos(math.radians(22.5 + 45 * i)), r * math.sin(math.radians(22.5 + 45 * i)))
            for i in range(8)]
    lines = ["# generated by tools/make_textures.py"]
    for x in (0.0, -thick):
        lines += [f"v {x} 0 0"] + [f"v {x} {y:.5f} {z:.5f}" for y, z in ring]  # viewer's right = +Y
    lines += ["vt 0.5 0.5"] + [f"vt {0.5 + y / (2 * r) * 0.99:.5f} {0.5 + z / (2 * r) * 0.99:.5f}" for y, z in ring]
    lines += ["vt 0.02 0.02", "vn 1 0 0", "vn -1 0 0"]
    for i in range(8):  # front fan (vertices 1..9), back fan (10..18) on the grey corner
        a, b = 2 + i, 2 + (i + 1) % 8
        lines.append(f"f 1/1/1 {a}/{a}/1 {b}/{b}/1")
        lines.append(f"f 10/11/2 {b + 9}/11/2 {a + 9}/11/2")
    return "\n".join(lines) + "\n"


def track():
    img = np.full((int(PLANE_H * PX_PER_M), int(PLANE_W * PX_PER_M), 3), GRASS, np.uint8)
    # Mown-lawn stripes: +-5 brightness, 1 m wide, so the edge of the texture stays invisible.
    for i in range(int(PLANE_W)):
        if i % 2:
            img[:, i * PX_PER_M:(i + 1) * PX_PER_M] = np.clip(np.array(GRASS) + 6, 0, 255)
    lawn = img.copy()
    outer, inner = CENTER_R + LANE, CENTER_R - LANE
    sidewalks(img)
    kerbs(img, outer)
    fill_stadium(img, outer, WHITE)
    fill_stadium(img, outer - LINE, ASPHALT)
    fill_stadium(img, inner + LINE, WHITE)
    infield = np.zeros(img.shape[:2], np.uint8)
    fill_stadium(infield, inner, 255)
    img[infield > 0] = lawn[infield > 0]
    asphalt_grain(img)
    dot = max(1, int(LINE / 2 * PX_PER_M))
    for s, x, y in centerline():
        if s % (2 * DASH) < DASH:
            cv2.circle(img, to_px(x, y), dot, YELLOW, -1)
    crossing(img)
    return img


def aruco(marker_id=0, px=600, border=100):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    return cv2.cvtColor(cv2.copyMakeBorder(marker, border, border, border, border,
                                           cv2.BORDER_CONSTANT, value=255), cv2.COLOR_GRAY2BGR)


if __name__ == "__main__":
    cv2.imwrite(str(HERE / "autodrive_track/materials/textures/track.png"), track())
    cv2.imwrite(str(HERE / "autodrive_track/materials/textures/grass.png"),
                np.full((8, 8, 3), GRASS, np.uint8))
    cv2.imwrite(str(HERE / "aruco_station/materials/textures/aruco_0.png"), aruco())
    cv2.imwrite(str(HERE / "autodrive_track/materials/textures/stop_sign.png"), stop_sign_texture())
    (HERE / "autodrive_track/meshes/stop_sign.obj").write_text(stop_sign_obj())
    for name, wall in (("facade_sand", (150, 190, 215)), ("facade_slate", (120, 112, 104))):
        cv2.imwrite(str(HERE / f"autodrive_track/materials/textures/{name}.png"), facade(wall))
    print("textures written under", HERE)
