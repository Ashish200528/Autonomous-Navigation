"""Offline evaluator-only ray/geometry labels for Phase 7 camera frames.

This module is never imported by anc_perception. It reads scenario geometry and
simulator truth solely to label saved RGB frames for measurement.
"""
import math

import numpy as np


def _ray_box(ox, oy, oz, dx, dy, dz, center, size, yaw):
    cx, cy, cz = (float(x) for x in center)
    sx, sy, sz = (float(x) for x in size)
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    rx, ry = ox - cx, oy - cy
    lx = c * rx + s * ry
    ly = -s * rx + c * ry
    ldx = c * dx + s * dy
    ldy = -s * dx + c * dy
    enter = np.full(dx.shape, -np.inf, dtype=np.float32)
    leave = np.full(dx.shape, np.inf, dtype=np.float32)
    valid = np.ones(dx.shape, dtype=bool)
    origins = (lx, ly, oz - cz)
    directions = (ldx, ldy, dz)
    half = (sx / 2.0, sy / 2.0, sz / 2.0)
    for origin, direction, radius in zip(origins, directions, half):
        parallel = np.abs(direction) < 1e-8
        valid &= (~parallel) | ((origin >= -radius) & (origin <= radius))
        safe_direction = np.where(parallel, 1.0, direction)
        a = (-radius - origin) / safe_direction
        b = (radius - origin) / safe_direction
        lo, hi = np.minimum(a, b), np.maximum(a, b)
        enter = np.maximum(enter, np.where(parallel, -np.inf, lo))
        leave = np.minimum(leave, np.where(parallel, np.inf, hi))
    hit = valid & (leave >= np.maximum(enter, 0.0)) & (leave >= 0.0)
    distance = np.where(enter > 0.0, enter, leave)
    return np.where(hit, distance, np.inf).astype(np.float32)


def _ray_sphere(ox, oy, oz, dx, dy, dz, center, radius):
    cx, cy, cz = (float(x) for x in center)
    rx, ry, rz = ox - cx, oy - cy, oz - cz
    a = dx * dx + dy * dy + dz * dz
    b = 2.0 * (rx * dx + ry * dy + rz * dz)
    c = rx * rx + ry * ry + rz * rz - float(radius) ** 2
    disc = b * b - 4.0 * a * c
    root = np.sqrt(np.maximum(disc, 0.0))
    t0 = (-b - root) / (2.0 * a)
    t1 = (-b + root) / (2.0 * a)
    t = np.where(t0 > 1e-6, t0, t1)
    return np.where((disc >= 0.0) & (t > 1e-6), t, np.inf).astype(np.float32)


def _ray_cylinder(ox, oy, oz, dx, dy, dz, center_xy, radius, height):
    cx, cy = (float(x) for x in center_xy)
    rx, ry = ox - cx, oy - cy
    a = dx * dx + dy * dy
    b = 2.0 * (rx * dx + ry * dy)
    c = rx * rx + ry * ry - float(radius) ** 2
    disc = b * b - 4.0 * a * c
    root = np.sqrt(np.maximum(disc, 0.0))
    safe_a = np.where(a < 1e-10, 1.0, a)
    t0 = (-b - root) / (2.0 * safe_a)
    t1 = (-b + root) / (2.0 * safe_a)
    side = np.full(dx.shape, np.inf, dtype=np.float32)
    for candidate in (t0, t1):
        z = oz + dz * candidate
        valid = (disc >= 0.0) & (candidate > 1e-6) & (z >= 0.0) & (z <= height)
        side = np.minimum(side, np.where(valid, candidate, np.inf))
    caps = np.full(dx.shape, np.inf, dtype=np.float32)
    nonhorizontal = np.abs(dz) > 1e-8
    for cap_z in (0.0, float(height)):
        t = (cap_z - oz) / np.where(nonhorizontal, dz, 1.0)
        px, py = rx + dx * t, ry + dy * t
        valid = nonhorizontal & (t > 1e-6) & (px * px + py * py <= radius * radius)
        caps = np.minimum(caps, np.where(valid, t, np.inf))
    return np.minimum(side, caps).astype(np.float32)


def _obstacle_boxes(obstacle):
    shape = obstacle["shape"]
    x, y, yaw = (float(v) for v in obstacle["pose_xyyaw"])
    if shape == "box":
        sx, sy, sz = (float(v) for v in obstacle["size_xyz"])
        return [(x, y, sz / 2.0, sx, sy, sz, yaw)]
    if shape == "chair":
        sx, sy, sz = (float(v) for v in obstacle["size_xyz"])
        seat_z = sz * 0.58
        parts = [
            (sx, sy * 0.78, 0.12, 0.0, sy * 0.03, seat_z),
            (sx, 0.12, sz * 0.43, 0.0, -sy * 0.40, sz * 0.78),
        ]
        leg_w = 0.08
        for lx in (-sx * 0.42, sx * 0.42):
            for ly in (-sy * 0.36, sy * 0.36):
                parts.append((leg_w, leg_w, seat_z, lx, ly, seat_z / 2.0))
        output = []
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py, pz, ox, oy, oz in parts:
            output.append((
                x + c * ox - s * oy, y + s * ox + c * oy, oz,
                px, py, pz, yaw,
            ))
        return output
    return []


def ground_truth_mask(image_shape, camera_info, layout, truth_pose, config):
    """Label first-hit floor/obstacle pixels; return UNKNOWN elsewhere."""
    height, width = (int(image_shape[0]), int(image_shape[1]))
    k = camera_info["k"]
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0 or fy <= 0 or camera_info.get("d", [0.0]) and any(
        abs(float(value)) > 1e-7 for value in camera_info.get("d", [])
    ):
        raise ValueError("evaluation_requires_valid_zero_distortion_camera_info")

    rows, cols = np.indices((height, width), dtype=np.float32)
    right = (cols - cx) / fx
    down = (rows - cy) / fy
    pitch = float(config["camera_pitch_down_rad"])
    cp, sp = math.cos(pitch), math.sin(pitch)
    # Camera optical (+z forward, +x right, +y down) into base (+x forward,
    # +y left, +z up), then apply the provisional downward pitch.
    base_forward = cp - sp * down
    base_left = -right
    base_up = -sp - cp * down
    yaw = float(truth_pose["yaw_rad"])
    c, s = math.cos(yaw), math.sin(yaw)
    dx = c * base_forward - s * base_left
    dy = s * base_forward + c * base_left
    dz = base_up

    base_x, base_y = float(truth_pose["x"]), float(truth_pose["y"])
    origin_x = base_x + c * float(config["camera_forward_offset_m"])
    origin_y = base_y + s * float(config["camera_forward_offset_m"])
    origin_z = float(truth_pose.get("z", 0.0)) + float(config["camera_height_m"])
    ox = np.full((height, width), origin_x, dtype=np.float32)
    oy = np.full((height, width), origin_y, dtype=np.float32)
    oz = np.full((height, width), origin_z, dtype=np.float32)

    nearest = np.full((height, width), np.inf, dtype=np.float32)
    xmin, xmax, ymin, ymax = (float(v) for v in layout["arena"]["outer_bounds_xy"])
    wall_h = float(layout["arena"]["wall_height_m"])
    wall_t = 0.15
    walls = [
        (xmin + wall_t / 2.0, 0.0, wall_h / 2.0, wall_t, ymax - ymin, wall_h, 0.0),
        (xmax - wall_t / 2.0, 0.0, wall_h / 2.0, wall_t, ymax - ymin, wall_h, 0.0),
        (0.0, ymin + wall_t / 2.0, wall_h / 2.0, xmax - xmin, wall_t, wall_h, 0.0),
        (0.0, ymax - wall_t / 2.0, wall_h / 2.0, xmax - xmin, wall_t, wall_h, 0.0),
    ]
    for bx, by, bz, sx, sy, sz, byaw in walls:
        hit = _ray_box(ox, oy, oz, dx, dy, dz,
                       (bx, by, bz), (sx, sy, sz), byaw)
        nearest = np.minimum(nearest, hit)

    for obstacle in layout.get("obstacles", []):
        shape = obstacle["shape"]
        x, y, object_yaw = (float(v) for v in obstacle["pose_xyyaw"])
        if shape in ("box", "chair"):
            for bx, by, bz, sx, sy, sz, byaw in _obstacle_boxes(obstacle):
                hit = _ray_box(ox, oy, oz, dx, dy, dz,
                               (bx, by, bz), (sx, sy, sz), byaw)
                nearest = np.minimum(nearest, hit)
        elif shape == "sphere":
            radius = float(obstacle["radius_m"])
            hit = _ray_sphere(ox, oy, oz, dx, dy, dz,
                              (x, y, radius), radius)
            nearest = np.minimum(nearest, hit)
        elif shape == "cylinder":
            radius = float(obstacle["radius_m"])
            height_m = float(obstacle["size_xyz"][2])
            hit = _ray_cylinder(ox, oy, oz, dx, dy, dz, (x, y), radius, height_m)
            nearest = np.minimum(nearest, hit)
        else:
            raise ValueError(f"unsupported evaluator geometry {shape}")

    falling = dz < -1e-8
    safe_dz = np.where(falling, dz, -1.0)
    ground_t = np.where(falling, -oz / safe_dz, np.inf)
    finite_floor = np.isfinite(ground_t)
    safe_ground_t = np.where(finite_floor, ground_t, 0.0)
    ground_x = origin_x + dx * safe_ground_t
    ground_y = origin_y + dy * safe_ground_t
    inner_x0, inner_x1, inner_y0, inner_y1 = (
        float(v) for v in layout["arena"]["inner_bounds_xy"]
    )
    inside = (
        (ground_x >= inner_x0) & (ground_x <= inner_x1)
        & (ground_y >= inner_y0) & (ground_y <= inner_y1)
    )
    hit_object_first = np.isfinite(nearest) & (
        (ground_t <= 0.0) | (nearest < ground_t - 1e-4)
    )
    forward_at_floor = ground_t * base_forward
    valid_range = (
        (forward_at_floor >= float(config["min_ground_forward_m"]))
        & (forward_at_floor <= float(config["max_ground_forward_m"]))
    )
    free_first = (ground_t > 0.0) & inside & valid_range & ~hit_object_first

    output = np.full((height, width), 255, dtype=np.uint8)
    output[free_first] = 0
    output[hit_object_first] = 100
    return output
