#!/usr/bin/env python3
"""Deterministic Phase 5 scenario layout, world, validation, and plot generator."""
import argparse
import hashlib
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "simulation" / "scenarios" / "scenario_registry.yaml"
BASE_WORLD = ROOT / "simulation" / "ros2_ws" / "src" / "anc_gazebo" / "worlds" / "phase2_test_world.sdf"
WORLD_NAMES = {
    "S00": "s00_empty_open_arena.sdf",
    "S01": "s01_single_box_detour.sdf",
    "S02": "s02_random_simple_objects.sdf",
    "S03": "s03_constricted_occlusion.sdf",
    "S04": "s04_seeded_known_start.sdf",
    "S09": "s09_no_route.sdf",
}
COLORS = {
    "box": ("0.76 0.25 0.12 1", "0.86 0.34 0.18 1"),
    "cylinder": ("0.16 0.44 0.72 1", "0.24 0.56 0.84 1"),
    "sphere": ("0.22 0.62 0.42 1", "0.31 0.73 0.51 1"),
    "chair": ("0.66 0.48 0.20 1", "0.76 0.58 0.28 1"),
}
WALL_COLOR = ("0.32 0.34 0.37 1", "0.42 0.44 0.47 1")


def registry_data():
    return yaml.safe_load(REGISTRY.read_text())


def find_scenario(data, scenario_id):
    for scenario in data["scenarios"]:
        if scenario["id"] == scenario_id:
            return scenario
    raise ValueError(f"Unknown scenario ID: {scenario_id}")


def shape_radius(obstacle):
    shape = obstacle["shape"]
    if shape in ("box", "chair"):
        sx, sy, _ = obstacle["size_xyz"]
        return math.hypot(sx / 2.0, sy / 2.0)
    if shape in ("cylinder", "sphere"):
        return float(obstacle["radius_m"])
    raise ValueError(f"Unsupported shape: {shape}")


def local_xy(x, y, obstacle):
    ox, oy, yaw = obstacle["pose_xyyaw"]
    dx, dy = x - ox, y - oy
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def point_obstacle_distance(x, y, obstacle):
    if obstacle["shape"] in ("cylinder", "sphere"):
        return max(0.0, math.hypot(x - obstacle["pose_xyyaw"][0],
                                   y - obstacle["pose_xyyaw"][1]) -
                   float(obstacle["radius_m"]))
    lx, ly = local_xy(x, y, obstacle)
    sx, sy, _ = obstacle["size_xyz"]
    dx = max(abs(lx) - sx / 2.0, 0.0)
    dy = max(abs(ly) - sy / 2.0, 0.0)
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.hypot(dx, dy)


def obstacle_corners(obstacle, expansion=0.0):
    if obstacle["shape"] in ("cylinder", "sphere"):
        return None
    x, y, yaw = obstacle["pose_xyyaw"]
    sx, sy, _ = obstacle["size_xyz"]
    hx, hy = sx / 2.0 + expansion, sy / 2.0 + expansion
    c, s = math.cos(yaw), math.sin(yaw)
    return [
        (x + c * px - s * py, y + s * px + c * py)
        for px, py in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))
    ]


def obstacle_inside(obstacle, inner_bounds, margin=0.02):
    xmin, xmax, ymin, ymax = inner_bounds
    corners = obstacle_corners(obstacle)
    if corners is None:
        x, y, _ = obstacle["pose_xyyaw"]
        radius = float(obstacle["radius_m"])
        return (x - radius >= xmin + margin and x + radius <= xmax - margin and
                y - radius >= ymin + margin and y + radius <= ymax - margin)
    return all(x >= xmin + margin and x <= xmax - margin and
               y >= ymin + margin and y <= ymax - margin
               for x, y in corners)


def _axes(poly):
    result = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length:
            result.append((-dy / length, dx / length))
    return result


def _project(poly, axis):
    values = [x * axis[0] + y * axis[1] for x, y in poly]
    return min(values), max(values)


def _circle_rect_overlap(circle, rect, padding=0.0):
    cx, cy, _ = circle["pose_xyyaw"]
    lx, ly = local_xy(cx, cy, rect)
    sx, sy, _ = rect["size_xyz"]
    dx = max(abs(lx) - sx / 2.0 - padding, 0.0)
    dy = max(abs(ly) - sy / 2.0 - padding, 0.0)
    return math.hypot(dx, dy) < float(circle["radius_m"]) + padding


def obstacles_overlap(a, b, padding=0.0):
    a_circle = a["shape"] in ("cylinder", "sphere")
    b_circle = b["shape"] in ("cylinder", "sphere")
    if a_circle and b_circle:
        ax, ay, _ = a["pose_xyyaw"]
        bx, by, _ = b["pose_xyyaw"]
        return math.hypot(ax - bx, ay - by) < (
            float(a["radius_m"]) + float(b["radius_m"]) + 2 * padding)
    if a_circle:
        return _circle_rect_overlap(a, b, padding)
    if b_circle:
        return _circle_rect_overlap(b, a, padding)
    pa = obstacle_corners(a, padding)
    pb = obstacle_corners(b, padding)
    for axis in _axes(pa) + _axes(pb):
        amin, amax = _project(pa, axis)
        bmin, bmax = _project(pb, axis)
        if amax <= bmin or bmax <= amin:
            return False
    return True


def point_is_free(x, y, layout):
    bounds = layout["arena"]["inner_bounds_xy"]
    xmin, xmax, ymin, ymax = bounds
    radius = (layout["arena"]["robot_circumscribed_radius_m"] +
              layout["arena"]["clearance_margin_m"])
    if min(x - xmin, xmax - x, y - ymin, ymax - y) < radius:
        return False
    return all(point_obstacle_distance(x, y, o) >= radius - 1e-9
               for o in layout["obstacles"])


def corridor_is_clear(points, layout, step=0.04):
    if len(points) < 2:
        return all(point_is_free(p[0], p[1], layout) for p in points)
    for a, b in zip(points[:-1], points[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, math.ceil(length / step))
        for i in range(n + 1):
            t = i / n
            if not point_is_free(a[0] + t * (b[0] - a[0]),
                                 a[1] + t * (b[1] - a[1]), layout):
                return False
    return True


def grid_route(layout):
    xmin, xmax, ymin, ymax = layout["arena"]["inner_bounds_xy"]
    radius = (layout["arena"]["robot_circumscribed_radius_m"] +
              layout["arena"]["clearance_margin_m"])
    step = layout["arena"]["grid_resolution_m"]
    xs = []
    x = xmin + radius + step / 2.0
    while x <= xmax - radius:
        xs.append(round(x, 6))
        x += step
    ys = []
    y = ymin + radius + step / 2.0
    while y <= ymax - radius:
        ys.append(round(y, 6))
        y += step
    free = set()
    for iy, yy in enumerate(ys):
        for ix, xx in enumerate(xs):
            if point_is_free(xx, yy, layout):
                free.add((ix, iy))

    def nearest_index(value, coords):
        return min(range(len(coords)), key=lambda i: abs(coords[i] - value))

    sx, sy, _ = layout["start_xyyaw"]
    gx, gy, _ = layout["goal_xyyaw"]
    start = (nearest_index(sx, xs), nearest_index(sy, ys))
    goal = (nearest_index(gx, xs), nearest_index(gy, ys))
    if start not in free or goal not in free:
        return {"found": False, "grid_cells": 0, "grid_resolution_m": step}
    frontier = [start]
    parent = {start: None}
    head = 0
    while head < len(frontier):
        here = frontier[head]
        head += 1
        if here == goal:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = (here[0] + dx, here[1] + dy)
            if nxt not in free or nxt in parent:
                continue
            if dx and dy and ((here[0] + dx, here[1]) not in free or
                              (here[0], here[1] + dy) not in free):
                continue
            parent[nxt] = here
            frontier.append(nxt)
    found = goal in parent
    path = []
    if found:
        cursor = goal
        while cursor is not None:
            path.append([xs[cursor[0]], ys[cursor[1]]])
            cursor = parent[cursor]
        path.reverse()
    return {
        "found": found,
        "grid_cells": len(path),
        "grid_resolution_m": step,
        "path_xy": path,
    }


def _random_shape(rng, shape):
    if shape == "box":
        return {"shape": "box",
                "size_xyz": [round(rng.uniform(0.30, 0.62), 4),
                             round(rng.uniform(0.30, 0.62), 4),
                             round(rng.uniform(0.40, 0.90), 4)]}
    if shape == "cylinder":
        return {"shape": shape, "radius_m": round(rng.uniform(0.18, 0.30), 4),
                "size_xyz": [0, 0, round(rng.uniform(0.45, 0.95), 4)]}
    if shape == "sphere":
        r = round(rng.uniform(0.18, 0.28), 4)
        return {"shape": shape, "radius_m": r, "size_xyz": [2*r, 2*r, 2*r]}
    if shape == "chair":
        return {"shape": shape, "size_xyz": [0.62, 0.62, 0.95]}
    raise ValueError(f"Unsupported random geometry: {shape}")


def generate_layout(scenario_id, seed, data=None):
    data = data or registry_data()
    scenario = find_scenario(data, scenario_id)
    if scenario.get("status") != "active_phase5":
        raise ValueError(f"{scenario_id} is not active in Phase 5")
    if scenario["id"] == "S04":
        x0, x1, y0, y1, yaw0, yaw1 = scenario["start_region_xyyaw"]
        rng_start = random.Random(int(seed) ^ 0x5A17)
        start = [round(rng_start.uniform(x0, x1), 6),
                 round(rng_start.uniform(y0, y1), 6),
                 round(rng_start.uniform(yaw0, yaw1), 6)]
    else:
        start = list(scenario["start_xyyaw"])
    goal = list(scenario["goal_xyyaw"])
    arena = dict(data["arena"])
    obstacles = [dict(o) for o in scenario.get("obstacles", [])]

    random_cfg = scenario.get("random_obstacles")
    if random_cfg:
        rng = random.Random(int(seed) ^ (0xA11CE if scenario_id == "S02" else 0xB04))
        layout_stub = {
            "arena": arena, "obstacles": [],
            "start_xyyaw": start, "goal_xyyaw": goal,
        }
        accepted = []
        xmin, xmax = random_cfg["x_range"]
        ymin, ymax = random_cfg["y_range"]
        for index in range(random_cfg["count"]):
            placed = None
            for _ in range(2500):
                shape = rng.choice(random_cfg["shapes"])
                candidate = _random_shape(rng, shape)
                candidate.update({
                    "name": f"obstacle_{index + 1:02d}",
                    "pose_xyyaw": [round(rng.uniform(xmin, xmax), 4),
                                   round(rng.uniform(ymin, ymax), 4),
                                   round(rng.uniform(-math.pi, math.pi), 5)],
                    "static": True,
                })
                if not obstacle_inside(candidate, arena["inner_bounds_xy"]):
                    continue
                clearance = (arena["robot_circumscribed_radius_m"] +
                             arena["clearance_margin_m"])
                if any(point_obstacle_distance(px, py, candidate) < clearance
                       for px, py, _ in (start, goal)):
                    continue
                if any(obstacles_overlap(candidate, old, padding=0.06)
                       for old in accepted):
                    continue
                if any(not point_is_free(px, py, {**layout_stub,
                                                   "obstacles": accepted + [candidate]})
                       for px, py in (start[:2], goal[:2])):
                    continue
                placed = candidate
                break
            if placed is None:
                raise RuntimeError(
                    f"Could not place obstacle {index + 1} for {scenario_id} seed {seed}"
                )
            accepted.append(placed)
        obstacles.extend(accepted)

    layout = {
        "schema_version": 1,
        "scenario_id": scenario_id,
        "scenario_name": scenario["name"],
        "world_type": scenario["world_type"],
        "seed": int(seed),
        "sensor_profile": scenario["sensor_profile"],
        "map_mode": scenario["map_mode"],
        "validity": scenario["validity"],
        "arena": arena,
        "start_xyyaw": start,
        "goal_xyyaw": goal,
        "obstacles": obstacles,
        "expected_free_corridors": scenario.get("expected_free_corridors", []),
        "route_expected": bool(scenario["route_expected"]),
        "navigation_attempted": False,
    }
    checks = []
    clearance = (arena["robot_circumscribed_radius_m"] +
                 arena["clearance_margin_m"])
    for label, pose in (("start", start), ("goal", goal)):
        checks.append({
            "name": f"{label}_footprint_clearance",
            "passed": point_is_free(pose[0], pose[1], layout),
            "detail": f"{label} center has at least {clearance:.3f} m clearance "
                      "from arena bounds and every obstacle.",
        })
    for obstacle in obstacles:
        inside = obstacle_inside(obstacle, arena["inner_bounds_xy"])
        checks.append({
            "name": f"{obstacle['name']}_inside_arena",
            "passed": inside,
            "detail": f"{obstacle['shape']} physical footprint lies inside the room.",
        })
    for i, first in enumerate(obstacles):
        for second in obstacles[i + 1:]:
            separated = not obstacles_overlap(first, second)
            checks.append({
                "name": f"{first['name']}_{second['name']}_nonoverlap",
                "passed": separated,
                "detail": "Obstacle physical footprints do not overlap.",
            })
    corridor_results = []
    for corridor in layout["expected_free_corridors"]:
        clear = corridor_is_clear(corridor["points"], layout)
        corridor_results.append({"name": corridor["name"], "clear": clear})
        checks.append({
            "name": f"corridor_{corridor['name']}_footprint_clear",
            "passed": clear,
            "detail": "The expected centerline remains clear after footprint inflation.",
        })
    route = grid_route(layout)
    checks.append({
        "name": "route_validity_matches_registry",
        "passed": route["found"] == layout["route_expected"],
        "detail": f"10 cm footprint-inflated grid found={route['found']}; "
                  f"expected={layout['route_expected']}.",
    })
    if layout["validity"] == "traversable":
        checks.append({
            "name": "scenario_is_traversable",
            "passed": route["found"],
            "detail": "Start and goal are connected in the footprint-inflated grid.",
        })
    if layout["validity"] == "no_route":
        checks.append({
            "name": "scenario_has_no_route",
            "passed": not route["found"],
            "detail": "The divider and room bounds leave no footprint-clear route.",
        })
    layout["corridor_results"] = corridor_results
    layout["route_validation"] = route
    layout["validation"] = {
        "passed": all(c["passed"] for c in checks),
        "checks": checks,
    }
    canonical = json.dumps(layout, sort_keys=True, separators=(",", ":"))
    layout["layout_sha256"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    return layout


def _add_box_geometry(parent, name, size, pose=None):
    collision = ET.SubElement(parent, "collision", {"name": f"{name}_collision"})
    if pose is not None:
        ET.SubElement(collision, "pose").text = " ".join(f"{v:.6f}" for v in pose)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box")
    ET.SubElement(geometry.find("box"), "size").text = " ".join(f"{v:.6f}" for v in size)
    visual = ET.SubElement(parent, "visual", {"name": f"{name}_visual"})
    if pose is not None:
        ET.SubElement(visual, "pose").text = " ".join(f"{v:.6f}" for v in pose)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box")
    ET.SubElement(geometry.find("box"), "size").text = " ".join(f"{v:.6f}" for v in size)
    return visual


def _set_material(visual, ambient, diffuse):
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = ambient
    ET.SubElement(material, "diffuse").text = diffuse


def add_model(world, name, x, y, z, yaw=0.0, static=True):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true" if static else "false"
    ET.SubElement(model, "pose").text = f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}"
    return model, ET.SubElement(model, "link", {"name": "link"})


def add_box_model(world, name, x, y, z, yaw, size, static=True, color=None):
    model, link = add_model(world, name, x, y, z, yaw, static)
    visual = _add_box_geometry(link, name, size)
    if color:
        _set_material(visual, *color)
    return model


def build_world_sdf(layout):
    tree = ET.parse(BASE_WORLD)
    root = tree.getroot()
    world = root.find("world")
    for model in list(world.findall("model")):
        if model.get("name") == "phase2_wall":
            world.remove(model)
    xmin, xmax, ymin, ymax = layout["arena"]["outer_bounds_xy"]
    wall_h = layout["arena"]["wall_height_m"]
    t = 0.15
    z = wall_h / 2.0
    add_box_model(world, "arena_wall_left", xmin + t/2, 0, z, 0,
                  (t, ymax - ymin, wall_h), color=WALL_COLOR)
    add_box_model(world, "arena_wall_right", xmax - t/2, 0, z, 0,
                  (t, ymax - ymin, wall_h), color=WALL_COLOR)
    add_box_model(world, "arena_wall_bottom", 0, ymin + t/2, z, 0,
                  (xmax - xmin, t, wall_h), color=WALL_COLOR)
    add_box_model(world, "arena_wall_top", 0, ymax - t/2, z, 0,
                  (xmax - xmin, t, wall_h), color=WALL_COLOR)

    for obstacle in layout["obstacles"]:
        shape = obstacle["shape"]
        x, y, yaw = obstacle["pose_xyyaw"]
        static = bool(obstacle.get("static", True))
        model, link = add_model(world, obstacle["name"], x, y, 0, yaw, static)
        ambient, diffuse = COLORS[shape]
        if shape in ("box",):
            sx, sy, sz = obstacle["size_xyz"]
            model.find("pose").text = f"{x:.6f} {y:.6f} {sz/2:.6f} 0 0 {yaw:.6f}"
            visual = _add_box_geometry(link, obstacle["name"], (sx, sy, sz))
            _set_material(visual, ambient, diffuse)
        elif shape in ("cylinder", "sphere"):
            radius = obstacle["radius_m"]
            sz = obstacle["size_xyz"][2]
            center_z = radius if shape == "sphere" else sz / 2.0
            model.find("pose").text = f"{x:.6f} {y:.6f} {center_z:.6f} 0 0 {yaw:.6f}"
            collision = ET.SubElement(link, "collision", {"name": f"{obstacle['name']}_collision"})
            geom = ET.SubElement(collision, "geometry")
            primitive = ET.SubElement(geom, shape)
            if shape == "cylinder":
                ET.SubElement(primitive, "radius").text = f"{radius:.6f}"
                ET.SubElement(primitive, "length").text = f"{sz:.6f}"
            else:
                ET.SubElement(primitive, "radius").text = f"{radius:.6f}"
            visual = ET.SubElement(link, "visual", {"name": f"{obstacle['name']}_visual"})
            geom = ET.SubElement(visual, "geometry")
            primitive = ET.SubElement(geom, shape)
            if shape == "cylinder":
                ET.SubElement(primitive, "radius").text = f"{radius:.6f}"
                ET.SubElement(primitive, "length").text = f"{sz:.6f}"
            else:
                ET.SubElement(primitive, "radius").text = f"{radius:.6f}"
            _set_material(visual, ambient, diffuse)
        elif shape == "chair":
            sx, sy, sz = obstacle["size_xyz"]
            seat_z = sz * 0.58
            parts = [
                ("seat", (sx, sy * 0.78, 0.12), (0, sy * 0.03, seat_z)),
                ("back", (sx, 0.12, sz * 0.43), (0, -sy * 0.40, sz * 0.78)),
            ]
            leg_w = 0.08
            for lx in (-sx * 0.42, sx * 0.42):
                for ly in (-sy * 0.36, sy * 0.36):
                    parts.append(("leg", (leg_w, leg_w, seat_z), (lx, ly, seat_z/2)))
            for part_index, (part, size, pose) in enumerate(parts):
                visual = _add_box_geometry(link, f"{obstacle['name']}_{part}_{part_index}",
                                           size, (*pose, 0, 0, 0))
                _set_material(visual, ambient, diffuse)
        else:
            raise ValueError(f"Unsupported geometry: {shape}")

    ET.indent(tree, space="  ")
    return (ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n")


def write_plot(layout, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    xmin, xmax, ymin, ymax = layout["arena"]["outer_bounds_xy"]
    fig, ax = plt.subplots(figsize=(8.5, 6.2), dpi=140)
    ax.set_facecolor("#f3f4f6")
    ax.add_patch(plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               facecolor="#f3f4f6", edgecolor="#30343b", linewidth=2))
    for obstacle in layout["obstacles"]:
        color = {"box": "#cf6148", "cylinder": "#4987b8",
                 "sphere": "#54a875", "chair": "#bd9347"}[obstacle["shape"]]
        corners = obstacle_corners(obstacle)
        if corners is None:
            x, y, _ = obstacle["pose_xyyaw"]
            ax.add_patch(Circle((x, y), obstacle["radius_m"], color=color, alpha=0.85))
        else:
            ax.add_patch(Polygon(corners, closed=True, facecolor=color,
                                 edgecolor="#30343b", alpha=0.88))
        x, y, _ = obstacle["pose_xyyaw"]
        ax.text(x, y, obstacle["name"][-2:], ha="center", va="center",
                fontsize=7, color="white", weight="bold")
    for corridor in layout["expected_free_corridors"]:
        xs = [p[0] for p in corridor["points"]]
        ys = [p[1] for p in corridor["points"]]
        ax.plot(xs, ys, linestyle="--", linewidth=1.3, color="#3a9b68", alpha=0.8)
    sx, sy, syaw = layout["start_xyyaw"]
    gx, gy, gyaw = layout["goal_xyyaw"]
    ax.scatter([sx], [sy], marker="o", s=80, color="#1f77b4", label="start", zorder=5)
    ax.arrow(sx, sy, 0.45*math.cos(syaw), 0.45*math.sin(syaw),
             head_width=0.13, color="#1f77b4", length_includes_head=True)
    ax.scatter([gx], [gy], marker="*", s=160, color="#9b2c7a", label="goal", zorder=5)
    ax.arrow(gx, gy, 0.38*math.cos(gyaw), 0.38*math.sin(gyaw),
             head_width=0.12, color="#9b2c7a", length_includes_head=True)
    if layout["route_validation"]["found"] and layout["route_validation"].get("path_xy"):
        path_xy = layout["route_validation"]["path_xy"]
        ax.plot([p[0] for p in path_xy], [p[1] for p in path_xy],
                color="#7359a5", linewidth=1.2, alpha=0.6, label="10 cm grid route")
    ax.set_xlim(xmin - 0.25, xmax + 0.25)
    ax.set_ylim(ymin - 0.25, ymax + 0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{layout['scenario_id']} {layout['scenario_name']} | seed {layout['seed']}\n"
                 f"profile={layout['sensor_profile']}  route={layout['route_validation']['found']}")
    ax.grid(True, alpha=0.22)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def write_outputs(scenario_id, seed, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    layout = generate_layout(scenario_id, seed)
    world_bytes = build_world_sdf(layout)
    world_path = output_dir / "world.sdf"
    world_path.write_bytes(world_bytes)
    layout["world_sha256"] = hashlib.sha256(world_bytes).hexdigest()
    layout["world_file"] = str(world_path)
    (output_dir / "manifest.json").write_text(
        json.dumps(layout, indent=2, sort_keys=True) + "\n")
    write_plot(layout, output_dir / "layout.png")
    if not layout["validation"]["passed"]:
        raise RuntimeError("Scenario validation failed: " + json.dumps(
            [c for c in layout["validation"]["checks"] if not c["passed"]]))
    return layout


def generate_canonical_worlds():
    data = registry_data()
    output_dir = ROOT / "simulation" / "scenarios" / "worlds"
    output_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for scenario_id, filename in WORLD_NAMES.items():
        scenario = find_scenario(data, scenario_id)
        seed = data["seed_sets"][scenario["seed_set"]][0]
        layout = generate_layout(scenario_id, seed, data)
        if not layout["validation"]["passed"]:
            failed = [c for c in layout["validation"]["checks"] if not c["passed"]]
            raise RuntimeError(f"{scenario_id} failed: {failed}")
        payload = build_world_sdf(layout)
        path = output_dir / filename
        path.write_bytes(payload)
        written.append({"scenario_id": scenario_id, "seed": seed,
                        "path": str(path.relative_to(ROOT)),
                        "sha256": hashlib.sha256(payload).hexdigest(),
                        "layout_sha256": layout["layout_sha256"]})
    return written


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=tuple(WORLD_NAMES))
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--generate-canonical-worlds", action="store_true")
    args = parser.parse_args()
    if args.generate_canonical_worlds:
        print(json.dumps(generate_canonical_worlds(), indent=2))
        return
    if not args.scenario or not args.output_dir:
        parser.error("--scenario and --output-dir are required unless --generate-canonical-worlds is used")
    layout = write_outputs(args.scenario, args.seed, args.output_dir)
    print(json.dumps({
        "scenario_id": args.scenario,
        "seed": args.seed,
        "layout_sha256": layout["layout_sha256"],
        "world_sha256": layout["world_sha256"],
        "route_found": layout["route_validation"]["found"],
        "validation_passed": layout["validation"]["passed"],
        "output_dir": str(args.output_dir),
    }, indent=2))


if __name__ == "__main__":
    main()
