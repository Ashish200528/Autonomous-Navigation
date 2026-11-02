
"""Evaluator-only projection and footprint corridor metrics for Phase 8B."""
import math

import cv2
import numpy as np

from anc_realistic_navigation.projection import project_free_pixels


def project_mask(mask, confidence, camera_info, camera_transform, map_params):
    translation = camera_transform["translation_m"]
    quaternion = camera_transform["quaternion_xyzw"]
    grid, counts = project_free_pixels(
        mask, confidence, camera_info, translation, quaternion,
        expected_frame=camera_info.get("frame_id", "camera_optical_frame"),
        grid_width=int(map_params["grid_width"]),
        grid_height=int(map_params["grid_height"]),
        resolution=float(map_params["resolution_m"]),
        origin_x=float(map_params["origin_x_m"]),
        origin_y=float(map_params["origin_y_m"]),
        floor_z=float(map_params["floor_z_in_base_m"]),
        min_confidence=int(map_params["min_free_confidence"]),
    )
    return grid, counts


def corridor_metrics(grid, map_params, robot_params, margin_m=0.02):
    model = robot_params["robot"]["simulation_model"]
    data = np.asarray(grid, dtype=np.int8).reshape(
        int(map_params["grid_height"]), int(map_params["grid_width"])
    )
    resolution = float(map_params["resolution_m"])
    origin_x = float(map_params["origin_x_m"])
    origin_y = float(map_params["origin_y_m"])
    footprint = model["footprint_xy_m"]
    front_x = max(float(point[0]) for point in footprint)
    half_width = max(abs(float(point[1])) for point in footprint)
    y_centers = origin_y + (np.arange(data.shape[0]) + 0.5) * resolution
    x_centers = origin_x + (np.arange(data.shape[1]) + 0.5) * resolution
    first_x = max(0, int(math.ceil((front_x-origin_x)/resolution - 1e-9)))

    def run(rows):
        if len(rows) == 0:
            return {
                "first_free_x_m": None,
                "gap_after_robot_front_m": None,
                "connected_free_length_from_front_m": 0.0,
                "free_length_after_first_gap_m": 0.0,
            }
        first = next((ix for ix in range(first_x, data.shape[1])
                      if np.all(data[rows, ix] == 0)), None)
        connected = 0
        if first == first_x:
            ix = first_x
            while ix < data.shape[1] and np.all(data[rows, ix] == 0):
                connected += 1
                ix += 1
        after = 0
        if first is not None:
            ix = first
            while ix < data.shape[1] and np.all(data[rows, ix] == 0):
                after += 1
                ix += 1
        return {
            "first_free_x_m": None if first is None else round(origin_x+first*resolution, 4),
            "gap_after_robot_front_m": None if first is None else round(
                max(0.0, origin_x+first*resolution-front_x), 4),
            "connected_free_length_from_front_m": round(connected*resolution, 4),
            "free_length_after_first_gap_m": round(after*resolution, 4),
        }

    footprint_rows = np.flatnonzero(
        (y_centers >= -half_width-margin_m) &
        (y_centers <= half_width+margin_m)
    )
    wheels = {}
    wheel_half_band = float(model["wheel_width_m"])/2.0 + margin_m
    separation = float(model["wheel_separation_m"])
    for name, center in (("left", separation/2.0), ("right", -separation/2.0)):
        rows = np.flatnonzero(
            (y_centers >= center-wheel_half_band) &
            (y_centers <= center+wheel_half_band)
        )
        wheels[name] = run(rows)
    near_cols = np.flatnonzero(
        (x_centers >= front_x) & (x_centers < front_x+0.10)
    )
    near_fraction = (
        float(np.count_nonzero(data[np.ix_(footprint_rows, near_cols)] == 0) /
              max(1, len(footprint_rows)*len(near_cols)))
        if len(near_cols) else 0.0
    )
    return {
        "robot_front_extent_m": front_x,
        "robot_half_width_m": half_width,
        "required_width_with_margin_m": 2.0*(half_width+margin_m),
        "lateral_margin_each_side_m": margin_m,
        "near_footprint_free_fraction_first_0p10m": near_fraction,
        "footprint_width": run(footprint_rows),
        "wheel_tracks": wheels,
    }


def grid_image(grid, map_params, robot_params, scale=8):
    width, height = int(map_params["grid_width"]), int(map_params["grid_height"])
    cells = np.asarray(grid, dtype=np.int8).reshape(height, width)
    image = np.full((height, width, 3), (130, 130, 130), dtype=np.uint8)
    image[cells == 0] = (40, 210, 40)
    image[cells == 100] = (20, 20, 230)
    image = cv2.resize(np.flipud(image), (width*scale, height*scale),
                       interpolation=cv2.INTER_NEAREST)
    resolution = float(map_params["resolution_m"])
    origin_x, origin_y = float(map_params["origin_x_m"]), float(map_params["origin_y_m"])
    polygon = []
    for x, y in robot_params["robot"]["simulation_model"]["footprint_xy_m"]:
        px = int(round((float(x)-origin_x)/resolution*scale))
        py = int(round((origin_y+height*resolution-float(y))/resolution*scale))
        polygon.append((px, py))
    cv2.polylines(image, [np.asarray(polygon, dtype=np.int32)], True, (255, 255, 0), 2)
    return image
