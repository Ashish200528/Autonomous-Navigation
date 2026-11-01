import math

import cv2
import numpy as np


FREE = 0
OCCUPIED = 100
UNKNOWN = 255


def _intrinsics(camera_info, width, height):
    if int(camera_info.get("width", 0)) != width or int(camera_info.get("height", 0)) != height:
        raise ValueError("camera_info_dimensions_mismatch")
    k = camera_info.get("k", [])
    if len(k) != 9:
        raise ValueError("camera_info_invalid_K")
    fx, fy = float(k[0]), float(k[4])
    cx, cy = float(k[2]), float(k[5])
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera_info_invalid_intrinsics")
    distortion = [float(x) for x in camera_info.get("d", [])]
    if camera_info.get("distortion_model", "plumb_bob") not in ("plumb_bob", ""):
        raise ValueError("camera_info_unsupported_distortion_model")
    if any(abs(x) > 1e-7 for x in distortion):
        raise ValueError("camera_info_nonzero_distortion_not_rectified")
    return fx, fy, cx, cy


def _ground_band(height, width, camera_info, config):
    fx, fy, cx, cy = _intrinsics(camera_info, width, height)
    rows = np.arange(height, dtype=np.float32).reshape(height, 1)
    down_angle = np.arctan2(rows - cy, fy) + float(config["camera_pitch_down_rad"])
    tangent = np.tan(down_angle)
    valid = tangent > 1e-5
    # Ground range is measured along the optical axis from the camera center.
    # Robot-frame x offset affects the evaluator's world ray origin, not range.
    forward = np.full((height, 1), np.inf, dtype=np.float32)
    forward[valid] = float(config["camera_height_m"]) / tangent[valid]
    valid &= forward >= float(config["min_ground_forward_m"])
    valid &= forward <= float(config["max_ground_forward_m"])
    return np.repeat(valid, width, axis=1), (fx, fy, cx, cy)


def classify_rgb(rgb, camera_info, config):
    """Return a conservative image-plane mask, confidence map, and diagnostics.

    FREE is limited to floor-colour pixels in the configured pinhole ground-plane
    image band that remain connected to the lower-center floor seed. Strongly
    chromatic, vertically extended regions in that band are OCCUPIED evidence.
    Everything else is UNKNOWN. This is not a metric obstacle-distance estimate.
    """
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("image_must_be_uint8_rgb8")
    height, width, _ = rgb.shape
    ground, intrinsics = _ground_band(height, width, camera_info, config)
    fx, fy, cx, cy = intrinsics
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
    lightness = lab[:, :, 0]
    grad_x = cv2.Sobel(lightness, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(lightness, cv2.CV_32F, 0, 1, ksize=3)
    gradient = cv2.magnitude(grad_x, grad_y)

    rows = np.arange(height).reshape(height, 1)
    cols = np.arange(width).reshape(1, width)
    seed_roi = (
        ground
        & (rows >= int(height * float(config["floor_seed_row_fraction"])))
        & (cols >= int(width * float(config["floor_seed_left_fraction"])))
        & (cols < int(width * float(config["floor_seed_right_fraction"])))
    )
    seed_gradients = gradient[seed_roi]
    if seed_gradients.size < int(config["floor_seed_min_pixels"]):
        raise ValueError("floor_seed_unobservable")
    seed_grad_median = float(np.median(seed_gradients))
    seed_grad_mad = float(np.median(np.abs(seed_gradients - seed_grad_median)))
    edge_threshold = max(
        float(config["free_gradient_floor"]),
        seed_grad_median + 3.0 * 1.4826 * seed_grad_mad + 4.0,
    )
    floor_lab = np.asarray(config["floor_reference_lab"], dtype=np.float32)
    if floor_lab.shape != (3,) or not np.all(np.isfinite(floor_lab)):
        raise ValueError("floor_reference_lab_invalid")
    delta = lab - floor_lab.reshape(1, 1, 3)
    d_l = delta[:, :, 0]
    d_a = delta[:, :, 1]
    d_b = delta[:, :, 2]
    color_distance = np.sqrt(d_l ** 2 + 0.5 * d_a ** 2 + 0.5 * d_b ** 2)
    chroma_distance = np.sqrt(d_a ** 2 + d_b ** 2)
    floor_threshold = float(config["floor_color_threshold"])
    calibrated_floor_support = (
        seed_roi
        & (color_distance <= float(config["floor_seed_support_tolerance"]))
        & (gradient <= edge_threshold)
    )
    seed_count = int(np.count_nonzero(calibrated_floor_support))
    if seed_count < int(config["floor_seed_min_pixels"]):
        raise ValueError("floor_seed_unobservable")
    seed = calibrated_floor_support
    free_candidate = (
        ground
        & (color_distance <= floor_threshold)
        & (gradient <= edge_threshold)
    )
    count, labels = cv2.connectedComponents(
        free_candidate.astype(np.uint8), connectivity=8
    )
    seed_labels = np.unique(labels[seed])
    seed_labels = seed_labels[seed_labels > 0]
    if seed_labels.size == 0:
        raise ValueError("floor_seed_disconnected")
    connected_floor = np.isin(labels, seed_labels)
    free_mask = free_candidate & connected_floor
    floor_component_areas = [
        int(np.count_nonzero(labels == label)) for label in seed_labels
    ]
    largest_floor_component_px = max(floor_component_areas, default=0)
    if largest_floor_component_px < int(config["floor_component_min_area_px"]):
        raise ValueError("floor_support_area_too_small")

    def retain_components(candidate, reject_top_border=False):
        component_count, component_labels, stats, _ = cv2.connectedComponentsWithStats(
            candidate.astype(np.uint8), connectivity=8
        )
        kept = np.zeros((height, width), dtype=bool)
        for label in range(1, component_count):
            x, y, w, h, area = (int(v) for v in stats[label])
            if (
                area >= int(config["occupied_component_min_area_px"])
                and h >= int(config["occupied_component_min_height_px"])
                and w >= int(config["occupied_component_min_width_px"])
                and y + h < height - 1
                and (not reject_top_border or y > 0)
            ):
                kept[component_labels == label] = True
        return kept

    # Chromatic object surfaces are useful evidence at any image row. Large
    # non-floor gray surfaces are called occupied only above the geometric
    # horizon; gray shadows on the ground band remain UNKNOWN.
    chroma_candidate = chroma_distance >= float(config["occupied_chroma_delta_min"])
    occupied_chroma = retain_components(chroma_candidate)
    down_angle = np.arctan2(rows - cy, fy) + float(config["camera_pitch_down_rad"])
    above_horizon = np.repeat(down_angle <= 0.0, width, axis=1)
    gray_surface = (
        above_horizon
        & (color_distance >= float(config["occupied_gray_delta_min"]))
        & (gradient <= float(config["occupied_gray_edge_max"]))
    )
    occupied_gray = retain_components(gray_surface, reject_top_border=True)
    # Strong chromatic components are image evidence independent of their
    # ground-plane projection; only the gray-wall heuristic uses the horizon.
    occupied_mask = occupied_chroma | occupied_gray
    occupied_mask &= ~free_mask

    mask = np.full((height, width), UNKNOWN, dtype=np.uint8)
    mask[free_mask] = FREE
    mask[occupied_mask] = OCCUPIED
    confidence = np.zeros((height, width), dtype=np.uint8)
    free_confidence = 255.0 * np.clip(
        1.0 - color_distance / max(floor_threshold * 1.6, 1.0), 0.12, 1.0
    )
    confidence[free_mask] = free_confidence[free_mask].astype(np.uint8)
    occupied_confidence = np.clip(
        108.0 + 2.2 * chroma_distance, 108.0, 235.0
    )
    confidence[occupied_mask] = occupied_confidence[occupied_mask].astype(np.uint8)

    counts = {
        "free_px": int(np.count_nonzero(mask == FREE)),
        "occupied_px": int(np.count_nonzero(mask == OCCUPIED)),
        "unknown_px": int(np.count_nonzero(mask == UNKNOWN)),
        "total_px": int(mask.size),
        "ground_band_px": int(np.count_nonzero(ground)),
        "floor_seed_px": seed_count,
    }
    diagnostics = {
        "intrinsics": {
            "fx_px": intrinsics[0], "fy_px": intrinsics[1],
            "cx_px": intrinsics[2], "cy_px": intrinsics[3],
        },
        "floor_reference_lab": [round(float(x), 3) for x in floor_lab],
        "floor_seed_support_tolerance_lab_weighted": float(config["floor_seed_support_tolerance"]),
        "floor_color_threshold_lab_weighted": round(floor_threshold, 3),
        "floor_seed_gradient_threshold": round(float(edge_threshold), 3),
        "floor_component_largest_px": largest_floor_component_px,
        "floor_component_min_area_px": int(config["floor_component_min_area_px"]),
        "camera_height_assumption_m": float(config["camera_height_m"]),
        "camera_pitch_down_assumption_rad": float(config["camera_pitch_down_rad"]),
        "ground_projection_valid_forward_band_m": [
            float(config["min_ground_forward_m"]),
            float(config["max_ground_forward_m"]),
        ],
        "counts": counts,
    }
    return mask, confidence, diagnostics
