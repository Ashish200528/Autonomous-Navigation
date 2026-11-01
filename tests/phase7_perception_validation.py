#!/usr/bin/env python3
"""Small deterministic checks for the image-plane classifier."""
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "ros2_ws" / "src" / "anc_perception"))
from anc_perception.classifier import FREE, OCCUPIED, UNKNOWN, classify_rgb

CONFIG = yaml.safe_load(
    (ROOT / "simulation/ros2_ws/src/anc_perception/config/realistic.yaml").read_text()
)["/**"]["ros__parameters"]
INFO = {
    "width": 320, "height": 240,
    "k": [277.128, 0.0, 160.0, 0.0, 277.128, 120.0, 0.0, 0.0, 1.0],
    "d": [0.0, 0.0, 0.0, 0.0, 0.0],
    "distortion_model": "plumb_bob",
}


def check(name, passed, detail):
    print(f'{"PASS" if passed else "FAIL"} {name}: {detail}')
    return bool(passed)


def main():
    results = []
    floor = np.full((240, 320, 3), 150, dtype=np.uint8)
    mask, confidence, detail = classify_rgb(floor, INFO, CONFIG)
    results.append(check(
        "useful_free_on_uniform_floor",
        np.count_nonzero(mask == FREE) > 5000,
        f'free={np.count_nonzero(mask == FREE)} ground_band={detail["counts"]["ground_band_px"]}',
    ))
    results.append(check(
        "above_ground_band_unknown",
        np.all(mask[:140] == UNKNOWN),
        f'unknown_above_horizon_roi={bool(np.all(mask[:140] == UNKNOWN))}',
    ))
    results.append(check(
        "unknown_confidence_zero",
        np.all(confidence[mask == UNKNOWN] == 0),
        f'nonzero_unknown_confidence={np.count_nonzero(confidence[mask == UNKNOWN])}',
    ))

    obstacle = floor.copy()
    obstacle[160:211, 135:185] = (190, 45, 25)
    mask_obstacle, _, detail_obstacle = classify_rgb(obstacle, INFO, CONFIG)
    object_area = mask_obstacle[160:211, 135:185]
    results.append(check(
        "chromatic_unknown_shape_reports_occupied",
        np.count_nonzero(object_area == OCCUPIED) > 200,
        f'occupied_in_generic_rectangle={np.count_nonzero(object_area == OCCUPIED)}',
    ))
    results.append(check(
        "free_regions_remain_useful_with_obstacle",
        np.count_nonzero(mask_obstacle == FREE) > 3000,
        f'free={detail_obstacle["counts"]["free_px"]}',
    ))

    low_contrast = floor.copy()
    low_contrast[160:211, 135:185] = (125, 125, 125)
    mask_low, _, _ = classify_rgb(low_contrast, INFO, CONFIG)
    low_region = mask_low[160:211, 135:185]
    results.append(check(
        "low_contrast_obstacle_not_declared_free",
        np.count_nonzero(low_region == FREE) == 0,
        f'free={np.count_nonzero(low_region == FREE)} unknown={np.count_nonzero(low_region == UNKNOWN)}',
    ))

    occlusion_scene = floor.copy()
    occlusion_scene[150:205, 96:160] = (80, 80, 80)
    mask_occlusion, _, detail_occlusion = classify_rgb(occlusion_scene, INFO, CONFIG)
    occluder = mask_occlusion[150:205, 96:160]
    results.append(check(
        "calibrated_floor_seed_survives_center_occlusion",
        detail_occlusion["counts"]["floor_seed_px"] >= CONFIG["floor_seed_min_pixels"]
        and np.count_nonzero(occluder == FREE) == 0
        and np.count_nonzero(mask_occlusion == FREE) > 3000,
        f'floor_seed={detail_occlusion["counts"]["floor_seed_px"]} '
        f'free_in_occluder={np.count_nonzero(occluder == FREE)} '
        f'free_total={detail_occlusion["counts"]["free_px"]}',
    ))

    hidden_floor = np.full((240, 320, 3), 80, dtype=np.uint8)
    try:
        classify_rgb(hidden_floor, INFO, CONFIG)
        unsupported_floor_rejected = False
    except ValueError as exc:
        unsupported_floor_rejected = "floor_seed_unobservable" in str(exc)
    results.append(check(
        "missing_calibrated_floor_support_fails_closed",
        unsupported_floor_rejected,
        "frame without visible calibrated floor appearance is rejected as unobservable",
    ))

    small_floor_patch = np.full((240, 320, 3), 80, dtype=np.uint8)
    small_floor_patch[210:239, 285:320] = 150
    try:
        classify_rgb(small_floor_patch, INFO, CONFIG)
        small_floor_patch_rejected = False
    except ValueError as exc:
        small_floor_patch_rejected = "floor_support_area_too_small" in str(exc)
    results.append(check(
        "small_ambiguous_floor_patch_fails_closed",
        small_floor_patch_rejected,
        "a calibrated-looking patch below the contiguous support minimum cannot publish FREE",
    ))

    wall_scene = floor.copy()
    wall_scene[:30, :] = (65, 75, 85)  # ambiguous top-edge backdrop
    wall_scene[40:100, :] = (65, 75, 85)  # bounded gray wall surface
    mask_wall, _, _ = classify_rgb(wall_scene, INFO, CONFIG)
    results.append(check(
        "distinct_non_floor_wall_reports_occupied",
        np.count_nonzero(mask_wall[40:100] == OCCUPIED) > 10000,
        f"occupied_wall_px={np.count_nonzero(mask_wall[40:100] == OCCUPIED)}",
    ))

    results.append(check(
        "ambiguous_top_border_stays_unknown",
        np.count_nonzero(mask_wall[:30] == OCCUPIED) == 0,
        f"occupied_top_border_px={np.count_nonzero(mask_wall[:30] == OCCUPIED)}",
    ))

    shadow_scene = floor.copy()
    shadow_scene[165:205, 120:200] = (110, 110, 110)
    mask_shadow, _, _ = classify_rgb(shadow_scene, INFO, CONFIG)
    shadow_patch = mask_shadow[165:205, 120:200]
    results.append(check(
        "gray_ground_shadow_remains_unknown",
        np.count_nonzero(shadow_patch == OCCUPIED) == 0
        and np.count_nonzero(shadow_patch == FREE) == 0,
        f"free={np.count_nonzero(shadow_patch == FREE)} "
        f"occupied={np.count_nonzero(shadow_patch == OCCUPIED)} "
        f"unknown={np.count_nonzero(shadow_patch == UNKNOWN)}",
    ))

    bad_info = dict(INFO)
    bad_info["d"] = [0.1, 0.0, 0.0, 0.0, 0.0]
    try:
        classify_rgb(floor, bad_info, CONFIG)
        distortion_rejected = False
    except ValueError as exc:
        distortion_rejected = "nonzero_distortion" in str(exc)
    results.append(check(
        "unrectified_distortion_rejected",
        distortion_rejected,
        "nonzero D is not silently treated as rectified",
    ))
    print(f"SUMMARY: {sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
