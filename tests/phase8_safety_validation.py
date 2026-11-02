#!/usr/bin/env python3
"""Geometry and fail-closed invariants for the Phase 8 RGB local map."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation/ros2_ws/src/anc_realistic_navigation"))
from anc_realistic_navigation.projection import project_free_pixels


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "docs/evidence/phase8/phase8-static-validation.json",
        help="JSON output path; defaults to the accepted Phase 8 evidence path",
    )
    args = parser.parse_args()
    checks = []
    def check(name, passed, detail):
        checks.append({"name": name, "status": "PASS" if passed else "FAIL",
                       "detail": detail})

    h, w = 240, 320
    info = {"width": w, "height": h, "frame_id": "camera_optical_frame",
            "distortion_model": "plumb_bob", "d": [0, 0, 0, 0, 0],
            "k": [277.128, 0, 160, 0, 277.128, 120, 0, 0, 1]}
    q = (-0.5, 0.5, -0.5, 0.5)  # URDF optical frame rotation into base_link.
    mask = np.full((h, w), 255, dtype=np.uint8)
    confidence = np.zeros((h, w), dtype=np.uint8)
    mask[204:220, :] = 0
    confidence[204:220, :] = 225
    grid, counts = project_free_pixels(mask, confidence, info, (0.10, 0.0, 0.10), q)
    check("free_pixels_project_to_metric_cells", counts["free_grid_cells"] > 0,
          f"projected={counts['projected_free_pixels']}, cells={counts['free_grid_cells']}")
    check("outside_observed_floor_remains_unknown",
          counts["grid_unknown_cells"] > counts["free_grid_cells"],
          f"unknown={counts['grid_unknown_cells']}, free={counts['free_grid_cells']}")
    check("occupancy_semantics_never_fabricate_obstacles",
          100 not in grid and counts["grid_occupied_cells"] == 0,
          "no RGB OCCUPIED pixel is assigned an arbitrary metric range")
    free_ix = [i % 75 for i, value in enumerate(grid) if value == 0]
    first_x = min(free_ix) * 0.02 if free_ix else None
    gap = first_x - 0.215 if first_x is not None else None
    check("free_patch_disconnected_from_robot_front",
          gap is not None and gap > 0.30, f"first cell x={first_x}; gap={gap}")

    obstacle_mask = np.full((h, w), 255, dtype=np.uint8)
    obstacle_conf = np.zeros((h, w), dtype=np.uint8)
    obstacle_mask[80:180, 90:230] = 100
    obstacle_conf[80:180, 90:230] = 220
    obstacle_grid, obstacle_counts = project_free_pixels(
        obstacle_mask, obstacle_conf, info, (0.10, 0.0, 0.10), q)
    check("unlocalized_occupied_evidence_stays_unknown",
          obstacle_counts["occupied_image_pixels_unlocalized"] > 0 and
          obstacle_counts["grid_occupied_cells"] == 0 and
          all(v == -1 for v in obstacle_grid),
          f"occupied pixels={obstacle_counts['occupied_image_pixels_unlocalized']}; metric occupied cells=0")

    low_conf = confidence.copy()
    low_conf[mask == 0] = 199
    low_grid, low_counts = project_free_pixels(mask, low_conf, info, (0.10, 0.0, 0.10), q)
    check("low_confidence_free_does_not_enter_grid",
          low_counts["free_grid_cells"] == 0 and all(v == -1 for v in low_grid),
          f"free grid cells={low_counts['free_grid_cells']}")

    bad_info = dict(info)
    bad_info["d"] = [0.01, 0, 0, 0, 0]
    rejected = False
    try:
        project_free_pixels(mask, confidence, bad_info, (0.10, 0.0, 0.10), q)
    except ValueError as exc:
        rejected = "distortion" in str(exc)
    check("nonzero_distortion_invalidates_projection", rejected,
          "nonzero CameraInfo distortion rejected")

    result = {"phase": 8, "status": "PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL",
              "checks": checks, "camera_info": info,
              "camera_transform_assumption": {"translation_base_m": [0.10, 0.0, 0.10],
                                              "quaternion_base": list(q)},
              "grid_state_counts": counts,
              "interpretation": "Synthetic unit fixtures validate projection invariants only; live Gazebo evidence is reported separately."}
    out = args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1

if __name__ == "__main__":
    raise SystemExit(main())
