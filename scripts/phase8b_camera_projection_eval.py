#!/usr/bin/env python3
"""Evaluate candidate RGB masks as base-relative Phase 8B traversability grids."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation/ros2_ws/src/anc_realistic_navigation"))
sys.path.insert(0, str(ROOT / "scripts"))
from phase8b_camera_metrics import corridor_metrics, grid_image, project_mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--additional-dataset-dir", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    dataset_roots = [args.dataset_dir.resolve(),
                     *(value.resolve() for value in args.additional_dataset_dir)]
    if any(ROOT not in root.parents for root in dataset_roots):
        raise ValueError("all dataset directories must remain inside the repository")
    output = (args.output.resolve() if args.output else
              dataset_roots[0] / "camera-projection-evaluation.json")
    if ROOT not in output.parents:
        raise ValueError("output must remain inside the repository")
    robot = yaml.safe_load((ROOT / "config/robot_parameters.yaml").read_text())
    map_params = {
        "grid_width": 75, "grid_height": 75, "resolution_m": 0.02,
        "origin_x_m": 0.0, "origin_y_m": -0.75,
        "floor_z_in_base_m": -0.08, "min_free_confidence": 200,
    }
    samples = []
    aggregate = {
        "known_truth_px": 0, "truth_free_px": 0, "truth_occupied_px": 0,
        "false_free_px": 0, "true_free_px": 0, "true_occupied_px": 0,
        "predicted_unknown_px": 0, "predicted_free_px": 0,
    }
    projection_root = output.parent / (output.stem + "-grids")
    projection_root.mkdir(parents=True, exist_ok=True)
    result_paths = [
        (dataset_root, result_path)
        for dataset_root in dataset_roots
        for result_path in sorted(dataset_root.glob("*/view_*/result.json"))
    ]
    for dataset_root, result_path in result_paths:
        data = json.loads(result_path.read_text())
        if not data.get("camera_transform_base_link"):
            raise ValueError(f"missing measured base-camera TF in {result_path}")
        mask = cv2.imread(str(result_path.parent / data["files"]["prediction_mask"]),
                          cv2.IMREAD_GRAYSCALE)
        confidence = cv2.imread(str(result_path.parent / data["files"]["confidence"]),
                                cv2.IMREAD_GRAYSCALE)
        if mask is None or confidence is None:
            raise ValueError(f"missing mask/confidence image in {result_path.parent}")
        camera_info = dict(data["camera_info"])
        projection, counts = project_mask(
            mask, confidence, camera_info, data["camera_transform_base_link"], map_params
        )
        projection = np.asarray(projection, dtype=np.int8).reshape(
            map_params["grid_height"], map_params["grid_width"]
        )
        corridor = corridor_metrics(projection.ravel(), map_params, robot)
        key = "/".join((dataset_root.name, *result_path.relative_to(dataset_root).parts[:-1]))
        grid_dir = projection_root / key
        grid_dir.mkdir(parents=True, exist_ok=True)
        np.save(grid_dir / "projected-grid.npy", projection)
        cv2.imwrite(str(grid_dir / "projected-grid.png"),
                    grid_image(projection.ravel(), map_params, robot))
        metrics = data["metrics"]
        truth_counts = metrics.get("evaluator_truth_counts_px", {})
        pred_counts = metrics.get("prediction_counts_px", {})
        aggregate["known_truth_px"] += int(metrics.get("known_truth_px", 0))
        aggregate["truth_free_px"] += int(truth_counts.get("free", 0))
        aggregate["truth_occupied_px"] += int(truth_counts.get("occupied", 0))
        aggregate["false_free_px"] += int(metrics.get("false_free_px_on_known_occupied", 0))
        aggregate["true_free_px"] += int(metrics.get("true_free_px", 0))
        aggregate["true_occupied_px"] += int(metrics.get("true_occupied_px", 0))
        aggregate["predicted_unknown_px"] += int(pred_counts.get("unknown", 0))
        aggregate["predicted_free_px"] += int(pred_counts.get("free", 0))
        samples.append({
            "dataset_run_id": dataset_root.name,
            "case_id": result_path.relative_to(dataset_root).parts[0],
            "scenario_id": data["scenario_id"],
            "seed": int(data["seed"]),
            "viewpoint_id": data["viewpoint_id"],
            "result_path": str(result_path.relative_to(ROOT)),
            "camera_transform_base_link": data["camera_transform_base_link"],
            "grid_counts": counts,
            "corridor": corridor,
            "pixel_metrics": metrics,
        })
    truth_occupied = aggregate["truth_occupied_px"]
    aggregate["false_free_rate_on_known_occupied"] = (
        aggregate["false_free_px"] / truth_occupied if truth_occupied else None
    )
    aggregate["occupied_recall"] = (
        aggregate["true_occupied_px"] / truth_occupied if truth_occupied else None
    )
    aggregate["useful_free_coverage_on_known_floor"] = (
        aggregate["true_free_px"] / aggregate["truth_free_px"]
        if aggregate["truth_free_px"] else None
    )
    aggregate["unknown_coverage_total"] = (
        aggregate["predicted_unknown_px"] /
        max(1, sum(int(s["pixel_metrics"]["prediction_counts_px"]["total"])
                   for s in samples))
    )
    corridors = [s["corridor"] for s in samples]
    report = {
        "phase": "8B",
        "dataset_dirs": [str(value.relative_to(ROOT)) for value in dataset_roots],
        "sample_count": len(samples),
        "map_geometry": map_params,
        "aggregate_pixel_metrics": aggregate,
        "projected_corridor_summary": {
            "samples_with_connected_full_footprint_free_from_front": sum(
                c["footprint_width"]["connected_free_length_from_front_m"] > 0.0
                for c in corridors
            ),
            "min_connected_full_footprint_free_m": min(
                (c["footprint_width"]["connected_free_length_from_front_m"]
                 for c in corridors), default=None),
            "max_connected_full_footprint_free_m": max(
                (c["footprint_width"]["connected_free_length_from_front_m"]
                 for c in corridors), default=None),
            "min_near_footprint_free_fraction": min(
                (c["near_footprint_free_fraction_first_0p10m"]
                 for c in corridors), default=None),
            "samples_with_both_wheel_tracks_connected": sum(
                all(c["wheel_tracks"][side]["connected_free_length_from_front_m"] > 0.0
                    for side in ("left", "right"))
                for c in corridors
            ),
        },
        "samples": samples,
        "runtime_boundary": (
            "Camera masks are projected using only recorded CameraInfo, measured "
            "base-to-camera TF, and the documented static grid geometry. Evaluator "
            "ground truth is used only for per-pixel metrics and never enters runtime."
        ),
    }
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "sample_count": len(samples),
        "aggregate_pixel_metrics": aggregate,
        "projected_corridor_summary": report["projected_corridor_summary"],
        "output": str(output),
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
