#!/usr/bin/env python3
"""Verify the parameterized launch still uses the Phase 2 default world."""
import json
import math
import subprocess
import sys
import time

import rclpy

from scenario_runner import ROOT, Phase5Capture, stop_process_group, validate_tf

RUN_DIR = ROOT / "docs" / "evidence" / "phase5" / "default-world-regression"


def check(items, name, passed, detail):
    items.append({"name": name, "status": "PASS" if passed else "FAIL",
                  "detail": detail})


def main():
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    log = (RUN_DIR / "launch.log").open("w")
    checks = []
    node = None
    process = None
    shutdown = None
    report = {"scope": "default launch regression; no robot command or navigation"}
    try:
        rclpy.init()
        node = Phase5Capture()
        process = subprocess.Popen(
            ["ros2", "launch", "anc_bringup", "sim.launch.py"],
            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        ready = node.spin_until(
            lambda: node.image_count > 0 and node.imu_count > 0 and node.odom_count > 0,
            75.0, "default-world sensors")
        check(checks, "default_world_sensor_start", ready,
              f"camera={node.image_count}, imu={node.imu_count}, odom={node.odom_count}")
        if not ready:
            raise RuntimeError("Default launch did not start all sensor streams")
        truth = node.query_truth()
        entities = node.query_entities()
        phase2_wall = "phase2_wall" in set(entities)
        check(checks, "phase2_default_world_loaded", phase2_wall,
              f"phase2_wall listed={phase2_wall}; entity_count={len(entities)}")
        pose_error = math.hypot(truth["x"], truth["y"])
        yaw_error = abs(math.atan2(math.sin(truth["yaw_rad"]),
                                   math.cos(truth["yaw_rad"])))
        check(checks, "zero_start_default_preserved",
              pose_error < 0.06 and yaw_error < 0.08,
              f"truth xy error={pose_error:.5f} m; yaw error={yaw_error:.5f} rad")
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        tf = validate_tf(node, [0.0, 0.0, 0.0], truth)
        check(checks, "zero_known_start_tf_preserved",
              tf["known_start_matches_launch"] and
              tf["composed_pose_matches_evaluator_truth"],
              json.dumps(tf, sort_keys=True))
        report["truth_evaluator_only"] = truth
        report["entities_evaluator_only"] = entities
        report["tf_validation"] = tf
        report["sensor_counts"] = {
            "camera": node.image_count, "imu": node.imu_count,
            "odom": node.odom_count, "clock": node.clock_count,
        }
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        shutdown = stop_process_group(process)
        report["shutdown"] = shutdown
        log.flush()
        log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        check(checks, "default_launch_clean_shutdown",
              shutdown["group_gone"] and not shutdown["forced"] and
              shutdown["returncode"] in (0, -2),
              json.dumps(shutdown, sort_keys=True))

    report["checks"] = checks
    report["status"] = "PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL"
    (RUN_DIR / "result.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "status": report["status"],
        "failed_checks": [c["name"] for c in checks if c["status"] != "PASS"],
        "result": str(RUN_DIR / "result.json"),
        "shutdown": report["shutdown"],
    }, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
