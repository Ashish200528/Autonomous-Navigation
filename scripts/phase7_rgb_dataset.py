#!/usr/bin/env python3
"""Capture fixed-seed RGB views; withheld Gazebo truth is evaluation-only."""
import argparse
import hashlib
import re
import json
import math
import os
import shutil
import subprocess
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String

from anc_perception.perception_node import _decode_rgb
from phase7_ground_truth import ground_truth_mask
from scenario_generator import ROOT, build_world_sdf, generate_layout, point_is_free
from scenario_runner import stop_process_group, yaw_from_quaternion


EVIDENCE = ROOT / "docs" / "evidence" / "phase7"
RUNS = EVIDENCE / "dataset"
CONFIG_PATH = ROOT / "simulation/ros2_ws/src/anc_perception/config/realistic.yaml"
INPUTS = ["/camera/image_raw", "/camera/camera_info", "/clock"]


def stamp_key(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def seconds_key(value):
    if value is None:
        return None
    return int(round(float(value) * 1_000_000_000))


def put(cache, key, value, limit=40):
    cache[key] = value
    cache.move_to_end(key)
    while len(cache) > limit:
        cache.popitem(last=False)


def mono_array(msg):
    if msg.encoding != "mono8":
        raise ValueError(f"Expected mono8, received {msg.encoding}")
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        int(msg.height), int(msg.step)
    )
    return raw[:, :int(msg.width)].copy()


def colorize(mask):
    output = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    output[mask == 0] = (35, 220, 35)
    output[mask == 100] = (20, 20, 240)
    output[mask == 255] = (230, 120, 20)
    return output


def pixel_metrics(prediction, truth):
    gt_free, gt_occ = truth == 0, truth == 100
    pred_free, pred_occ = prediction == 0, prediction == 100
    n_free, n_occ = int(np.count_nonzero(gt_free)), int(np.count_nonzero(gt_occ))
    n_pred_occ = int(np.count_nonzero(pred_occ))
    tp_free = int(np.count_nonzero(pred_free & gt_free))
    tp_occ = int(np.count_nonzero(pred_occ & gt_occ))
    false_free = int(np.count_nonzero(pred_free & gt_occ))
    known = n_free + n_occ
    return {
        "prediction_counts_px": {
            "free": int(np.count_nonzero(pred_free)),
            "occupied": n_pred_occ,
            "unknown": int(np.count_nonzero(prediction == 255)),
            "total": int(prediction.size),
        },
        "evaluator_truth_counts_px": {
            "free": n_free,
            "occupied": n_occ,
            "unknown": int(np.count_nonzero(truth == 255)),
        },
        "known_truth_px": known,
        "true_free_px": tp_free,
        "true_occupied_px": tp_occ,
        "false_free_px_on_known_occupied": false_free,
        "false_free_rate_on_known_occupied": false_free / n_occ if n_occ else None,
        "occupied_precision": tp_occ / n_pred_occ if n_pred_occ else None,
        "occupied_recall": tp_occ / n_occ if n_occ else None,
        "useful_free_coverage_on_known_floor": tp_free / n_free if n_free else None,
        "unknown_coverage_total": float(np.count_nonzero(prediction == 255) / prediction.size),
        "known_truth_classification_coverage": (
            float(np.count_nonzero((prediction != 255) & (truth != 255)) / known)
            if known else None
        ),
    }


def _header_rate(stamps):
    if len(stamps) < 2:
        return {"count": len(stamps), "rate_hz": None}
    diffs = np.diff(np.asarray(stamps, dtype=np.float64)) / 1e9
    diffs = diffs[diffs > 0]
    return {
        "count": len(stamps),
        "median_interval_s": float(np.median(diffs)) if diffs.size else None,
        "rate_hz": float(1.0 / np.median(diffs)) if diffs.size else None,
        "max_interval_s": float(np.max(diffs)) if diffs.size else None,
    }


class CaptureNode(Node):
    """Evaluator only: records sensor outputs and queries withheld Gazebo pose."""
    def __init__(self):
        super().__init__("anc_phase7_evaluation_harness")
        self.images, self.masks = OrderedDict(), OrderedDict()
        self.confidences, self.overlays, self.health = (
            OrderedDict(), OrderedDict(), OrderedDict()
        )
        self.health_history = []
        self.camera_info = None
        self.clock_seconds = None
        self.image_stamps, self.clock_stamps = [], []
        self.truth = self.create_client(GetEntityState, "/gzserver/get_entity_state")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image_raw", self._image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._info, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_sim_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(Image, "/perception/image_mask",
                                 lambda m: put(self.masks, stamp_key(m.header.stamp), m),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "/perception/confidence",
                                 lambda m: put(self.confidences, stamp_key(m.header.stamp), m),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "/perception/overlay",
                                 lambda m: put(self.overlays, stamp_key(m.header.stamp), m),
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/perception/health", self._health, 20)

    def _image(self, msg):
        key = stamp_key(msg.header.stamp)
        put(self.images, key, msg)
        self.image_stamps.append(key)
        if len(self.image_stamps) > 3000:
            self.image_stamps = self.image_stamps[-1500:]

    def _info(self, msg):
        self.camera_info = msg

    def on_sim_clock(self, msg):
        self.clock_seconds = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.clock_stamps.append(int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec))
        if len(self.clock_stamps) > 3000:
            self.clock_stamps = self.clock_stamps[-1500:]

    def _odom(self, _msg):
        pass  # Evaluator graph visibility only; odometry never labels pixels.

    def _health(self, msg):
        try:
            health = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.health_history.append(health)
        if len(self.health_history) > 3000:
            self.health_history = self.health_history[-1500:]
        key = seconds_key(health.get("image_stamp_s"))
        if key is not None:
            put(self.health, key, health)

    def clear_run(self):
        for cache in (self.images, self.masks, self.confidences, self.overlays, self.health):
            cache.clear()
        self.health_history.clear()
        self.camera_info = None
        self.clock_seconds = None
        self.image_stamps.clear()
        self.clock_stamps.clear()

    def bundle(self, after_ns=0):
        for key in reversed(self.images):
            if key <= after_ns:
                continue
            if all(key in cache for cache in (
                self.masks, self.confidences, self.overlays, self.health
            )):
                return {
                    "key": key,
                    "image": self.images[key],
                    "mask": self.masks[key],
                    "confidence": self.confidences[key],
                    "overlay": self.overlays[key],
                    "health": self.health[key],
                }
        return None

    def spin_until(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.03)
        return bool(predicate())

    def query_truth(self):
        if not self.truth.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("Gazebo evaluator truth service unavailable")
        last_error = None
        for attempt in range(4):
            request = GetEntityState.Request()
            request.entity = "anc_robot"
            future = self.truth.call_async(request)
            end = time.monotonic() + 1.5
            while rclpy.ok() and time.monotonic() < end and not future.done():
                rclpy.spin_once(self, timeout_sec=0.03)
            if not future.done() or future.result() is None:
                last_error = "Gazebo evaluator truth query timed out"
            else:
                response = future.result()
                if response.result.result == 1:
                    state = response.state
                    return {
                        "sim_time_s": float(state.header.stamp.sec) + float(state.header.stamp.nanosec) * 1e-9,
                        "frame_id": state.header.frame_id,
                        "x": float(state.pose.position.x),
                        "y": float(state.pose.position.y),
                        "z": float(state.pose.position.z),
                        "yaw_rad": yaw_from_quaternion(state.pose.orientation),
                        "linear_x_mps": float(state.twist.linear.x),
                        "angular_z_radps": float(state.twist.angular.z),
                    }
                last_error = response.result.error_message or "Gazebo entity state query failed"
                if "not found" not in last_error.lower():
                    raise RuntimeError(last_error)
            if attempt < 3:
                time.sleep(0.15)
        raise RuntimeError(f"Gazebo evaluator truth query failed after retries: {last_error}")

    def command(self, angular):
        msg = Twist()
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def rotate_and_settle(self):
        start = self.clock_seconds
        if start is None:
            raise RuntimeError("Simulation clock unavailable before turn.")
        deadline = time.monotonic() + 12.0
        while rclpy.ok() and (self.clock_seconds or start) - start < 1.25:
            if time.monotonic() >= deadline:
                self.command(0.0)
                raise RuntimeError("Viewpoint rotation timed out.")
            self.command(0.28)
            rclpy.spin_once(self, timeout_sec=0.03)
        self.command(0.0)
        settle = (self.clock_seconds or start) + 0.65
        self.spin_until(
            lambda: self.clock_seconds is not None and self.clock_seconds >= settle,
            8.0, "viewpoint rotation settle",
        )
        self.command(0.0)
def _challenge_layout():
    layout = generate_layout("S01", 101)
    layout["base_phase5_scenario"] = "S01"
    layout["scenario_id"] = "P7C1"
    layout["scenario_name"] = "low_contrast_unnamed_box_with_shadow"
    layout["phase7_challenge"] = "low_contrast_box_and_directional_shadow"
    layout["start_xyyaw"] = [-1.10, -0.20, 0.0]
    if not point_is_free(layout["start_xyyaw"][0], layout["start_xyyaw"][1], layout):
        raise RuntimeError("Phase 7 close-view start is not geometry-clear.")
    layout["obstacles"][0]["visual_material"] = [
        "0.45 0.45 0.45 1", "0.55 0.55 0.55 1"
    ]
    canonical = json.dumps(
        {k: v for k, v in layout.items() if k != "layout_sha256"},
        sort_keys=True, separators=(",", ":"),
    )
    layout["layout_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return layout


def _process_rss_snapshot():
    records = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
            if not any(name in cmd for name in ("gzserver", "rgb_perception_node")):
                continue
            status = (entry / "status").read_text(errors="ignore")
            line = next(line for line in status.splitlines() if line.startswith("VmRSS:"))
            records.append({
                "pid": int(entry.name),
                "rss_kib": int(line.split()[1]),
                "command": cmd[:180],
            })
        except (OSError, StopIteration, ValueError):
            continue
    return records


def _save_sample(node, bundle, layout, case_dir, view_id, config, motion=None):
    image_msg = bundle["image"]
    if image_msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
        raise ValueError(f"Unsupported captured image encoding {image_msg.encoding}")
    rgb = _decode_rgb(image_msg)
    prediction = mono_array(bundle["mask"])
    confidence = mono_array(bundle["confidence"])
    overlay_msg = bundle["overlay"]
    overlay = np.frombuffer(overlay_msg.data, dtype=np.uint8).reshape(
        int(overlay_msg.height), int(overlay_msg.step)
    )[:, :int(overlay_msg.width) * 3].reshape(
        int(overlay_msg.height), int(overlay_msg.width), 3
    ).copy()
    if prediction.shape != rgb.shape[:2] or confidence.shape != prediction.shape:
        raise ValueError("Perception output dimensions differ from RGB input.")
    info_msg = node.camera_info
    if info_msg is None:
        raise ValueError("CameraInfo missing during RGB capture.")
    info = {
        "width": int(info_msg.width), "height": int(info_msg.height),
        "k": [float(x) for x in info_msg.k],
        "d": [float(x) for x in info_msg.d],
        "distortion_model": info_msg.distortion_model,
        "frame_id": info_msg.header.frame_id,
    }
    truth_pose = node.query_truth()
    gt = ground_truth_mask(rgb.shape[:2], info, layout, truth_pose, config)
    metrics = pixel_metrics(prediction, gt)
    stamp_ns = bundle["key"]
    sample_dir = case_dir / view_id
    sample_dir.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(sample_dir / "rgb_input.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(sample_dir / "prediction_mask.png"), prediction)
    cv2.imwrite(str(sample_dir / "prediction_confidence.png"), confidence)
    cv2.imwrite(str(sample_dir / "prediction_overlay.png"), overlay)
    cv2.imwrite(str(sample_dir / "prediction_mask_color.png"), colorize(prediction))
    cv2.imwrite(str(sample_dir / "evaluator_truth_mask.png"), gt)
    cv2.imwrite(str(sample_dir / "evaluator_truth_color.png"), colorize(gt))
    data = {
        "scenario_id": layout["scenario_id"],
        "base_phase5_scenario": layout.get("base_phase5_scenario", layout["scenario_id"]),
        "seed": int(layout["seed"]),
        "viewpoint_id": view_id,
        "runtime_profile": "REALISTIC_RGB_ONLY",
        "perception_input_topics": [
            "/camera/image_raw", "/camera/camera_info", "/clock"
        ],
        "perception_forbidden_inputs": [
            "/odom", "/imu/data", "/map", "/depth", "/scan",
            "/gzserver/get_entity_state", "scenario geometry",
        ],
        "image_stamp_ns": stamp_ns,
        "image_stamp_s": stamp_ns / 1e9,
        "image_frame_id": image_msg.header.frame_id,
        "camera_info": info,
        "perception_health": bundle["health"],
        "evaluator_truth_pose": truth_pose,
        "truth_query_time_minus_image_stamp_s": (
            truth_pose["sim_time_s"] - stamp_ns / 1e9
        ),
        "viewpoint_rotation_evaluator_only": motion,
        "metrics": metrics,
        "ground_truth_source": (
            "Offline ray intersections against deterministic scenario geometry "
            "and Gazebo truth pose; withheld from the runtime ROS perception node."
        ),
        "files": {
            "rgb": "rgb_input.png",
            "prediction_mask": "prediction_mask.png",
            "confidence": "prediction_confidence.png",
            "overlay": "prediction_overlay.png",
            "evaluator_truth": "evaluator_truth_mask.png",
        },
        "process_rss_snapshot": _process_rss_snapshot(),
    }
    (sample_dir / "result.json").write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n"
    )
    return {
        "scenario_id": data["scenario_id"],
        "base_phase5_scenario": data["base_phase5_scenario"],
        "seed": data["seed"],
        "viewpoint_id": view_id,
        "sample_dir": str(sample_dir.relative_to(ROOT)),
        "health": bundle["health"],
        "metrics": metrics,
        "truth_query_time_minus_image_stamp_s": data["truth_query_time_minus_image_stamp_s"],
        "process_rss_snapshot": data["process_rss_snapshot"],
    }


def _perception_subscriber_info(node, timeout_sec=8.0):
    """Read the remote node's actual subscriptions from the live ROS graph."""
    expected = set(INPUTS)
    deadline = time.monotonic() + float(timeout_sec)
    subscriptions = []
    last_error = None
    while time.monotonic() < deadline:
        try:
            subscriptions = node.get_subscriber_names_and_types_by_node(
                "anc_rgb_perception", "/"
            )
            names = {name for name, _types in subscriptions}
            if expected.issubset(names):
                break
        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    names = {name for name, _types in subscriptions}
    rows = ["Node: /anc_rgb_perception", "Subscriptions:"]
    rows.extend(
        f"  {name}: {', '.join(types)}"
        for name, types in sorted(subscriptions)
    )
    if last_error:
        rows.append(f"Last graph query error: {last_error}")
    return "\n".join(rows) + "\n", names


def _run_case(node, layout, case_id, config):
    case_dir = RUNS / case_id
    case_dir.mkdir(parents=True, exist_ok=False)
    world_path = case_dir / "world.sdf"
    world_data = build_world_sdf(layout)
    world_path.write_bytes(world_data)
    layout["world_file"] = str(world_path)
    layout["world_sha256"] = hashlib.sha256(world_data).hexdigest()
    (case_dir / "manifest.json").write_text(
        json.dumps(layout, indent=2, sort_keys=True) + "\n"
    )
    node.clear_run()
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"anc_phase7_{case_id}"
    env["ROS_LOG_DIR"] = str(case_dir / "ros_logs")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    x, y, yaw = layout["start_xyyaw"]
    command = [
        "ros2", "launch", "anc_perception", "realistic_perception.launch.py",
        f"world_file:={world_path}",
        f"start_x:={float(x):.6f}",
        f"start_y:={float(y):.6f}",
        f"start_yaw:={float(yaw):.6f}",
    ]
    launch_log_path = case_dir / "launch.log"
    process, shutdown, samples = None, None, []
    start_wall = time.monotonic()
    run_contract = {}
    try:
        with launch_log_path.open("w") as launch_log:
            process = subprocess.Popen(
                command, cwd=ROOT, env=env, stdout=launch_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            ready = node.spin_until(
                lambda: node.clock_seconds is not None
                and node.bundle() is not None
                and node.bundle()["health"].get("valid", False),
                75.0, "valid synchronized RGB perception output",
            )
            if not ready:
                bundle = node.bundle()
                raise RuntimeError(
                    "No valid RGB perception bundle: "
                    + (str(bundle["health"]) if bundle else "no synchronized output")
                )
            settle_to = float(node.clock_seconds or 0.0) + 0.8
            node.spin_until(
                lambda: node.clock_seconds is not None and node.clock_seconds >= settle_to,
                8.0, "initial camera settling",
            )
            first = node.bundle()
            if first is None:
                raise RuntimeError("Missing synchronized start-view bundle.")
            samples.append(_save_sample(node, first, layout, case_dir, "view_start", config))

            truth_before = node.query_truth()
            node.rotate_and_settle()
            truth_after = node.query_truth()
            motion = {
                "xy_change_m": math.hypot(
                    truth_after["x"] - truth_before["x"],
                    truth_after["y"] - truth_before["y"],
                ),
                "yaw_change_rad": math.atan2(
                    math.sin(truth_after["yaw_rad"] - truth_before["yaw_rad"]),
                    math.cos(truth_after["yaw_rad"] - truth_before["yaw_rad"]),
                ),
                "source": "evaluator-only Gazebo pose; not a perception input",
            }
            second = node.bundle(after_ns=first["key"])
            if second is None:
                node.spin_until(
                    lambda: node.bundle(after_ns=first["key"]) is not None,
                    8.0, "new camera output after safe in-place turn",
                )
                second = node.bundle(after_ns=first["key"])
            if second is None:
                raise RuntimeError("Missing synchronized post-turn bundle.")
            samples.append(_save_sample(
                node, second, layout, case_dir, "view_turned", config, motion
            ))
        node_info, observed = _perception_subscriber_info(node)
        (case_dir / "perception-node-info.txt").write_text(node_info)
        if observed != set(INPUTS):
            raise RuntimeError(
                f"Perception ROS input contract differs: {sorted(observed)}"
            )
        node_info_ok = True
        free_text = subprocess.run(
            ["free", "-h"], text=True, stdout=subprocess.PIPE, check=False
        ).stdout
        health_path = case_dir / "runtime-health.jsonl"
        health_path.write_text(
            "".join(json.dumps(x, sort_keys=True) + "\n" for x in node.health_history)
        )
        timing_rows = []
        for h in node.health_history:
            timing_rows.append({
                "image_stamp_s": h.get("image_stamp_s"),
                "valid": h.get("valid"),
                "camera_sim_age_s": h.get("camera_sim_age_s"),
                "camera_wall_age_s": h.get("camera_wall_age_s"),
                "processing_latency_ms": h.get("processing_latency_ms"),
                "measured_wall_fps": h.get("measured_wall_fps"),
                "free_px": h.get("counts", {}).get("free_px"),
                "occupied_px": h.get("counts", {}).get("occupied_px"),
                "unknown_px": h.get("counts", {}).get("unknown_px"),
            })
        timing_path = case_dir / "runtime-health.csv"
        columns = list(timing_rows[0]) if timing_rows else [
            "image_stamp_s", "valid", "camera_sim_age_s", "camera_wall_age_s",
            "processing_latency_ms", "measured_wall_fps",
            "free_px", "occupied_px", "unknown_px",
        ]
        import csv
        with timing_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(timing_rows)
        run_contract = {
            "profile": "REALISTIC_RGB_ONLY",
            "observed_perception_subscriber_topics": sorted(observed),
            "expected_perception_subscriber_topics": INPUTS,
            "contract_passed": node_info_ok,
            "perception_node_info": node_info,
            "camera_header_timing": _header_rate(node.image_stamps),
            "clock_timing": _header_rate(node.clock_stamps),
            "health_sample_count": len(node.health_history),
            "health_samples_file": "runtime-health.jsonl",
            "health_timing_csv": "runtime-health.csv",
            "free_memory_snapshot": free_text,
            "process_rss_sampled": _process_rss_snapshot(),
            "launch_command": command,
            "reference_map_loaded": False,
            "nav2_started": False,
            "gazebo_truth_subscribed_by_perception": False,
        }
        (case_dir / "runtime-contract.json").write_text(
            json.dumps(run_contract, indent=2, sort_keys=True) + "\n"
        )
    finally:
        node.command(0.0)
        shutdown = stop_process_group(process)
        (case_dir / "shutdown.json").write_text(
            json.dumps(shutdown, indent=2, sort_keys=True) + "\n"
        )
    return {
        "case_id": case_id,
        "run_directory": str(case_dir.relative_to(ROOT)),
        "scenario_id": layout["scenario_id"],
        "seed": int(layout["seed"]),
        "start_xyyaw": layout["start_xyyaw"],
        "sample_count": len(samples),
        "samples": samples,
        "launch_command": command,
        "elapsed_wall_s": round(time.monotonic() - start_wall, 3),
        "perception_node_input_contract_passed": node_info_ok,
        "camera_header_timing": run_contract["camera_header_timing"],
        "clock_timing": run_contract["clock_timing"],
        "health_sample_count": len(node.health_history),
        "shutdown": shutdown,
        "status": (
            "PASS" if len(samples) == 2 and node_info_ok
            and shutdown and shutdown["group_gone"] else "FAIL"
        ),
        "limitations": (
            ["Gazebo child required signal escalation during teardown."]
            if shutdown and (
                shutdown.get("child_sigterm_escalated")
                or shutdown.get("child_sigkill_escalated")
            ) else []
        ),
    }


def _aggregate(samples):
    counts = {
        "pred_free": 0, "pred_occupied": 0, "pred_unknown": 0,
        "gt_free": 0, "gt_occupied": 0, "tp_free": 0, "tp_occupied": 0,
        "false_free": 0, "total_px": 0,
    }
    for sample in samples:
        m = sample["metrics"]
        p, g = m["prediction_counts_px"], m["evaluator_truth_counts_px"]
        counts["pred_free"] += p["free"]
        counts["pred_occupied"] += p["occupied"]
        counts["pred_unknown"] += p["unknown"]
        counts["gt_free"] += g["free"]
        counts["gt_occupied"] += g["occupied"]
        counts["tp_free"] += m["true_free_px"]
        counts["tp_occupied"] += m["true_occupied_px"]
        counts["false_free"] += m["false_free_px_on_known_occupied"]
        counts["total_px"] += p["total"]
    nf, no, npocc = counts["gt_free"], counts["gt_occupied"], counts["pred_occupied"]
    total = counts["total_px"] or 1
    return {
        "pixel_totals": counts,
        "false_free_rate_on_known_occupied": counts["false_free"] / no if no else None,
        "occupied_precision": counts["tp_occupied"] / npocc if npocc else None,
        "occupied_recall": counts["tp_occupied"] / no if no else None,
        "useful_free_coverage_on_known_floor": counts["tp_free"] / nf if nf else None,
        "unknown_coverage_total": counts["pred_unknown"] / total,
    }


def _summarize_health(results):
    latency, fps, age, wall_age = [], [], [], []
    for result in results:
        path = ROOT / result.get("run_directory", "") / "runtime-health.jsonl"
        if not path.exists():
            continue
        for line in path.read_text().splitlines():
            h = json.loads(line)
            if not h.get("valid"):
                continue
            for target, key in (
                (latency, "processing_latency_ms"),
                (fps, "measured_wall_fps"),
                (age, "camera_sim_age_s"),
                (wall_age, "camera_wall_age_s"),
            ):
                if h.get(key) is not None:
                    target.append(float(h[key]))

    def stats(values):
        if not values:
            return {"count": 0, "p50": None, "p95": None, "max": None}
        return {
            "count": len(values),
            "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
            "max": float(max(values)),
        }

    return {
        "processing_latency_ms": stats(latency),
        "measured_wall_fps": {
            "count": len(fps),
            "p10": float(np.percentile(fps, 10)) if fps else None,
            "median": float(np.median(fps)) if fps else None,
            "min": min(fps) if fps else None,
        },
        "camera_sim_timestamp_age_s": stats(age),
        "camera_wall_age_s": stats(wall_age),
    }


def main():
    global RUNS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="accepted-final")
    parser.add_argument("--scenarios", default="S00,S01,S02,S03,S04,P7C1")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,60}", args.run_id):
        raise ValueError("run-id must contain only letters, digits, dot, underscore, or hyphen")
    RUNS = EVIDENCE / "dataset" / args.run_id
    RUNS.mkdir(parents=True, exist_ok=False)
    selected = {item.strip() for item in args.scenarios.split(",") if item.strip()}
    config = yaml.safe_load(CONFIG_PATH.read_text())["/**"]["ros__parameters"]
    os.environ["ROS_DOMAIN_ID"] = "73"
    rclpy.init()
    node = CaptureNode()
    all_cases = [
        ("S00", 101, generate_layout("S00", 101)),
        ("S01", 101, generate_layout("S01", 101)),
        ("S02", 101, generate_layout("S02", 101)),
        ("S03", 101, generate_layout("S03", 101)),
        ("S04", 202, generate_layout("S04", 202)),
        ("P7C1", 101, _challenge_layout()),
    ]
    valid_ids = {item[0] for item in all_cases}
    if not selected or not selected <= valid_ids:
        raise ValueError(f"scenarios must be a subset of {sorted(valid_ids)}")
    cases = [item for item in all_cases if item[0] in selected]
    results = []
    disk_before = shutil.disk_usage(ROOT)
    try:
        for scenario_id, seed, layout in cases:
            case_id = f"{scenario_id.lower()}-seed{seed}-rgb-only"
            print(f"START {case_id}", flush=True)
            try:
                result = _run_case(node, layout, case_id, config)
            except Exception as exc:
                result = {
                    "case_id": case_id, "scenario_id": scenario_id,
                    "seed": seed, "status": "FAIL",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            results.append(result)
            print(json.dumps({
                "case_id": case_id, "status": result["status"],
                "samples": result.get("sample_count"),
                "error": result.get("error"),
                "shutdown": result.get("shutdown"),
            }, sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    samples = [s for result in results for s in result.get("samples", [])]
    aggregate = _aggregate(samples)
    aggregate.update(_summarize_health(results))
    disk_after = shutil.disk_usage(ROOT)
    report = {
        "phase": 7,
        "profile": "REALISTIC_RGB_ONLY",
        "source_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
            stdout=subprocess.PIPE, check=False,
        ).stdout.strip(),
        "scenario_cases": results,
        "sample_count": len(samples),
        "aggregate_metrics": aggregate,
        "runtime_inputs_to_anc_perception": INPUTS,
        "runtime_outputs": [
            "/perception/image_mask", "/perception/confidence",
            "/perception/overlay", "/perception/health",
        ],
        "forbidden_runtime_data": [
            "/odom", "/imu/data", "/map", "/depth", "/scan",
            "/gzserver/get_entity_state", "scenario manifests",
        ],
        "extra_challenge": (
            "P7C1 derives from S01 seed 101, uses a floor-like gray unnamed box "
            "at a clear viewpoint; the existing directional light casts shadows."
        ),
        "resource_observation": {
            "free_memory_after": subprocess.run(
                ["free", "-h"], text=True, stdout=subprocess.PIPE, check=False
            ).stdout,
            "disk_free_before_bytes": int(disk_before.free),
            "disk_free_after_bytes": int(disk_after.free),
            "disk_bytes_added": int(disk_before.free - disk_after.free),
            "process_rss_sampled": [
                record for result in results
                for sample in result.get("samples", [])
                for record in sample.get("process_rss_snapshot", [])
            ],
        },
        "assessment": "PENDING_SENSOR_FAULT_TEST_AND_FINAL_REVIEW",
    }
    path = RUNS / "dataset-results.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "sample_count": len(samples),
        "scenario_statuses": [
            [x.get("case_id"), x.get("status")] for x in results
        ],
        "aggregate_metrics": aggregate,
        "report": str(path),
    }, indent=2), flush=True)
    return 0 if all(x.get("status") == "PASS" for x in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
