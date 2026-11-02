#!/usr/bin/env python3
"""Bounded live test: nonzero request is always stopped in Phase 8."""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import yaml

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from phase8b_camera_metrics import corridor_metrics, grid_image


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec)*1e-9


class Monitor(Node):
    def __init__(self):
        super().__init__("phase8_live_safety_test")
        self.request_pub = self.create_publisher(Twist, "/cmd_vel/requested", 10)
        self.cmd_samples = []
        self.grid = None
        self.map_state = None
        self.supervisor_state = None
        self.health = None
        self.camera_image = None
        self.mask_image = None
        self.confidence_image = None
        self.odom = None
        self.localization_covariance = None
        self.contact_messages = 0
        self.contact_events = 0
        self.non_ground_contact_events = 0
        self.contact_pair_examples = []
        self.contact_pair_seen = set()
        self.tf_edges = set()
        self.tf_static_edges = set()
        self.rows = []
        self.t0 = time.monotonic()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(OccupancyGrid, "/realistic/local_traversability", self.on_grid, 10)
        self.create_subscription(String, "/realistic/navigation_state", self.on_map, 10)
        self.create_subscription(String, "/realistic/supervisor_state", self.on_supervisor, 10)
        self.create_subscription(String, "/perception/health", self.on_health, 10)
        self.create_subscription(ROSImage, "/camera/image_raw", self.on_camera_image, qos_profile_sensor_data)
        self.create_subscription(ROSImage, "/perception/image_mask", self.on_mask_image, qos_profile_sensor_data)
        self.create_subscription(ROSImage, "/perception/confidence", self.on_confidence_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Contacts, "/contacts", self.on_contacts, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf_static, static_qos)
        self.truth = self.create_client(GetEntityState, "/gzserver/get_entity_state")

    def on_cmd(self, msg):
        nonzero = any(abs(float(v)) > 1e-12 for v in (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z))
        t = time.monotonic() - self.t0
        sample = {"wall_elapsed_s": round(t, 4), "linear_x": float(msg.linear.x),
                  "angular_z": float(msg.angular.z), "nonzero": nonzero}
        self.cmd_samples.append(sample)
        self.rows.append({"wall_elapsed_s": t, "event": "cmd_vel_output",
                          "linear_x": float(msg.linear.x), "angular_z": float(msg.angular.z),
                          "nonzero": nonzero})

    def on_grid(self, msg):
        self.grid = msg

    def on_map(self, msg):
        try:
            self.map_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.map_state = {"status": "INVALID_UNKNOWN", "reason": "bad_json"}

    def on_supervisor(self, msg):
        try:
            self.supervisor_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.supervisor_state = {"status": "STOP", "reason": "bad_json"}

    def on_health(self, msg):
        try:
            self.health = json.loads(msg.data)
        except json.JSONDecodeError:
            self.health = {"valid": False, "status": "INVALID_UNKNOWN", "reason": "bad_json"}

    def on_camera_image(self, msg):
        self.camera_image = msg

    def on_mask_image(self, msg):
        self.mask_image = msg

    def on_confidence_image(self, msg):
        self.confidence_image = msg

    def on_odom(self, msg):
        self.odom = msg
        covariance = list(msg.pose.covariance)
        self.localization_covariance = {
            "pose_x_variance_m2": float(covariance[0]),
            "pose_y_variance_m2": float(covariance[7]),
            "pose_yaw_variance_rad2": float(covariance[35]),
            "source_topic": "/odometry/filtered",
        }

    def on_tf(self, msg):
        for transform in msg.transforms:
            self.tf_edges.add((transform.header.frame_id.lstrip("/"),
                               transform.child_frame_id.lstrip("/")))

    def on_tf_static(self, msg):
        for transform in msg.transforms:
            self.tf_static_edges.add((transform.header.frame_id.lstrip("/"),
                                      transform.child_frame_id.lstrip("/")))

    def on_contacts(self, msg):
        self.contact_messages += 1
        for contact in msg.contacts:
            self.contact_events += 1
            c1, c2 = contact.collision1.name, contact.collision2.name
            pair = (c1, c2)
            if pair not in self.contact_pair_seen and len(self.contact_pair_examples) < 12:
                self.contact_pair_seen.add(pair)
                self.contact_pair_examples.append({"collision1": c1, "collision2": c2,
                                                   "max_depth_m": max(contact.depths, default=0.0)})
            if "ground_plane" not in (c1 + " " + c2).lower():
                self.non_ground_contact_events += 1

    def query_truth(self, timeout=8.0):
        if not self.truth.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("evaluator_service_missing:/gzserver/get_entity_state")
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.truth.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("evaluator_truth_query_timeout")
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError("evaluator_truth_query_failed:" + response.result.error_message)
        p = response.state.pose.position
        q = response.state.pose.orientation
        return {"sim_time_s": stamp_seconds(response.state.header.stamp),
                "x_m": float(p.x), "y_m": float(p.y), "z_m": float(p.z),
                "yaw_quaternion": [float(q.x), float(q.y), float(q.z), float(q.w)],
                "linear_x_mps": float(response.state.twist.linear.x),
                "angular_z_radps": float(response.state.twist.angular.z),
                "source": "Gazebo GetEntityState evaluator-only; not subscribed by runtime nodes"}

    def spin_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if predicate():
                return True
        raise RuntimeError("timeout_waiting_for:" + description)

    def request_for(self, duration_s, linear_x=0.10, angular_z=0.0):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.request_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def mem_available_kib():
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1])
    except OSError:
        pass
    return None


def process_sample():
    try:
        raw = subprocess.run(["ps", "-eo", "comm,%cpu,rss"], capture_output=True,
                             text=True, timeout=2.0, check=True).stdout
        rows = []
        for line in raw.splitlines()[1:]:
            bits = line.split()
            if len(bits) >= 3 and any(token in bits[0].lower()
                                      for token in ("gz", "gazebo", "python3", "ekf")):
                rows.append({"process": bits[0], "cpu_percent": float(bits[1]),
                             "rss_kib": int(bits[2])})
        return rows
    except Exception:
        return []


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="S00")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--world-file", type=Path, required=True)
    parser.add_argument("--camera-enabled", choices=("true", "false"), default="true")
    parser.add_argument("--expect-invalid-camera", action="store_true",
                        help="Expect the configured RGB candidate to fail closed and save raw/mask evidence")
    parser.add_argument("--camera-candidate-mode", choices=("valid", "either"),
                        default="valid",
                        help="In either mode, accept valid or invalid camera health for a bounded geometry check")
    parser.add_argument("--odometry-topic", default="/odometry/filtered")
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--camera-mount-z-m", type=float, default=0.10)
    parser.add_argument("--camera-pitch-down-rad", type=float, default=0.0)
    parser.add_argument("--camera-height-m", type=float, default=0.18)
    parser.add_argument("--min-ground-forward-m", type=float, default=0.50)
    parser.add_argument("--max-ground-forward-m", type=float, default=0.60)
    parser.add_argument("--floor-seed-row-fraction", type=float, default=0.90)
    parser.add_argument(
        "--output-root", type=Path,
        default=ROOT / "docs/evidence/phase8/runs",
        help="Evidence directory root; defaults to the Phase 8 accepted runs directory",
    )
    args = parser.parse_args()
    out_dir = args.output_root / args.run_id
    out_dir.mkdir(parents=True, exist_ok=True)
    launch_log_path = out_dir / "launch.log"
    launch_log = launch_log_path.open("w")
    launch = [
        "ros2", "launch", "anc_realistic_navigation", "phase8_realistic.launch.py",
        f"world_file:={args.world_file.resolve()}",
        f"start_x:={args.start_x}", f"start_y:={args.start_y}",
        f"start_yaw:={args.start_yaw}", f"camera_enabled:={args.camera_enabled}",
        f"odometry_topic:={args.odometry_topic}",
        f"camera_mount_z_m:={args.camera_mount_z_m}",
        f"camera_pitch_down_rad:={args.camera_pitch_down_rad}",
        f"camera_height_m:={args.camera_height_m}",
        f"min_ground_forward_m:={args.min_ground_forward_m}",
        f"max_ground_forward_m:={args.max_ground_forward_m}",
        f"floor_seed_row_fraction:={args.floor_seed_row_fraction}",
    ]
    proc = subprocess.Popen(launch, cwd=ROOT, stdout=launch_log, stderr=subprocess.STDOUT,
                            start_new_session=True, text=True)
    result = {"phase": 8, "run_id": args.run_id, "scenario": args.scenario,
              "seed": args.seed, "camera_enabled": args.camera_enabled == "true",
              "requested_velocity_mps": 0.10, "duration_s": args.duration,
              "status": "FAIL", "launch_command": launch,
              "evaluator_truth_used_at_runtime": False}
    monitor = None
    rclpy.init()
    try:
        monitor = Monitor()
        if args.expect_invalid_camera:
            monitor.spin_until(
                lambda: monitor.health is not None and monitor.health.get("valid") is False
                and monitor.grid is not None and all(v == -1 for v in monitor.grid.data),
                30.0, "candidate_fail_closed_unknown_grid",
            )
        elif args.camera_candidate_mode == "either":
            def candidate_state_ready():
                if monitor.health is None or monitor.grid is None:
                    return False
                if monitor.health.get("valid") is False:
                    return all(v == -1 for v in monitor.grid.data)
                if monitor.health.get("valid") is True:
                    return (monitor.map_state is not None
                            and not monitor.map_state.get("input_failures", [])
                            and monitor.map_state.get("camera_transform") is not None)
                return False
            monitor.spin_until(candidate_state_ready, 45.0,
                               "candidate_camera_health_and_synchronized_projection")
        elif args.camera_enabled == "true" and args.odometry_topic == "/odometry/filtered":
            monitor.spin_until(
                lambda: monitor.health is not None and monitor.health.get("valid") is True
                and monitor.map_state is not None
                and monitor.map_state.get("counts", {}).get("free_grid_cells", 0) > 0,
                75.0, "valid_rgb_and_projected_free_cells",
            )
        elif args.camera_enabled == "true":
            monitor.spin_until(
                lambda: monitor.health is not None and monitor.health.get("valid") is True
                and monitor.map_state is not None
                and any("filtered_localization" in reason
                        for reason in monitor.map_state.get("input_failures", [])),
                75.0, "valid_rgb_and_missing_localization_stop",
            )
        else:
            monitor.spin_until(
                lambda: monitor.health is not None and monitor.health.get("valid") is False
                and monitor.grid is not None and all(v == -1 for v in monitor.grid.data),
                45.0, "camera_absent_all_unknown_grid",
            )
        # Wait for the delayed robot spawn before querying evaluator-only pose.
        monitor.spin_until(lambda: monitor.odom is not None, 45.0, "spawned_robot_odometry")
        rclpy.spin_once(monitor, timeout_sec=0.3)
        publishers = monitor.get_publishers_info_by_topic("/cmd_vel")
        publisher_names = sorted({f"{p.node_namespace.rstrip('/')}/{p.node_name}"
                                  for p in publishers})
        node_subscriptions = {}
        for node_name in ("anc_rgb_perception", "anc_local_traversability",
                          "anc_realistic_safe_stop_supervisor"):
            try:
                node_subscriptions[node_name] = sorted(
                    name for name, _types in
                    monitor.get_subscriber_names_and_types_by_node(node_name, "/")
                )
            except Exception as exc:
                node_subscriptions[node_name] = {"error": f"{type(exc).__name__}: {exc}"}
        topic_publishers = {}
        for topic in ("/cmd_vel", "/tf", "/tf_static"):
            topic_publishers[topic] = sorted({
                f"{p.node_namespace.rstrip('/')}/{p.node_name}"
                for p in monitor.get_publishers_info_by_topic(topic)
            })
        truth_before = monitor.query_truth()
        mem_start = mem_available_kib()
        samples = []
        max_process_rss_kib = 0
        max_process_cpu_percent = 0.0
        last_resource_sample = 0.0
        duration_deadline = time.monotonic() + args.duration
        while time.monotonic() < duration_deadline:
            req = Twist()
            req.linear.x = 0.10
            monitor.request_pub.publish(req)
            monitor.rows.append({"wall_elapsed_s": time.monotonic()-monitor.t0,
                                 "event": "requested_nonzero_command", "linear_x": 0.10,
                                 "angular_z": 0.0, "nonzero": True})
            rclpy.spin_once(monitor, timeout_sec=0.05)
            now_sample = time.monotonic()
            ps = process_sample() if now_sample-last_resource_sample >= 0.5 else []
            if ps:
                last_resource_sample = now_sample
                samples.extend(ps)
                max_process_rss_kib = max(max_process_rss_kib,
                    max(x["rss_kib"] for x in ps))
                max_process_cpu_percent = max(max_process_cpu_percent,
                    max(x["cpu_percent"] for x in ps))
        truth_after = monitor.query_truth()
        silence_deadline = time.monotonic() + 0.55
        while time.monotonic() < silence_deadline:
            rclpy.spin_once(monitor, timeout_sec=0.05)
        dx = truth_after["x_m"]-truth_before["x_m"]
        dy = truth_after["y_m"]-truth_before["y_m"]
        displacement = math.hypot(dx, dy)
        zero_count = sum(not sample["nonzero"] for sample in monitor.cmd_samples)
        nonzero_count = len(monitor.cmd_samples)-zero_count
        grid_message = monitor.grid
        grid_data = list(grid_message.data) if grid_message else []
        grid_counts = {"free": grid_data.count(0), "occupied": grid_data.count(100),
                       "unknown": grid_data.count(-1)}
        map_params = {
            "grid_width": int(grid_message.info.width) if grid_message else 0,
            "grid_height": int(grid_message.info.height) if grid_message else 0,
            "resolution_m": float(grid_message.info.resolution) if grid_message else 0.0,
            "origin_x_m": float(grid_message.info.origin.position.x) if grid_message else 0.0,
            "origin_y_m": float(grid_message.info.origin.position.y) if grid_message else 0.0,
        }
        robot_params = yaml.safe_load(
            (ROOT / "config/robot_parameters.yaml").read_text()
        )
        projected_corridor = (
            corridor_metrics(grid_data, map_params, robot_params)
            if grid_data else None
        )
        cmd_zero = bool(monitor.cmd_samples) and nonzero_count == 0
        localization_uncertainty = None
        if monitor.localization_covariance is not None:
            cov = monitor.localization_covariance
            sigma_xy = math.sqrt(max(cov["pose_x_variance_m2"],
                                     cov["pose_y_variance_m2"], 0.0))
            localization_uncertainty = {
                **cov,
                "sigma_xy_m": sigma_xy,
                "three_sigma_xy_allowance_m": 3.0 * sigma_xy,
                "interpretation": "sqrt(max EKF x/y pose variance); evaluator truth is excluded",
            }
        publisher_audit = (len(publisher_names) == 1 and
                           publisher_names[0].endswith("/anc_realistic_safe_stop_supervisor"))
        map_failures = (monitor.map_state or {}).get("input_failures", [])
        obstacle_pixels = (monitor.map_state or {}).get("counts", {}).get(
            "occupied_image_pixels_unlocalized", 0)
        base_checks = {
            "single_cmd_vel_publisher_is_phase8_supervisor": publisher_audit,
            "requested_nonzero_command_seen": bool(
                (monitor.supervisor_state or {}).get("requested_command_count", 0) > 0),
            "every_actual_cmd_vel_sample_zero": cmd_zero,
            "evaluator_truth_displacement_le_0_02m": displacement <= 0.02,
            "supervisor_reports_motion_lock": bool(
                (monitor.supervisor_state or {}).get("phase8_motion_lock") is True),
            "controller_timeout_keeps_output_zero": bool(
                (monitor.supervisor_state or {}).get("watchdog_state") ==
                "controller_command_timeout_stop" and cmd_zero),
            "runtime_graph_has_no_nav2_nodes": not any(
                "nav2" in name.lower() or "map_server" in name.lower()
                for name in monitor.get_node_names()),
            "no_known_start_map_to_odom_transform": (
                "anc_known_start_map_to_odom" not in monitor.get_node_names() and
                ("map", "odom") not in monitor.tf_edges and
                ("map", "odom") not in monitor.tf_static_edges),
            "perception_subscribes_only_camera_and_clock": set(
                node_subscriptions.get("anc_rgb_perception", [])
            ) == {"/camera/image_raw", "/camera/camera_info", "/clock"},
            "supervisor_subscriptions_are_isolated": set(
                node_subscriptions.get("anc_realistic_safe_stop_supervisor", [])
            ) == {"/clock", "/cmd_vel/requested", "/realistic/navigation_state"},
            "occupancy_grid_never_marks_obstacles": grid_counts["occupied"] == 0,
        }
        if args.expect_invalid_camera or (
            args.camera_candidate_mode == "either" and
            monitor.health is not None and monitor.health.get("valid") is False
        ):
            base_checks["candidate_camera_invalid_fails_closed"] = bool(
                monitor.health and monitor.health.get("valid") is False)
            base_checks["invalid_candidate_grid_all_unknown"] = grid_counts == {
                "free": 0, "occupied": 0, "unknown": len(grid_data)}
        elif args.camera_enabled == "true":
            base_checks["camera_health_valid"] = bool(monitor.health and monitor.health.get("valid") is True)
            if args.odometry_topic == "/odometry/filtered":
                base_checks["some_geometrically_projected_free_cells"] = grid_counts["free"] > 0
                base_checks["perception_state_reports_forward_unknown_block"] = bool(
                    monitor.map_state and "forward_corridor_not_proven_free" in
                    monitor.map_state.get("block_reasons", []))
                if obstacle_pixels:
                    base_checks["unlocalized_occupied_evidence_blocks"] = (
                        "unlocalized_occupied_evidence" in monitor.map_state.get("block_reasons", []))
            else:
                base_checks["missing_localization_invalidates_grid"] = bool(
                    monitor.map_state and any("filtered_localization" in reason
                        for reason in monitor.map_state.get("input_failures", []))
                    and grid_counts == {"free": 0, "occupied": 0, "unknown": len(grid_data)})
        else:
            base_checks["camera_health_invalid_unknown"] = bool(
                monitor.health and monitor.health.get("valid") is False)
            base_checks["no_camera_means_all_grid_unknown"] = grid_counts == {
                "free": 0, "occupied": 0, "unknown": len(grid_data)}
        result.update({
            "status": "PASS" if all(base_checks.values()) else "FAIL",
            "checks": base_checks,
            "actual_cmd_vel": {"samples": len(monitor.cmd_samples), "zero_samples": zero_count,
                               "nonzero_samples": nonzero_count,
                               "publisher_nodes": publisher_names},
            "request_count": (monitor.supervisor_state or {}).get("requested_command_count"),
            "map_state": monitor.map_state,
            "supervisor_state": monitor.supervisor_state,
            "perception_health": monitor.health,
            "grid_counts": grid_counts,
            "camera_geometry_candidate": {
                "mount_z_relative_to_base_m": args.camera_mount_z_m,
                "camera_height_above_floor_m": args.camera_height_m,
                "pitch_down_rad": args.camera_pitch_down_rad,
                "min_ground_forward_m": args.min_ground_forward_m,
                "max_ground_forward_m": args.max_ground_forward_m,
            },
            "projected_corridor_metrics": projected_corridor,
            "runtime_graph": {
                "node_subscriptions": node_subscriptions,
                "topic_publishers": topic_publishers,
                "dynamic_tf_edges": sorted(monitor.tf_edges),
                "static_tf_edges": sorted(monitor.tf_static_edges),
                "note": "Topic-level publisher ownership is observed; per-edge /tf publisher attribution is not provided by current ROS message metadata.",
            },
            "unlocalized_occupied_image_pixels": obstacle_pixels,
            "evaluator_truth_before": truth_before,
            "evaluator_truth_after": truth_after,
            "evaluator_displacement_m": displacement,
            "contact_telemetry": {"messages": monitor.contact_messages,
                                  "ground_or_surface_events": monitor.contact_events-monitor.non_ground_contact_events,
                                  "non_ground_contact_events": monitor.non_ground_contact_events,
                                  "collision_pair_examples": monitor.contact_pair_examples},
            "resources": {"mem_available_kib_start": mem_start,
                          "mem_available_kib_end": mem_available_kib(),
                          "max_sampled_process_rss_kib": max_process_rss_kib,
                          "max_sampled_process_cpu_percent": max_process_cpu_percent,
                          "process_samples": len(samples)},
            "camera_metrics": (monitor.health or {}).get("measured_wall_fps"),
            "camera_sim_age_s": (monitor.health or {}).get("camera_sim_age_s"),
            "perception_latency_ms": (monitor.health or {}).get("processing_latency_ms"),
            "localization_runtime_input": args.odometry_topic,
            "localization_uncertainty": localization_uncertainty,
            "evaluator_truth_runtime_input": False,
        })
        if grid_data:
            (out_dir / "local-grid.json").write_text(json.dumps({
                "geometry": map_params, "data": grid_data,
                "counts": grid_counts, "connected_corridor": projected_corridor,
            }, indent=2, sort_keys=True) + "\n")
            cv2 = __import__("cv2")
            cv2.imwrite(str(out_dir / "projected-local-grid.png"),
                        grid_image(grid_data, map_params, robot_params))
        for label, msg in (("camera-rgb.png", monitor.camera_image),
                           ("perception-mask.png", monitor.mask_image),
                           ("perception-confidence.png", monitor.confidence_image)):
            if msg is None:
                continue
            import numpy as np
            cv2 = __import__("cv2")
            channels = 1 if msg.encoding == "mono8" else 3
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            if channels == 1:
                image = raw[:, :msg.width].copy()
            else:
                image = raw[:, :msg.width * channels].reshape(msg.height, msg.width, channels).copy()
                if msg.encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(out_dir / label), image)
        (out_dir / "camera-image-metadata.json").write_text(json.dumps({
            label: ({"encoding": msg.encoding, "width": msg.width, "height": msg.height,
                     "step": msg.step, "stamp": stamp_seconds(msg.header.stamp)}
                    if msg is not None else None)
            for label, msg in (("camera-rgb.png", monitor.camera_image),
                               ("perception-mask.png", monitor.mask_image),
                               ("perception-confidence.png", monitor.confidence_image))
        }, indent=2, sort_keys=True) + "\n")
        with (out_dir / "telemetry.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "wall_elapsed_s", "event", "linear_x", "angular_z", "nonzero"])
            writer.writeheader()
            writer.writerows(monitor.rows)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if monitor is not None:
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=18.0)
            result["launch_shutdown"] = {"signal": "SIGINT", "return_code": proc.returncode}
        except Exception as exc:
            result["launch_shutdown"] = {"signal": "SIGINT", "error": str(exc)}
            try:
                proc.terminate()
                proc.wait(timeout=4.0)
                result["launch_shutdown"]["escalation"] = "SIGTERM"
                result["launch_shutdown"]["return_code"] = proc.returncode
            except Exception:
                proc.kill()
                proc.wait(timeout=3.0)
                result["launch_shutdown"]["escalation"] = "SIGKILL"
                result["launch_shutdown"]["return_code"] = proc.returncode
        launch_log.close()
        result_path = out_dir / "result.json"
        result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        print(json.dumps({
            "run_id": result.get("run_id"), "scenario": result.get("scenario"),
            "seed": result.get("seed"), "status": result.get("status"),
            "checks": result.get("checks"),
            "evaluator_displacement_m": result.get("evaluator_displacement_m"),
            "actual_cmd_vel": result.get("actual_cmd_vel"),
            "grid_counts": result.get("grid_counts"),
            "input_failures": (result.get("map_state") or {}).get("input_failures"),
            "contact_telemetry": result.get("contact_telemetry"),
            "launch_shutdown": result.get("launch_shutdown"),
            "error": result.get("error"),
        }, indent=2, sort_keys=True))
    return 0 if result.get("status") == "PASS" else 1

if __name__ == "__main__":
    raise SystemExit(main())
