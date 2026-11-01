#!/usr/bin/env python3
"""Run a deterministic Phase 5 world headlessly and capture evaluator evidence.

Gazebo truth is queried by this evaluator only. The runner never publishes truth,
obstacle layouts, or goal data into the ROS graph and does not command motion.
"""
import argparse
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from simulation_interfaces.srv import GetEntities, GetEntityState

from scenario_generator import ROOT, generate_layout, write_plot, build_world_sdf


EVIDENCE = ROOT / "docs" / "evidence" / "phase5" / "runs"


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class Phase5Capture(Node):
    def __init__(self):
        super().__init__("anc_phase5_scenario_evaluator")
        self.image_count = 0
        self.imu_count = 0
        self.odom_count = 0
        self.clock_count = 0
        self.clock_latest = None
        self.tf_edges = {}
        self.create_subscription(Image, "/camera/image_raw",
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data",
                                 self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        static_qos = QoSProfile(depth=100,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, static_qos)
        self.state_client = self.create_client(GetEntityState, "/gzserver/get_entity_state")
        self.entities_client = self.create_client(GetEntities, "/gzserver/get_entities")

    def on_image(self, _msg):
        self.image_count += 1

    def on_imu(self, _msg):
        self.imu_count += 1

    def on_odom(self, _msg):
        self.odom_count += 1

    def on_clock(self, msg):
        self.clock_count += 1
        self.clock_latest = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def on_tf(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            t = transform.transform.translation
            q = transform.transform.rotation
            self.tf_edges[(parent, child)] = {
                "x": float(t.x), "y": float(t.y), "z": float(t.z),
                "yaw": yaw_from_quaternion(q),
                "topic": "tf_static" if parent == "map" and child == "odom" else "tf",
            }

    def spin_until(self, predicate, timeout_s, label):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(predicate())

    def _service_call(self, client, request, label, timeout_s=6.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"Evaluator service unavailable: {label}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"Evaluator service timed out: {label}")
        response = future.result()
        if response is None or response.result.result != 1:
            detail = "no response" if response is None else response.result.error_message
            raise RuntimeError(f"Evaluator service failed ({label}): {detail}")
        return response

    def query_truth(self):
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        response = self._service_call(self.state_client, request, "get_entity_state")
        pose = response.state.pose
        return {
            "sim_time_s": float(response.state.header.stamp.sec) +
                          float(response.state.header.stamp.nanosec) * 1e-9,
            "frame_id": response.state.header.frame_id,
            "x": float(pose.position.x), "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw_rad": yaw_from_quaternion(pose.orientation),
            "linear_x_mps": float(response.state.twist.linear.x),
            "angular_z_radps": float(response.state.twist.angular.z),
        }

    def query_entities(self):
        response = self._service_call(
            self.entities_client, GetEntities.Request(), "get_entities")
        return sorted(str(name) for name in response.entities)


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def validate_tf(node, start, truth):
    map_odom = node.tf_edges.get(("map", "odom"))
    odom_base = node.tf_edges.get(("odom", "base_footprint"))
    result = {
        "map_to_odom_present": map_odom is not None,
        "odom_to_base_footprint_present": odom_base is not None,
        "known_start_matches_launch": False,
        "composed_pose_matches_evaluator_truth": False,
        "map_to_odom": map_odom,
        "odom_to_base_footprint": odom_base,
    }
    if map_odom is None or odom_base is None:
        return result
    start_error_xy = math.hypot(map_odom["x"] - start[0], map_odom["y"] - start[1])
    start_error_yaw = abs(wrap_angle(map_odom["yaw"] - start[2]))
    result["map_to_odom_start_error_xy_m"] = start_error_xy
    result["map_to_odom_start_error_yaw_rad"] = start_error_yaw
    result["known_start_matches_launch"] = start_error_xy < 1e-4 and start_error_yaw < 1e-4

    c, s = math.cos(map_odom["yaw"]), math.sin(map_odom["yaw"])
    composed_x = map_odom["x"] + c * odom_base["x"] - s * odom_base["y"]
    composed_y = map_odom["y"] + s * odom_base["x"] + c * odom_base["y"]
    composed_yaw = wrap_angle(map_odom["yaw"] + odom_base["yaw"])
    dx, dy = composed_x - truth["x"], composed_y - truth["y"]
    dyaw = wrap_angle(composed_yaw - truth["yaw_rad"])
    result["map_to_base_composed"] = {
        "x": composed_x, "y": composed_y, "yaw_rad": composed_yaw,
    }
    result["truth_error_xy_m"] = math.hypot(dx, dy)
    result["truth_error_yaw_rad"] = abs(dyaw)
    result["composed_pose_matches_evaluator_truth"] = (
        result["truth_error_xy_m"] <= 0.15 and result["truth_error_yaw_rad"] <= 0.20
    )
    return result


def stop_process_group(process):
    if process is None:
        return {"signal": None, "returncode": None, "group_gone": True,
                "forced": False}
    sent = None
    forced = False
    try:
        os.killpg(process.pid, signal.SIGINT)
        sent = "SIGINT"
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=25)
    except subprocess.TimeoutExpired:
        forced = True
        try:
            os.killpg(process.pid, signal.SIGTERM)
            sent = "SIGTERM"
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
                sent = "SIGKILL"
            except ProcessLookupError:
                pass
            process.wait(timeout=3)
    # ros2 launch may exit after sending SIGINT while one child remains.
    group_gone = False
    for _ in range(30):
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            group_gone = True
            break
        time.sleep(0.1)
    if not group_gone:
        forced = True
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        for _ in range(20):
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                group_gone = True
                break
            time.sleep(0.1)
    return {"signal": sent, "returncode": process.poll(),
            "group_gone": group_gone, "forced": forced}


def run(args):
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", args.run_id):
        raise ValueError("run-id must contain only letters, digits, dot, underscore, or hyphen")
    run_dir = EVIDENCE / args.run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    layout = generate_layout(args.scenario, args.seed)
    if not layout["validation"]["passed"]:
        raise RuntimeError("Scenario geometry is invalid")
    world_data = build_world_sdf(layout)
    world_path = run_dir / "world.sdf"
    world_path.write_bytes(world_data)
    write_plot(layout, run_dir / "layout.png")
    layout["world_sha256"] = sha256_bytes(world_data)
    layout["world_file"] = str(world_path)
    (run_dir / "manifest.json").write_text(
        json.dumps(layout, indent=2, sort_keys=True) + "\n")
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except Exception:
        commit = "unavailable"
    result = {
        "schema_version": 1,
        "run_id": args.run_id,
        "scenario_id": args.scenario,
        "seed": args.seed,
        "source_commit": commit,
        "sensor_profile": layout["sensor_profile"],
        "layout_sha256": layout["layout_sha256"],
        "world_sha256": layout["world_sha256"],
        "start_xyyaw": layout["start_xyyaw"],
        "goal_xyyaw": layout["goal_xyyaw"],
        "obstacle_count": len(layout["obstacles"]),
        "route_expected": layout["route_expected"],
        "route_found_by_geometry_validator": layout["route_validation"]["found"],
        "navigation_attempted": False,
        "truth_published_to_ros": False,
        "tests": [],
    }

    def add_check(name, passed, detail):
        result["tests"].append({
            "name": name, "status": "PASS" if passed else "FAIL",
            "detail": detail,
        })

    process = None
    log_file = None
    node = None
    shutdown = None
    try:
        rclpy.init()
        node = Phase5Capture()
        log_file = (run_dir / "launch.log").open("w")
        workspace = ROOT / "simulation" / "ros2_ws"
        command = [
            "ros2", "launch", "anc_bringup", "sim.launch.py",
            f"world_file:={world_path}",
            f"start_x:={layout['start_xyyaw'][0]:.6f}",
            f"start_y:={layout['start_xyyaw'][1]:.6f}",
            f"start_yaw:={layout['start_xyyaw'][2]:.6f}",
        ]
        process = subprocess.Popen(
            command, cwd=ROOT, stdout=log_file, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        result["launch_command"] = command
        result["duration_s"] = float(args.duration_s)

        ready = node.spin_until(
            lambda: node.image_count > 0 and node.imu_count > 0 and node.odom_count > 0,
            args.startup_timeout_s, "camera/IMU/odometry")
        add_check("camera_imu_odom_started", ready,
                  f"image={node.image_count}, imu={node.imu_count}, odom={node.odom_count}")
        if not ready:
            raise RuntimeError("Camera, IMU, and odometry did not all start")
        if process.poll() is not None:
            raise RuntimeError(f"Simulation launch exited early with code {process.returncode}")

        truth = node.query_truth()
        entities = node.query_entities()
        result["evaluator_only"] = {
            "truth": truth,
            "entity_names": entities,
            "entity_state_source": "/gzserver/get_entity_state",
            "entity_list_source": "/gzserver/get_entities",
        }
        expected_names = [o["name"] for o in layout["obstacles"]]
        entity_set = set(entities)
        robot_present = "anc_robot" in entity_set
        obstacles_present = all(name in entity_set for name in expected_names)
        add_check("spawned_robot_listed_by_evaluator", robot_present,
                  f"anc_robot listed={robot_present}; entity_count={len(entities)}")
        add_check("all_scenario_obstacles_listed_by_evaluator", obstacles_present,
                  f"expected={expected_names}; observed_match={obstacles_present}")
        pose_error = math.hypot(truth["x"] - layout["start_xyyaw"][0],
                                truth["y"] - layout["start_xyyaw"][1])
        yaw_error = abs(wrap_angle(truth["yaw_rad"] - layout["start_xyyaw"][2]))
        result["evaluator_only"]["spawn_pose_error_xy_m"] = pose_error
        result["evaluator_only"]["spawn_pose_error_yaw_rad"] = yaw_error
        add_check("gazebo_truth_matches_requested_start",
                  pose_error <= 0.06 and yaw_error <= 0.08,
                  f"xy error={pose_error:.4f} m, yaw error={yaw_error:.4f} rad")

        sampling_deadline = time.monotonic() + args.duration_s
        while rclpy.ok() and time.monotonic() < sampling_deadline:
            if process.poll() is not None:
                raise RuntimeError(f"Simulation launch exited during capture: {process.returncode}")
            rclpy.spin_once(node, timeout_sec=0.05)
        tf_result = validate_tf(node, layout["start_xyyaw"], truth)
        result["tf_validation"] = tf_result
        add_check("nonzero_known_start_map_to_odom",
                  tf_result["known_start_matches_launch"],
                  f"map->odom matches start pose={tf_result['known_start_matches_launch']}; "
                  f"start={layout['start_xyyaw']}")
        add_check("map_base_tf_consistent_with_evaluator",
                  tf_result["composed_pose_matches_evaluator_truth"],
                  f"composed truth error={tf_result.get('truth_error_xy_m')} m / "
                  f"{tf_result.get('truth_error_yaw_rad')} rad")

        result["sensor_counts"] = {
            "camera_image_raw": node.image_count,
            "imu_data": node.imu_count,
            "odom": node.odom_count,
            "clock": node.clock_count,
            "last_clock_s": node.clock_latest,
        }
        add_check("camera_stream_seen", node.image_count >= 5,
                  f"{node.image_count} image messages during startup and capture")
        add_check("imu_stream_seen", node.imu_count >= 10,
                  f"{node.imu_count} IMU messages during startup and capture")
        add_check("odometry_stream_seen", node.odom_count >= 5,
                  f"{node.odom_count} odometry messages during startup and capture")
        add_check("clock_stream_seen", node.clock_count >= 5,
                  f"{node.clock_count} clock messages during startup and capture")
        add_check("geometry_route_matches_expected",
                  layout["route_validation"]["found"] == layout["route_expected"],
                  f"grid route={layout['route_validation']['found']}; "
                  f"expected={layout['route_expected']}")
        add_check("layout_validation_passed", layout["validation"]["passed"],
                  f"{sum(1 for c in layout['validation']['checks'] if c['passed'])}/"
                  f"{len(layout['validation']['checks'])} geometry checks passed")
    except Exception as exc:
        result["runner_error"] = f"{type(exc).__name__}: {exc}"
    finally:
        shutdown = stop_process_group(process)
        result["shutdown"] = shutdown
        if log_file:
            log_file.flush()
            log_file.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        add_check("clean_simulation_shutdown",
                  shutdown["group_gone"] and not shutdown["forced"] and
                  shutdown["returncode"] in (0, -signal.SIGINT),
                  json.dumps(shutdown, sort_keys=True))

    result["status"] = ("PASS" if all(t["status"] == "PASS"
                                      for t in result["tests"])
                        else "FAIL")
    (run_dir / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "run_id": args.run_id, "scenario_id": args.scenario,
        "seed": args.seed, "status": result["status"],
        "layout_sha256": result["layout_sha256"],
        "world_sha256": result["world_sha256"],
        "run_dir": str(run_dir),
        "failed_checks": [t["name"] for t in result["tests"]
                          if t["status"] != "PASS"],
        "runner_error": result.get("runner_error"),
        "shutdown": result["shutdown"],
    }, indent=2))
    return 0 if result["status"] == "PASS" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, choices=("S00", "S01", "S02", "S03", "S04", "S09"))
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--startup-timeout-s", type=float, default=75.0)
    args = parser.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
