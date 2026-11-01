#!/usr/bin/env python3
"""Repeatable Phase 4 evaluator; Gazebo truth is evaluator-only.

Run while anc_bringup sim.launch.py is active. Only this script requests
GetEntityState; truth is saved locally and never published to ROS or the EKF.
"""
import argparse
import bisect
import csv
import json
import math
import subprocess
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from simulation_interfaces.srv import GetEntityState
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs" / "evidence"


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def gid_string(value):
    if value is None:
        return "unknown"
    if isinstance(value, str):
        return value.replace(".", "").replace(":", "").lower()
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        return str(value)


class LocalizationTrial(Node):
    def __init__(self, mode, slip_mu=0.02, slip_speed_mps=0.10, slip_duration_s=4.0):
        super().__init__("phase4_localization_evaluator")
        self.mode = mode
        self.slip_mu = float(slip_mu)
        self.slip_speed_mps = float(slip_speed_mps)
        self.slip_duration_s = float(slip_duration_s)
        self.phase = "startup"
        self.samples = []
        self.truth = []
        self.phases = []
        self.tf_edges = {"/tf": {}, "/tf_static": {}}
        self.tf_rows = []
        self.tf_message_info_debug = None
        self.clock_stamps = []
        self.latest = {"raw": None, "wheel": None, "filtered": None}
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", lambda m: self.on_odom("raw", m), 20)
        self.create_subscription(Odometry, "/odometry/wheel", lambda m: self.on_odom("wheel", m), 20)
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: self.on_odom("filtered", m), 20)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf_static, static_qos)
        self.truth_client = self.create_client(GetEntityState, "/gzserver/get_entity_state")
        self.imu_parameter_client = self.create_client(SetParameters,
                                                        "/anc_imu_sanitizer/set_parameters")
        self.imu_dropout = {"requested": False, "during_count": None,
                            "filtered_count": None, "imu_recovery_count": None}

    def add_sample(self, row):
        row["scenario"] = self.mode
        row["phase"] = self.phase
        self.samples.append(row)

    def on_odom(self, topic, msg):
        pose, twist = msg.pose.pose, msg.twist.twist
        row = {
            "topic": topic, "sim_time_s": stamp_seconds(msg.header.stamp),
            "x": float(pose.position.x), "y": float(pose.position.y),
            "z": float(pose.position.z), "yaw": yaw_from_quaternion(pose.orientation),
            "vx": float(twist.linear.x), "vy": float(twist.linear.y),
            "wz": float(twist.angular.z),
            "pose_var_x": float(msg.pose.covariance[0]),
            "pose_var_y": float(msg.pose.covariance[7]),
            "pose_var_yaw": float(msg.pose.covariance[35]),
            "twist_var_x": float(msg.twist.covariance[0]),
            "twist_var_y": float(msg.twist.covariance[7]),
            "twist_var_yaw": float(msg.twist.covariance[35]),
            "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id,
        }
        self.latest[topic] = row
        self.add_sample(row)

    def on_imu(self, msg):
        self.add_sample({
            "topic": "imu", "sim_time_s": stamp_seconds(msg.header.stamp),
            "gyro_z": float(msg.angular_velocity.z),
            "gyro_var_z": float(msg.angular_velocity_covariance[8]),
            "accel_x": float(msg.linear_acceleration.x),
            "accel_y": float(msg.linear_acceleration.y),
            "accel_z": float(msg.linear_acceleration.z),
            "orientation_covariance_0": float(msg.orientation_covariance[0]),
            "frame_id": msg.header.frame_id,
        })

    def on_clock(self, msg):
        value = stamp_seconds(msg.clock)
        self.clock_stamps.append(value)
        self.add_sample({"topic": "clock", "sim_time_s": value})

    def on_tf(self, msg, info):
        self.record_tf("/tf", msg, info)

    def on_tf_static(self, msg, info):
        self.record_tf("/tf_static", msg, info)

    def record_tf(self, topic, msg, info):
        publisher_gid = (info.get("publisher_gid") if isinstance(info, dict)
                         else getattr(info, "publisher_gid", None))
        if self.tf_message_info_debug is None:
            self.tf_message_info_debug = {
                "type": type(info).__name__,
                "keys": sorted(info.keys()) if isinstance(info, dict) else [],
                "publisher_gid_repr": repr(publisher_gid),
            }
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            self.tf_edges[topic][(parent, child)] = None
            self.tf_rows.append({
                "topic": "tf_static" if topic == "/tf_static" else "tf",
                "sim_time_s": stamp_seconds(transform.header.stamp),
                "frame_id": parent, "child_frame_id": child,
                "x": float(transform.transform.translation.x),
                "y": float(transform.transform.translation.y),
                "z": float(transform.transform.translation.z),
                "yaw": yaw_from_quaternion(transform.transform.rotation),
                "scenario": self.mode, "phase": self.phase,
            })

    def get_truth(self):
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.truth_client.call_async(request)
        deadline = time.monotonic() + 4.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
        if not future.done():
            raise RuntimeError("Gazebo evaluator service timed out")
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError("Gazebo evaluator service failed: " + str(response.result.error_message))
        state = response.state
        pose, twist = state.pose, state.twist
        row = {
            "topic": "truth", "scenario": self.mode, "phase": self.phase,
            "sim_time_s": stamp_seconds(state.header.stamp),
            "x": float(pose.position.x), "y": float(pose.position.y),
            "z": float(pose.position.z), "yaw": yaw_from_quaternion(pose.orientation),
            "vx": float(twist.linear.x), "vy": float(twist.linear.y),
            "wz": float(twist.angular.z), "frame_id": state.header.frame_id,
        }
        self.truth.append(row)
        self.samples.append(row)
        return row

    def set_imu_enabled(self, enabled):
        if not self.imu_parameter_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("IMU sanitizer parameter service is unavailable")
        request = SetParameters.Request()
        request.parameters = [Parameter("publish_enabled", value=bool(enabled)).to_parameter_msg()]
        future = self.imu_parameter_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
        if not future.done():
            raise RuntimeError("IMU output toggle timed out")
        results = future.result().results
        if not results or not results[0].successful:
            reason = results[0].reason if results else "no result"
            raise RuntimeError("IMU output toggle rejected: " + reason)

    def wait_for_inputs(self):
        deadline = time.monotonic() + 25.0
        while rclpy.ok() and time.monotonic() < deadline:
            if (self.latest["raw"] is not None and self.latest["wheel"] is not None
                    and self.latest["filtered"] is not None and self.truth_client.service_is_ready()):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("Required odometry/filter inputs or evaluator service did not start")

    def run_phase(self, name, duration_s, linear_x=0.0, angular_z=0.0, settle_s=0.35):
        self.phase = name
        start_truth = self.get_truth()
        start_raw = dict(self.latest["raw"] or {})
        start_filtered = dict(self.latest["filtered"] or {})
        sample_start = len(self.samples)
        deadline = time.monotonic() + duration_s
        next_truth = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            command = Twist()
            command.linear.x = float(linear_x)
            command.angular.z = float(angular_z)
            self.command_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.015)
            now = time.monotonic()
            if now >= next_truth:
                self.get_truth()
                next_truth = now + 0.25
        self.command_pub.publish(Twist())
        settle_deadline = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < settle_deadline:
            self.command_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.015)
        end_truth = self.get_truth()
        self.phases.append({
            "name": name, "duration_requested_s": duration_s,
            "command": {"linear_x_mps": linear_x, "angular_z_radps": angular_z},
            "start_truth": start_truth, "end_truth": end_truth,
            "start_raw": start_raw, "end_raw": dict(self.latest["raw"] or {}),
            "start_filtered": start_filtered,
            "end_filtered": dict(self.latest["filtered"] or {}),
            "sample_count": len(self.samples) - sample_start,
        })

    def run_normal(self):
        self.run_phase("stationary_initial", 3.0)
        imu_before = sum(row["topic"] == "imu" for row in self.samples)
        filtered_before = sum(row["topic"] == "filtered" for row in self.samples)
        self.set_imu_enabled(False)
        self.imu_dropout["requested"] = True
        self.run_phase("imu_dropout_stationary", 1.2)
        self.set_imu_enabled(True)
        self.run_phase("imu_recovery_stationary", 0.8)
        imu_after = sum(row["topic"] == "imu" for row in self.samples)
        filtered_after = sum(row["topic"] == "filtered" for row in self.samples)
        self.imu_dropout["during_count"] = sum(
            row["topic"] == "imu" and row["phase"] == "imu_dropout_stationary" for row in self.samples
        )
        self.imu_dropout["filtered_count"] = filtered_after - filtered_before
        self.imu_dropout["imu_recovery_count"] = imu_after - imu_before

        self.run_phase("straight_out", 3.0, linear_x=0.10)
        self.run_phase("straight_return", 3.0, linear_x=-0.10)
        self.run_phase("rotation_ccw", 3.14, angular_z=0.50)
        self.run_phase("rotation_cw", 3.14, angular_z=-0.50)
        self.run_phase("rectangle_x_positive", 2.5, linear_x=0.10)
        self.run_phase("rectangle_turn_1", 3.14, angular_z=0.50)
        self.run_phase("rectangle_y_positive", 2.0, linear_x=0.10)
        self.run_phase("rectangle_turn_2", 3.14, angular_z=0.50)
        self.run_phase("rectangle_x_negative", 2.5, linear_x=0.10)
        self.run_phase("rectangle_turn_3", 3.14, angular_z=0.50)
        self.run_phase("rectangle_y_negative", 2.0, linear_x=0.10)
        self.run_phase("rectangle_turn_4", 3.14, angular_z=0.50)
        self.run_phase("wall_stall_approach", 10.0, linear_x=0.12, settle_s=0.5)
        self.run_phase("stationary_wall", 2.0)

    def run_slip(self):
        self.run_phase("stationary_before_slip", 1.5)
        self.run_phase("low_friction_slip_drive", self.slip_duration_s, linear_x=self.slip_speed_mps)
        self.run_phase("stationary_after_slip", 1.5)


def interpolate_truth(timestamp, truths):
    if not truths:
        return None
    ordered = sorted(truths, key=lambda row: row["sim_time_s"])
    times = [row["sim_time_s"] for row in ordered]
    index = bisect.bisect_left(times, timestamp)
    if index == 0:
        return ordered[0]
    if index >= len(ordered):
        return ordered[-1]
    before, after = ordered[index - 1], ordered[index]
    span = after["sim_time_s"] - before["sim_time_s"]
    fraction = 0.0 if span <= 0.0 else (timestamp - before["sim_time_s"]) / span
    yaw_delta = wrap_angle(after["yaw"] - before["yaw"])
    return {
        "sim_time_s": timestamp,
        "x": before["x"] + fraction * (after["x"] - before["x"]),
        "y": before["y"] + fraction * (after["y"] - before["y"]),
        "yaw": wrap_angle(before["yaw"] + fraction * yaw_delta),
    }


def compare_estimate(rows, truths):
    errors = []
    for row in rows:
        gt = interpolate_truth(row["sim_time_s"], truths)
        if gt is None:
            continue
        dx, dy = row["x"] - gt["x"], row["y"] - gt["y"]
        errors.append({
            "time_s": row["sim_time_s"], "dx": dx, "dy": dy,
            "xy": math.hypot(dx, dy), "dyaw": wrap_angle(row["yaw"] - gt["yaw"]),
            "pose_var_x": row.get("pose_var_x", math.nan),
            "pose_var_y": row.get("pose_var_y", math.nan),
            "pose_var_yaw": row.get("pose_var_yaw", math.nan),
        })
    if not errors:
        return {"samples": 0}
    xy = [e["xy"] for e in errors]
    yaw = [abs(e["dyaw"]) for e in errors]
    nis = {}
    for name, error_key, var_key in (("x", "dx", "pose_var_x"),
                                     ("y", "dy", "pose_var_y"),
                                     ("yaw", "dyaw", "pose_var_yaw")):
        ratios = [e[error_key] ** 2 / e[var_key] for e in errors
                  if math.isfinite(e[var_key]) and e[var_key] > 0.0]
        nis[name] = sum(ratios) / len(ratios) if ratios else None
    last = errors[-1]
    return {
        "samples": len(errors),
        "xy_rmse_m": math.sqrt(sum(v * v for v in xy) / len(xy)),
        "xy_mean_m": sum(xy) / len(xy), "xy_max_m": max(xy),
        "yaw_rmse_rad": math.sqrt(sum(v * v for v in yaw) / len(yaw)),
        "yaw_abs_mean_rad": sum(yaw) / len(yaw), "yaw_abs_max_rad": max(yaw),
        "endpoint_xy_error_m": last["xy"],
        "endpoint_yaw_error_rad": abs(last["dyaw"]),
        "mean_squared_normalized_error": nis,
        "mean_estimator_sigma": {
            "x_m": math.sqrt(max(0.0, sum(e["pose_var_x"] for e in errors) / len(errors))),
            "y_m": math.sqrt(max(0.0, sum(e["pose_var_y"] for e in errors) / len(errors))),
            "yaw_rad": math.sqrt(max(0.0, sum(e["pose_var_yaw"] for e in errors) / len(errors))),
        },
        "error_series": errors,
    }


def rate_hz(rows):
    times = [row["sim_time_s"] for row in rows if row.get("sim_time_s") is not None]
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])

def summarize(trial):
    by_topic = {}
    for row in trial.samples:
        by_topic.setdefault(row["topic"], []).append(row)
    raw, wheel = by_topic.get("raw", []), by_topic.get("wheel", [])
    filtered, imu = by_topic.get("filtered", []), by_topic.get("imu", [])
    truth = sorted(trial.truth, key=lambda row: row["sim_time_s"])
    raw_metrics = compare_estimate(raw, truth)
    filtered_metrics = compare_estimate(filtered, truth)
    phases = {}
    for phase in trial.phases:
        name = phase["name"]
        phase_truth = [row for row in truth if row["phase"] == name]
        phases[name] = {
            "raw": compare_estimate([r for r in raw if r["phase"] == name], phase_truth),
            "filtered": compare_estimate([r for r in filtered if r["phase"] == name], phase_truth),
            **{key: phase[key] for key in (
                "start_truth", "end_truth", "start_raw", "end_raw",
                "start_filtered", "end_filtered", "duration_requested_s")},
        }
        for key in ("raw", "filtered"):
            phases[name][key].pop("error_series", None)
    excluded = ("stationary", "imu_")
    motion_truth = [r for r in truth if not r["phase"].startswith(excluded)]
    raw_motion = [r for r in raw if not r["phase"].startswith(excluded)]
    filtered_motion = [r for r in filtered if not r["phase"].startswith(excluded)]
    raw_motion_metrics = compare_estimate(raw_motion, motion_truth)
    filtered_motion_metrics = compare_estimate(filtered_motion, motion_truth)

    stationary_phases = ("stationary_initial", "imu_dropout_stationary",
                         "imu_recovery_stationary", "stationary_before_slip",
                         "stationary_after_slip")
    stationary_imu = [r for r in imu if r["phase"] in stationary_phases]
    gyro = [r["gyro_z"] for r in stationary_imu]
    gyro_mean = sum(gyro) / len(gyro) if gyro else None
    gyro_std = (math.sqrt(sum((x - gyro_mean) ** 2 for x in gyro) / (len(gyro) - 1))
                if len(gyro) > 1 else None)
    accel = [r for r in stationary_imu if "accel_z" in r]
    imu_noise = {
        "stationary_sample_count": len(stationary_imu),
        "gyro_z_mean_radps": gyro_mean,
        "gyro_z_stddev_radps": gyro_std,
        "gyro_z_covariance_variance": stationary_imu[0].get("gyro_var_z") if stationary_imu else None,
        "accel_x_mean_mps2": sum(r["accel_x"] for r in accel) / len(accel) if accel else None,
        "accel_y_mean_mps2": sum(r["accel_y"] for r in accel) / len(accel) if accel else None,
        "accel_z_mean_mps2": sum(r["accel_z"] for r in accel) / len(accel) if accel else None,
    }

    stamps = {key: [r["sim_time_s"] for r in rows]
              for key, rows in (("raw", raw), ("wheel", wheel),
                                ("filtered", filtered), ("imu", imu))}
    odom_stamp_set = {round(value, 9) for value in stamps["raw"]}
    wheel_stamp_match = sum(round(value, 9) in odom_stamp_set for value in stamps["wheel"])
    latest_clock = max(trial.clock_stamps) if trial.clock_stamps else None
    latest_filtered = max(stamps["filtered"]) if stamps["filtered"] else None
    stale_age = (latest_clock - latest_filtered) if latest_clock is not None and latest_filtered is not None else None
    monotonic = {key: all(b >= a for a, b in zip(values, values[1:]))
                 for key, values in stamps.items()}
    clock_monotonic = all(b > a for a, b in zip(trial.clock_stamps, trial.clock_stamps[1:]))

    jumps = []
    ordered_filtered = sorted(filtered, key=lambda r: r["sim_time_s"])
    for before, after in zip(ordered_filtered, ordered_filtered[1:]):
        dt = after["sim_time_s"] - before["sim_time_s"]
        if 0.0 < dt <= 0.20:
            dp = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
            dyaw = abs(wrap_angle(after["yaw"] - before["yaw"]))
            if dp > 0.10 or dyaw > 0.30:
                jumps.append({"dt_s": dt, "position_m": dp, "yaw_rad": dyaw})
    covariance_finite = all(
        math.isfinite(r[key]) and r[key] >= 0.0
        for r in filtered for key in ("pose_var_x", "pose_var_y", "pose_var_yaw")
    )
    wheel_covariance_ok = bool(wheel) and all(
        math.isclose(r["twist_var_x"], 0.0004, rel_tol=0.02, abs_tol=1e-8)
        and math.isclose(r["twist_var_y"], 0.01, rel_tol=0.02, abs_tol=1e-8)
        and r["pose_var_x"] >= 1e5 for r in wheel[:min(100, len(wheel))]
    )
    imu_orientation_unavailable = bool(imu) and all(
        r.get("orientation_covariance_0") == -1.0 for r in imu
    )
    dynamic_edges, static_edges = trial.tf_edges["/tf"], trial.tf_edges["/tf_static"]
    required_dynamic = ("odom", "base_footprint")
    required_static = (("map", "odom"), ("base_footprint", "base_link"),
                       ("base_link", "imu_link"), ("base_link", "camera_link"),
                       ("camera_link", "camera_optical_frame"))
    static_missing = [edge for edge in required_static if edge not in static_edges]
    def topic_publisher_nodes(topic):
        return sorted({
            name for endpoint in trial.get_publishers_info_by_topic(topic)
            if (name := getattr(endpoint, "node_name", None))
        })

    tf_topic_publishers = {
        "/tf": topic_publisher_nodes("/tf"),
        "/tf_static": topic_publisher_nodes("/tf_static"),
    }
    child_parents = {}
    for parent, child in list(dynamic_edges) + list(static_edges):
        child_parents.setdefault(child, set()).add(parent)
    frame_parent_conflicts = {child: sorted(parents) for child, parents in child_parents.items()
                              if len(parents) > 1}
    map_odom = [r for r in trial.tf_rows if r["topic"] == "tf_static"
                and r["frame_id"] == "map" and r["child_frame_id"] == "odom"]
    map_odom_identity = bool(map_odom) and all(
        abs(r["x"]) < 1e-6 and abs(r["y"]) < 1e-6 and abs(r["yaw"]) < 1e-6
        for r in map_odom
    )

    first_truth = truth[0] if truth else None
    return_to_start = None
    if trial.mode == "normal" and first_truth:
        record = next((p for p in trial.phases if p["name"] == "rectangle_turn_4"), None)
        if record:
            end_truth = record["end_truth"]
            end_raw, end_filtered = record["end_raw"], record["end_filtered"]
            return_to_start = {
                "truth_xy_error_m": math.hypot(end_truth["x"] - first_truth["x"],
                                                end_truth["y"] - first_truth["y"]),
                "truth_yaw_error_rad": abs(wrap_angle(end_truth["yaw"] - first_truth["yaw"])),
                "raw_xy_error_m": math.hypot(end_raw.get("x", math.inf) - first_truth["x"],
                                              end_raw.get("y", math.inf) - first_truth["y"]),
                "raw_yaw_error_rad": abs(wrap_angle(end_raw.get("yaw", math.inf) - first_truth["yaw"])),
                "filtered_xy_error_m": math.hypot(end_filtered.get("x", math.inf) - first_truth["x"],
                                                   end_filtered.get("y", math.inf) - first_truth["y"]),
                "filtered_yaw_error_rad": abs(wrap_angle(end_filtered.get("yaw", math.inf) - first_truth["yaw"])),
            }

    wall_record = next((p for p in trial.phases if p["name"] == "wall_stall_approach"), None)
    wall_metrics = None
    if wall_record:
        wall_x = wall_record["end_truth"]["x"]
        raw_wall, filtered_wall = wall_record["end_raw"], wall_record["end_filtered"]
        wall_truth = wall_record["end_truth"]
        wall_metrics = {
            "evaluator_x_m": wall_x, "evaluator_y_m": wall_truth.get("y"),
            "evaluator_yaw_rad": wall_truth.get("yaw"),
            "raw_odom_x_m": raw_wall.get("x"),
            "filtered_x_m": filtered_wall.get("x"),
            "raw_minus_truth_m": raw_wall.get("x", 0.0) - wall_x,
            "filtered_minus_truth_m": filtered_wall.get("x", 0.0) - wall_x,
            "evaluator_linear_speed_mps": wall_truth.get("vx"),
        }

    slip_metrics = None
    if trial.mode == "slip":
        record = next((p for p in trial.phases if p["name"] == "low_friction_slip_drive"), None)
        if record:
            truth_delta = record["end_truth"]["x"] - record["start_truth"]["x"]
            raw_delta = record["end_raw"].get("x", 0.0) - record["start_raw"].get("x", 0.0)
            filtered_delta = record["end_filtered"].get("x", 0.0) - record["start_filtered"].get("x", 0.0)
            slip_metrics = {
                "wheel_surface_mu": trial.slip_mu,
                "requested_drive_speed_mps": trial.slip_speed_mps,
                "requested_drive_duration_s": trial.slip_duration_s,
                "evaluator_x_delta_m": truth_delta,
                "raw_odom_x_delta_m": raw_delta,
                "filtered_x_delta_m": filtered_delta,
                "raw_minus_truth_delta_m": raw_delta - truth_delta,
                "filtered_minus_truth_delta_m": filtered_delta - truth_delta,
            }

    checks = []
    def check(name, passed, detail):
        checks.append({"name": name, "status": "PASS" if passed else "FAIL", "detail": detail})
    check("raw, covariance-qualified wheel, filtered odometry and IMU received",
          bool(raw and wheel and filtered and imu),
          {"raw": len(raw), "wheel": len(wheel), "filtered": len(filtered), "imu": len(imu)})
    smaller_stream_count = min(len(set(round(value, 9) for value in stamps["raw"])),
                               len(set(round(value, 9) for value in stamps["wheel"])))
    shared_stamp_coverage = wheel_stamp_match / smaller_stream_count if smaller_stream_count else 0.0
    check("raw and wheel streams share exact stamps for >=98% of the smaller stream",
          smaller_stream_count > 0 and shared_stamp_coverage >= 0.98,
          {"matched": wheel_stamp_match, "raw_samples": len(raw),
           "wheel_samples": len(wheel),
           "raw_unmatched_samples": len(raw) - wheel_stamp_match,
           "wheel_unmatched_samples": len(wheel) - wheel_stamp_match,
           "shared_coverage_of_smaller_stream": shared_stamp_coverage})
    check("wheel velocity covariance is nonzero and pose is excluded",
          wheel_covariance_ok,
          {"vx_variance": wheel[0]["twist_var_x"] if wheel else None,
           "vy_variance": wheel[0]["twist_var_y"] if wheel else None,
           "pose_x_variance": wheel[0]["pose_var_x"] if wheel else None})
    check("sanitized IMU orientation stays unavailable", imu_orientation_unavailable,
          {"orientation_covariance_0": imu[0].get("orientation_covariance_0") if imu else None})
    dropout_imu_count = sum(r["phase"] == "imu_dropout_stationary" for r in imu)
    filter_during_dropout = sum(r["phase"] == "imu_dropout_stationary" for r in filtered)
    imu_recovery = any(r["phase"] == "imu_recovery_stationary" for r in imu)
    if trial.mode == "normal":
        check("IMU dropout stops data, EKF continues, and input recovers",
              trial.imu_dropout["requested"] and dropout_imu_count == 0
              and filter_during_dropout >= 20 and imu_recovery,
              {"imu_during_dropout": dropout_imu_count,
               "filtered_during_dropout": filter_during_dropout,
               "imu_recovered": imu_recovery})
    else:
        check("IMU stream remains available during slip trial", bool(imu),
              {"imu_samples": len(imu)})
    check("simulation clock and sensor stamps are monotonic",
          clock_monotonic and monotonic["raw"] and monotonic["wheel"] and monotonic["imu"],
          {"clock": clock_monotonic, "topics": monotonic})
    check("EKF output rate is stable", rate_hz(filtered) >= 20.0,
          {"filtered_rate_hz": rate_hz(filtered), "target_hz": 30.0})
    check("filtered odometry is fresh against /clock",
          stale_age is not None and -0.05 <= stale_age <= 0.20,
          {"latest_clock_s": latest_clock, "latest_filtered_s": latest_filtered, "age_s": stale_age})
    check("filtered covariance remains finite and nonnegative", covariance_finite,
          {"samples_checked": len(filtered)})
    check("known-start TF chain has required edges and no parent conflicts",
          map_odom_identity and required_dynamic in dynamic_edges
          and not static_missing and not frame_parent_conflicts,
          {"map_to_odom_identity": map_odom_identity,
           "odom_to_base_footprint_edge_present": required_dynamic in dynamic_edges,
           "tf_topic_publisher_nodes": tf_topic_publishers["/tf"],
           "per_edge_publisher_gid_available":
               "publisher_gid" in (trial.tf_message_info_debug or {}).get("keys", []),
           "static_edges_missing": static_missing,
           "frame_parent_conflicts": frame_parent_conflicts})
    check("filtered pose has no one-sample jumps", not jumps,
          {"jump_count": len(jumps), "examples": jumps[:5],
           "limits": {"position_m": 0.10, "yaw_rad": 0.30}})
    if trial.mode == "normal":
        straight = phases.get("straight_out", {}).get("end_truth")
        rotation = phases.get("rotation_ccw", {}).get("end_truth")
        check("straight, in-place rotation, rectangle, and return are recorded",
              straight is not None and rotation is not None and return_to_start is not None,
              {"straight_end_truth": straight, "rotation_end_truth": rotation,
               "return_to_start": return_to_start})
        check("wall stall is stationary and the EKF reduces raw wheel drift",
              wall_metrics is not None
              and abs(wall_metrics["evaluator_linear_speed_mps"] or 0.0) < 0.01
              and abs(wall_metrics["raw_minus_truth_m"]) >= 0.20
              and abs(wall_metrics["filtered_minus_truth_m"])
                  < abs(wall_metrics["raw_minus_truth_m"]),
              wall_metrics)
    else:
        slip_observed = (slip_metrics is not None
                         and abs(slip_metrics["raw_minus_truth_delta_m"]) >= 0.05)
        check("low-friction trial induced measurable wheel slip (>=0.05 m)",
              slip_observed, slip_metrics)

    try:
        source_commit = subprocess.check_output(["git", "rev-parse", "HEAD"],
                                                cwd=ROOT, text=True).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        source_commit = "unknown"
    clean_phases = {}
    for name, data in phases.items():
        clean_phases[name] = dict(data)
        for key in ("raw", "filtered"):
            clean_phases[name][key] = {k: v for k, v in data[key].items() if k != "error_series"}

    return {
        "status": "PASS_WITH_LIMITS" if all(c["status"] == "PASS" for c in checks) else "FAIL",
        "mode": trial.mode,
        "source_commit": source_commit,
        "evaluator_truth_policy": "Only this test calls /gzserver/get_entity_state; truth is saved to local evidence and is not published or consumed by the EKF.",
        "configuration": {
            "estimator": "robot_localization EKF", "frequency_hz": 30.0,
            "sensor_timeout_s": 0.25, "two_d_mode": True,
            "world_frame": "odom", "base_link_frame": "base_footprint",
            "known_start_map_pose_xyyaw": [0.0, 0.0, 0.0],
            "wheel_inputs": ["/odometry/wheel.twist.linear.x", "/odometry/wheel.twist.linear.y"],
            "wheel_velocity_variances": {"x_m2ps2": 0.0004, "y_m2ps2": 0.01},
            "imu_input": "/imu/data.angular_velocity.z only; orientation unavailable",
            "imu_angular_velocity_z_variance_rad2ps2": 4e-6,
        },
        "rates_hz": {"raw_odom": rate_hz(raw), "wheel_odometry": rate_hz(wheel),
                     "filtered": rate_hz(filtered), "imu": rate_hz(imu)},
        "timing": {"latest_clock_s": latest_clock, "latest_filtered_s": latest_filtered,
                   "filtered_age_s": stale_age,
                   "monotonic": {**monotonic, "clock": clock_monotonic}},
        "imu_dropout": trial.imu_dropout,
        "stationary_imu_noise": imu_noise,
        "raw_odometry_vs_evaluator": {k: v for k, v in raw_metrics.items() if k != "error_series"},
        "filtered_vs_evaluator": {k: v for k, v in filtered_metrics.items() if k != "error_series"},
        "motion_raw_vs_evaluator": {k: v for k, v in raw_motion_metrics.items() if k != "error_series"},
        "motion_filtered_vs_evaluator": {k: v for k, v in filtered_motion_metrics.items() if k != "error_series"},
        "phase_metrics": clean_phases, "return_to_start": return_to_start,
        "wall_stall": wall_metrics, "wheel_slip": slip_metrics,
        "tf": {
            "dynamic_edges": sorted(f"{a}->{b}" for a, b in dynamic_edges),
            "static_edges": sorted(f"{a}->{b}" for a, b in static_edges),
            "topic_publishers": tf_topic_publishers,
            "per_edge_publisher_gid_available":
                "publisher_gid" in (trial.tf_message_info_debug or {}).get("keys", []),
            "required_static_edges_missing": static_missing,
            "frame_parent_conflicts": frame_parent_conflicts,
        },
        "checks": checks,
    }

def add_error_series(rows, truths):
    result = []
    for source in rows:
        gt = interpolate_truth(source["sim_time_s"], truths)
        if gt is None:
            continue
        row = dict(source)
        row["truth_x"], row["truth_y"], row["truth_yaw"] = gt["x"], gt["y"], gt["yaw"]
        row["xy_error_m"] = math.hypot(row["x"] - gt["x"], row["y"] - gt["y"])
        row["yaw_error_rad"] = wrap_angle(row["yaw"] - gt["yaw"])
        result.append(row)
    return result


def write_plot(mode, samples, truths, path):
    raw = add_error_series([r for r in samples if r["topic"] == "raw"], truths)
    filtered = add_error_series([r for r in samples if r["topic"] == "filtered"], truths)
    truth = sorted(truths, key=lambda row: row["sim_time_s"])
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    if truth:
        axes[0].plot([r["x"] for r in truth], [r["y"] for r in truth],
                     color="black", linewidth=2.2, label="Gazebo truth (offline evaluator)")
    if raw:
        axes[0].plot([r["x"] for r in raw], [r["y"] for r in raw],
                     color="#d97706", alpha=0.8, label="Raw wheel odom")
    if filtered:
        axes[0].plot([r["x"] for r in filtered], [r["y"] for r in filtered],
                     color="#2563eb", linewidth=1.2, label="Filtered EKF")
    axes[0].set_title("XY track")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    for rows, color, label in ((raw, "#d97706", "Raw"), (filtered, "#2563eb", "Filtered")):
        if rows:
            t0 = rows[0]["sim_time_s"]
            elapsed = [r["sim_time_s"] - t0 for r in rows]
            axes[1].plot(elapsed, [r["xy_error_m"] for r in rows], color=color, label=label)
            axes[2].plot(elapsed, [r["yaw_error_rad"] for r in rows], color=color, label=label)
    axes[1].set_title("Position error vs evaluator")
    axes[1].set_xlabel("simulation time (s)")
    axes[1].set_ylabel("XY error (m)")
    axes[2].set_title("Yaw error vs evaluator")
    axes[2].set_xlabel("simulation time (s)")
    axes[2].set_ylabel("yaw error (rad)")
    for axis in axes[1:]:
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)
    fig.suptitle(f"Phase 4 localization — {mode} trial; truth is evaluator-only")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("normal", "slip"), default="normal")
    parser.add_argument("--run-id", default="phase4-localization-normal")
    parser.add_argument("--slip-mu", type=float, default=0.02)
    parser.add_argument("--slip-speed", type=float, default=0.10)
    parser.add_argument("--slip-duration", type=float, default=4.0)
    args = parser.parse_args()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    trial = LocalizationTrial(args.mode, args.slip_mu, args.slip_speed, args.slip_duration)
    try:
        trial.wait_for_inputs()
        trial.get_truth()
        trial.run_normal() if args.mode == "normal" else trial.run_slip()
        trial.command_pub.publish(Twist())
        summary = summarize(trial)
        summary_path = EVIDENCE / f"{args.run_id}.json"
        csv_path = EVIDENCE / f"{args.run_id}.csv"
        plot_path = EVIDENCE / f"{args.run_id}.png"
        with summary_path.open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, allow_nan=False)
            stream.write("\n")
        fields = ["scenario", "phase", "topic", "sim_time_s", "frame_id", "child_frame_id",
                  "x", "y", "z", "yaw", "vx", "vy", "wz", "gyro_z", "gyro_var_z",
                  "accel_x", "accel_y", "accel_z", "orientation_covariance_0",
                  "pose_var_x", "pose_var_y", "pose_var_yaw", "twist_var_x", "twist_var_y",
                  "twist_var_yaw"]
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
            writer.writeheader()
            writer.writerows(trial.samples)
            writer.writerows(trial.tf_rows)
        write_plot(args.mode, trial.samples, trial.truth, plot_path)
        print(json.dumps({
            "status": summary["status"], "mode": args.mode,
            "summary": str(summary_path), "telemetry": str(csv_path), "plot": str(plot_path),
            "rates_hz": summary["rates_hz"],
            "checks": summary["checks"],
            "motion_raw_xy_rmse_m": summary["motion_raw_vs_evaluator"].get("xy_rmse_m"),
            "motion_filtered_xy_rmse_m": summary["motion_filtered_vs_evaluator"].get("xy_rmse_m"),
            "return_to_start": summary["return_to_start"],
            "wall_stall": summary["wall_stall"], "wheel_slip": summary["wheel_slip"],
        }, indent=2, allow_nan=False))
        return 0 if summary["status"] != "FAIL" else 1
    finally:
        trial.command_pub.publish(Twist())
        trial.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
