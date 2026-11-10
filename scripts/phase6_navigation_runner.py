#!/usr/bin/env python3
"""Run bounded Nav2 REFERENCE trials on generated Phase 5 worlds.

The navigation graph receives only a geometry-derived reference occupancy map,
the Phase 4 known-start map->odom transform, and wheel/IMU EKF odometry.
Gazebo truth and obstacle geometry are queried only by this evaluator.
"""
import csv
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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu
from simulation_interfaces.srv import GetEntityState
from tf2_msgs.msg import TFMessage

from scenario_generator import (
    ROOT, build_world_sdf, find_scenario, generate_layout,
    point_obstacle_distance, registry_data, write_plot,
)

EVIDENCE = ROOT / "docs" / "evidence" / "phase6" / "runs"
ARENA = registry_data()["arena"]
FOOTPRINT_RADIUS = float(ARENA["robot_circumscribed_radius_m"])
MAP_RESOLUTION = 0.05


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def scenario_local_xy(x, y, obstacle):
    ox, oy, yaw = obstacle["pose_xyyaw"]
    dx, dy = x - ox, y - oy
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def make_oracle_map(layout, run_dir):
    """Rasterize generated SDF geometry into a reference-only occupancy map."""
    xmin, xmax, ymin, ymax = ARENA["outer_bounds_xy"]
    ixmin, ixmax, iymin, iymax = ARENA["inner_bounds_xy"]
    width = int(round((xmax - xmin) / MAP_RESOLUTION))
    height = int(round((ymax - ymin) / MAP_RESOLUTION))
    rows = []
    for row in range(height):
        y = ymax - (row + 0.5) * MAP_RESOLUTION
        values = []
        for col in range(width):
            x = xmin + (col + 0.5) * MAP_RESOLUTION
            occupied = (x <= ixmin or x >= ixmax or y <= iymin or y >= iymax)
            if not occupied:
                for obstacle in layout["obstacles"]:
                    if obstacle["shape"] in ("cylinder", "sphere"):
                        ox, oy, _ = obstacle["pose_xyyaw"]
                        occupied = math.hypot(x - ox, y - oy) <= float(obstacle["radius_m"])
                    else:
                        lx, ly = scenario_local_xy(x, y, obstacle)
                        sx, sy, _ = obstacle["size_xyz"]
                        occupied = abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0
                    if occupied:
                        break
            values.append("0" if occupied else "254")
        rows.append(" ".join(values))
    pgm = run_dir / "oracle_reference_map.pgm"
    pgm.write_text(f"P2\n{width} {height}\n255\n" + "\n".join(rows) + "\n")
    yaml_path = run_dir / "oracle_reference_map.yaml"
    yaml_path.write_text(
        "image: oracle_reference_map.pgm\n"
        f"resolution: {MAP_RESOLUTION:.6f}\n"
        f"origin: [{xmin:.6f}, {ymin:.6f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    occupied_cells = sum(row.count("0") for row in rows)
    return yaml_path, pgm, {
        "width": width, "height": height, "resolution_m": MAP_RESOLUTION,
        "origin_xy_m": [xmin, ymin], "occupied_cells": occupied_cells,
        "free_cells": width * height - occupied_cells,
        "profile": "ORACLE_REFERENCE",
    }


def distance_to_obstacles_and_walls(x, y, layout):
    xmin, xmax, ymin, ymax = ARENA["inner_bounds_xy"]
    clearance = min(x - xmin, xmax - x, y - ymin, ymax - y) - FOOTPRINT_RADIUS
    for obstacle in layout["obstacles"]:
        clearance = min(clearance,
                        point_obstacle_distance(x, y, obstacle) - FOOTPRINT_RADIUS)
    return clearance


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points[:-1], points[1:]))


def truth_pose_window(samples, window_s=1.0):
    if len(samples) < 2:
        return None
    end = samples[-1]
    recent = [item for item in samples
              if end["monotonic_s"] - item["monotonic_s"] <= window_s]
    if len(recent) < 2:
        return None
    start = recent[0]
    return {
        "sample_count": len(recent),
        "duration_s": round(end["monotonic_s"] - start["monotonic_s"], 3),
        "xy_delta_m": round(math.hypot(end["x"] - start["x"],
                                       end["y"] - start["y"]), 5),
        "yaw_delta_rad": round(abs(wrap_angle(end["yaw_rad"] -
                                              start["yaw_rad"])), 5),
    }


class Phase6Capture(Node):
    def __init__(self):
        super().__init__("anc_phase6_navigation_evaluator")
        self.image_count = 0
        self.imu_count = 0
        self.odom_count = 0
        self.filtered_odom_count = 0
        self.clock_count = 0
        self.map_count = 0
        self.clock_latest = None
        self.map_summary = None
        self.costmaps = {}
        self.tf_edges = {}
        self.tf_edge_topics = {}
        self.truth_samples = []
        self.truth_future = None
        self.truth_enabled = False
        self.next_truth_at = 0.0
        self.lifecycle_state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("map_server", "planner_server", "controller_server",
                         "behavior_server", "bt_navigator")
        }
        self.lifecycle_states = {}
        self.truth_errors = []
        self.raw_odom_points = []
        self.filtered_odom_points = []
        self.raw_odom_velocity = None
        self.filtered_odom_velocity = None
        self.plan_points = []
        self.plan_points_peak = []
        self.plan_length_latest = 0.0
        self.plan_length_peak = 0.0
        self.plan_count = 0
        self.feedback_count = 0
        self.last_feedback = None
        self.recovery_count = 0
        self.cmd_nav_samples = []
        self.cmd_gz_samples = []
        self.contact_pairs_active = set()
        self.contact_events = []
        self.ground_contact_messages = 0
        self.other_contact_messages = 0
        transient_map = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_filtered_odom, 20)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_dynamic_tf, 100)
        static_tf_qos = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, "/tf_static", self.on_static_tf, static_tf_qos)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, transient_map)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self.on_global_costmap, transient_map)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self.on_local_costmap, transient_map)
        self.create_subscription(PathMsg, "/plan", self.on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_cmd_nav, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_gz, 20)
        self.create_subscription(Contacts, "/contacts", self.on_contacts, qos_profile_sensor_data)
        self.state_client = self.create_client(GetEntityState, "/gzserver/get_entity_state")
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    @staticmethod
    def pose_xy(message):
        return float(message.pose.pose.position.x), float(message.pose.pose.position.y)

    def on_image(self, _msg):
        self.image_count += 1

    def on_imu(self, _msg):
        self.imu_count += 1

    def on_odom(self, msg):
        self.odom_count += 1
        self.raw_odom_points.append(self.pose_xy(msg))
        self.raw_odom_velocity = (
            float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))

    def on_filtered_odom(self, msg):
        self.filtered_odom_count += 1
        self.filtered_odom_points.append(self.pose_xy(msg))
        self.filtered_odom_velocity = (
            float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))

    def on_clock(self, msg):
        self.clock_count += 1
        self.clock_latest = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def _save_tf(self, msg, topic):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            edge = (parent, child)
            t = transform.transform.translation
            q = transform.transform.rotation
            self.tf_edges[edge] = {
                "x": float(t.x), "y": float(t.y), "z": float(t.z),
                "yaw": yaw_from_quaternion(q),
            }
            self.tf_edge_topics.setdefault(edge, set()).add(topic)

    def on_dynamic_tf(self, msg):
        self._save_tf(msg, "/tf")

    def on_static_tf(self, msg):
        self._save_tf(msg, "/tf_static")

    def on_map(self, msg):
        self.map_count += 1
        self.map_summary = {
            "frame_id": msg.header.frame_id,
            "width": int(msg.info.width), "height": int(msg.info.height),
            "resolution_m": float(msg.info.resolution),
            "origin_xy_m": [float(msg.info.origin.position.x),
                            float(msg.info.origin.position.y)],
            "data_count": len(msg.data),
            "occupied_cells": sum(1 for value in msg.data if value >= 65),
            "free_cells": sum(1 for value in msg.data if 0 <= value < 20),
            "unknown_cells": sum(1 for value in msg.data if value < 0),
        }

    def on_global_costmap(self, msg):
        self.costmaps["global"] = msg

    def on_local_costmap(self, msg):
        self.costmaps["local"] = msg

    def on_plan(self, msg):
        self.plan_count += 1
        self.plan_points = [(float(p.pose.position.x), float(p.pose.position.y))
                            for p in msg.poses]
        self.plan_length_latest = path_length(self.plan_points)
        if self.plan_length_latest > self.plan_length_peak:
            self.plan_length_peak = self.plan_length_latest
            self.plan_points_peak = list(self.plan_points)

    def on_cmd_nav(self, msg):
        self.cmd_nav_samples.append((time.monotonic(), float(msg.linear.x),
                                     float(msg.angular.z)))

    def on_cmd_gz(self, msg):
        self.cmd_gz_samples.append((time.monotonic(), float(msg.linear.x),
                                    float(msg.angular.z)))

    def on_contacts(self, msg):
        current = set()
        for contact in msg.contacts:
            a, b = str(contact.collision1.name), str(contact.collision2.name)
            if any(word in (a + " " + b).lower()
                   for word in ("ground_plane", "ground::", "floor")):
                self.ground_contact_messages += 1
                continue
            if a and b:
                current.add(tuple(sorted((a, b))))
        if current:
            self.other_contact_messages += 1
        for pair in current - self.contact_pairs_active:
            self.contact_events.append({
                "time_monotonic_s": time.monotonic(),
                "collision1": pair[0], "collision2": pair[1],
            })
        self.contact_pairs_active = current

    def schedule_truth(self):
        if (self.truth_enabled and self.truth_future is None and
                time.monotonic() >= self.next_truth_at and
                self.state_client.service_is_ready()):
            request = GetEntityState.Request()
            request.entity = "anc_robot"
            self.truth_future = self.state_client.call_async(request)
            self.next_truth_at = time.monotonic() + 0.20

    def collect_truth_if_ready(self):
        if self.truth_future is None or not self.truth_future.done():
            return
        future = self.truth_future
        self.truth_future = None
        try:
            response = future.result()
            if response is None or response.result.result != 1:
                detail = "no response" if response is None else response.result.error_message
                self.truth_errors.append(str(detail))
                return
            pose = response.state.pose
            self.truth_samples.append({
                "monotonic_s": time.monotonic(),
                "sim_time_s": float(response.state.header.stamp.sec) +
                              float(response.state.header.stamp.nanosec) * 1e-9,
                "frame_id": response.state.header.frame_id,
                "x": float(pose.position.x), "y": float(pose.position.y),
                "z": float(pose.position.z),
                "yaw_rad": yaw_from_quaternion(pose.orientation),
                "linear_x_mps": float(response.state.twist.linear.x),
                "angular_z_radps": float(response.state.twist.angular.z),
            })
        except Exception as exc:
            self.truth_errors.append(f"{type(exc).__name__}: {exc}")

    def lifecycle_nodes_active(self, per_node_timeout_s=0.25):
        states = {}
        for name, client in self.lifecycle_state_clients.items():
            if not client.service_is_ready():
                self.lifecycle_states = {**states, name: "service_unavailable"}
                return False
            future = client.call_async(GetState.Request())
            deadline = time.monotonic() + per_node_timeout_s
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
            if not future.done():
                self.lifecycle_states = {**states, name: "state_query_timeout"}
                return False
            state = future.result().current_state
            states[name] = {"id": int(state.id), "label": str(state.label)}
        self.lifecycle_states = states
        return all(state["id"] == 3 for state in states.values())

    def on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.feedback_count += 1
        self.last_feedback = {
            "x": float(feedback.current_pose.pose.position.x),
            "y": float(feedback.current_pose.pose.position.y),
            "distance_remaining_m": float(feedback.distance_remaining),
            "navigation_time_s": float(feedback.navigation_time.sec) +
                                 float(feedback.navigation_time.nanosec) * 1e-9,
            "number_of_recoveries": int(feedback.number_of_recoveries),
        }
        self.recovery_count = max(self.recovery_count,
                                  self.last_feedback["number_of_recoveries"])

    def sample_loop(self, deadline, process=None):
        while time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError(f"Navigation launch exited early with code {process.returncode}")
            rclpy.spin_once(self, timeout_sec=0.04)
            self.schedule_truth()
            self.collect_truth_if_ready()



def shutdown_group(process, grace_s=18):
    if process is None:
        return {"signal": None, "returncode": None, "group_gone": True, "forced": False}
    sent, forced = None, False
    try:
        process.send_signal(signal.SIGINT)
        sent = "SIGINT"
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        forced = True
        try:
            os.killpg(process.pid, signal.SIGTERM)
            sent = "SIGTERM"
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
                sent = "SIGKILL"
            except ProcessLookupError:
                pass
            process.wait(timeout=3)
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
    return {"signal": sent, "returncode": process.poll(),
            "group_gone": group_gone, "forced": forced}


def stop_bag(process):
    if process is None:
        return {"returncode": None, "stopped": True}
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            return {"returncode": process.poll(), "stopped": False}
    except ProcessLookupError:
        pass
    return {"returncode": process.poll(), "stopped": process.poll() == 0}


def inspect_bag_tf(bag_path):
    """Verify transform edges from the closed MCAP independently of callback timing."""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="mcap"),
                ConverterOptions("", ""))
    edges = {}
    while reader.has_next():
        topic, raw, _timestamp = reader.read_next()
        if topic not in ("/tf", "/tf_static"):
            continue
        message = deserialize_message(raw, TFMessage)
        for transform in message.transforms:
            key = (topic, transform.header.frame_id.lstrip("/"),
                   transform.child_frame_id.lstrip("/"))
            entry = edges.setdefault(key, {"count": 0, "latest": None})
            entry["count"] += 1
            q = transform.transform.rotation
            entry["latest"] = {
                "x": float(transform.transform.translation.x),
                "y": float(transform.transform.translation.y),
                "z": float(transform.transform.translation.z),
                "yaw": yaw_from_quaternion(q),
            }
    return {
        f"{topic}:{parent}->{child}": value
        for (topic, parent, child), value in sorted(edges.items())
    }


def compose_map_pose(tf_edges):
    map_odom = tf_edges.get(("map", "odom"))
    odom_base = tf_edges.get(("odom", "base_footprint"))
    if map_odom is None or odom_base is None:
        return None
    c, s = math.cos(map_odom["yaw"]), math.sin(map_odom["yaw"])
    return {
        "x": map_odom["x"] + c * odom_base["x"] - s * odom_base["y"],
        "y": map_odom["y"] + s * odom_base["x"] + c * odom_base["y"],
        "yaw_rad": wrap_angle(map_odom["yaw"] + odom_base["yaw"]),
    }


def write_trace_plot(layout, capture, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
        fig, ax = plt.subplots(figsize=(10, 7))
        xmin, xmax, ymin, ymax = ARENA["inner_bounds_xy"]
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               fill=False, linewidth=2, edgecolor="black"))
        for obstacle in layout["obstacles"]:
            if obstacle["shape"] in ("cylinder", "sphere"):
                x, y, _ = obstacle["pose_xyyaw"]
                patch = Circle((x, y), float(obstacle["radius_m"]),
                               color="firebrick", alpha=0.65)
            else:
                x, y, yaw = obstacle["pose_xyyaw"]
                sx, sy, _ = obstacle["size_xyz"]
                patch = Rectangle((x - sx / 2.0, y - sy / 2.0), sx, sy,
                                  angle=math.degrees(yaw), rotation_point="center",
                                  color="firebrick", alpha=0.65)
            ax.add_patch(patch)
        if capture.plan_points_peak:
            ax.plot([p[0] for p in capture.plan_points_peak],
                    [p[1] for p in capture.plan_points_peak], "b--", linewidth=1.8,
                    label="longest observed Nav2 plan")
        if capture.truth_samples:
            ax.plot([p["x"] for p in capture.truth_samples],
                    [p["y"] for p in capture.truth_samples], "g-", linewidth=2,
                    label="evaluator truth trajectory")
            ax.plot(capture.truth_samples[0]["x"], capture.truth_samples[0]["y"],
                    "go", label="start")
            ax.plot(capture.truth_samples[-1]["x"], capture.truth_samples[-1]["y"],
                    "gx", markersize=10, label="final")
        goal = layout["goal_xyyaw"]
        ax.plot(goal[0], goal[1], "r*", markersize=13, label="goal")
        ax.set(xlim=(xmin - 0.2, xmax + 0.2), ylim=(ymin - 0.2, ymax + 0.2),
               aspect="equal", xlabel="x (m)", ylabel="y (m)",
               title=f"{layout['scenario_id']} ORACLE_REFERENCE navigation trace")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(path, dpi=140)
        plt.close(fig)
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"
    return None


def write_costmap_image(capture, run_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        written = {}
        for name, msg in capture.costmaps.items():
            if not msg.info.width or not msg.info.height:
                continue
            values = np.asarray(msg.data, dtype=float).reshape(
                (msg.info.height, msg.info.width))
            fig, ax = plt.subplots(figsize=(7, 7))
            ax.imshow(np.flipud(values), cmap="gray_r", vmin=-1, vmax=100,
                      origin="upper")
            ax.set_title(f"{name.title()} Nav2 costmap")
            ax.set_xlabel("cell x")
            ax.set_ylabel("cell y")
            fig.tight_layout()
            out = run_dir / f"{name}_costmap.png"
            fig.savefig(out, dpi=120)
            plt.close(fig)
            written[name] = str(out)
        return written, None
    except Exception as exc:
        return {}, f"{type(exc).__name__}: {exc}"


def run(args):
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", args.run_id):
        raise ValueError("run-id must contain only letters, digits, dot, underscore, or hyphen")
    run_dir = EVIDENCE / args.run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    scenario_data = registry_data()
    scenario = find_scenario(scenario_data, args.scenario)
    layout = generate_layout(args.scenario, args.seed, scenario_data)
    if not layout["validation"]["passed"]:
        raise RuntimeError("Scenario geometry is invalid")
    if layout["route_validation"]["found"] != layout["route_expected"]:
        raise RuntimeError("Phase 5 route validator does not match scenario expectation")

    world_data = build_world_sdf(layout)
    world_path = run_dir / "world.sdf"
    world_path.write_bytes(world_data)
    write_plot(layout, run_dir / "layout.png")
    map_yaml, map_pgm, map_summary = make_oracle_map(layout, run_dir)
    layout["world_sha256"] = hashlib.sha256(world_data).hexdigest()
    layout["world_file"] = str(world_path)
    layout["navigation_profile"] = "ORACLE_REFERENCE"
    layout["navigation_map_yaml"] = str(map_yaml)
    layout["navigation_map_pgm"] = str(map_pgm)
    (run_dir / "manifest.json").write_text(json.dumps(layout, indent=2, sort_keys=True) + "\n")
    try:
        source_commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except Exception:
        source_commit = "unavailable"
    result = {
        "schema_version": 1, "run_id": args.run_id, "phase": 6,
        "scenario_id": args.scenario, "seed": args.seed,
        "source_commit": source_commit,
        "navigation_profile": "ORACLE_REFERENCE",
        "display_options": {"gazebo_gui": bool(args.gui), "rviz": bool(args.rviz)},
        "original_phase5_sensor_profile": layout["sensor_profile"],
        "sensor_profile_override": "ORACLE_REFERENCE",
        "navigation_inputs": {
            "map": "run-specific geometry-derived static occupancy map from scenario layout",
            "localization": "Phase 4 wheel/IMU EKF odometry plus known-start map->odom",
            "rgb_camera": "Disabled for this headless reference profile; RGB is not a navigation input.",
            "gazebo_truth_used_by_navigation": False,
            "obstacle_geometry_sent_to_navigation": False,
            "reference_map_available_to_realistic_profile": False,
        },
        "layout_sha256": layout["layout_sha256"],
        "world_sha256": layout["world_sha256"],
        "map_yaml_sha256": sha256_file(map_yaml),
        "map_pgm_sha256": sha256_file(map_pgm),
        "map": map_summary,
        "start_xyyaw": layout["start_xyyaw"],
        "goal_xyyaw": layout["goal_xyyaw"],
        "obstacle_count": len(layout["obstacles"]),
        "route_expected": bool(layout["route_expected"]),
        "route_found_by_geometry_validator": bool(layout["route_validation"]["found"]),
        "truth_published_to_ros": False,
        "rosbag": {"requested": True, "path": str(run_dir / "rosbag")},
        "tests": [],
    }

    def add_check(name, passed, detail):
        result["tests"].append({
            "name": name, "status": "PASS" if passed else "FAIL",
            "detail": str(detail),
        })

    def set_check(name, passed, detail):
        for check in result["tests"]:
            if check["name"] == name:
                check["status"] = "PASS" if passed else "FAIL"
                check["detail"] = str(detail)
                return
        add_check(name, passed, detail)

    env = os.environ.copy()
    domain_id = 41 + (int(hashlib.sha256(args.run_id.encode()).hexdigest()[:4], 16) % 100)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = f"anc_phase6_{args.run_id}"
    env["ROS_LOG_DIR"] = str(run_dir / "ros_logs")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    # The evaluator and its child launch must join the same isolated ROS domain.
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]
    os.environ["ROS_LOG_DIR"] = env["ROS_LOG_DIR"]
    result["ros_domain_id"] = domain_id
    rclpy.init(args=None)
    capture = Phase6Capture()
    process, bag_process = None, None
    launch_log = (run_dir / "launch.log").open("w")
    bag_log = (run_dir / "rosbag.log").open("w")
    action_handle = None
    start_wall = time.monotonic()
    shutdown = bag_shutdown = None

    try:
        command = [
            "ros2", "launch", "anc_navigation", "reference_navigation.launch.py",
            f"world_file:={world_path}", f"map_yaml:={map_yaml}",
            f"start_x:={layout['start_xyyaw'][0]:.6f}",
            f"start_y:={layout['start_xyyaw'][1]:.6f}",
            f"start_yaw:={layout['start_xyyaw'][2]:.6f}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.rviz).lower()}",
        ]
        result["launch_command"] = command
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=launch_log,
                                   stderr=subprocess.STDOUT, start_new_session=True)

        startup_started = time.monotonic()
        readiness_deadline = startup_started + float(args.startup_timeout_s)
        lifecycle_ready = False
        next_lifecycle_query = startup_started
        while time.monotonic() < readiness_deadline:
            if process.poll() is not None:
                raise RuntimeError(f"Navigation launch exited during startup: {process.returncode}")
            if time.monotonic() - startup_started > 28.0:
                current_log = (run_dir / "launch.log").read_text(errors="replace")
                if "InvalidParameterTypeException" in current_log or (
                        "[ERROR] [controller_server" in current_log and
                        "process has died" in current_log):
                    raise RuntimeError(
                        "Nav2 controller failed during startup; see launch.log for the node error"
                    )
            rclpy.spin_once(capture, timeout_sec=0.05)
            capture.schedule_truth()
            capture.collect_truth_if_ready()
            topics_ready = (
                capture.imu_count > 0 and capture.odom_count > 0 and
                capture.filtered_odom_count > 0 and
                capture.clock_count > 0 and capture.map_count > 0
            )
            if (topics_ready and capture.action_client.server_is_ready() and
                    time.monotonic() >= next_lifecycle_query):
                lifecycle_ready = capture.lifecycle_nodes_active()
                next_lifecycle_query = time.monotonic() + 0.50
                if lifecycle_ready:
                    break
        ready_topics = (
            capture.imu_count > 0 and capture.odom_count > 0 and
            capture.filtered_odom_count > 0 and
            capture.clock_count > 0 and capture.map_count > 0
        )
        lifecycle_ready = lifecycle_ready or (
            ready_topics and capture.action_client.server_is_ready() and
            capture.lifecycle_nodes_active()
        )
        ready = ready_topics and capture.action_client.server_is_ready() and lifecycle_ready
        capture.truth_enabled = ready
        add_check(
            "reference_navigation_graph_ready", ready,
            f"camera={capture.image_count}, imu={capture.imu_count}, "
            f"odom={capture.odom_count}, filtered_odom={capture.filtered_odom_count}, "
            f"clock={capture.clock_count}, map={capture.map_count}, "
            f"navigate_to_pose_server={capture.action_client.server_is_ready()}, "
            f"lifecycle_active={lifecycle_ready}; states={capture.lifecycle_states}",
        )
        add_check("reference_navigation_lifecycle_active", lifecycle_ready,
                  json.dumps(capture.lifecycle_states, sort_keys=True))
        add_check("rgb_camera_disabled_for_headless_reference",
                  capture.image_count == 0,
                  f"RGB frames observed={capture.image_count}; camera sensor is disabled in reference launch")
        if not ready:
            raise RuntimeError("Required simulation/Nav2 topics or NavigateToPose action did not become ready")

        bag_topics = [
            "/clock", "/tf", "/tf_static", "/map", "/global_costmap/costmap",
            "/local_costmap/costmap", "/global_costmap/costmap_raw",
            "/local_costmap/costmap_raw", "/odom", "/odometry/filtered", "/plan",
            "/cmd_vel_nav", "/cmd_vel", "/contacts",
            "/navigate_to_pose/_action/feedback", "/navigate_to_pose/_action/status",
        ]
        bag_command = ["ros2", "bag", "record", "--storage", "mcap", "-o",
                       str(run_dir / "rosbag"), *bag_topics]
        result["rosbag"]["command"] = bag_command
        bag_process = subprocess.Popen(bag_command, cwd=ROOT, env=env,
                                       stdout=bag_log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        time.sleep(0.5)
        if bag_process.poll() is not None:
            raise RuntimeError(f"Rosbag recorder exited during startup: {bag_process.returncode}")

        initial_truth_deadline = time.monotonic() + 8.0
        while not capture.truth_samples and time.monotonic() < initial_truth_deadline:
            rclpy.spin_once(capture, timeout_sec=0.05)
            capture.schedule_truth()
            capture.collect_truth_if_ready()
        if not capture.truth_samples:
            raise RuntimeError("Evaluator-only Gazebo truth service returned no robot pose")

        start = layout["start_xyyaw"]
        first_truth = capture.truth_samples[0]
        spawn_xy_error = math.hypot(first_truth["x"] - start[0], first_truth["y"] - start[1])
        spawn_yaw_error = abs(wrap_angle(first_truth["yaw_rad"] - start[2]))
        add_check("known_start_spawn_matches_scenario",
                  spawn_xy_error <= 0.08 and spawn_yaw_error <= 0.12,
                  f"truth start error={spawn_xy_error:.4f} m / {spawn_yaw_error:.4f} rad")

        map_ok = (
            capture.map_summary is not None and capture.map_summary["frame_id"] == "map" and
            capture.map_summary["width"] == map_summary["width"] and
            capture.map_summary["height"] == map_summary["height"] and
            capture.map_summary["occupied_cells"] == map_summary["occupied_cells"]
        )
        result["published_map"] = capture.map_summary
        result["lifecycle_states"] = capture.lifecycle_states
        add_check("oracle_reference_map_published", map_ok,
                  f"map_server geometry grid matched={map_ok}; {capture.map_summary}")

        map_odom = capture.tf_edges.get(("map", "odom"))
        odom_base = capture.tf_edges.get(("odom", "base_footprint"))
        result["tf_initial"] = {"map_to_odom": map_odom,
                                "odom_to_base_footprint": odom_base}
        add_check("single_known_start_map_to_odom_transform_present",
                  map_odom is not None,
                  "map->odom observed; sole configured publisher is anc_known_start_map_to_odom")
        add_check("single_ekf_odom_to_base_transform_present",
                  odom_base is not None,
                  "odom->base_footprint observed; sole configured publisher is anc_ekf_filter")
        add_check("amcl_not_in_reference_profile", True,
                  "Dedicated launch does not start AMCL or another map->odom publisher")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        now_sim = capture.clock_latest or 0.0
        goal.pose.header.stamp.sec = int(now_sim)
        goal.pose.header.stamp.nanosec = int((now_sim % 1.0) * 1e9)
        goal.pose.pose.position.x = float(layout["goal_xyyaw"][0])
        goal.pose.pose.position.y = float(layout["goal_xyyaw"][1])
        goal.pose.pose.position.z = 0.0
        goal_yaw = float(layout["goal_xyyaw"][2])
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        send_future = capture.action_client.send_goal_async(goal,
                                                           feedback_callback=capture.on_feedback)
        accept_deadline = time.monotonic() + 20.0
        while not send_future.done() and time.monotonic() < accept_deadline:
            capture.sample_loop(time.monotonic() + 0.04, process)
        if not send_future.done():
            raise RuntimeError("NavigateToPose did not respond to goal request within 20 seconds")
        action_handle = send_future.result()
        accepted = bool(action_handle and action_handle.accepted)
        result["action"] = {
            "name": "/navigate_to_pose", "goal_accepted": accepted,
            "goal_frame": "map", "goal_xyyaw": layout["goal_xyyaw"],
        }
        add_check("navigate_to_pose_goal_accepted", accepted,
                  "Action goal acceptance returned by Nav2")
        if accepted:
            action_future = action_handle.get_result_async()
            scenario_limit = 38.0 if args.scenario == "S09" else float(args.navigation_timeout_s)
            action_deadline = time.monotonic() + scenario_limit
            while not action_future.done() and time.monotonic() < action_deadline:
                capture.sample_loop(time.monotonic() + 0.04, process)
            timed_out = not action_future.done()
            if timed_out:
                cancel_future = action_handle.cancel_goal_async()
                cancel_deadline = time.monotonic() + 4.0
                while not cancel_future.done() and time.monotonic() < cancel_deadline:
                    capture.sample_loop(time.monotonic() + 0.04, process)
            if action_future.done():
                wrapped = action_future.result()
                status_code = int(wrapped.status)
                status_name = {
                    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                    GoalStatus.STATUS_CANCELED: "CANCELED",
                    GoalStatus.STATUS_ABORTED: "ABORTED",
                }.get(status_code, f"STATUS_{status_code}")
                error_code, error_msg = int(wrapped.result.error_code), str(wrapped.result.error_msg)
            else:
                status_code, status_name = None, "BOUNDED_TIMEOUT_CANCELED"
                error_code = None
                error_msg = "Action exceeded bounded trial deadline; cancellation was requested."
        else:
            timed_out = False
            status_code, status_name = None, "REJECTED"
            error_code, error_msg = None, "Nav2 rejected the goal."
        result["action"].update({
            "status_code": status_code, "status": status_name,
            "error_code": error_code, "error_msg": error_msg,
            "timed_out": timed_out,
            "feedback_samples": capture.feedback_count,
            "last_feedback": capture.last_feedback,
            "number_of_recoveries": capture.recovery_count,
        })

        settle_start = time.monotonic()
        settle_deadline = settle_start + 6.0
        while time.monotonic() < settle_deadline:
            latest_truth = capture.truth_samples[-1] if capture.truth_samples else None
            latest_command = capture.cmd_gz_samples[-1] if capture.cmd_gz_samples else None
            command_is_zero = (
                latest_command is None or
                (abs(latest_command[1]) <= 0.02 and abs(latest_command[2]) <= 0.02)
            )
            pose_window = truth_pose_window(capture.truth_samples)
            truth_pose_is_stationary = (
                pose_window is not None and pose_window["duration_s"] >= 0.8 and
                pose_window["xy_delta_m"] <= 0.02 and
                pose_window["yaw_delta_rad"] <= 0.03
            )
            raw_odom_is_stopped = (
                capture.raw_odom_velocity is not None and
                abs(capture.raw_odom_velocity[0]) < 0.04 and
                abs(capture.raw_odom_velocity[1]) < 0.08
            )
            filtered_odom_is_stopped = (
                capture.filtered_odom_velocity is not None and
                abs(capture.filtered_odom_velocity[0]) < 0.04 and
                abs(capture.filtered_odom_velocity[1]) < 0.08
            )
            if (command_is_zero and truth_pose_is_stationary and
                    raw_odom_is_stopped and filtered_odom_is_stopped):
                break
            remaining = settle_deadline - time.monotonic()
            if remaining <= 0:
                break
            capture.sample_loop(min(settle_deadline, time.monotonic() + 0.20), process)
        if capture.truth_future is not None:
            truth_deadline = time.monotonic() + 2.0
            while capture.truth_future is not None and time.monotonic() < truth_deadline:
                rclpy.spin_once(capture, timeout_sec=0.04)
                capture.collect_truth_if_ready()

        latest_truth = capture.truth_samples[-1] if capture.truth_samples else None
        latest_command = capture.cmd_gz_samples[-1] if capture.cmd_gz_samples else None
        final_pose_window = truth_pose_window(capture.truth_samples)
        truth_pose_is_stationary = (
            final_pose_window is not None and final_pose_window["duration_s"] >= 0.8 and
            final_pose_window["xy_delta_m"] <= 0.02 and
            final_pose_window["yaw_delta_rad"] <= 0.03
        )
        raw_odom_is_stopped = (
            capture.raw_odom_velocity is not None and
            abs(capture.raw_odom_velocity[0]) < 0.04 and
            abs(capture.raw_odom_velocity[1]) < 0.08
        )
        filtered_odom_is_stopped = (
            capture.filtered_odom_velocity is not None and
            abs(capture.filtered_odom_velocity[0]) < 0.04 and
            abs(capture.filtered_odom_velocity[1]) < 0.08
        )
        command_is_zero = (
            latest_command is None or
            (abs(latest_command[1]) <= 0.02 and abs(latest_command[2]) <= 0.02)
        )
        settle_expired = not (
            truth_pose_is_stationary and raw_odom_is_stopped and
            filtered_odom_is_stopped and command_is_zero
        )
        truth = capture.truth_samples
        truth_xy = [(p["x"], p["y"]) for p in truth]
        traveled = path_length(truth_xy)
        min_clearance = min(
            (distance_to_obstacles_and_walls(p["x"], p["y"], layout) for p in truth),
            default=None,
        )
        final_truth = truth[-1] if truth else None
        goal_xy = layout["goal_xyyaw"][:2]
        if final_truth:
            final_xy_error = math.hypot(final_truth["x"] - goal_xy[0],
                                        final_truth["y"] - goal_xy[1])
            final_yaw_error = abs(wrap_angle(final_truth["yaw_rad"] -
                                             layout["goal_xyyaw"][2]))
        else:
            final_xy_error, final_yaw_error = None, None
        final_map_pose = compose_map_pose(capture.tf_edges)
        localization_drift = None
        if final_map_pose and final_truth:
            localization_drift = math.hypot(final_map_pose["x"] - final_truth["x"],
                                            final_map_pose["y"] - final_truth["y"])
        max_linear = max((abs(v[1]) for v in capture.cmd_gz_samples), default=0.0)
        max_angular = max((abs(v[2]) for v in capture.cmd_gz_samples), default=0.0)
        command_stopped = command_is_zero
        goal_succeeded = status_name == "SUCCEEDED" and error_code == 0
        truth_twist_stopped = (
            final_truth is not None and abs(final_truth["linear_x_mps"]) < 0.04 and
            abs(final_truth["angular_z_radps"]) < 0.08
        )
        safe_ended = (
            final_truth is not None and truth_pose_is_stationary and
            raw_odom_is_stopped and filtered_odom_is_stopped and command_stopped
        )
        collisions = len(capture.contact_events)
        result["metrics"] = {
            "action_accepted": accepted,
            "action_result": status_name,
            "action_error_code": error_code,
            "action_error_message": error_msg,
            "wall_elapsed_s": round(time.monotonic() - start_wall, 3),
            "sim_elapsed_s": (
                round((capture.clock_latest or 0.0) - first_truth["sim_time_s"], 3)
                if capture.clock_latest is not None else None
            ),
            "truth_path_length_m": round(traveled, 4),
            "latest_planned_path_length_m": round(capture.plan_length_latest, 4),
            "peak_planned_path_length_m": round(capture.plan_length_peak, 4),
            "plan_message_count": capture.plan_count,
            "final_truth_pose": final_truth,
            "final_xy_goal_error_m": None if final_xy_error is None else round(final_xy_error, 4),
            "final_yaw_goal_error_rad": None if final_yaw_error is None else round(final_yaw_error, 4),
            "final_estimated_map_pose": final_map_pose,
            "final_localization_drift_xy_m": (
                None if localization_drift is None else round(localization_drift, 4)
            ),
            "minimum_conservative_clearance_m": (
                None if min_clearance is None else round(min_clearance, 4)
            ),
            "contact_collision_events": collisions,
            "contact_collision_pairs": capture.contact_events,
            "ground_contact_messages_filtered": capture.ground_contact_messages,
            "non_ground_contact_messages": capture.other_contact_messages,
            "max_abs_cmd_vel_linear_x_mps": round(max_linear, 4),
            "max_abs_cmd_vel_angular_z_radps": round(max_angular, 4),
            "command_guard_limits_mps_radps": [0.25, 1.0],
            "post_action_robot_stopped": safe_ended,
            "post_action_settle_time_s": round(time.monotonic() - settle_start, 3),
            "post_action_settle_window_expired": settle_expired,
            "post_action_truth_pose_window": final_pose_window,
            "post_action_raw_odom_velocity": capture.raw_odom_velocity,
            "post_action_filtered_odom_velocity": capture.filtered_odom_velocity,
            "post_action_truth_twist_stopped": truth_twist_stopped,
            "truth_samples": len(truth),
            "raw_odom_samples": capture.odom_count,
            "filtered_odom_samples": capture.filtered_odom_count,
            "raw_odom_path_length_m": round(path_length(capture.raw_odom_points), 4),
            "filtered_odom_path_length_m": round(path_length(capture.filtered_odom_points), 4),
            "command_nav_samples": len(capture.cmd_nav_samples),
            "command_gazebo_samples": len(capture.cmd_gz_samples),
            "sensor_counts": {
                "camera_image_raw": capture.image_count,
                "imu_data": capture.imu_count, "clock": capture.clock_count,
                "map": capture.map_count, "odom": capture.odom_count,
                "odometry_filtered": capture.filtered_odom_count,
            },
            "evaluator_truth_errors": capture.truth_errors,
        }

        tf_ok = all(key in capture.tf_edges for key in
                    (("map", "odom"), ("odom", "base_footprint")))
        sensor_ok = (capture.imu_count > 0 and capture.clock_count > 0 and
                     capture.odom_count > 0 and capture.filtered_odom_count > 0)
        motion_ok = traveled > 0.15 and max_linear <= 0.251 and max_angular <= 1.001
        no_collision = collisions == 0 and (min_clearance is None or min_clearance >= -0.03)
        if args.scenario == "S09":
            safe_no_route = (
                accepted and status_name != "SUCCEEDED" and not timed_out and
                traveled <= 0.60 and no_collision and safe_ended
            )
            add_check("no_route_not_reported_as_success",
                      accepted and status_name != "SUCCEEDED",
                      f"accepted={accepted}, result={status_name}, error={error_code}")
            add_check("no_route_run_stays_bounded_and_stopped", safe_no_route,
                      f"travel={traveled:.3f} m, timeout={timed_out}, "
                      f"stopped={safe_ended}, contacts={collisions}, "
                      f"settle_s={result['metrics']['post_action_settle_time_s']}, "
                      f"settle_window_expired={settle_expired}, "
                      f"truth_pose_window={final_pose_window}, "
                      f"raw_odom={capture.raw_odom_velocity}, "
                      f"filtered_odom={capture.filtered_odom_velocity}, "
                      f"truth_twist_stopped={truth_twist_stopped}")
            result["classification"] = "PASS_EXPECTED_NO_ROUTE" if safe_no_route else "FAIL"
        else:
            goal_accuracy = (
                final_xy_error is not None and final_xy_error <= 0.30 and
                final_yaw_error is not None and final_yaw_error <= 0.35
            )
            passed = (goal_succeeded and goal_accuracy and motion_ok and no_collision and
                      sensor_ok and tf_ok and safe_ended)
            add_check("navigate_to_pose_succeeded_with_zero_error_code",
                      goal_succeeded, f"status={status_name}, error_code={error_code}")
            add_check("goal_reached_against_evaluator_truth", goal_accuracy,
                      f"truth final goal error={final_xy_error} m / {final_yaw_error} rad")
            add_check("robot_motion_observed_with_velocity_limits", motion_ok,
                      f"truth travel={traveled:.3f} m, max command={max_linear:.3f} m/s / "
                      f"{max_angular:.3f} rad/s")
            add_check("no_obstacle_collision_and_clearance_recorded", no_collision,
                      f"contact events={collisions}; conservative min clearance={min_clearance}")
            add_check("simulation_sensor_streams_present", sensor_ok,
                      f"reference profile uses IMU/clock/odom; camera disabled; "
                      f"imu={capture.imu_count}, odom={capture.odom_count}, "
                      f"filtered={capture.filtered_odom_count}")
            add_check("single_authority_tf_chain_present", tf_ok,
                      "odom->base_footprint observed; map->odom is reconciled from the bag below")
            add_check("controller_stopped_after_action", safe_ended,
                      f"post-action truth speeds="
                      f"{None if not final_truth else [final_truth['linear_x_mps'], final_truth['angular_z_radps']]}; "
                      f"latest command zero={command_stopped}")
            result["classification"] = "PASS" if passed else "FAIL"

        result["runtime_graph"] = {}
        for name, command in (
            ("tf_publishers", ["ros2", "topic", "info", "/tf", "--verbose"]),
            ("tf_static_publishers", ["ros2", "topic", "info", "/tf_static", "--verbose"]),
            ("cmd_vel_publishers", ["ros2", "topic", "info", "/cmd_vel", "--verbose"]),
            ("nodes", ["ros2", "node", "list"]),
        ):
            try:
                completed = subprocess.run(
                    command, cwd=ROOT, env=env, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    timeout=12, check=False,
                )
                (run_dir / f"graph-{name}.log").write_text(completed.stdout)
                result["runtime_graph"][name] = {
                    "returncode": completed.returncode,
                    "output_file": f"graph-{name}.log",
                    "summary": completed.stdout[-3500:],
                }
            except Exception as exc:
                result["runtime_graph"][name] = {
                    "error": f"{type(exc).__name__}: {exc}",
                }
        nodes_output = result["runtime_graph"].get("nodes", {}).get("summary", "")
        add_check("no_amcl_or_truth_navigation_node",
                  "amcl" not in nodes_output.lower(),
                  "AMCL is absent; evaluator is the only Gazebo entity-state service client")

        map_images, map_image_error = write_costmap_image(capture, run_dir)
        result["costmap_images"] = map_images
        if map_image_error:
            result["costmap_image_error"] = map_image_error
        result["trace_image"] = "navigation_trace.png"
        trace_error = write_trace_plot(layout, capture, run_dir / "navigation_trace.png")
        if trace_error:
            result["trace_image_error"] = trace_error

        with (run_dir / "truth_trajectory.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=[
                "monotonic_s", "sim_time_s", "x", "y", "yaw_rad",
                "linear_x_mps", "angular_z_radps",
            ])
            writer.writeheader()
            for sample in truth:
                writer.writerow({key: sample[key] for key in writer.fieldnames})
        result["evidence_files"] = [
            "manifest.json", "world.sdf", "oracle_reference_map.yaml",
            "oracle_reference_map.pgm", "layout.png", "navigation_trace.png",
            "truth_trajectory.csv", "launch.log", "rosbag.log", "rosbag/",
        ]
        result["status"] = (
            "PASS" if result["classification"] in ("PASS", "PASS_EXPECTED_NO_ROUTE")
            and all(item["status"] == "PASS" for item in result["tests"])
            else "FAIL"
        )
    except Exception as exc:
        result["runner_error"] = f"{type(exc).__name__}: {exc}"
        result["status"], result["classification"] = "FAIL", "FAIL"
    finally:
        pause_result = {"requested": False, "success": False, "detail": "simulation unavailable"}
        if process is not None and process.poll() is None:
            pause_result["requested"] = True
            try:
                pause_call = subprocess.run(
                    [
                        "ros2", "service", "call", "/gzserver/set_simulation_state",
                        "simulation_interfaces/srv/SetSimulationState",
                        "{state: {state: 2}}",
                    ],
                    cwd=ROOT, env=env, capture_output=True, text=True, timeout=8,
                )
                pause_output = (pause_call.stdout + pause_call.stderr).strip()
                pause_result.update({
                    "returncode": pause_call.returncode,
                    "success": pause_call.returncode == 0 and
                               ("result=1" in pause_output or "result: 1" in pause_output),
                    "detail": pause_output[-2000:],
                })
            except Exception as exc:
                pause_result["detail"] = f"{type(exc).__name__}: {exc}"
        result["simulation_pause_before_shutdown"] = pause_result
        if bag_process is not None:
            bag_shutdown = stop_bag(bag_process)
        if bag_shutdown and bag_shutdown["stopped"] and (run_dir / "rosbag").exists():
            try:
                bag_tf = inspect_bag_tf(run_dir / "rosbag")
                result["tf_bag_validation"] = bag_tf
                tf_publishers = []
                for endpoint in capture.get_publishers_info_by_topic("/tf"):
                    tf_publishers.append({
                        "node_name": endpoint.node_name,
                        "node_namespace": endpoint.node_namespace,
                        "topic_type": endpoint.topic_type,
                        "endpoint_gid": bytes(endpoint.endpoint_gid).hex(),
                    })
                result["tf_dynamic_publishers"] = tf_publishers
                map_key = "/tf_static:map->odom"
                odom_key = "/tf:odom->base_footprint"
                map_edge = bag_tf.get(map_key)
                odom_edge = bag_tf.get(odom_key)
                expected = layout["start_xyyaw"]
                map_pose = None if not map_edge else map_edge["latest"]
                map_tf_ok = (
                    map_edge is not None and map_edge["count"] == 1 and
                    math.hypot(map_pose["x"] - expected[0],
                               map_pose["y"] - expected[1]) <= 1e-4 and
                    abs(wrap_angle(map_pose["yaw"] - expected[2])) <= 1e-4
                )
                ekf_publishers = [
                    item for item in tf_publishers
                    if item["node_name"] == "anc_ekf_filter"
                ]
                publisher_gids = {item["endpoint_gid"] for item in ekf_publishers}
                allowed_tf_nodes = {"anc_ekf_filter", "robot_state_publisher"}
                unexpected_tf_nodes = {
                    item["node_name"] for item in tf_publishers
                    if item["node_name"] not in allowed_tf_nodes
                }
                odom_tf_ok = (
                    odom_edge is not None and odom_edge["count"] > 0 and
                    len(ekf_publishers) == 1 and len(publisher_gids) == 1 and
                    not unexpected_tf_nodes
                )
                set_check(
                    "single_known_start_map_to_odom_transform_present",
                    map_tf_ok,
                    f"MCAP /tf_static map->odom count="
                    f"{None if map_edge is None else map_edge['count']}; "
                    f"transform={map_pose}; expected known start={expected}; "
                    "configured sole broadcaster=anc_known_start_map_to_odom",
                )
                set_check(
                    "single_ekf_odom_to_base_transform_present",
                    odom_tf_ok,
                    f"MCAP /tf odom->base_footprint count="
                    f"{None if odom_edge is None else odom_edge['count']}; "
                    f"active /tf publishers={[item['node_name'] for item in tf_publishers]}; "
                    f"EKF endpoints={len(ekf_publishers)}, distinct GIDs={len(publisher_gids)}; "
                    f"unexpected endpoints={sorted(unexpected_tf_nodes)}",
                )
                tf_chain_ok = map_tf_ok and odom_tf_ok
                set_check(
                    "single_authority_tf_chain_present",
                    tf_chain_ok,
                    "One known-start map->odom transform in /tf_static and one "
                    "EKF publisher GID for odom->base_footprint in /tf.",
                )
                result["tf_authorities"] = {
                    "map_to_odom": {
                        "node": "anc_known_start_map_to_odom",
                        "topic": "/tf_static",
                        "unique_static_edge_messages": (
                            None if map_edge is None else map_edge["count"]),
                    },
                    "odom_to_base_footprint": {
                        "node": "anc_ekf_filter",
                        "topic": "/tf",
                        "publisher_gid_count": len(publisher_gids),
                        "publisher_node_names": sorted({
                            item["node_name"] for item in tf_publishers}),
                        "unexpected_publisher_node_names": sorted(unexpected_tf_nodes),
                    },
                    "amcl_launched": False,
                    "gazebo_truth_in_navigation_graph": False,
                }
                if map_tf_ok and odom_tf_ok:
                    dynamic_pose = odom_edge["latest"]
                    composed = compose_map_pose({
                        ("map", "odom"): map_pose,
                        ("odom", "base_footprint"): dynamic_pose,
                    })
                    result["tf_final_from_bag"] = composed
                    if result.get("metrics") is not None:
                        result["metrics"]["final_estimated_map_pose"] = composed
                        if result["metrics"].get("final_truth_pose"):
                            truth_pose = result["metrics"]["final_truth_pose"]
                            drift = math.hypot(composed["x"] - truth_pose["x"],
                                               composed["y"] - truth_pose["y"])
                            result["metrics"]["final_localization_drift_xy_m"] = round(drift, 4)
            except Exception as exc:
                result["tf_bag_validation_error"] = f"{type(exc).__name__}: {exc}"
                set_check("single_authority_tf_chain_present", False,
                          result["tf_bag_validation_error"])
        if args.hold_window and process is not None and process.poll() is None:
            action_summary = result.get("action", {})
            metric_summary = result.get("metrics", {})
            print("\n=== REFERENCE NAVIGATION — PREDEFINED MAP ===", flush=True)
            print(f"Scenario {args.scenario}, seed {args.seed}", flush=True)
            print(f"Goal status: {action_summary.get('status', 'unavailable')} / "
                  f"classification: {result.get('classification', 'unavailable')}", flush=True)
            print(f"Final goal error: {metric_summary.get('final_xy_goal_error_m', 'unavailable')} m; "
                  f"collisions: {metric_summary.get('contact_collision_events', 'unavailable')}", flush=True)
            print(f"Live result folder: {run_dir}", flush=True)
            print("The simulation is paused with the final scene visible. "
                  "Press Ctrl+C here or select Stop from the launcher menu to clean up.", flush=True)
            while process.poll() is None:
                try:
                    time.sleep(0.25)
                except KeyboardInterrupt:
                    print("\nStopping the held reference scene cleanly...", flush=True)
                    break
        if process is not None:
            shutdown = shutdown_group(process)
        result["shutdown"] = shutdown or {
            "signal": None, "returncode": None, "group_gone": True, "forced": False,
        }
        result["rosbag"]["shutdown"] = bag_shutdown or {
            "returncode": None, "stopped": True,
        }
        launch_log.flush()
        launch_log.close()
        bag_log.flush()
        bag_log.close()
        launch_text = (run_dir / "launch.log").read_text(errors="replace")
        shutdown["child_sigterm_escalated"] = (
            "escalating to 'SIGTERM'" in launch_text or
            "sending signal 'SIGTERM'" in launch_text
        )
        shutdown["child_sigkill_escalated"] = (
            "escalating to 'SIGKILL'" in launch_text or
            "sending signal 'SIGKILL'" in launch_text
        )
        shutdown["gazebo_child_shutdown_clean"] = not (
            shutdown["child_sigterm_escalated"] or shutdown["child_sigkill_escalated"]
        )
        if not shutdown["gazebo_child_shutdown_clean"]:
            result.setdefault("limitations", []).append(
                "Gazebo required launch-managed signal escalation during shutdown; "
                "the runner still verified the simulation process group was gone."
            )
        capture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        clean = (
            result["shutdown"]["group_gone"] and not result["shutdown"]["forced"] and
            result["shutdown"]["returncode"] in (0, -signal.SIGINT, None)
        )
        add_check("simulation_shutdown_bounded", clean,
                  json.dumps(result["shutdown"], sort_keys=True))
        add_check(
            "gazebo_pause_requested_before_shutdown",
            result["simulation_pause_before_shutdown"]["success"],
            json.dumps(result["simulation_pause_before_shutdown"], sort_keys=True),
        )
        add_check("rosbag_closed", result["rosbag"]["shutdown"]["stopped"],
                  json.dumps(result["rosbag"]["shutdown"], sort_keys=True))
        result["status"] = (
            "PASS" if result.get("classification") in ("PASS", "PASS_EXPECTED_NO_ROUTE")
            and all(item["status"] == "PASS" for item in result["tests"])
            else "FAIL"
        )
        (run_dir / "result.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n")
        print(json.dumps({
            "run_id": args.run_id, "scenario_id": args.scenario,
            "seed": args.seed, "profile": result["navigation_profile"],
            "status": result["status"], "classification": result.get("classification"),
            "action": result.get("action"), "metrics": result.get("metrics"),
            "runner_error": result.get("runner_error"),
            "limitations": result.get("limitations", []),
            "failed_checks": [item["name"] for item in result["tests"]
                              if item["status"] != "PASS"],
            "run_dir": str(run_dir), "shutdown": result.get("shutdown"),
        }, indent=2))
    return 0 if result["status"] == "PASS" else 1


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True,
                        choices=("S00", "S01", "S02", "S03", "S04", "S09"))
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    parser.add_argument("--navigation-timeout-s", type=float, default=100.0)
    parser.add_argument("--gui", action="store_true", help="Start Gazebo with its graphical interface.")
    parser.add_argument("--rviz", action="store_true", help="Also start optional RViz displays.")
    parser.add_argument("--hold-window", action="store_true", help="Keep the paused final Gazebo scene visible until interrupted.")
    args = parser.parse_args()
    if args.rviz and not args.gui:
        parser.error("--rviz requires --gui")
    if args.hold_window and not args.gui:
        parser.error("--hold-window requires --gui")
    try:
        sys.exit(run(args))
    except Exception as exc:
        print(f"Phase 6 runner failed before evidence initialization: "
              f"{type(exc).__name__}: {exc}", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()

