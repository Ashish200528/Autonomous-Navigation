#!/usr/bin/env python3
"""Stationary, low-memory SLAM diagnostic. It publishes no velocity requests."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class SlamMonitor(Node):
    def __init__(self):
        super().__init__("phase8c_slam_stationary_evaluator")
        self.scan_count = 0
        self.map_count = 0
        self.pose_count = 0
        self.tf_map_odom_count = 0
        self.latest_map = None
        self.latest_pose = None
        self.latest_safety = None
        self.sim_clock = None
        self.outputs = []
        self.contact_records = 0
        self.non_ground_contacts = 0
        self.create_subscription(LaserScan, "/realistic/scan",
                                 lambda msg: setattr(self, "scan_count", self.scan_count+1), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/slam_toolbox/pose",
                                 self.on_pose, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.create_subscription(Clock, "/clock",
                                 lambda msg: setattr(self, "sim_clock",
                                     msg.clock.sec+msg.clock.nanosec*1e-9), qos_profile_sensor_data)
        self.create_subscription(String, "/realistic/safety_state",
                                 lambda msg: setattr(self, "latest_safety",
                                     json.loads(msg.data)), 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
        self.create_subscription(Contacts, "/contacts", self.on_contacts, 20)
        self.truth = self.create_client(GetEntityState, "/gzserver/get_entity_state")

    def on_map(self, msg):
        self.map_count += 1
        self.latest_map = {
            "frame_id": msg.header.frame_id,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution_m": float(msg.info.resolution),
            "free_cells": sum(v == 0 for v in msg.data),
            "occupied_cells": sum(v == 100 for v in msg.data),
            "unknown_cells": sum(v < 0 for v in msg.data),
            "sim_stamp_s": msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
        }

    def on_pose(self, msg):
        self.pose_count += 1
        self.latest_pose = {
            "frame_id": msg.header.frame_id,
            "x_m": float(msg.pose.pose.position.x),
            "y_m": float(msg.pose.pose.position.y),
            "yaw_rad": yaw(msg.pose.pose.orientation),
            "sim_stamp_s": msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
        }

    def on_tf(self, msg):
        self.tf_map_odom_count += sum(
            t.header.frame_id.lstrip("/") == "map"
            and t.child_frame_id.lstrip("/") == "odom"
            for t in msg.transforms)

    def on_cmd(self, msg):
        self.outputs.append({
            "linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z),
        })

    def on_contacts(self, msg):
        for contact in msg.contacts:
            self.contact_records += 1
            if "ground_plane" not in (
                    contact.collision1.name+" "+contact.collision2.name).lower():
                self.non_ground_contacts += 1

    def get_truth(self, timeout=6.0):
        if not self.truth.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("truth_service_not_ready")
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.truth.call_async(request)
        deadline = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
        if not future.done() or future.result() is None:
            raise RuntimeError("truth_service_timeout")
        result = future.result()
        if result.result.result != 1:
            raise RuntimeError("truth_query_error:"+result.result.error_message)
        state = result.state
        return {
            "sim_stamp_s": state.header.stamp.sec+state.header.stamp.nanosec*1e-9,
            "x_m": float(state.pose.position.x),
            "y_m": float(state.pose.position.y),
            "yaw_rad": yaw(state.pose.orientation),
        }


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: probe.py OUT.json DURATION_S")
    output = Path(sys.argv[1])
    duration_s = float(sys.argv[2])
    rclpy.init()
    node = SlamMonitor()
    error = None
    try:
        before = node.get_truth()
        started = time.monotonic()
        sim_start = node.sim_clock
        while rclpy.ok() and time.monotonic()-started < duration_s:
            rclpy.spin_once(node, timeout_sec=0.05)
        wall_elapsed = time.monotonic()-started
        after = node.get_truth()
        sim_elapsed = ((node.sim_clock-sim_start)
                       if node.sim_clock is not None and sim_start is not None else None)
        publishers = node.get_publishers_info_by_topic("/cmd_vel")
        max_v = max((abs(item["linear_x_mps"]) for item in node.outputs), default=0.0)
        max_w = max((abs(item["angular_z_radps"]) for item in node.outputs), default=0.0)
        xy_drift = math.hypot(after["x_m"]-before["x_m"], after["y_m"]-before["y_m"])
        yaw_drift = math.atan2(math.sin(after["yaw_rad"]-before["yaw_rad"]),
                               math.cos(after["yaw_rad"]-before["yaw_rad"]))
        result = {
            "profile": "REALISTIC_RGB_LIDAR_STATIONARY_SLAM_DIAGNOSTIC",
            "scenario": "S00",
            "duration_requested_wall_s": duration_s,
            "duration_observed_wall_s": wall_elapsed,
            "duration_observed_sim_s": sim_elapsed,
            "runtime_nodes_receive_no_gazebo_truth": True,
            "evaluator_truth_source": "/gzserver/get_entity_state",
            "velocity_requests_published": 0,
            "scan_messages": node.scan_count,
            "map_messages": node.map_count,
            "map_snapshot": node.latest_map,
            "slam_pose_messages": node.pose_count,
            "last_slam_pose": node.latest_pose,
            "map_to_odom_tf_messages": node.tf_map_odom_count,
            "safety_state": node.latest_safety,
            "cmd_vel_publisher_count": len(publishers),
            "cmd_vel_publishers": [str(p.node_name) for p in publishers],
            "cmd_vel_samples": len(node.outputs),
            "max_abs_cmd_vel_linear_mps": max_v,
            "max_abs_cmd_vel_angular_radps": max_w,
            "contact_records_all": node.contact_records,
            "contact_records_non_ground": node.non_ground_contacts,
            "truth_start_evaluator_only": before,
            "truth_end_evaluator_only": after,
            "truth_xy_drift_m": xy_drift,
            "truth_yaw_drift_rad": yaw_drift,
            "diagnostic_pass": (
                node.scan_count >= duration_s*3.5
                and node.map_count >= 2
                and node.tf_map_odom_count > 0
                and len(publishers) == 1
                and max_v <= 1e-8 and max_w <= 1e-8
                and xy_drift <= 0.002 and abs(yaw_drift) <= 0.002
                and node.non_ground_contacts == 0
            ),
        }
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        result = {"scenario": "S00", "diagnostic_pass": False, "error": error}
    finally:
        result["error"] = error
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        print(json.dumps({
            "diagnostic_pass": result.get("diagnostic_pass"),
            "scan_messages": result.get("scan_messages"),
            "map_messages": result.get("map_messages"),
            "map_snapshot": result.get("map_snapshot"),
            "slam_pose_messages": result.get("slam_pose_messages"),
            "map_to_odom_tf_messages": result.get("map_to_odom_tf_messages"),
            "cmd_vel_publisher_count": result.get("cmd_vel_publisher_count"),
            "max_abs_cmd_vel": [
                result.get("max_abs_cmd_vel_linear_mps"),
                result.get("max_abs_cmd_vel_angular_radps")],
            "truth_xy_drift_m": result.get("truth_xy_drift_m"),
            "truth_yaw_drift_rad": result.get("truth_yaw_drift_rad"),
            "error": error,
        }, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result.get("diagnostic_pass") else 1)


if __name__ == "__main__":
    main()