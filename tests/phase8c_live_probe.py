#!/usr/bin/env python3
"""Collect bounded live Phase 8C sensor and safety metrics from ROS topics."""
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
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)/1_000_000_000.0


def parse_json(msg):
    try:
        return json.loads(msg.data)
    except (TypeError, json.JSONDecodeError):
        return {"parse_error": True, "text": msg.data[:120]}


class Probe(Node):
    def __init__(self):
        super().__init__("phase8c_live_probe")
        self.start = time.monotonic()
        self.scan_records = []
        self.camera_records = []
        self.info_count = self.imu_count = self.odom_count = 0
        self.lidar_map = self.rgb_map = None
        self.lidar_map_wall = self.rgb_map_wall = None
        self.scan_health = self.map_health = self.rgb_state = None
        self.safety = self.guard = None
        self.command_count = self.nonzero_command_count = 0
        self.max_abs_linear = self.max_abs_angular = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_subscription(LaserScan, "/realistic/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/realistic/scan/health",
                                 lambda msg: setattr(self, "scan_health", parse_json(msg)), 10)
        self.create_subscription(OccupancyGrid, "/realistic/local_map/lidar",
                                 self.on_lidar_map, 10)
        self.create_subscription(String, "/realistic/local_map/lidar/health",
                                 lambda msg: setattr(self, "map_health", parse_json(msg)), 10)
        self.create_subscription(OccupancyGrid, "/realistic/local_map/fused",
                                 self.on_rgb_map, 10)
        self.create_subscription(String, "/realistic/navigation_state",
                                 lambda msg: setattr(self, "rgb_state", parse_json(msg)), 10)
        self.create_subscription(String, "/realistic/safety_state",
                                 lambda msg: setattr(self, "safety", parse_json(msg)), 10)
        self.create_subscription(String, "/realistic/command_guard_state",
                                 lambda msg: setattr(self, "guard", parse_json(msg)), 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 lambda _msg: setattr(self, "info_count", self.info_count+1),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data",
                                 lambda _msg: setattr(self, "imu_count", self.imu_count+1),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda _msg: setattr(self, "odom_count", self.odom_count+1),
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

    def on_scan(self, msg):
        wall = time.monotonic()
        usable = [float(v) for v in msg.ranges
                  if math.isfinite(float(v)) and msg.range_min <= float(v) < msg.range_max-1e-6]
        no_return = sum(math.isfinite(float(v)) and float(v) >= msg.range_max-1e-6
                        for v in msg.ranges)
        self.scan_records.append({
            "wall_s": wall, "stamp_s": stamp_seconds(msg.header.stamp),
            "frame_id": msg.header.frame_id, "samples": len(msg.ranges),
            "angle_min_rad": float(msg.angle_min), "angle_max_rad": float(msg.angle_max),
            "angle_increment_rad": float(msg.angle_increment),
            "range_min_m": float(msg.range_min), "range_max_m": float(msg.range_max),
            "usable_hits": len(usable), "no_returns": int(no_return),
            "invalid_or_dropped": len(msg.ranges)-len(usable)-int(no_return),
            "minimum_hit_m": min(usable) if usable else None,
            "maximum_hit_m": max(usable) if usable else None,
        })
        self.scan_records = self.scan_records[-100:]

    def on_camera(self, msg):
        self.camera_records.append((time.monotonic(), stamp_seconds(msg.header.stamp),
                                    msg.header.frame_id, int(msg.width), int(msg.height)))
        self.camera_records = self.camera_records[-100:]

    def on_lidar_map(self, msg):
        self.lidar_map_wall = time.monotonic()
        values = list(msg.data)
        self.lidar_map = {
            "frame_id": msg.header.frame_id, "stamp_s": stamp_seconds(msg.header.stamp),
            "width": int(msg.info.width), "height": int(msg.info.height),
            "resolution_m": float(msg.info.resolution),
            "origin_xy_m": [float(msg.info.origin.position.x), float(msg.info.origin.position.y)],
            "free": sum(v == 0 for v in values), "occupied": sum(v == 100 for v in values),
            "unknown": sum(v == -1 for v in values), "total": len(values),
        }

    def on_rgb_map(self, msg):
        self.rgb_map_wall = time.monotonic()
        self.rgb_map = {
            "frame_id": msg.header.frame_id, "width": int(msg.info.width),
            "height": int(msg.info.height), "resolution_m": float(msg.info.resolution),
            "free": sum(v == 0 for v in msg.data), "occupied": sum(v == 100 for v in msg.data),
            "unknown": sum(v == -1 for v in msg.data),
        }

    def on_cmd(self, msg):
        self.command_count += 1
        vx, wz = float(msg.linear.x), float(msg.angular.z)
        self.max_abs_linear = max(self.max_abs_linear, abs(vx))
        self.max_abs_angular = max(self.max_abs_angular, abs(wz))
        if abs(vx) > 1e-8 or abs(wz) > 1e-8:
            self.nonzero_command_count += 1


def summarize_rate(records, wall_index=0):
    if len(records) < 2:
        return None
    span = (records[-1][wall_index]-records[0][wall_index])
    return None if span <= 0 else (len(records)-1)/span


def main():
    output_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("phase8c-live.json")
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        scans = node.scan_records
        cameras = node.camera_records
        tf_data = None
        tf_error = None
        try:
            tf = node.tf_buffer.lookup_transform("base_link", "lidar_link", Time())
            t, q = tf.transform.translation, tf.transform.rotation
            tf_data = {"translation_m": [t.x, t.y, t.z],
                       "rotation_xyzw": [q.x, q.y, q.z, q.w]}
        except TransformException as exc:
            tf_error = str(exc)
        result = {
            "classification": "LIVE_SENSOR_VALIDATION_NO_MOTION_COMMANDS_ISSUED",
            "duration_s": time.monotonic()-node.start,
            "scan": {
                "messages": len(scans),
                "measured_wall_rate_hz": summarize_rate(scans, "wall_s"),
                "first": None if not scans else scans[0],
                "last": None if not scans else scans[-1],
                "minimum_hit_m_over_run": min((r["minimum_hit_m"] for r in scans
                                                if r["minimum_hit_m"] is not None), default=None),
                "health": node.scan_health,
                "topic": "/realistic/scan",
            },
            "camera": {
                "frames": len(cameras),
                "measured_wall_rate_hz": summarize_rate(cameras),
                "last_frame": None if not cameras else {
                    "stamp_s": cameras[-1][1], "frame_id": cameras[-1][2],
                    "width": cameras[-1][3], "height": cameras[-1][4]},
                "camera_info_messages": node.info_count,
                "topic": "/camera/image_raw",
            },
            "imu_messages": node.imu_count,
            "filtered_odom_messages": node.odom_count,
            "lidar_local_map": node.lidar_map,
            "lidar_map_health": node.map_health,
            "rgb_fused_map": node.rgb_map,
            "rgb_navigation_state": node.rgb_state,
            "safety_state": node.safety,
            "command_guard_state": node.guard,
            "cmd_vel": {"messages": node.command_count,
                        "nonzero_messages": node.nonzero_command_count,
                        "max_abs_linear_mps": node.max_abs_linear,
                        "max_abs_angular_radps": node.max_abs_angular},
            "lidar_static_tf": tf_data,
            "lidar_tf_error": tf_error,
            "runtime_navigation_nodes_use_oracle_map_or_truth": False,
        }
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        print(json.dumps(result, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
