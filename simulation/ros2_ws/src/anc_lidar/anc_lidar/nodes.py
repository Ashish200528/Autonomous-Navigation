"""ROS 2 nodes for the provisional RGB + 2D LiDAR profile."""
import copy
import json
import math
import random
import time
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from anc_lidar.scan_model import (
    Grid, condition_ranges, fuse_lidar_with_rgb_free, inflate_obstacles,
    raytrace_scan, validate_scan,
)

NANO = 1_000_000_000


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec)/NANO


def yaw_roll_pitch(q):
    sinr = 2.0*(q.w*q.x + q.y*q.z)
    cosr = 1.0 - 2.0*(q.x*q.x + q.y*q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0*(q.w*q.y - q.z*q.x)
    pitch = math.copysign(math.pi/2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                     1.0 - 2.0*(q.y*q.y + q.z*q.z))
    return roll, pitch, yaw


def grid_to_ros(grid, data, stamp):
    msg = OccupancyGrid()
    msg.header.frame_id = grid.frame_id
    msg.header.stamp = stamp
    msg.info.resolution = float(grid.resolution_m)
    msg.info.width = int(grid.width)
    msg.info.height = int(grid.height)
    msg.info.origin.position.x = float(grid.origin_x_m)
    msg.info.origin.position.y = float(grid.origin_y_m)
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(value) for value in data]
    return msg


def occupancy_counts(data):
    return {"free": sum(v == 0 for v in data),
            "occupied": sum(v == 100 for v in data),
            "unknown": sum(v == -1 for v in data),
            "total": len(data)}


class ScanConditioner(Node):
    def __init__(self):
        super().__init__("anc_lidar_scan_conditioner")
        defaults = {
            "input_topic": "/realistic/scan/raw",
            "output_topic": "/realistic/scan",
            "health_topic": "/realistic/scan/health",
            "clock_topic": "/clock",
            "expected_frame": "lidar_link",
            "expected_samples": 720,
            "angle_min_rad": -math.pi,
            "angle_max_rad": math.pi,
            "range_min_m": 0.12,
            "range_max_m": 8.0,
            "expected_rate_hz": 8.0,
            "minimum_rate_hz": 6.0,
            "minimum_usable_fraction": 0.90,
            "missed_probability": 0.005,
            "invalid_probability": 0.002,
            "random_seed": 20261102,
            "max_wall_age_s": 0.25,
            "max_sim_age_s": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.rng = random.Random(int(self.p["random_seed"]))
        self.clock_s = None
        self.clock_wall = None
        self.last_wall = None
        self.last_scan_stamp_s = None
        self.rate_stamps = deque(maxlen=32)
        self.last_counts = {}
        self.last_reason = "waiting_for_scan"
        self.pub = self.create_publisher(LaserScan, self.p["output_topic"],
                                          qos_profile_sensor_data)
        self.health_pub = self.create_publisher(String, self.p["health_topic"], 10)
        self.create_subscription(LaserScan, self.p["input_topic"], self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, self.p["clock_topic"], self.on_clock,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.check_stale, clock=RclClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Conditioning only measured Gazebo LaserScan values.")

    def on_clock(self, msg):
        self.clock_s = stamp_seconds(msg.clock)
        self.clock_wall = time.monotonic()

    def publish_health(self, valid, reason, meta, counts, rate_hz, age_s):
        payload = {
            "node": "anc_lidar_scan_conditioner",
            "profile": "REALISTIC_RGB_LIDAR",
            "valid": bool(valid),
            "status": "VALID" if valid else "INVALID_UNKNOWN",
            "reason": str(reason),
            "frame_id": str(meta.get("frame_id", "")),
            "scan_stamp_s": meta.get("scan_stamp_s"),
            "sim_age_s": age_s,
            "wall_age_s": (None if self.last_wall is None
                           else max(0.0, time.monotonic()-self.last_wall)),
            "measured_rate_hz": rate_hz,
            "expected_rate_hz": float(self.p["expected_rate_hz"]),
            "angle_min_rad": meta.get("angle_min_rad"),
            "angle_max_rad": meta.get("angle_max_rad"),
            "angle_increment_rad": meta.get("angle_increment_rad"),
            "range_min_m": meta.get("range_min_m"),
            "range_max_m": meta.get("range_max_m"),
            "counts": counts,
            "runtime_uses_world_map_or_truth": False,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.health_pub.publish(msg)

    def on_scan(self, scan):
        wall_now = time.monotonic()
        stamp = stamp_seconds(scan.header.stamp)
        self.last_wall = wall_now
        self.last_scan_stamp_s = stamp
        self.rate_stamps.append(wall_now)
        rate = None
        if len(self.rate_stamps) >= 4:
            span = self.rate_stamps[-1]-self.rate_stamps[0]
            if span > 0:
                rate = (len(self.rate_stamps)-1)/span
        reason = validate_scan(
            scan, frame_id=str(self.p["expected_frame"]),
            samples=int(self.p["expected_samples"]),
            angle_min_rad=float(self.p["angle_min_rad"]),
            angle_max_rad=float(self.p["angle_max_rad"]),
            range_min_m=float(self.p["range_min_m"]),
            range_max_m=float(self.p["range_max_m"]),
        )
        out = copy.deepcopy(scan)
        counts = {"valid_hit_beams": 0, "no_return_beams": 0,
                  "missed_beams": 0, "invalid_beams": len(scan.ranges),
                  "usable_beams": 0, "total_beams": len(scan.ranges)}
        if reason is None:
            out.ranges, counts = condition_ranges(
                scan.ranges, range_min_m=float(self.p["range_min_m"]),
                range_max_m=float(self.p["range_max_m"]), rng=self.rng,
                missed_probability=float(self.p["missed_probability"]),
                invalid_probability=float(self.p["invalid_probability"]),
            )
        else:
            out.ranges = [float("nan")]*len(scan.ranges)
        self.pub.publish(out)
        usable_fraction = (counts["usable_beams"]/max(counts["total_beams"], 1))
        sim_age = None if self.clock_s is None else self.clock_s-stamp
        if reason is None and self.clock_s is None:
            reason = "simulation_clock_missing"
        elif reason is None and (sim_age < -0.05 or sim_age > float(self.p["max_sim_age_s"])):
            reason = "scan_sim_timestamp_stale_or_future"
        elif reason is None and rate is None:
            reason = "scan_rate_not_yet_measured"
        elif reason is None and rate < float(self.p["minimum_rate_hz"]):
            reason = "scan_rate_below_minimum"
        elif reason is None and usable_fraction < float(self.p["minimum_usable_fraction"]):
            reason = "usable_beam_fraction_below_minimum"
        self.last_counts = counts
        self.last_reason = reason or "scan_contract_valid"
        meta = {
            "frame_id": scan.header.frame_id, "scan_stamp_s": stamp,
            "angle_min_rad": float(scan.angle_min),
            "angle_max_rad": float(scan.angle_max),
            "angle_increment_rad": float(scan.angle_increment),
            "range_min_m": float(scan.range_min), "range_max_m": float(scan.range_max),
        }
        self.publish_health(reason is None, self.last_reason, meta, counts, rate, sim_age)

    def check_stale(self):
        if self.last_wall is None:
            return
        age = time.monotonic()-self.last_wall
        if age <= float(self.p["max_wall_age_s"]):
            return
        meta = {"frame_id": str(self.p["expected_frame"]),
                "scan_stamp_s": self.last_scan_stamp_s}
        self.publish_health(False, "scan_wall_stale", meta, self.last_counts,
                            None, None)


class LocalScanMapper(Node):
    def __init__(self):
        super().__init__("anc_local_scan_mapper")
        defaults = {
            "scan_topic": "/realistic/scan",
            "scan_health_topic": "/realistic/scan/health",
            "map_topic": "/realistic/local_map/lidar",
            "inflated_map_topic": "/realistic/local_map/lidar_inflated",
            "health_topic": "/realistic/local_map/lidar/health",
            "base_frame": "base_link",
            "scan_frame": "lidar_link",
            "grid_width": 180,
            "grid_height": 180,
            "resolution_m": 0.10,
            "origin_x_m": -9.0,
            "origin_y_m": -9.0,
            "inflation_radius_m": 0.28,
            "expected_lidar_z_in_base_m": 0.08,
            "mount_tolerance_m": 0.02,
            "max_wall_age_s": 0.25,
            "max_sim_age_s": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.grid = Grid(int(self.p["grid_width"]), int(self.p["grid_height"]),
                         float(self.p["resolution_m"]), float(self.p["origin_x_m"]),
                         float(self.p["origin_y_m"]), str(self.p["base_frame"]))
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self, spin_thread=False)
        self.scan_health = None
        self.scan_health_wall = None
        self.latest_scan_wall = None
        self.clock_s = None
        self.last_scan = None
        self.map_pub = self.create_publisher(OccupancyGrid, self.p["map_topic"], 10)
        self.inflated_pub = self.create_publisher(OccupancyGrid,
                                                   self.p["inflated_map_topic"], 10)
        self.health_pub = self.create_publisher(String, self.p["health_topic"], 10)
        self.create_subscription(LaserScan, self.p["scan_topic"], self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, self.p["scan_health_topic"], self.on_scan_health, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_timer(0.10, self.check_stale, clock=RclClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Publishing one-scan base_link local maps from LaserScan + TF only.")

    def on_clock(self, msg):
        self.clock_s = stamp_seconds(msg.clock)

    def on_scan_health(self, msg):
        try:
            self.scan_health = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.scan_health = {"valid": False, "reason": "scan_health_json_invalid"}
        self.scan_health_wall = time.monotonic()

    def publish_map(self, scan, data, valid, reason, transform, started):
        stamp = scan.header.stamp
        self.map_pub.publish(grid_to_ros(self.grid, data, stamp))
        inflated = inflate_obstacles(data, self.grid, float(self.p["inflation_radius_m"]))
        self.inflated_pub.publish(grid_to_ros(self.grid, inflated, stamp))
        counts = occupancy_counts(data)
        payload = {
            "node": "anc_local_scan_mapper",
            "valid": bool(valid), "status": "VALID" if valid else "INVALID_UNKNOWN",
            "reason": str(reason), "frame_id": self.grid.frame_id,
            "scan_frame": scan.header.frame_id,
            "scan_stamp_s": stamp_seconds(stamp),
            "map_build_wall_s": time.monotonic()-started,
            "scanner_transform_base_xyz_m": transform.get("xyz_m") if transform else None,
            "scanner_transform_rpy_rad": transform.get("rpy_rad") if transform else None,
            "counts": counts,
            "grid": {"width": self.grid.width, "height": self.grid.height,
                     "resolution_m": self.grid.resolution_m,
                     "origin_xy_m": [self.grid.origin_x_m, self.grid.origin_y_m]},
            "occupancy_semantics": {"FREE": 0, "OCCUPIED": 100, "UNKNOWN": -1},
            "unknown_preserved": True,
            "runtime_uses_world_map_or_truth": False,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.health_pub.publish(msg)

    def on_scan(self, scan):
        started = time.monotonic()
        self.last_scan = copy.deepcopy(scan)
        self.latest_scan_wall = started
        data = [-1]*(self.grid.width*self.grid.height)
        reason = None
        transform_meta = None
        if self.scan_health is None or not self.scan_health.get("valid", False):
            reason = "scan_health_invalid_or_missing"
        elif time.monotonic()-self.scan_health_wall > float(self.p["max_wall_age_s"]):
            reason = "scan_health_stale"
        elif scan.header.frame_id.lstrip("/") != str(self.p["scan_frame"]).lstrip("/"):
            reason = "scan_frame_mismatch"
        elif self.clock_s is None or self.clock_s-stamp_seconds(scan.header.stamp) > float(self.p["max_sim_age_s"]):
            reason = "scan_sim_time_stale_or_clock_missing"
        if reason is None:
            try:
                tf = self.buffer.lookup_transform(
                    self.grid.frame_id, str(self.p["scan_frame"]),
                    Time.from_msg(scan.header.stamp),
                )
                t, q = tf.transform.translation, tf.transform.rotation
                roll, pitch, yaw = yaw_roll_pitch(q)
                xyz = (float(t.x), float(t.y), float(t.z))
                if abs(xyz[2]-float(self.p["expected_lidar_z_in_base_m"])) > float(self.p["mount_tolerance_m"]):
                    raise ValueError("lidar_mount_height_mismatch")
                if max(abs(roll), abs(pitch)) > 0.02:
                    raise ValueError("lidar_scan_plane_not_horizontal")
                transform_meta = {"xyz_m": list(xyz), "rpy_rad": [roll, pitch, yaw]}
                data = raytrace_scan(scan, self.grid, scanner_x_m=xyz[0],
                                     scanner_y_m=xyz[1], scanner_yaw_rad=yaw)
            except (TransformException, ValueError, TypeError) as exc:
                reason = "tf_or_scan_geometry_invalid:" + str(exc)
        if reason is not None:
            data = [-1]*(self.grid.width*self.grid.height)
        self.publish_map(scan, data, reason is None,
                         "single_scan_raytrace" if reason is None else reason,
                         transform_meta, started)

    def check_stale(self):
        if self.last_scan is None or self.latest_scan_wall is None:
            return
        if time.monotonic()-self.latest_scan_wall <= float(self.p["max_wall_age_s"]):
            return
        unknown = [-1]*(self.grid.width*self.grid.height)
        self.publish_map(self.last_scan, unknown, False, "scan_wall_stale", None, time.monotonic())


class LocalGridFuser(Node):
    def __init__(self):
        super().__init__("anc_rgb_lidar_grid_fuser")
        defaults = {
            "lidar_map_topic": "/realistic/local_map/lidar",
            "rgb_map_topic": "/realistic/local_traversability",
            "fused_map_topic": "/realistic/local_map/fused",
            "health_topic": "/realistic/local_map/fused/health",
            "mode": "rgb_lidar",
            "allow_projected_rgb_free": True,
            "max_wall_age_s": 0.40,
            "publish_rate_hz": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.lidar = self.rgb = None
        self.lidar_wall = self.rgb_wall = None
        self.pub = self.create_publisher(OccupancyGrid, self.p["fused_map_topic"], 10)
        self.health_pub = self.create_publisher(String, self.p["health_topic"], 10)
        self.create_subscription(OccupancyGrid, self.p["lidar_map_topic"], self.on_lidar, 10)
        self.create_subscription(OccupancyGrid, self.p["rgb_map_topic"], self.on_rgb, 10)
        self.create_timer(1.0/float(self.p["publish_rate_hz"]), self.publish_fused, clock=RclClock(clock_type=ClockType.STEADY_TIME))

    def on_lidar(self, msg):
        self.lidar = msg
        self.lidar_wall = time.monotonic()

    def on_rgb(self, msg):
        self.rgb = msg
        self.rgb_wall = time.monotonic()

    def publish_fused(self):
        if self.lidar is None:
            return
        age = time.monotonic()-self.lidar_wall
        lidar_data = list(self.lidar.data)
        use_rgb = (self.p["mode"] == "rgb_lidar"
                   and bool(self.p["allow_projected_rgb_free"])
                   and self.rgb is not None
                   and time.monotonic()-self.rgb_wall <= float(self.p["max_wall_age_s"]))
        reason = "lidar_only" if not use_rgb else "rgb_free_only_supplement"
        data = lidar_data
        try:
            if use_rgb:
                ql = self.lidar.info.origin.orientation
                qr = self.rgb.info.origin.orientation
                if (abs(ql.x)+abs(ql.y)+abs(ql.z) > 1e-4
                        or abs(qr.x)+abs(qr.y)+abs(qr.z) > 1e-4
                        or self.lidar.header.frame_id != self.rgb.header.frame_id):
                    raise ValueError("fusion_frame_or_grid_orientation_mismatch")
                rgb_geometry = (
                    int(self.rgb.info.width), int(self.rgb.info.height),
                    float(self.rgb.info.resolution),
                    float(self.rgb.info.origin.position.x),
                    float(self.rgb.info.origin.position.y),
                    self.rgb.header.frame_id,
                )
                grid = Grid(int(self.lidar.info.width), int(self.lidar.info.height),
                            float(self.lidar.info.resolution),
                            float(self.lidar.info.origin.position.x),
                            float(self.lidar.info.origin.position.y),
                            self.lidar.header.frame_id)
                data = fuse_lidar_with_rgb_free(
                    lidar_data, grid, list(self.rgb.data), rgb_geometry)
        except (ValueError, TypeError) as exc:
            reason = "fusion_disabled:" + str(exc)
            data = lidar_data
            use_rgb = False
        out = copy.deepcopy(self.lidar)
        out.data = [int(v) for v in data]
        if age > float(self.p["max_wall_age_s"]):
            out.data = [-1]*len(out.data)
            reason = "lidar_map_stale_unknown"
        self.pub.publish(out)
        payload = {
            "node": "anc_rgb_lidar_grid_fuser", "valid": age <= float(self.p["max_wall_age_s"]),
            "mode": str(self.p["mode"]), "rgb_supplement_used": bool(use_rgb),
            "reason": reason, "rgb_occupied_used": False,
            "lidar_occupied_or_free_overwritten": False,
            "counts": occupancy_counts(out.data),
        }
        health = String()
        health.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.health_pub.publish(health)


def _spin(node_type):
    rclpy.init()
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def scan_conditioner_main():
    _spin(ScanConditioner)


def local_scan_mapper_main():
    _spin(LocalScanMapper)


def local_grid_fuser_main():
    _spin(LocalGridFuser)
