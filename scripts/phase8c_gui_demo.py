#!/usr/bin/env python3
"""Presentation monitor for the existing REALISTIC_RGB_LIDAR safety chain.

It only publishes an optional short +0.05 m/s request to /cmd_vel_nav for
S00 after a fresh valid/authorized safety state. The existing guard remains
the sole owner of /cmd_vel. It never queries Gazebo truth or requests yaw.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GuiDemoMonitor(Node):
    def __init__(self, scenario, output_path):
        super().__init__("anc_windows_gui_demo_monitor")
        self.scenario = scenario
        self.output_path = Path(output_path)
        self.safety = None
        self.safety_wall = None
        self.scan_count = 0
        self.scan_wall_times = []
        self.image_count = 0
        self.slam_map_count = 0
        self.local_map_count = 0
        self.slam_map_summary = None
        self.local_map_summary = None
        self.odom = None
        self.initial_odom = None
        self.requested = None
        self.guard_output = None
        self.guard_outputs = []
        self.exercise = {
            "requested": False,
            "status": "NOT_REQUESTED",
            "detail": "No velocity request was issued.",
        }
        self.started_wall = time.monotonic()
        self.last_print_wall = 0.0
        self.request_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(String, "/realistic/safety_state",
                                 self.on_safety, 10)
        self.create_subscription(LaserScan, "/realistic/scan",
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw",
                                 self.on_image, qos_profile_sensor_data)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map",
                                 self.on_slam_map, map_qos)
        self.create_subscription(OccupancyGrid, "/realistic/local_map/lidar",
                                 self.on_local_map, 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_request, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_guard_output, 10)

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.safety = {"valid": False, "reason": "safety_state_json_invalid"}
        self.safety_wall = time.monotonic()

    def on_scan(self, msg):
        now = time.monotonic()
        self.scan_count += 1
        self.scan_wall_times.append(now)
        self.scan_wall_times = self.scan_wall_times[-12:]

    def on_image(self, _msg):
        self.image_count += 1

    @staticmethod
    def map_counts(msg):
        data = list(msg.data)
        return {
            "frame_id": msg.header.frame_id,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "free": sum(v == 0 for v in data),
            "occupied": sum(v == 100 for v in data),
            "unknown": sum(v == -1 for v in data),
        }

    def on_slam_map(self, msg):
        self.slam_map_count += 1
        self.slam_map_summary = self.map_counts(msg)

    def on_local_map(self, msg):
        self.local_map_count += 1
        self.local_map_summary = self.map_counts(msg)

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.odom = {
            "x_m": float(pos.x),
            "y_m": float(pos.y),
            "yaw_rad": yaw_from_quaternion(msg.pose.pose.orientation),
            "linear_x_mps": float(msg.twist.twist.linear.x),
            "angular_z_radps": float(msg.twist.twist.angular.z),
            "wall_s": time.monotonic(),
        }
        if self.initial_odom is None:
            self.initial_odom = dict(self.odom)

    @staticmethod
    def twist_record(msg):
        return {
            "linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z),
        }

    def on_request(self, msg):
        self.requested = self.twist_record(msg)

    def on_guard_output(self, msg):
        self.guard_output = self.twist_record(msg)
        if abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6:
            self.guard_outputs.append({
                "wall_s": time.monotonic(),
                **self.guard_output,
            })
            self.guard_outputs = self.guard_outputs[-100:]

    def safety_is_fresh_and_authorized(self, minimum_speed):
        age = None if self.safety_wall is None else time.monotonic() - self.safety_wall
        state = self.safety or {}
        ok = (
            state.get("valid") is True
            and state.get("authorized") is True
            and age is not None and age <= 0.35
            and float(state.get("max_linear_speed_mps", 0.0)) >= minimum_speed
            and float(state.get("max_angular_speed_radps", 0.0)) <= 1e-9
        )
        return ok, age

    def publish_speed(self, linear_x):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        self.request_pub.publish(msg)

    def exercise_guarded_forward(self, duration_s):
        if self.scenario != "S00":
            self.exercise = {
                "requested": False, "status": "BLOCKED_SCENARIO",
                "detail": "Forward requests are restricted to clear S00; this scenario is sensing-only.",
            }
            return
        deadline = time.monotonic() + 8.0
        ready, age = False, None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            ready, age = self.safety_is_fresh_and_authorized(0.05)
            if ready:
                break
        if not ready:
            self.exercise = {
                "requested": False, "status": "BLOCKED_SAFETY_GATE",
                "detail": "No fresh valid/authorized safety state with a >=0.05 m/s forward cap; no motion request was issued.",
                "safety_state": self.safety,
                "safety_state_age_s": age,
            }
            self.publish_status()
            print("Guarded forward movement BLOCKED by the existing safety gate; no request issued.", flush=True)
            return

        self.exercise = {
            "requested": True,
            "status": "REQUESTING_THROUGH_GUARD",
            "detail": "0.05 m/s straight request on /cmd_vel_nav for at most 0.75 s; angular request is zero.",
            "requested_speed_mps": 0.05,
            "duration_s": duration_s,
            "safety_state_at_start": self.safety,
            "safety_state_age_s": age,
            "start_odometry": self.odom,
        }
        print("S00: sending bounded 0.05 m/s straight request only to /cmd_vel_nav for at most 0.75 s.", flush=True)
        stop_deadline = time.monotonic() + duration_s
        sent_nonzero = False
        try:
            while time.monotonic() < stop_deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.025)
                ready, age = self.safety_is_fresh_and_authorized(0.05)
                if not ready:
                    self.exercise["status"] = "STOPPED_SAFETY_STATE"
                    self.exercise["stop_reason"] = "Safety state became invalid, stale, or unauthorized."
                    break
                self.publish_speed(0.05)
                sent_nonzero = True
        finally:
            self.publish_speed(0.0)
            self.exercise["zero_request_sent"] = True
            settle_until = time.monotonic() + 0.65
            while time.monotonic() < settle_until and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.03)

        self.exercise["nonzero_request_published"] = sent_nonzero
        self.exercise["safety_state_at_end"] = self.safety
        self.exercise["end_odometry"] = self.odom
        if self.initial_odom is not None and self.odom is not None:
            dx = self.odom["x_m"] - self.initial_odom["x_m"]
            dy = self.odom["y_m"] - self.initial_odom["y_m"]
            self.exercise["odometry_displacement_m"] = round(math.hypot(dx, dy), 5)
        self.exercise["max_guard_output_linear_x_mps"] = round(
            max((abs(v["linear_x_mps"]) for v in self.guard_outputs), default=0.0), 5)
        self.exercise["guard_output_samples"] = len(self.guard_outputs)
        if (self.exercise.get("status") == "REQUESTING_THROUGH_GUARD"
                and self.exercise["max_guard_output_linear_x_mps"] > 0.0):
            self.exercise["status"] = "GUARDED_OUTPUT_OBSERVED"
            self.exercise["detail"] = (
                "Nonzero /cmd_vel output was observed from the existing command guard; "
                "see odometry and saved status for the measured simulator response."
            )
        elif self.exercise.get("status") == "REQUESTING_THROUGH_GUARD":
            self.exercise["status"] = "NO_GUARD_OUTPUT_OBSERVED"
            self.exercise["detail"] = "The safety guard did not pass a nonzero output."
        self.publish_status()
        print("Guarded movement result: " + self.exercise["status"], flush=True)
        print("Requested command: /cmd_vel_nav; authoritative output: /cmd_vel; angular request: 0 rad/s.", flush=True)
        print(f"Saved status: {self.output_path}", flush=True)

    def snapshot(self):
        now = time.monotonic()
        scan_rate = None
        if len(self.scan_wall_times) >= 2:
            span = self.scan_wall_times[-1] - self.scan_wall_times[0]
            if span > 0:
                scan_rate = round((len(self.scan_wall_times) - 1) / span, 2)
        safety_age = None if self.safety_wall is None else round(now - self.safety_wall, 3)
        odometry_displacement = None
        if self.initial_odom is not None and self.odom is not None:
            odometry_displacement = round(math.hypot(
                self.odom["x_m"] - self.initial_odom["x_m"],
                self.odom["y_m"] - self.initial_odom["y_m"]), 5)
        return {
            "profile": "REALISTIC_RGB_LIDAR",
            "scenario": self.scenario,
            "timestamp_unix_s": time.time(),
            "elapsed_s": round(now - self.started_wall, 2),
            "nav2_goal_navigation": "DISABLED_BY_LAUNCHER; readiness flags unchanged",
            "sensor_counts": {
                "/realistic/scan": self.scan_count,
                "/camera/image_raw": self.image_count,
                "/map": self.slam_map_count,
                "/realistic/local_map/lidar": self.local_map_count,
            },
            "scan_rate_hz_observed": scan_rate,
            "safety_state_age_s": safety_age,
            "safety_state": self.safety,
            "requested_command": self.requested,
            "guard_output_command": self.guard_output,
            "odometry": self.odom,
            "odometry_displacement_since_monitor_start_m": odometry_displacement,
            "guarded_forward_demo": self.exercise,
            "slam_map": self.slam_map_summary,
            "lidar_local_map": self.local_map_summary,
        }

    def publish_status(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.output_path.with_suffix(self.output_path.suffix + ".tmp")
        temp.write_text(json.dumps(self.snapshot(), indent=2, sort_keys=True) + "\n",
                        encoding="utf-8")
        temp.replace(self.output_path)

    def print_status(self):
        snapshot = self.snapshot()
        safety = snapshot["safety_state"] or {}
        reason = safety.get("reason", "waiting")
        failures = ",".join(str(v) for v in safety.get("failures", []))
        caps = f"{safety.get('max_linear_speed_mps', 0.0)} m/s / {safety.get('max_angular_speed_radps', 0.0)} rad/s"
        scan_rate = snapshot["scan_rate_hz_observed"]
        print(
            f"[{self.scenario}] RGB+LiDAR | valid={safety.get('valid', False)} "
            f"authorized={safety.get('authorized', False)} reason={reason} "
            f"failures={failures or '-'} cap={caps} "
            f"scan={self.scan_count} ({scan_rate if scan_rate is not None else 'n/a'} Hz) "
            f"images={self.image_count} map={self.slam_map_count} "
            f"local_map={self.local_map_count} request={self.requested} "
            f"guarded_output={self.guard_output}",
            flush=True,
        )
        self.publish_status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True,
                        choices=("S00", "S01", "S02", "S03", "S04", "S09"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--request-forward", action="store_true",
                        help="Attempt the bounded, straight S00 request through /cmd_vel_nav if the fresh safety state authorizes it.")
    parser.add_argument("--request-duration-s", type=float, default=0.75)
    args = parser.parse_args()
    if args.request_forward and args.scenario != "S00":
        parser.error("--request-forward is permitted only in S00")
    if not 0.1 <= args.request_duration_s <= 0.75:
        parser.error("--request-duration-s must be 0.1..0.75 seconds")

    rclpy.init()
    node = GuiDemoMonitor(args.scenario, args.output)
    try:
        print("REALISTIC RGB + LiDAR — CURRENT VERIFIED CAPABILITIES", flush=True)
        print("LiDAR sensing and guarded straight motion are limited; obstacle avoidance and Nav2 goal navigation are not validated.", flush=True)
        if args.request_forward:
            node.exercise_guarded_forward(args.request_duration_s)
        else:
            node.exercise = {
                "requested": False, "status": "SENSING_ONLY",
                "detail": "No velocity request was issued in this scenario.",
            }
            print("Sensing-only scenario: no velocity request will be published.", flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() - node.last_print_wall >= 2.0:
                node.last_print_wall = time.monotonic()
                node.print_status()
    except KeyboardInterrupt:
        print("\nStopping the monitor and issuing a zero request.", flush=True)
        node.publish_speed(0.0)
        for _ in range(5):
            if not rclpy.ok():
                break
            rclpy.spin_once(node, timeout_sec=0.03)
    finally:
        node.publish_status()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
