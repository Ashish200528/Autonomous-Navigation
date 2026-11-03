#!/usr/bin/env python3
"""Show and measure a bounded S00 forward request through the LiDAR guard."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from phase8c_demo_policy import (REQUEST_SPEED_MPS, TARGET_TRAVEL_M,
    forward_block_reason, process_identity_is_live, stream_problem)


class DemoRuntimeFailure(RuntimeError):
    pass


class GuiDemoMonitor(Node):
    def __init__(self, scenario, output_path, launch_pid=None, startup_timeout_s=30.0,
                 monitor_duration_s=90.0):
        super().__init__("anc_windows_gui_demo_monitor")
        self.scenario = scenario
        self.output_path = Path(output_path)
        self.launch_pid = launch_pid
        self.launch_ticks = None
        if launch_pid is not None:
            try:
                stat = Path(f"/proc/{launch_pid}/stat").read_text()
                self.launch_ticks = stat[stat.rindex(")") + 2:].split()[19]
            except (OSError, ValueError, IndexError):
                pass
        self.startup_timeout_s = startup_timeout_s
        self.started_wall = time.monotonic()
        self.deadline_wall = self.started_wall + monitor_duration_s
        self.last_print_wall = 0.0
        self.runtime_status = "STARTING"
        self.runtime_error = None
        self.safety = self.safety_wall = None
        self.scan_count = self.image_count = 0
        self.slam_map_count = self.local_map_count = 0
        self.scan_wall_times = []
        self.scan_wall = self.image_wall = None
        self.slam_map_summary = self.local_map_summary = None
        self.slam_summary_wall = self.local_summary_wall = None
        self.odom = self.initial_odom = None
        self.requested = self.guard_output = self.guard_output_wall = None
        self.guard_outputs = []
        self.exercise = {"requested": False, "status": "NOT_REQUESTED"}
        self.request_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(String, "/realistic/safety_state", self.on_safety, 10)
        self.create_subscription(LaserScan, "/realistic/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_slam_map, map_qos)
        self.create_subscription(OccupancyGrid, "/realistic/local_map/lidar", self.on_local_map, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_request, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_guard_output, 10)

    def on_safety(self, msg):
        try:
            value = json.loads(msg.data)
            self.safety = value if isinstance(value, dict) else {}
        except (TypeError, json.JSONDecodeError):
            self.safety = {"valid": False, "reason": "safety_state_json_invalid"}
        self.safety_wall = time.monotonic()

    def on_scan(self, _msg):
        self.scan_wall = time.monotonic()
        self.scan_count += 1
        self.scan_wall_times.append(self.scan_wall)
        self.scan_wall_times = self.scan_wall_times[-12:]

    def on_image(self, _msg):
        self.image_count += 1
        self.image_wall = time.monotonic()

    @staticmethod
    def map_counts(msg):
        data = list(msg.data)
        return {"frame_id": msg.header.frame_id, "width": int(msg.info.width),
                "height": int(msg.info.height), "free": data.count(0),
                "occupied": data.count(100), "unknown": data.count(-1)}

    def on_slam_map(self, msg):
        self.slam_map_count += 1
        now = time.monotonic()
        if self.slam_summary_wall is None or now - self.slam_summary_wall >= 1.0:
            self.slam_map_summary = self.map_counts(msg)
            self.slam_summary_wall = now

    def on_local_map(self, msg):
        self.local_map_count += 1
        now = time.monotonic()
        if self.local_summary_wall is None or now - self.local_summary_wall >= 1.0:
            self.local_map_summary = self.map_counts(msg)
            self.local_summary_wall = now

    def on_odom(self, msg):
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = {"x_m": float(pos.x), "y_m": float(pos.y),
                     "yaw_rad": math.atan2(2 * (q.w*q.z + q.x*q.y),
                                           1 - 2 * (q.y*q.y + q.z*q.z)),
                     "linear_x_mps": float(msg.twist.twist.linear.x),
                     "angular_z_radps": float(msg.twist.twist.angular.z),
                     "wall_s": time.monotonic()}
        if self.initial_odom is None:
            self.initial_odom = dict(self.odom)

    @staticmethod
    def twist_record(msg):
        return {"linear_x_mps": float(msg.linear.x), "angular_z_radps": float(msg.angular.z)}

    def on_request(self, msg):
        self.requested = self.twist_record(msg)

    def on_guard_output(self, msg):
        self.guard_output = self.twist_record(msg)
        self.guard_output_wall = time.monotonic()
        if abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6:
            self.guard_outputs.append({"wall_s": self.guard_output_wall, **self.guard_output})
            self.guard_outputs = self.guard_outputs[-200:]

    def ages(self):
        now = time.monotonic()
        return {"safety": None if self.safety_wall is None else now-self.safety_wall,
                "scan": None if self.scan_wall is None else now-self.scan_wall,
                "odometry": None if self.odom is None else now-self.odom["wall_s"]}

    def runtime_problem(self):
        if self.launch_pid is not None:
            try:
                stat = Path(f"/proc/{self.launch_pid}/stat").read_text()
            except OSError:
                return "ros_launch_exited"
            if not process_identity_is_live(stat, self.launch_ticks):
                return "ros_launch_exited"
        return stream_problem(time.monotonic()-self.started_wall, self.ages(),
                              self.startup_timeout_s)

    def check_runtime(self):
        if time.monotonic() >= self.deadline_wall:
            raise DemoRuntimeFailure("demo_duration_expired_before_completion")
        problem = self.runtime_problem()
        if problem:
            raise DemoRuntimeFailure(problem)

    def block_reason(self, travelled_m=0.0):
        ages = self.ages()
        return forward_block_reason(self.safety, ages["safety"], ages["odometry"],
                                    ages["scan"], travelled_m)

    def publish_speed(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        self.request_pub.publish(msg)

    def progress(self):
        if time.monotonic()-self.last_print_wall >= 2.0:
            self.last_print_wall = time.monotonic()
            self.print_status()

    def exercise_guarded_forward(self, duration_s):
        if self.scenario != "S00":
            self.exercise = {"requested": False, "status": "BLOCKED_SCENARIO"}
            return 4
        self.exercise = {"requested": False, "status": "WAITING_FOR_LIVE_SAFETY"}
        print("Waiting for fresh LiDAR, odometry and stable safety authorization...", flush=True)
        ready_since = None
        deadline = time.monotonic()+self.startup_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            self.check_runtime()
            reason = self.block_reason()
            self.exercise["block_reason"] = reason
            if reason is None:
                ready_since = ready_since or time.monotonic()
                if time.monotonic()-ready_since >= 0.5:
                    break
            else:
                ready_since = None
            self.progress()
        else:
            self.exercise.update(status="BLOCKED_SAFETY_GATE", block_reason=self.block_reason())
            self.print_status()
            return 4
        for remaining in (3, 2, 1):
            print(f"S00 guarded forward demonstration starts in {remaining}...", flush=True)
            until = time.monotonic()+1.0
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=0.03)
                self.check_runtime()
                reason = self.block_reason()
                if reason:
                    self.exercise.update(status="BLOCKED_SAFETY_GATE", block_reason=reason)
                    self.print_status()
                    return 4
        start_odom = dict(self.odom)
        start_wall = time.monotonic()
        self.exercise = {"requested": True, "status": "REQUESTING_THROUGH_GUARD",
                         "requested_speed_mps": REQUEST_SPEED_MPS,
                         "duration_limit_s": duration_s, "target_distance_m": TARGET_TRAVEL_M,
                         "hard_distance_limit_m": 0.20, "start_odometry": start_odom,
                         "safety_state_at_start": self.safety}
        print("Moving straight at 0.05 m/s, up to 0.15 m or 4 seconds; live LiDAR safety remains in control.", flush=True)
        stop_reason = "duration_limit"
        try:
            while rclpy.ok() and time.monotonic()-start_wall < duration_s:
                rclpy.spin_once(self, timeout_sec=0.025)
                self.check_runtime()
                distance = math.hypot(self.odom["x_m"]-start_odom["x_m"],
                                      self.odom["y_m"]-start_odom["y_m"])
                reason = self.block_reason(distance)
                if reason:
                    self.exercise.update(status="STOPPED_SAFETY_STATE", block_reason=reason)
                    stop_reason = reason
                    break
                if distance >= TARGET_TRAVEL_M:
                    stop_reason = "target_distance"
                    break
                self.publish_speed(REQUEST_SPEED_MPS)
                self.progress()
        finally:
            settle_until = time.monotonic()+1.5
            zero_sent = False
            while rclpy.ok() and time.monotonic() < settle_until:
                self.publish_speed(0.0)
                zero_sent = True
                rclpy.spin_once(self, timeout_sec=0.03)
            self.exercise["zero_request_sent"] = zero_sent
        distance = math.hypot(self.odom["x_m"]-start_odom["x_m"],
                              self.odom["y_m"]-start_odom["y_m"])
        outputs = [v for v in self.guard_outputs if v["wall_s"] >= start_wall]
        maximum = max((abs(v["linear_x_mps"]) for v in outputs), default=0.0)
        now = time.monotonic()
        stop_confirmed = (self.odom is not None and now-self.odom["wall_s"] <= 0.5
                          and abs(self.odom["linear_x_mps"]) <= 0.005
                          and abs(self.odom["angular_z_radps"]) <= 0.01
                          and self.guard_output_wall is not None
                          and now-self.guard_output_wall <= 0.35
                          and abs(self.guard_output["linear_x_mps"]) <= 1e-6
                          and abs(self.guard_output["angular_z_radps"]) <= 1e-6)
        self.exercise.update(stop_reason=stop_reason, end_odometry=self.odom,
                             odometry_displacement_m=round(distance, 5),
                             max_guard_output_linear_x_mps=maximum,
                             guard_output_samples=len(outputs), stop_confirmed=stop_confirmed)
        if self.exercise["status"] == "STOPPED_SAFETY_STATE":
            code = 4
        elif maximum > 0.005 and 0.02 <= distance < 0.20 and stop_confirmed:
            self.exercise["status"] = "MOVEMENT_VERIFIED"
            code = 0
        else:
            self.exercise["status"] = "MOVEMENT_NOT_VERIFIED"
            code = 5
        self.print_status()
        print(f"Movement result: {self.exercise['status']}; measured {distance:.3f} m; stop confirmed={stop_confirmed}", flush=True)
        return code

    def snapshot(self):
        now = time.monotonic()
        ages = self.ages()
        effective_safety = dict(self.safety or {})
        if ages["safety"] is None or ages["safety"] > 0.35:
            effective_safety.update(valid=False, authorized=False, reason="monitor_safety_missing_or_stale")
        scan_rate = None
        if ages["scan"] is not None and ages["scan"] <= 0.6 and len(self.scan_wall_times) >= 2:
            span = self.scan_wall_times[-1]-self.scan_wall_times[0]
            if span > 0:
                scan_rate = round((len(self.scan_wall_times)-1)/span, 2)
        displacement = None
        if self.initial_odom and self.odom:
            displacement = round(math.hypot(self.odom["x_m"]-self.initial_odom["x_m"],
                                             self.odom["y_m"]-self.initial_odom["y_m"]), 5)
        return {"profile": "REALISTIC_RGB_LIDAR", "scenario": self.scenario,
                "timestamp_unix_s": time.time(), "elapsed_s": round(now-self.started_wall, 2),
                "runtime_status": self.runtime_status, "runtime_error": self.runtime_error,
                "nav2_goal_navigation": "DISABLED_BY_LAUNCHER; readiness flags unchanged",
                "sensor_counts": {"/realistic/scan": self.scan_count,
                                  "/camera/image_raw": self.image_count, "/map": self.slam_map_count,
                                  "/realistic/local_map/lidar": self.local_map_count},
                "stream_ages_s": ages, "scan_rate_hz_observed": scan_rate,
                "safety_state_age_s": ages["safety"], "safety_state": effective_safety,
                "last_received_safety_state": self.safety,
                "requested_command": self.requested, "guard_output_command": self.guard_output,
                "odometry": self.odom, "odometry_displacement_since_monitor_start_m": displacement,
                "guarded_forward_demo": self.exercise, "slam_map": self.slam_map_summary,
                "lidar_local_map": self.local_map_summary}

    def publish_status(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.output_path.with_suffix(self.output_path.suffix+".tmp")
        temp.write_text(json.dumps(self.snapshot(), indent=2, sort_keys=True)+"\n", encoding="utf-8")
        temp.replace(self.output_path)

    def print_status(self):
        data = self.snapshot()
        safety = data["safety_state"]
        print(f"[{self.scenario}] {self.runtime_status} | motion={self.exercise['status']} "
              f"| safety={safety.get('reason', 'waiting')} valid={safety.get('valid', False)} "
              f"| scans={self.scan_count} ({data['scan_rate_hz_observed']} Hz) "
              f"| images={self.image_count} | measured travel={data['odometry_displacement_since_monitor_start_m']} m "
              f"| block={self.exercise.get('block_reason') or '-'}", flush=True)
        self.publish_status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, choices=("S00", "S01", "S02", "S03", "S04", "S09"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--request-forward", action="store_true")
    parser.add_argument("--request-duration-s", type=float, default=4.0)
    parser.add_argument("--startup-timeout-s", type=float, default=30.0)
    parser.add_argument("--monitor-duration-s", type=float, default=90.0)
    parser.add_argument("--launch-pid", type=int)
    args = parser.parse_args()
    if args.request_forward and args.scenario != "S00":
        parser.error("--request-forward is permitted only in S00")
    for name, value, lower, upper in (("request-duration-s", args.request_duration_s, 0.1, 4.0),
                                      ("startup-timeout-s", args.startup_timeout_s, 1.0, 60.0),
                                      ("monitor-duration-s", args.monitor_duration_s, 1.0, 180.0)):
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f"--{name} must be {lower}..{upper} seconds")
    rclpy.init()
    node = GuiDemoMonitor(args.scenario, args.output, args.launch_pid,
                          args.startup_timeout_s, args.monitor_duration_s)
    result = 0
    try:
        print("RGB + LiDAR demonstration: live sensing and bounded guarded straight motion.", flush=True)
        print("Obstacle avoidance/Nav2 goal navigation requires further readiness evidence.", flush=True)
        if args.request_forward:
            result = node.exercise_guarded_forward(args.request_duration_s)
        else:
            node.exercise = {"requested": False, "status": "SENSING_ONLY"}
            print("Stationary sensor scenario: no movement request is configured.", flush=True)
        if result == 0:
            node.runtime_status = "MONITORING"
            while rclpy.ok() and time.monotonic() < node.deadline_wall:
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() >= node.deadline_wall:
                    break
                node.check_runtime()
                node.progress()
            if any(age is None for age in node.ages().values()):
                node.runtime_status = "BLOCKED"
                node.runtime_error = "demo_duration_ended_before_sensor_readiness"
                result = 4
            else:
                node.runtime_status = "COMPLETED"
        else:
            node.runtime_status = "BLOCKED" if result == 4 else "FAILED"
    except (KeyboardInterrupt, ExternalShutdownException):
        node.runtime_status = "STOPPED_BY_USER"
        result = 130
    except Exception as exc:
        node.runtime_status = "FAILED"
        node.runtime_error = str(exc)
        if node.exercise.get("status") in {"WAITING_FOR_LIVE_SAFETY", "REQUESTING_THROUGH_GUARD"}:
            node.exercise["status"] = "RUNTIME_FAILED"
        print("LiDAR demo stopped: " + str(exc), flush=True)
        result = 6
    finally:
        for _ in range(5):
            try:
                node.publish_speed(0.0)
                if node.exercise.get("requested"):
                    node.exercise["zero_request_sent"] = True
                if rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.02)
            except Exception:
                break
        node.print_status()
        print(f"Saved LiDAR result: {node.output_path}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
