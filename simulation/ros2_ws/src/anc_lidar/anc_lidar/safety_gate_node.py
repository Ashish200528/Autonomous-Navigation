"""Fail-closed speed gate for the REALISTIC_RGB_LIDAR profile."""
import json
import math
import time
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from anc_navigation_common.contracts import GridGeometry, validate_grid
from anc_navigation_common.safety import speed_gate
from anc_lidar.scan_model import (Grid, latest_matching_map_health_pair,
                                  swept_rotation_clearance)


def stamp_seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)/1_000_000_000.0


class LidarSafetyGate(Node):
    def __init__(self):
        super().__init__("anc_lidar_safety_gate")
        defaults = {
            "scan_health_topic": "/realistic/scan/health",
            "lidar_map_topic": "/realistic/local_map/lidar",
            "lidar_map_health_topic": "/realistic/local_map/lidar/health",
            "rgb_map_topic": "/realistic/local_traversability",
            "rgb_state_topic": "/realistic/navigation_state",
            "odometry_topic": "/odometry/filtered",
            "clock_topic": "/clock",
            "safety_state_topic": "/realistic/safety_state",
            "base_frame": "base_link",
            "max_scan_health_wall_age_s": 0.20,
            "max_map_wall_age_s": 0.22,
            "max_rgb_wall_age_s": 0.35,
            "max_odom_wall_age_s": 0.20,
            "max_clock_wall_age_s": 0.20,
            "max_scan_sim_age_s": 0.22,
            "max_odom_sim_age_s": 0.25,
            "minimum_scan_rate_hz": 6.0,
            "minimum_usable_beam_fraction": 0.90,
            "maximum_odom_xy_sigma_m": 0.50,
            "maximum_odom_yaw_sigma_rad": 0.50,
            "max_linear_speed_mps": 0.18,
            "localization_allowance_m": 0.4705050313257851,
            "explicit_safety_margin_m": 0.10,
            "measured_stop_distance_m": 0.00298,
            "camera_age_bound_s": 0.174,
            "perception_latency_bound_s": 0.2368,
            "supervisor_period_s": 0.05,
            "publish_rate_hz": 20.0,
            "rotation_calibration_max_radps": 0.0,
            "turn_footprint_vertices_xy_m": [0.215, 0.1525, 0.215, -0.1525,
                                               -0.16, -0.1525, -0.16, 0.1525],
            "turn_center_drift_allowance_m": 0.14,
            "turn_sensor_mount_uncertainty_m": 0.02,
            "turn_safety_margin_m": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.scan_health = self.map_health = self.rgb_state = None
        self.scan_health_wall = self.map_health_wall = self.rgb_state_wall = None
        self.map_msg = self.rgb_map = self.odom = None
        self.map_wall = self.rgb_map_wall = self.odom_wall = None
        self.lidar_map_records = deque(maxlen=8)
        self.lidar_health_records = deque(maxlen=8)
        self.clock_s = self.clock_wall = None
        self.pub = self.create_publisher(String, self.p["safety_state_topic"], 10)
        self.create_subscription(String, self.p["scan_health_topic"],
                                 self.on_scan_health, 10)
        self.create_subscription(OccupancyGrid, self.p["lidar_map_topic"],
                                 self.on_map, 10)
        self.create_subscription(String, self.p["lidar_map_health_topic"],
                                 self.on_map_health, 10)
        self.create_subscription(OccupancyGrid, self.p["rgb_map_topic"],
                                 self.on_rgb_map, 10)
        self.create_subscription(String, self.p["rgb_state_topic"],
                                 self.on_rgb_state, 10)
        self.create_subscription(Odometry, self.p["odometry_topic"],
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Clock, self.p["clock_topic"],
                                 self.on_clock, qos_profile_sensor_data)
        self.create_timer(1.0/float(self.p["publish_rate_hz"]), self.tick, clock=RclClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            "Forward motion uses the measured linear gate. Rotation stays disabled by default; "
            "an explicit calibration cap allows rotation only inside a fully known-free swept disk."
        )

    def on_scan_health(self, msg):
        try:
            self.scan_health = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.scan_health = {"valid": False, "reason": "scan_health_json_invalid"}
        self.scan_health_wall = time.monotonic()

    def on_map(self, msg):
        self.lidar_map_records.append({
            "stamp_s": stamp_seconds(msg.header.stamp),
            "wall_s": time.monotonic(), "payload": msg,
        })
        self._synchronize_lidar_map_health()

    def on_map_health(self, msg):
        wall = time.monotonic()
        try:
            value = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.map_health = {"valid": False, "reason": "map_health_json_invalid"}
            self.map_health_wall = wall
            return
        try:
            scan_stamp = float(value["scan_stamp_s"])
            if not math.isfinite(scan_stamp):
                raise ValueError("map_health_scan_stamp_nonfinite")
        except (KeyError, TypeError, ValueError):
            self.map_health = {"valid": False,
                               "reason": "map_health_scan_stamp_missing_or_invalid"}
            self.map_health_wall = wall
            return
        if value.get("valid") is not True:
            # Any explicit invalid report fails closed while its matching grid
            # catches up on the separate OccupancyGrid topic.
            self.map_health = value
            self.map_health_wall = wall
        self.lidar_health_records.append({
            "stamp_s": scan_stamp, "wall_s": wall, "payload": value,
        })
        self._synchronize_lidar_map_health()

    def _synchronize_lidar_map_health(self):
        pair = latest_matching_map_health_pair(
            self.lidar_map_records, self.lidar_health_records, 0.001)
        if pair is None:
            return
        map_record, health_record = pair
        # A newer explicitly invalid, unmatched status remains authoritative
        # until its matching grid arrives.
        latest_health = (self.lidar_health_records[-1]
                         if self.lidar_health_records else None)
        if (latest_health is not None
                and latest_health["payload"].get("valid") is not True
                and abs(latest_health["stamp_s"]-map_record["stamp_s"]) > 0.001):
            return
        self.map_msg = map_record["payload"]
        self.map_wall = map_record["wall_s"]
        self.map_health = health_record["payload"]
        self.map_health_wall = health_record["wall_s"]

    def on_rgb_map(self, msg):
        self.rgb_map = msg
        self.rgb_map_wall = time.monotonic()

    def on_rgb_state(self, msg):
        try:
            self.rgb_state = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.rgb_state = {"input_failures": ["rgb_state_json_invalid"]}
        self.rgb_state_wall = time.monotonic()

    def on_odom(self, msg):
        self.odom = msg
        self.odom_wall = time.monotonic()

    def on_clock(self, msg):
        self.clock_s = stamp_seconds(msg.clock)
        self.clock_wall = time.monotonic()

    def tick(self):
        now = time.monotonic()
        p = self.p
        failures = []
        try:
            rotation_calibration_cap = float(p["rotation_calibration_max_radps"])
        except (TypeError, ValueError):
            rotation_calibration_cap = 0.0
            failures.append("rotation_calibration_limit_invalid")
        if not math.isfinite(rotation_calibration_cap) or rotation_calibration_cap < 0:
            rotation_calibration_cap = 0.0
            failures.append("rotation_calibration_limit_invalid")
        elif rotation_calibration_cap > 0.10:
            rotation_calibration_cap = 0.0
            failures.append("rotation_calibration_limit_exceeds_0.10_radps")
        scalar_inputs = {}
        if self.scan_health is None:
            failures.append("scan_health_missing")
        elif now-self.scan_health_wall > float(p["max_scan_health_wall_age_s"]):
            failures.append("scan_health_stale")
        elif self.scan_health.get("valid") is not True:
            failures.append("scan_invalid:" + str(self.scan_health.get("reason", "unknown")))
        if self.clock_s is None or now-self.clock_wall > float(p["max_clock_wall_age_s"]):
            failures.append("simulation_clock_missing_or_stale")
        if self.scan_health and self.clock_s is not None:
            stamp = self.scan_health.get("scan_stamp_s")
            try:
                scan_age = self.clock_s-float(stamp)
                scalar_inputs["scan_sim_age_s"] = scan_age
                if scan_age < -0.05 or scan_age > float(p["max_scan_sim_age_s"]):
                    failures.append("scan_sim_timestamp_stale_or_future")
            except (TypeError, ValueError):
                scan_age = None
                failures.append("scan_timestamp_invalid")
            rate = self.scan_health.get("measured_rate_hz")
            counts = self.scan_health.get("counts", {})
            usable = counts.get("usable_beams", 0)
            total = max(counts.get("total_beams", 0), 1)
            scalar_inputs["measured_scan_rate_hz"] = rate
            scalar_inputs["usable_beam_fraction"] = usable/total
            if rate is None or float(rate) < float(p["minimum_scan_rate_hz"]):
                failures.append("measured_scan_rate_below_minimum")
            if usable/total < float(p["minimum_usable_beam_fraction"]):
                failures.append("usable_beam_fraction_below_minimum")

        if self.map_msg is None or self.map_wall is None:
            failures.append("lidar_local_map_missing")
        elif now-self.map_wall > float(p["max_map_wall_age_s"]):
            failures.append("lidar_local_map_stale")
        if self.map_health is None or self.map_health_wall is None:
            failures.append("lidar_map_health_missing")
        elif now-self.map_health_wall > float(p["max_map_wall_age_s"]):
            failures.append("lidar_map_health_stale")
        elif self.map_health.get("valid") is not True:
            failures.append("lidar_map_invalid:" + str(self.map_health.get("reason", "unknown")))
        if self.map_msg is not None and self.map_health is not None:
            try:
                geometry = GridGeometry(
                    int(self.map_msg.info.width), int(self.map_msg.info.height),
                    float(self.map_msg.info.resolution),
                    float(self.map_msg.info.origin.position.x),
                    float(self.map_msg.info.origin.position.y),
                    self.map_msg.header.frame_id,
                )
                data = validate_grid(self.map_msg.data, geometry)
                if geometry.frame_id.lstrip("/") != str(p["base_frame"]).lstrip("/"):
                    failures.append("lidar_grid_frame_mismatch")
                q = self.map_msg.info.origin.orientation
                if abs(q.x)+abs(q.y)+abs(q.z) > 1e-4:
                    failures.append("lidar_grid_origin_rotation_unsupported")
                counts = self.map_health.get("counts", {})
                total = len(data)
                if counts and sum(int(counts.get(k, -1)) for k in
                                  ("free", "occupied", "unknown")) != total:
                    failures.append("lidar_map_count_mismatch")
                if self.map_health.get("scan_stamp_s") is not None:
                    delta = abs(stamp_seconds(self.map_msg.header.stamp)
                                - float(self.map_health["scan_stamp_s"]))
                    if delta > 1e-3:
                        failures.append("lidar_map_stamp_mismatch")
                grid = geometry
            except (TypeError, ValueError, OverflowError) as exc:
                failures.append("lidar_grid_invalid:" + str(exc))
                data = []
                grid = None
        else:
            data = []
            grid = None

        if self.rgb_map is None or self.rgb_map_wall is None:
            failures.append("rgb_local_map_missing")
        elif now-self.rgb_map_wall > float(p["max_rgb_wall_age_s"]):
            failures.append("rgb_local_map_stale")
        if self.rgb_state is None or self.rgb_state_wall is None:
            failures.append("rgb_perception_health_missing")
        elif now-self.rgb_state_wall > float(p["max_rgb_wall_age_s"]):
            failures.append("rgb_perception_health_stale")
        else:
            input_failures = self.rgb_state.get("input_failures", ["missing_input_failures"])
            if input_failures:
                failures.extend("rgb_input:" + str(item) for item in input_failures)
            if self.rgb_map is not None and self.rgb_state.get("map_frame") not in (
                    None, self.rgb_map.header.frame_id):
                failures.append("rgb_local_map_frame_mismatch")

        if self.odom is None or self.odom_wall is None:
            failures.append("filtered_localization_missing")
        elif now-self.odom_wall > float(p["max_odom_wall_age_s"]):
            failures.append("filtered_localization_stale")
        else:
            odom_age = None if self.clock_s is None else (
                self.clock_s-stamp_seconds(self.odom.header.stamp))
            scalar_inputs["odom_sim_age_s"] = odom_age
            if (self.odom.header.frame_id.lstrip("/") != "odom"
                    or self.odom.child_frame_id.lstrip("/") not in
                    ("base_footprint", "base_link")):
                failures.append("filtered_localization_frame_invalid")
            if odom_age is None or odom_age < -0.05 or odom_age > float(p["max_odom_sim_age_s"]):
                failures.append("filtered_localization_timestamp_stale_or_future")
            covariance = self.odom.pose.covariance
            xy_vars = (float(covariance[0]), float(covariance[7]))
            yaw_var = float(covariance[35])
            if (not all(math.isfinite(v) and v >= 0 for v in (*xy_vars, yaw_var))
                    or max(math.sqrt(v) for v in xy_vars) >
                    float(p["maximum_odom_xy_sigma_m"])
                    or math.sqrt(yaw_var) > float(p["maximum_odom_yaw_sigma_rad"])):
                failures.append("filtered_localization_uncertainty")

        turn_clearance = {
            "authorized": False,
            "reason": "required_inputs_invalid",
            "checked_cells": 0,
            "occupied_cells": 0,
            "unknown_cells": 0,
        }
        if grid is not None:
            try:
                flat_vertices = list(p["turn_footprint_vertices_xy_m"])
                if len(flat_vertices) < 6 or len(flat_vertices) % 2:
                    raise ValueError("rotation_footprint_invalid")
                vertices = list(zip(flat_vertices[::2], flat_vertices[1::2]))
                turn_grid = Grid(
                    geometry.width, geometry.height, geometry.resolution_m,
                    geometry.origin_x_m, geometry.origin_y_m, geometry.frame_id)
                turn_clearance = swept_rotation_clearance(
                    data, turn_grid, vertices,
                    center_drift_allowance_m=float(p["turn_center_drift_allowance_m"]),
                    sensor_mount_uncertainty_m=float(p["turn_sensor_mount_uncertainty_m"]),
                    safety_margin_m=float(p["turn_safety_margin_m"]))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                turn_clearance = {
                    "authorized": False,
                    "reason": str(exc) or "rotation_clearance_config_invalid",
                    "checked_cells": 0,
                    "occupied_cells": 0,
                    "unknown_cells": 0,
                }

        corridor = 0.0
        if not failures and grid is not None:
            width = grid.width
            corridor = 0.0
            y0 = math.floor((-0.1725-grid.origin_y_m)/grid.resolution_m)
            y1 = math.floor((0.1725-grid.origin_y_m)/grid.resolution_m)
            x0 = math.floor((0.215-grid.origin_x_m)/grid.resolution_m)
            if y0 < 0 or y1 >= grid.height or x0 < 0:
                failures.append("footprint_corridor_outside_grid")
            else:
                cells = 0
                for ix in range(x0, grid.width):
                    if any(data[iy*width+ix] != 0 for iy in range(y0, y1+1)):
                        break
                    cells += 1
                corridor = cells*grid.resolution_m
        scalar_inputs["connected_full_footprint_corridor_m"] = corridor

        map_age_wall = (float("inf") if self.map_wall is None
                        else max(0.0, now-self.map_wall))
        scalar_inputs["lidar_map_wall_age_s"] = map_age_wall
        speed_result = None
        if not failures:
            scan_age = max(0.0, float(scalar_inputs.get("scan_sim_age_s", 0.0)))
            processing = float((self.map_health or {}).get("map_build_wall_s", 0.0))
            if not math.isfinite(processing) or processing < 0:
                failures.append("lidar_map_processing_latency_invalid")
                processing = 0.0
            processing = max(float(p["perception_latency_bound_s"]), processing)
            speed_result = speed_gate(
                corridor_m=corridor,
                age_s=max(scan_age, float(p["camera_age_bound_s"]), map_age_wall),
                processing_latency_s=processing,
                supervisor_period_s=float(p["supervisor_period_s"]),
                stop_distance_by_speed=[
                    (0.02, float(p["measured_stop_distance_m"])),
                    (0.05, float(p["measured_stop_distance_m"])),
                    (0.10, float(p["measured_stop_distance_m"])),
                    (0.18, float(p["measured_stop_distance_m"])),
                ],
                localization_allowance_m=float(p["localization_allowance_m"]),
                footprint_margin_m=0.0,
                safety_margin_m=float(p["explicit_safety_margin_m"]),
                max_speed_mps=float(p["max_linear_speed_mps"]),
            )
            if not speed_result.get("valid"):
                failures.append(str(speed_result.get("reason", "speed_gate_invalid")))
            elif (float(speed_result.get("max_linear_speed_mps", 0.0)) <= 0
                  and rotation_calibration_cap <= 0):
                failures.append(str(speed_result.get("reason", "no_safe_speed")))
        selected_speed = (
            0.0 if failures or speed_result is None or rotation_calibration_cap > 0
            else float(speed_result["max_linear_speed_mps"])
        )
        selected_angular = (
            rotation_calibration_cap
            if (not failures and rotation_calibration_cap > 0
                and turn_clearance.get("authorized") is True)
            else 0.0
        )
        gate_authorized = not failures and (selected_speed > 0 or selected_angular > 0)
        if failures:
            gate_reason = failures[0]
        elif rotation_calibration_cap > 0 and selected_angular <= 0:
            gate_reason = str(turn_clearance.get("reason", "rotation_clearance_denied"))
        else:
            gate_reason = "pass"
        result = {
            "node": "anc_lidar_safety_gate",
            "profile": "REALISTIC_RGB_LIDAR",
            "valid": not failures,
            "authorized": gate_authorized,
            "reason": gate_reason,
            "failures": failures,
            "motion_mode": ("rotation_calibration_only" if rotation_calibration_cap > 0
                            else "forward_only"),
            "combined_motion_authorized": False,
            "max_linear_speed_mps": selected_speed,
            "max_angular_speed_radps": selected_angular,
            "angular_motion_status": (
                "calibration_only_swept_disk_clear" if selected_angular > 0 else
                (str(turn_clearance.get("reason", "rotation_clearance_denied"))
                 if rotation_calibration_cap > 0 else
                 "disabled_unmeasured_yaw_stopping")),
            "rotation_calibration_speed_cap_radps": rotation_calibration_cap,
            "rotation_clearance": turn_clearance,
            "angular_stopping_model": {
                "measured": False,
                "production_authorized": False,
                "note": "Calibration-only capped pulses; production yaw remains disabled pending measured stopping bound and review.",
            },
            "corridor_m": corridor,
            "speed_gate": speed_result,
            "speed_gate_inputs": scalar_inputs,
            "stopping_model": {
                "source": "Phase 8B isolated simulator stopping test",
                "worst_stop_distance_m": float(p["measured_stop_distance_m"]),
                "tested_speeds_mps": [0.02, 0.05, 0.10, 0.18],
                "friction_conditions": [0.5, 1.2],
                "hardware_bound": False,
            },
            "localization_allowance_m": float(p["localization_allowance_m"]),
            "safety_margin_m": float(p["explicit_safety_margin_m"]),
            "footprint_width_m": 0.345,
            "forward_distance_reference": "front_footprint_boundary",
            "map_frame": None if self.map_msg is None else self.map_msg.header.frame_id,
            "map_counts": None if self.map_msg is None else {
                "free": sum(v == 0 for v in self.map_msg.data),
                "occupied": sum(v == 100 for v in self.map_msg.data),
                "unknown": sum(v == -1 for v in self.map_msg.data),
            },
            "runtime_uses_world_map_or_truth": False,
            "truth_used_only_by_evaluator": True,
        }
        msg = String()
        msg.data = json.dumps(result, sort_keys=True, separators=(",", ":"))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
