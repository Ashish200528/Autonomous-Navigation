#!/usr/bin/env python3
"""Headless Phase 2 motion, contact, and stability checks.

Gazebo entity state is used only as a test evaluator for physical motion and
wall clearance. This script is not a perception or navigation input.
"""
import csv
import json
import math
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contacts
from simulation_interfaces.srv import GetEntityState

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs" / "evidence"
RESULT_PATH = EVIDENCE / "phase2-motion-results.json"
CSV_PATH = EVIDENCE / "phase2-motion-telemetry.csv"
CONFIG_PATH = ROOT / "config" / "robot_parameters.yaml"
WORLD_PATH = (
    ROOT / "simulation" / "ros2_ws" / "src" / "anc_gazebo"
    / "worlds" / "phase2_test_world.sdf"
)


def expected_wall_stop_center_x_m():
    config_text = CONFIG_PATH.read_text(encoding="utf-8")
    def parameter(name):
        match = re.search(
            rf"(?m)^    {re.escape(name)}:\s*([-+]?[0-9]+(?:\.[0-9]+)?)\s*$",
            config_text,
        )
        if match is None:
            raise RuntimeError(f"Could not read {name} from robot parameters")
        return float(match.group(1))

    chassis_length = parameter("chassis_length_m")
    caster_front = parameter("caster_x_m") + parameter("caster_radius_m")
    wheel_front = parameter("wheel_x_m") + parameter("wheel_radius_m")
    robot_front_extent = max(chassis_length / 2.0, caster_front, wheel_front)

    world = ET.parse(WORLD_PATH).getroot()
    wall = world.find(".//model[@name='phase2_wall']")
    if wall is None:
        raise RuntimeError("Could not find phase2_wall in the test world")
    wall_pose = [float(value) for value in wall.findtext("pose").split()]
    wall_size_text = wall.findtext(
        "./link/collision/geometry/box/size"
    )
    wall_length_x = float(wall_size_text.split()[0])
    return wall_pose[0] - wall_length_x / 2.0 - robot_front_extent


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


def roll_pitch(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


class MotionTest(Node):
    def __init__(self):
        super().__init__(
            "anc_phase2_motion_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Contacts, "/contacts", self.on_contacts, 20)
        self.state_client = self.create_client(
            GetEntityState, "/gzserver/get_entity_state"
        )
        self.latest_odom = None
        self.odom_count = 0
        self.contact_events = []
        self.phases = []
        self.samples = []
        self.start_wall = time.monotonic()
        self.collision_truth = []
        self.contact_count_start_collision = 0

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist
        sample = {
            "elapsed_s": time.monotonic() - self.start_wall,
            "sim_time_s": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": p.x, "y": p.y, "z": p.z, "yaw_rad": yaw(q),
            "linear_x": v.linear.x, "angular_z": v.angular.z,
        }
        self.latest_odom = sample
        self.odom_count += 1
        self.samples.append({"topic": "odom", **sample})

    def on_contacts(self, msg):
        for contact in msg.contacts:
            event = {
                "elapsed_s": time.monotonic() - self.start_wall,
                "collision1": contact.collision1.name,
                "collision2": contact.collision2.name,
                "max_depth_m": max(contact.depths, default=0.0),
            }
            self.contact_events.append(event)
            self.samples.append({"topic": "contact", **event})

    def command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.samples.append({
            "topic": "command",
            "elapsed_s": time.monotonic() - self.start_wall,
            "linear_x": float(linear_x),
            "angular_z": float(angular_z),
        })

    def run_for(self, duration, linear_x=0.0, angular_z=0.0):
        deadline = time.monotonic() + duration
        last_truth_query = 0.0
        while time.monotonic() < deadline:
            self.command(linear_x, angular_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_phase == "wall_collision":
                now = time.monotonic()
                if now - last_truth_query >= 0.5:
                    truth = self.get_truth()
                    self.collision_truth.append(truth)
                    last_truth_query = now
        self.stop_for(0.6)

    def stop_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_inputs(self):
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and self.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_odom is None:
            raise RuntimeError("No /odom received within 20 seconds")
        if not self.state_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Missing /gzserver/get_entity_state evaluator service")
        deadline = time.monotonic() + 10.0
        while self.count_publishers("/contacts") == 0 and time.monotonic() < deadline:
            self.command()
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.count_publishers("/contacts") == 0:
            raise RuntimeError("No /contacts publisher from Gazebo bridge")
        self.stop_for(0.5)

    def get_truth(self):
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.state_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("GetEntityState timed out")
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError(
                "GetEntityState failed: "
                + str(response.result.error_message)
            )
        state = response.state
        p, q = state.pose.position, state.pose.orientation
        item = {
            "sim_time_s": state.header.stamp.sec + state.header.stamp.nanosec * 1e-9,
            "x": p.x, "y": p.y, "z": p.z, "yaw_rad": yaw(q),
            "roll_rad": roll_pitch(q)[0], "pitch_rad": roll_pitch(q)[1],
            "linear_x": state.twist.linear.x,
            "angular_z": state.twist.angular.z,
            "frame_id": state.header.frame_id,
        }
        self.samples.append({
            "topic": "gazebo_evaluator_state",
            "elapsed_s": time.monotonic() - self.start_wall,
            **item,
        })
        return item

    def phase(self, name, linear_x, angular_z, duration):
        self.current_phase = name
        start_truth = self.get_truth()
        start_odom = dict(self.latest_odom)
        start_contact_count = len(self.contact_events)
        if name == "wall_collision":
            self.contact_count_start_collision = start_contact_count
            self.collision_truth = [start_truth]
        self.run_for(duration, linear_x, angular_z)
        end_truth = self.get_truth()
        end_odom = dict(self.latest_odom)
        record = {
            "name": name,
            "command": {"linear_x_mps": linear_x, "angular_z_radps": angular_z},
            "duration_wall_s": duration,
            "start_truth": start_truth,
            "end_truth": end_truth,
            "truth_delta_x_m": end_truth["x"] - start_truth["x"],
            "truth_delta_y_m": end_truth["y"] - start_truth["y"],
            "truth_delta_yaw_rad": angle_delta(start_truth["yaw_rad"], end_truth["yaw_rad"]),
            "start_odom": start_odom,
            "end_odom": end_odom,
            "odom_delta_x_m": end_odom["x"] - start_odom["x"],
            "odom_delta_y_m": end_odom["y"] - start_odom["y"],
            "odom_delta_yaw_rad": angle_delta(start_odom["yaw_rad"], end_odom["yaw_rad"]),
            "contact_messages_since_start": len(self.contact_events) - start_contact_count,
        }
        if name == "wall_collision":
            record["contacts"] = self.contact_events[start_contact_count:]
            record["evaluator_truth_samples"] = self.collision_truth
            record["max_truth_x_during_collision_m"] = max(
                (p["x"] for p in self.collision_truth), default=end_truth["x"]
            )
        self.phases.append(record)
        return record


def main():
    rclpy.init()
    node = MotionTest()
    try:
        node.wait_for_inputs()
        start = node.get_truth()
        forward = node.phase("forward", 0.15, 0.0, 2.0)
        reverse = node.phase("reverse", -0.15, 0.0, 2.0)
        collision = node.phase("wall_collision", 0.20, 0.0, 7.0)
        backoff = node.phase("wall_backoff", -0.12, 0.0, 1.5)
        rotation = node.phase("in_place_rotation", 0.0, 0.60, 2.0)
        node.stop_for(0.8)
        final = node.get_truth()

        wall_contacts = [
            contact for contact in node.contact_events
            if "phase2_wall" in contact["collision1"]
            or "phase2_wall" in contact["collision2"]
        ]
        truth_points = [start, final]
        for phase_record in node.phases:
            truth_points.extend([phase_record["start_truth"], phase_record["end_truth"]])
        truth_points.extend(collision["evaluator_truth_samples"])
        max_abs_tilt = max(
            max(abs(point["roll_rad"]), abs(point["pitch_rad"]))
            for point in truth_points
        )
        expected_wall_x = expected_wall_stop_center_x_m()
        wall_stop_error = abs(collision["end_truth"]["x"] - expected_wall_x)
        checks = {
            "forward_motion_gt_0_10m": forward["truth_delta_x_m"] > 0.10,
            "reverse_motion_lt_minus_0_10m": reverse["truth_delta_x_m"] < -0.10,
            "wall_constraint_stopped_robot_at_expected_clearance": (
                wall_stop_error <= 0.07
                and abs(collision["end_truth"]["linear_x"]) < 0.03
            ),
            "no_tunneling_past_wall": (
                collision["max_truth_x_during_collision_m"] < expected_wall_x + 0.02
            ),
            "robot_remained_near_ground": (
                max(abs(point["z"]) for point in truth_points) < 0.15
            ),
            "absolute_roll_pitch_below_0_25rad": max_abs_tilt < 0.25,
            "in_place_yaw_change_gt_0_30rad": abs(rotation["truth_delta_yaw_rad"]) > 0.30,
            "odom_matches_forward_reverse_motion": (
                forward["odom_delta_x_m"] > 0.08 and reverse["odom_delta_x_m"] < -0.08
            ),
            "odom_rotation_observed": abs(rotation["odom_delta_yaw_rad"]) > 0.25,
            "odom_received": node.odom_count > 20,
        }
        result = {
            "classification": "PHASE_2_SIMULATION_MODEL_TEST",
            "truth_usage": "Gazebo entity state is used only by this physical-motion evaluator; not a perception or navigation input.",
            "parameter_status": "Design placeholders only; not measured hardware specifications.",
            "robot_entity": "anc_robot",
            "world": "anc_phase2_test",
            "expected_wall_stop_center_x_m": expected_wall_x,
            "wall_stop_error_m": wall_stop_error,
            "max_abs_roll_pitch_rad": max_abs_tilt,
            "contact_sensor_message_count": len(node.contact_events),
            "contact_sensor_status": (
                "WALL_EVENT_OBSERVED" if wall_contacts else "NO_WALL_EVENT_OBSERVED"
            ),
            "wall_stop_evidence": (
                "Evaluator pose held at the geometry-derived wall contact plane "
                "while forward velocity was near zero; evaluator data is test-only."
            ),
            "start_truth": start,
            "phases": node.phases,
            "wall_contact_event_count": len(wall_contacts),
            "all_contact_events": node.contact_events,
            "final_truth": final,
            "odom_message_count": node.odom_count,
            "checks": checks,
            "overall_pass": all(checks.values()),
            "telemetry_csv": "docs/evidence/phase2-motion-telemetry.csv",
        }
        EVIDENCE.mkdir(parents=True, exist_ok=True)
        RESULT_PATH.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        fields = [
            "topic", "elapsed_s", "sim_time_s", "x", "y", "z", "yaw_rad",
            "linear_x", "angular_z", "collision1", "collision2", "max_depth_m", "frame_id",
        ]
        with CSV_PATH.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(node.samples)
        print(json.dumps(result, indent=2))
        if not result["overall_pass"]:
            raise SystemExit(2)
    finally:
        node.stop_for(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
