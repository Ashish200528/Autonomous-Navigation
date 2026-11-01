#!/usr/bin/env python3
"""Repeatable headless Phase 3 sensor, odometry, TF, and wall-contact test.

Gazebo entity state is queried only by this evaluator. It is never published to
navigation or perception topics.
"""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from simulation_interfaces.srv import GetEntityState
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs" / "evidence"


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def angle_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rate_hz(stamps):
    if len(stamps) < 2:
        return 0.0
    duration = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / duration if duration > 0 else 0.0


def monotonic(stamps):
    return all(b > a for a, b in zip(stamps, stamps[1:]))


class SensorSmoke(Node):
    def __init__(self):
        super().__init__(
            "anc_phase3_sensor_smoke_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image_raw",
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data",
                                 self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(JointState, "/joint_states",
                                 self.on_joint_states, qos_profile_sensor_data)
        self.create_subscription(Contacts, "/contacts",
                                 self.on_contacts, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        static_tf_qos = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, "/tf_static", self.on_tf, static_tf_qos
        )
        self.state_client = self.create_client(
            GetEntityState, "/gzserver/get_entity_state"
        )
        self.image_messages = []
        self.image_candidates = []
        self.latest_image = None
        self.camera_info_messages = []
        self.imu_messages = []
        self.stationary_imu = []
        self.moving = False
        self.odom_messages = []
        self.joint_messages = []
        self.contact_events = []
        self.contact_message_count = 0
        self.wall_contact_message_count = 0
        self.clock_stamps = []
        self.tf_edges = {}
        self.tf_stamps = []
        self.tf_conflicts = []
        self.csv_rows = []
        self.start_wall = time.monotonic()
        self.last_clock_s = 0.0

    def add_row(self, topic, stamp, **values):
        self.csv_rows.append({
            "topic": topic, "sim_time_s": stamp,
            **values,
        })

    def on_image(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.image_messages.append((stamp, msg.header.frame_id))
        self.latest_image = msg
        if not self.moving and len(self.image_candidates) < 15:
            self.image_candidates.append(msg)
        self.add_row("camera_image", stamp, frame_id=msg.header.frame_id,
                     width=msg.width, height=msg.height, encoding=msg.encoding)

    def on_camera_info(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.camera_info_messages.append(msg)
        self.add_row("camera_info", stamp, frame_id=msg.header.frame_id,
                     fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])

    def on_imu(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        values = {
            "frame_id": msg.header.frame_id,
            "orientation_covariance_0": msg.orientation_covariance[0],
            "gyro_x": msg.angular_velocity.x,
            "gyro_y": msg.angular_velocity.y,
            "gyro_z": msg.angular_velocity.z,
            "accel_x": msg.linear_acceleration.x,
            "accel_y": msg.linear_acceleration.y,
            "accel_z": msg.linear_acceleration.z,
            "gyro_var_x": msg.angular_velocity_covariance[0],
            "gyro_var_y": msg.angular_velocity_covariance[4],
            "gyro_var_z": msg.angular_velocity_covariance[8],
            "accel_var_x": msg.linear_acceleration_covariance[0],
            "accel_var_y": msg.linear_acceleration_covariance[4],
            "accel_var_z": msg.linear_acceleration_covariance[8],
        }
        self.imu_messages.append((stamp, msg, values))
        if not self.moving and len(self.stationary_imu) < 200:
            self.stationary_imu.append(values)
        self.add_row("imu", stamp, **values)

    def on_odom(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        item = {
            "stamp": stamp,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": angle_from_quaternion(msg.pose.pose.orientation),
            "linear_x": msg.twist.twist.linear.x,
            "angular_z": msg.twist.twist.angular.z,
        }
        self.odom_messages.append(item)
        self.add_row("odom", stamp, **{k: v for k, v in item.items()
                                       if k != "stamp"})

    def on_joint_states(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        item = {
            "stamp": stamp,
            "names": list(msg.name),
            "positions": list(msg.position),
            "velocities": list(msg.velocity),
        }
        self.joint_messages.append(item)
        self.add_row("joint_states", stamp, names=";".join(msg.name),
                     positions=";".join(str(v) for v in msg.position))

    def on_contacts(self, msg):
        self.contact_message_count += 1
        message_has_wall_contact = False
        for contact in msg.contacts:
            item = {
                "wall_elapsed_s": time.monotonic() - self.start_wall,
                "collision1": contact.collision1.name,
                "collision2": contact.collision2.name,
                "max_depth_m": max(contact.depths, default=0.0),
            }
            self.contact_events.append(item)
            if "phase2_wall" in item["collision1"] or "phase2_wall" in item["collision2"]:
                message_has_wall_contact = True
            self.csv_rows.append({"topic": "contact", **item})
        if message_has_wall_contact:
            self.wall_contact_message_count += 1

    def on_clock(self, msg):
        stamp = stamp_seconds(msg.clock)
        self.clock_stamps.append(stamp)
        self.last_clock_s = stamp
        self.add_row("clock", stamp)

    def on_tf(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            edge = (parent, child)
            prior = self.tf_edges.get(child)
            if prior is not None and prior != parent:
                self.tf_conflicts.append({
                    "child": child, "parent_a": prior, "parent_b": parent,
                })
            self.tf_edges[child] = parent
            if transform.header.stamp.sec or transform.header.stamp.nanosec:
                self.tf_stamps.append(stamp_seconds(transform.header.stamp))

    def command(self, linear_x=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        self.cmd_pub.publish(msg)

    def run_for(self, duration_s, linear_x=0.0):
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.command(linear_x)
            rclpy.spin_once(self, timeout_sec=0.025)
        self.command(0.0)

    def wait_for_initial_data(self, timeout_s=20.0):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self.odom_messages and self.latest_image and self.imu_messages:
                return
            self.command(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(
            "Timed out waiting for initial /odom, camera image, and /imu/data"
        )

    def query_truth(self):
        if not self.state_client.wait_for_service(timeout_sec=5.0):
            return None, "evaluator service unavailable"
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.state_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None, "evaluator query timed out"
        response = future.result()
        if response.result.result != 1:
            return None, response.result.error_message
        state = response.state
        p, q = state.pose.position, state.pose.orientation
        return {
            "sim_time_s": stamp_seconds(state.header.stamp),
            "frame_id": state.header.frame_id,
            "x": p.x, "y": p.y, "z": p.z,
            "yaw_rad": angle_from_quaternion(q),
            "linear_x_mps": state.twist.linear.x,
            "angular_z_radps": state.twist.angular.z,
        }, None


def build_report(node, run_id):
    checks = []

    def check(name, passed, detail):
        checks.append({"name": name, "status": "PASS" if passed else "FAIL",
                       "detail": detail})

    def sampled_color_count(msg):
        raw = bytes(msg.data)
        if msg.encoding.lower() not in ("rgb8", "bgr8") or len(raw) < 3:
            return 0
        return len({tuple(raw[i:i + 3])
                    for i in range(0, len(raw) - 2, 30)})
    candidates = node.image_candidates or ([node.latest_image]
                                           if node.latest_image else [])
    image = max(candidates, key=sampled_color_count) if candidates else None
    image_info = None
    image_path = EVIDENCE / f"phase3-camera-frame-{run_id}.png"
    if image is not None:
        try:
            from PIL import Image as PILImage
            expected_bytes = image.width * image.height * 3
            raw = bytes(image.data)
            if image.encoding.lower() not in ("rgb8", "bgr8"):
                raise ValueError(f"unsupported encoding {image.encoding}")
            if len(raw) != expected_bytes:
                raise ValueError(
                    f"expected {expected_bytes} raw bytes, got {len(raw)}"
                )
            pil = PILImage.frombytes("RGB", (image.width, image.height), raw)
            if image.encoding.lower() == "bgr8":
                # Convert channel order without modifying the captured pixels.
                rawrgb = bytearray(raw)
                for index in range(0, len(rawrgb), 3):
                    rawrgb[index], rawrgb[index + 2] = rawrgb[index + 2], rawrgb[index]
                pil = PILImage.frombytes("RGB", (image.width, image.height), bytes(rawrgb))
            pil.save(image_path)
            colors = len(set(pil.getdata()))
            extrema = pil.getextrema()
            image_info = {
                "path": str(image_path.relative_to(ROOT)),
                "frame_id": image.header.frame_id,
                "stamp_s": stamp_seconds(image.header.stamp),
                "width": image.width, "height": image.height,
                "encoding": image.encoding, "unique_rgb_pixels": colors,
                "channel_extrema": extrema,
                "capture_stage": "pre-contact stationary camera frame",
            }
        except Exception as exc:
            image_info = {"error": str(exc)}
    check("camera image exists", image_info is not None and "error" not in image_info,
          image_info or "no image received")
    if image_info and "error" not in image_info:
        check("rendered RGB scene", image_info["unique_rgb_pixels"] > 20,
              f'{image_info["unique_rgb_pixels"]} distinct pixel colors')
        check("camera dimensions and frame",
              image_info["width"] == 320 and image_info["height"] == 240
              and image_info["frame_id"] == "camera_optical_frame",
              f'{image_info["width"]}x{image_info["height"]}, '
              f'frame={image_info["frame_id"]}')

    img_stamps = [x[0] for x in node.image_messages]
    info_stamps = [stamp_seconds(x.header.stamp)
                   for x in node.camera_info_messages]
    imu_stamps = [x[0] for x in node.imu_messages]
    odom_stamps = [x["stamp"] for x in node.odom_messages]
    joint_stamps = [x["stamp"] for x in node.joint_messages]

    camera_rate = rate_hz(img_stamps)
    info_rate = rate_hz(info_stamps)
    imu_rate = rate_hz(imu_stamps)
    odom_rate = rate_hz(odom_stamps)
    joint_rate = rate_hz(joint_stamps)
    check("camera and CameraInfo frequency",
          10.5 <= camera_rate <= 20.0 and 10.5 <= info_rate <= 20.0,
          f"image={camera_rate:.2f} Hz, info={info_rate:.2f} Hz; expected about 15 Hz")
    check("sensor timestamps monotonic",
          monotonic(img_stamps) and monotonic(info_stamps) and monotonic(imu_stamps),
          "strictly increasing camera, CameraInfo and IMU simulation stamps")
    check("camera frame and calibration",
          bool(node.camera_info_messages)
          and all(m.header.frame_id == "camera_optical_frame"
                  for m in node.camera_info_messages)
          and all(len(m.k) == 9 and len(m.p) == 12 and len(m.d) == 5
                  and m.distortion_model == "plumb_bob"
                  for m in node.camera_info_messages),
          "CameraInfo frame, K/P/D lengths and plumb_bob model")
    expected_focal = 320.0 / (2.0 * math.tan(1.0471975512 / 2.0))
    info = node.camera_info_messages[-1] if node.camera_info_messages else None
    focal_ok = info is not None and abs(info.k[0] - expected_focal) < 0.5 \
        and abs(info.k[4] - expected_focal) < 0.5
    check("camera calibration values", focal_ok,
          (f"fx={info.k[0]:.3f}, fy={info.k[4]:.3f}, expected "
           f"{expected_focal:.3f} px for provisional 60 deg HFOV") if info else "no CameraInfo")
    check("IMU frequency", 70.0 <= imu_rate <= 130.0,
          f"{imu_rate:.2f} Hz; expected about 100 Hz")
    imu_ok = bool(node.imu_messages) and all(
        msg.header.frame_id == "imu_link"
        and msg.orientation_covariance[0] == -1.0
        and msg.orientation.x == 0.0 and msg.orientation.y == 0.0
        and msg.orientation.z == 0.0 and msg.orientation.w == 0.0
        for _, msg, _ in node.imu_messages
    )
    check("IMU orientation truth suppressed", imu_ok,
          "zero quaternion and covariance[0] = -1 on /imu/data; frame imu_link")
    covariance_ok = bool(node.imu_messages) and all(
        abs(msg.angular_velocity_covariance[0] - 4e-6) < 1e-9
        and abs(msg.linear_acceleration_covariance[0] - 4e-4) < 1e-8
        for _, msg, _ in node.imu_messages
    )
    check("IMU configured covariance", covariance_ok,
          "gyro variance 4e-6 (0.002 rad/s sigma), acceleration variance "
          "4e-4 (0.02 m/s^2 sigma)")
    if node.stationary_imu:
        means = {
            axis: statistics.mean(s[f"accel_{axis}"]
                                  for s in node.stationary_imu)
            for axis in "xyz"
        }
        gravity_ok = abs(means["z"] - 9.80665) < 0.7 \
            and abs(means["x"]) < 0.7 and abs(means["y"]) < 0.7
        check("IMU stationary gravity convention", gravity_ok,
              f"stationary mean acceleration x/y/z = "
              f"{means['x']:.3f}/{means['y']:.3f}/{means['z']:.3f} m/s^2")
    else:
        check("IMU stationary gravity convention", False, "no stationary samples")
    check("odometry interface and frequency",
          bool(node.odom_messages)
          and all(m["frame_id"] == "odom"
                  and m["child_frame_id"] == "base_footprint"
                  for m in node.odom_messages)
          and 24.0 <= odom_rate <= 36.0,
          f"{len(node.odom_messages)} messages at {odom_rate:.2f} Hz; "
          "expected 30 Hz from robot_parameters.yaml; frames odom/base_footprint")
    expected_joints = {"left_wheel_joint", "right_wheel_joint"}
    joint_names = set(node.joint_messages[-1]["names"]) if node.joint_messages else set()
    check("wheel joint states",
          expected_joints.issubset(joint_names) and joint_rate >= 10.0,
          f"last names={sorted(joint_names)}, rate={joint_rate:.2f} Hz")
    check("simulation clock", len(node.clock_stamps) > 10
          and monotonic(node.clock_stamps),
          f"{len(node.clock_stamps)} increasing /clock messages")
    required_edges = {
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
        ("base_link", "camera_link"),
        ("camera_link", "camera_optical_frame"),
        ("base_link", "imu_link"),
        ("base_link", "left_wheel_link"),
        ("base_link", "right_wheel_link"),
    }
    found_edges = {(parent, child) for child, parent in node.tf_edges.items()}
    missing_edges = sorted(required_edges - found_edges)
    check("TF chain and wheel frames", not missing_edges
          and not node.tf_conflicts,
          f"missing={missing_edges}, conflicting_children={node.tf_conflicts}")
    if node.tf_stamps and node.last_clock_s:
        tf_fresh = node.last_clock_s - max(node.tf_stamps) <= 0.75
    else:
        tf_fresh = False
    check("dynamic TF timestamp freshness", tf_fresh,
          f"latest clock={node.last_clock_s:.3f}s, newest dynamic TF="
          f"{max(node.tf_stamps, default=0.0):.3f}s")

    truth, truth_error = node.query_truth()
    odom = node.odom_messages[-1] if node.odom_messages else None
    expected_wall = 0.735
    wall_stopped = truth is not None and abs(truth["x"] - expected_wall) <= 0.04 \
        and abs(truth["linear_x_mps"]) < 0.03
    check("reproducible wall stall", wall_stopped,
          f"evaluator pose={truth}; expected x about {expected_wall:.3f} m; "
          f"error={truth_error}")
    wall_contact_events = [
        item for item in node.contact_events
        if "phase2_wall" in item["collision1"]
        or "phase2_wall" in item["collision2"]
    ]
    contact_ok = node.wall_contact_message_count > 0 and len(wall_contact_events) > 0
    check("wall contact telemetry", contact_ok,
          f"{node.contact_message_count} contact messages; "
          f"{len(node.contact_events)} contact records; "
          f"{node.wall_contact_message_count} messages and "
          f"{len(wall_contact_events)} records include phase2_wall; first="
          f"{wall_contact_events[0] if wall_contact_events else None}")
    odom_truth = None
    if truth is not None and odom is not None:
        odom_truth = {
            "odom_x_m": odom["x"], "evaluator_x_m": truth["x"],
            "x_divergence_m": odom["x"] - truth["x"],
            "odom_yaw_rad": odom["yaw"], "evaluator_yaw_rad": truth["yaw_rad"],
            "yaw_divergence_rad": odom["yaw"] - truth["yaw_rad"],
        }
    if odom_truth:
        check("wheel odometry stall characterization",
              abs(odom_truth["x_divergence_m"]) < 2.0,
              f"wheel odometry/evaluator comparison={odom_truth}; "
              "evaluator data used only by this test")

    latest_sensor_stamps = [
        max(img_stamps, default=0.0),
        max(info_stamps, default=0.0),
        max(imu_stamps, default=0.0),
        max(odom_stamps, default=0.0),
    ]
    freshness = [node.last_clock_s - stamp for stamp in latest_sensor_stamps]
    check("sensor timestamps not stale",
          node.last_clock_s > 0 and all(0.0 <= age <= 0.75 for age in freshness),
          f"age behind /clock [image, CameraInfo, IMU, odom] = "
          f"{[round(x, 3) for x in freshness]} s")

    summary = {
        "run_id": run_id,
        "profile": "REALISTIC",
        "mode": "headless Gazebo Harmonic",
        "camera": {
            "image_count": len(img_stamps), "image_rate_hz": camera_rate,
            "camera_info_count": len(info_stamps), "camera_info_rate_hz": info_rate,
            "frame": "camera_optical_frame",
            "provisional_horizontal_fov_rad": 1.0471975512,
            "resolution": [320, 240],
            "image_evidence": image_info,
        },
        "imu": {
            "count": len(imu_stamps), "rate_hz": imu_rate, "frame": "imu_link",
            "orientation": "unavailable; Gazebo truth-derived quaternion suppressed",
            "stationary_sample_count": len(node.stationary_imu),
        },
        "odometry": {
            "count": len(odom_stamps), "rate_hz": odom_rate,
            "frame": "odom", "child_frame": "base_footprint",
            "stall_comparison": odom_truth,
        },
        "joint_states": {
            "count": len(joint_stamps), "rate_hz": joint_rate,
            "names": sorted(joint_names),
        },
        "clock": {"count": len(node.clock_stamps), "latest_s": node.last_clock_s},
        "tf": {
            "edges": sorted([list(edge) for edge in found_edges]),
            "conflicts": node.tf_conflicts, "missing": missing_edges,
        },
        "contact": {
            "message_count": node.contact_message_count,
            "messages_with_wall_contact": node.wall_contact_message_count,
            "contact_record_count": len(node.contact_events),
            "wall_event_count": len(wall_contact_events),
            "wall_event_sample": wall_contact_events[:20],
            "events": node.contact_events[:20],
            "robot_collision_test": "forward command toward fixed phase2_wall",
        },
        "evaluator_only_truth": truth,
        "checks": checks,
        "status": "PASS" if all(x["status"] == "PASS" for x in checks) else "FAIL",
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="final",
                        help="suffix for evidence filenames")
    parser.add_argument("--warmup-seconds", type=float, default=2.0)
    parser.add_argument("--approach-seconds", type=float, default=10.0)
    args = parser.parse_args()

    EVIDENCE.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = SensorSmoke()
    try:
        node.wait_for_initial_data()
        node.run_for(args.warmup_seconds, 0.0)
        node.moving = True
        node.run_for(args.approach_seconds, 0.12)
        node.run_for(0.8, 0.0)
        # Flush callbacks and let simulated timestamps settle before final samples.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        summary = build_report(node, args.run_id)
        result_path = EVIDENCE / f"phase3-sensor-smoke-{args.run_id}.json"
        csv_path = EVIDENCE / f"phase3-sensor-telemetry-{args.run_id}.csv"
        with result_path.open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2)
            stream.write("\n")
        columns = sorted({key for row in node.csv_rows for key in row})
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(node.csv_rows)
        print(json.dumps({
            "status": summary["status"],
            "result": str(result_path),
            "telemetry": str(csv_path),
            "checks": summary["checks"],
        }, indent=2))
        return 0 if summary["status"] == "PASS" else 1
    finally:
        node.command(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
