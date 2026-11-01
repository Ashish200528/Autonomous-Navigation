#!/usr/bin/env python3
"""Exercise runtime fail-closed behavior for absent and stale RGB."""
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from anc_perception.perception_node import RGBPerceptionNode


class FaultHarness(Node):
    def __init__(self):
        super().__init__("phase7_sensor_fault_harness")
        self.clock_pub = self.create_publisher(Clock, "/clock", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.health_sub = self.create_subscription(String, "/perception/health", self.on_health, 10)
        self.mask_sub = self.create_subscription(Image, "/perception/image_mask", self.on_mask, qos_profile_sensor_data)
        self.confidence_sub = self.create_subscription(Image, "/perception/confidence", self.on_confidence, qos_profile_sensor_data)
        self.health = None
        self.mask = None
        self.mask_stamp_ns = None
        self.confidence = None
        self.health_history = []

    def on_health(self, msg):
        try:
            self.health = json.loads(msg.data)
            self.health_history.append(self.health)
        except json.JSONDecodeError:
            pass

    def _mono(self, msg):
        if msg.encoding != "mono8" or msg.step < msg.width:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step)
        return raw.reshape(msg.height, msg.step)[:, :msg.width].copy()

    def on_mask(self, msg):
        self.mask = self._mono(msg)
        self.mask_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def on_confidence(self, msg):
        self.confidence = self._mono(msg)

    def publish_time(self, seconds):
        sec = int(seconds)
        clock = Clock()
        clock.clock = RosTime(sec=sec, nanosec=int((seconds - sec) * 1e9))
        self.clock_pub.publish(clock)

    def publish_camera(self, seconds):
        info = CameraInfo()
        info.header.frame_id = "camera_optical_frame"
        sec = int(seconds)
        info.header.stamp = RosTime(sec=sec, nanosec=int((seconds - sec) * 1e9))
        info.width = 320
        info.height = 240
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [277.128, 0.0, 160.0, 0.0, 277.128, 120.0, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)

    def publish_floor_rgb(self, seconds):
        msg = Image()
        msg.header.frame_id = "camera_optical_frame"
        sec = int(seconds)
        msg.header.stamp = RosTime(sec=sec, nanosec=int((seconds - sec) * 1e9))
        msg.height = 240
        msg.width = 320
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 320 * 3
        msg.data = np.full((240, 320, 3), 150, dtype=np.uint8).tobytes()
        self.image_pub.publish(msg)


def wait_for(predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    raise RuntimeError(f"timeout_waiting_for:{label}")


def main():
    rclpy.init()
    perception = RGBPerceptionNode()
    harness = FaultHarness()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(perception)
    executor.add_node(harness)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    result = {"status": "FAIL", "checks": {}}
    try:
        wait_for(
            lambda: harness.health is not None
            and harness.health.get("reason") == "no_rgb_received",
            1.5, "no_rgb_received health",
        )
        wait_for(
            lambda: harness.mask is not None and np.all(harness.mask == 255)
            and harness.confidence is not None and np.all(harness.confidence == 0),
            0.5, "all-UNKNOWN mask before first RGB",
        )
        result["checks"]["missing_rgb"] = {
            "pass": True,
            "health": harness.health,
            "mask_values": sorted(int(v) for v in np.unique(harness.mask)),
            "mask_stamp_s": harness.mask_stamp_ns / 1e9,
            "confidence_values": sorted(int(v) for v in np.unique(harness.confidence)),
        }

        harness.publish_time(2.0)
        harness.publish_camera(2.0)
        time.sleep(0.12)
        for _ in range(4):
            harness.publish_floor_rgb(2.0)
            time.sleep(0.08)
        wait_for(
            lambda: harness.health is not None and harness.health.get("valid") is True,
            2.0, "valid RGB perception output",
        )
        wait_for(
            lambda: harness.mask is not None and np.count_nonzero(harness.mask == 0) > 5000,
            0.5, "useful FREE output for fresh RGB",
        )
        valid_health = dict(harness.health)
        valid_free = int(np.count_nonzero(harness.mask == 0))
        result["checks"]["fresh_rgb"] = {
            "pass": True,
            "health": valid_health,
            "free_pixels": valid_free,
        }

        harness.publish_time(2.5)
        wait_for(
            lambda: harness.health is not None
            and harness.health.get("valid") is False
            and harness.health.get("reason") == "rgb_sim_timestamp_stale",
            0.35, "stale simulated timestamp invalidation",
        )
        wait_for(
            lambda: harness.mask is not None and np.all(harness.mask == 255)
            and harness.mask_stamp_ns == 2_000_000_000
            and harness.confidence is not None and np.all(harness.confidence == 0),
            0.5, "source-stamped all-UNKNOWN output after stale timestamp",
        )
        result["checks"]["stale_timestamp"] = {
            "pass": True,
            "health": harness.health,
            "mask_values": sorted(int(v) for v in np.unique(harness.mask)),
            "mask_stamp_s": harness.mask_stamp_ns / 1e9,
            "confidence_values": sorted(int(v) for v in np.unique(harness.confidence)),
        }
        result["status"] = "PASS"
    finally:
        result["health_messages"] = len(harness.health_history)
        output = Path(__file__).resolve().parents[1] / "docs/evidence/phase7/fault-injection.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        harness.destroy_node()
        perception.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
