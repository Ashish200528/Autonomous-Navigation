import copy
import json
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from anc_perception.classifier import FREE, OCCUPIED, UNKNOWN, classify_rgb


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _stamp_from_seconds(value):
    if not math.isfinite(value) or value < 0:
        return Time()
    sec = int(value)
    return Time(sec=sec, nanosec=int((value - sec) * 1e9))


def _decode_rgb(msg):
    width, height = int(msg.width), int(msg.height)
    if width <= 0 or height <= 0:
        raise ValueError("image_invalid_dimensions")
    encodings = {
        "rgb8": (3, "rgb"),
        "bgr8": (3, "bgr"),
        "rgba8": (4, "rgba"),
        "bgra8": (4, "bgra"),
    }
    if msg.encoding not in encodings:
        raise ValueError(f"image_unsupported_encoding:{msg.encoding}")
    channels, order = encodings[msg.encoding]
    minimum_step = width * channels
    if int(msg.step) < minimum_step or len(msg.data) < height * int(msg.step):
        raise ValueError("image_data_length_or_step_invalid")
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=height * int(msg.step))
    rows = rows.reshape(height, int(msg.step))[:, :minimum_step]
    array = rows.reshape(height, width, channels)
    if order == "rgb":
        return array.copy()
    if order == "bgr":
        return cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
    if order == "rgba":
        return cv2.cvtColor(array, cv2.COLOR_RGBA2RGB)
    return cv2.cvtColor(array, cv2.COLOR_BGRA2RGB)


def _image_message(array, encoding, header):
    msg = Image()
    msg.header = copy.deepcopy(header)
    msg.height, msg.width = int(array.shape[0]), int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    if array.ndim == 2:
        msg.step = int(array.shape[1])
    else:
        msg.step = int(array.shape[1] * array.shape[2])
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


class RGBPerceptionNode(Node):
    def __init__(self):
        super().__init__("anc_rgb_perception")
        defaults = {
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "clock_topic": "/clock",
            "mask_topic": "/perception/image_mask",
            "confidence_topic": "/perception/confidence",
            "overlay_topic": "/perception/overlay",
            "health_topic": "/perception/health",
            "expected_frame_id": "camera_optical_frame",
            "camera_height_m": 0.18,
            "camera_forward_offset_m": 0.10,
            "camera_pitch_down_rad": 0.0,
            "min_ground_forward_m": 0.50,
            "max_ground_forward_m": 0.60,
            "floor_seed_row_fraction": 0.90,
            "floor_seed_left_fraction": 0.02,
            "floor_seed_right_fraction": 0.98,
            "floor_seed_min_pixels": 24,
            "floor_component_min_area_px": 4000,
            "floor_seed_support_tolerance": 30.0,
            "floor_reference_lab": [171.0, 128.0, 128.0],
            "free_gradient_floor": 22.0,
            "floor_color_threshold": 16.0,
            "occupied_chroma_delta_min": 24.0,
            "occupied_gray_delta_min": 24.0,
            "occupied_gray_edge_max": 80.0,
            "occupied_component_min_area_px": 8,
            "occupied_component_min_height_px": 4,
            "occupied_component_min_width_px": 2,
            "max_sim_age_sec": 0.35,
            "max_wall_age_sec": 0.45,
            "camera_info_max_age_sec": 0.35,
            "stale_check_period_sec": 0.10,
            "fallback_width_px": 320,
            "fallback_height_px": 240,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        self._classifier_config = {
            name: self.params[name] for name in (
                "camera_height_m", "camera_forward_offset_m",
                "camera_pitch_down_rad", "min_ground_forward_m",
                "max_ground_forward_m", "floor_seed_row_fraction",
                "floor_seed_left_fraction", "floor_seed_right_fraction",
                "floor_seed_min_pixels", "floor_component_min_area_px",
                "floor_seed_support_tolerance", "floor_reference_lab",
                "free_gradient_floor",
                "floor_color_threshold", "occupied_chroma_delta_min",
                "occupied_gray_delta_min", "occupied_gray_edge_max",
                "occupied_component_min_area_px",
                "occupied_component_min_height_px",
                "occupied_component_min_width_px",
            )
        }
        self.last_info = None
        self.last_info_wall = None
        self.clock_seconds = None
        self.clock_wall = None
        self.last_image_wall = None
        self.last_image_msg = None
        self.last_rgb = None
        self.last_valid = False
        self.last_stale_reason = None
        self.invalid_publish_wall = 0.0
        self.image_count = 0
        self.image_times = deque(maxlen=60)
        self.warned_reasons = set()

        self.mask_pub = self.create_publisher(
            Image, self.params["mask_topic"], qos_profile_sensor_data
        )
        self.confidence_pub = self.create_publisher(
            Image, self.params["confidence_topic"], qos_profile_sensor_data
        )
        self.overlay_pub = self.create_publisher(
            Image, self.params["overlay_topic"], qos_profile_sensor_data
        )
        self.health_pub = self.create_publisher(String, self.params["health_topic"], 10)
        self.image_sub = self.create_subscription(
            Image, self.params["image_topic"], self.on_image, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, self.params["camera_info_topic"], self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.clock_sub = self.create_subscription(
            Clock, self.params["clock_topic"], self.on_clock, qos_profile_sensor_data
        )
        self.timer = self.create_timer(
            float(self.params["stale_check_period_sec"]), self.check_stale
        )
        self.get_logger().info(
            "RGB-only inputs: "
            f"{self.params['image_topic']}, {self.params['camera_info_topic']}, "
            f"{self.params['clock_topic']} (time validity only); "
            "no odometry, IMU, Gazebo truth, depth, map, or scenario geometry."
        )

    def on_clock(self, msg):
        now = _stamp_seconds(msg.clock)
        if self.clock_seconds is not None and now + 0.5 < self.clock_seconds:
            self.last_info = None
            self.last_info_wall = None
            self.last_valid = False
            self.get_logger().warning("Simulation clock reset; camera calibration cache cleared.")
        self.clock_seconds = now
        self.clock_wall = time.monotonic()

    def on_camera_info(self, msg):
        self.last_info = msg
        self.last_info_wall = time.monotonic()

    def _health(self, valid, reason, image_stamp=None, info_stamp=None,
                sim_age=None, wall_age=None, latency_ms=None, details=None):
        rates = 0.0
        if len(self.image_times) >= 2:
            elapsed = self.image_times[-1] - self.image_times[0]
            if elapsed > 0:
                rates = (len(self.image_times) - 1) / elapsed
        info = {
            "node": "anc_rgb_perception",
            "profile": "REALISTIC_RGB_ONLY",
            "valid": bool(valid),
            "status": "VALID" if valid else "INVALID_UNKNOWN",
            "reason": reason,
            "input_topics": [
                self.params["image_topic"],
                self.params["camera_info_topic"],
                self.params["clock_topic"],
            ],
            "clock_use": "timestamp-age only; not a geometric sensor",
            "forbidden_inputs": [
                "odometry", "IMU", "Gazebo entity state", "depth",
                "oracle map", "scenario geometry",
            ],
            "image_stamp_s": image_stamp,
            "camera_info_stamp_s": info_stamp,
            "camera_sim_age_s": sim_age,
            "camera_wall_age_s": wall_age,
            "processing_latency_ms": latency_ms,
            "measured_wall_fps": round(float(rates), 3),
            "processed_frames": int(self.image_count),
            "mask_encoding": "mono8: FREE=0, OCCUPIED=100, UNKNOWN=255",
            "unknown_confidence_value": 0,
        }
        if details:
            info.update(details)
        msg = String()
        msg.data = json.dumps(info, sort_keys=True, separators=(",", ":"))
        self.health_pub.publish(msg)

    def _publish_images(self, mask, confidence, overlay, source_header):
        self.mask_pub.publish(_image_message(mask, "mono8", source_header))
        self.confidence_pub.publish(_image_message(confidence, "mono8", source_header))
        self.overlay_pub.publish(_image_message(overlay, "bgr8", source_header))

    def _unknown(self, reason, now_wall=None):
        now_wall = time.monotonic() if now_wall is None else now_wall
        if self.last_image_msg is not None:
            height = max(1, int(self.last_image_msg.height))
            width = max(1, int(self.last_image_msg.width))
            source_header = copy.deepcopy(self.last_image_msg.header)
        elif self.last_info is not None:
            height = max(1, int(self.last_info.height))
            width = max(1, int(self.last_info.width))
            source_header = copy.deepcopy(self.last_info.header)
        else:
            height = int(self.params["fallback_height_px"])
            width = int(self.params["fallback_width_px"])
            source_header = Image().header
            source_header.frame_id = str(self.params["expected_frame_id"])
        # Keep invalid output paired to its source image. Only synthesize a
        # current stamp when no RGB header exists (e.g. startup/no-input state).
        if self.last_image_msg is None and self.clock_seconds is not None:
            source_header.stamp = _stamp_from_seconds(self.clock_seconds)
        mask = np.full((height, width), UNKNOWN, dtype=np.uint8)
        confidence = np.zeros((height, width), dtype=np.uint8)
        if self.last_rgb is not None and self.last_rgb.shape[:2] == (height, width):
            overlay = cv2.cvtColor(self.last_rgb, cv2.COLOR_RGB2BGR)
            overlay = cv2.addWeighted(
                overlay, 0.55,
                np.full_like(overlay, (255, 80, 0)), 0.45, 0.0,
            )
        else:
            overlay = np.zeros((height, width, 3), dtype=np.uint8)
            overlay[:, :] = (255, 80, 0)
        cv2.putText(
            overlay, f"UNKNOWN / INVALID: {reason[:30]}",
            (4, max(16, min(height - 4, 20))), cv2.FONT_HERSHEY_SIMPLEX,
            0.38, (255, 255, 255), 1, cv2.LINE_AA,
        )
        self._publish_images(mask, confidence, overlay, source_header)
        image_stamp = _stamp_seconds(self.last_image_msg.header.stamp) if self.last_image_msg else None
        info_stamp = _stamp_seconds(self.last_info.header.stamp) if self.last_info else None
        sim_age = None
        if image_stamp is not None and self.clock_seconds is not None:
            sim_age = round(self.clock_seconds - image_stamp, 6)
        wall_age = (
            None if self.last_image_wall is None
            else round(now_wall - self.last_image_wall, 6)
        )
        self._health(False, reason, image_stamp, info_stamp, sim_age, wall_age)
        self.last_valid = False
        self.last_stale_reason = reason
        self.invalid_publish_wall = now_wall

    def check_stale(self):
        now = time.monotonic()
        if self.last_image_wall is None:
            if now - self.invalid_publish_wall >= 0.5:
                self._unknown("no_rgb_received", now)
            return
        wall_age = now - self.last_image_wall
        if wall_age > float(self.params["max_wall_age_sec"]):
            if self.last_valid or now - self.invalid_publish_wall >= 0.5:
                self._unknown("rgb_wall_timeout", now)
            return
        if self.clock_seconds is None:
            if self.last_valid or self.last_stale_reason != "clock_unavailable":
                self._unknown("clock_unavailable", now)
            return
        sim_age = self.clock_seconds - _stamp_seconds(self.last_image_msg.header.stamp)
        if sim_age > float(self.params["max_sim_age_sec"]):
            if self.last_valid or self.last_stale_reason != "rgb_sim_timestamp_stale":
                self._unknown("rgb_sim_timestamp_stale", now)

    def on_image(self, msg):
        started = time.perf_counter()
        now_wall = time.monotonic()
        self.last_image_wall = now_wall
        self.last_image_msg = msg
        self.image_count += 1
        self.image_times.append(now_wall)
        try:
            rgb = _decode_rgb(msg)
            self.last_rgb = rgb
            if self.clock_seconds is None:
                raise ValueError("clock_unavailable")
            image_stamp = _stamp_seconds(msg.header.stamp)
            sim_age = self.clock_seconds - image_stamp
            if sim_age < -0.05:
                raise ValueError("rgb_timestamp_in_future")
            if sim_age > float(self.params["max_sim_age_sec"]):
                raise ValueError("rgb_sim_timestamp_stale")
            if self.last_info is None:
                raise ValueError("camera_info_missing")
            if self.last_info.header.frame_id != msg.header.frame_id:
                raise ValueError("camera_info_frame_mismatch")
            if msg.header.frame_id != str(self.params["expected_frame_id"]):
                raise ValueError("unexpected_camera_frame")
            if int(self.last_info.width) != int(msg.width) or int(self.last_info.height) != int(msg.height):
                raise ValueError("camera_info_dimensions_mismatch")
            info_stamp = _stamp_seconds(self.last_info.header.stamp)
            info_age = abs(image_stamp - info_stamp)
            if info_age > float(self.params["camera_info_max_age_sec"]):
                raise ValueError("camera_info_timestamp_stale")
            info = {
                "width": int(self.last_info.width),
                "height": int(self.last_info.height),
                "k": list(self.last_info.k),
                "d": list(self.last_info.d),
                "distortion_model": self.last_info.distortion_model,
            }
            mask, confidence, details = classify_rgb(
                rgb, info, self._classifier_config
            )
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            colors = np.zeros_like(bgr)
            colors[mask == FREE] = (30, 210, 30)
            colors[mask == OCCUPIED] = (20, 20, 235)
            colors[mask == UNKNOWN] = (220, 110, 20)
            overlay = cv2.addWeighted(bgr, 0.62, colors, 0.38, 0.0)
            latency_ms = (time.perf_counter() - started) * 1000.0
            details["processing_latency_ms"] = round(float(latency_ms), 3)
            self._publish_images(mask, confidence, overlay, msg.header)
            self._health(
                True, "ok", image_stamp, info_stamp, round(sim_age, 6),
                round(now_wall - self.last_image_wall, 6), latency_ms,
                details,
            )
            self.last_valid = True
            self.last_stale_reason = None
        except Exception as exc:
            reason = str(exc) or type(exc).__name__
            if reason not in self.warned_reasons:
                self.get_logger().warning(f"Publishing UNKNOWN for invalid RGB input: {reason}")
                self.warned_reasons.add(reason)
            self._unknown(reason, now_wall)


def main(args=None):
    rclpy.init(args=args)
    node = RGBPerceptionNode()
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
