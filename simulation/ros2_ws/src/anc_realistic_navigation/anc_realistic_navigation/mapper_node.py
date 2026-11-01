"""Fresh-snapshot, base_link-relative RGB traversability map."""
import json
import math
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from anc_realistic_navigation.projection import project_free_pixels, quaternion_matrix

NANO = 1_000_000_000

def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec)/NANO

def decode_mono8(msg):
    if msg.encoding != "mono8" or msg.width <= 0 or msg.height <= 0:
        raise ValueError("image_not_valid_mono8")
    if msg.step < msg.width or len(msg.data) < msg.height*msg.step:
        raise ValueError("image_data_length_or_step_invalid")
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height*msg.step)
    return rows.reshape(msg.height, msg.step)[:, :msg.width].copy()

class LocalTraversabilityNode(Node):
    def __init__(self):
        super().__init__("anc_local_traversability")
        defaults = {
            "mask_topic": "/perception/image_mask",
            "confidence_topic": "/perception/confidence",
            "health_topic": "/perception/health",
            "camera_info_topic": "/camera/camera_info",
            "clock_topic": "/clock",
            "odometry_topic": "/odometry/filtered",
            "grid_topic": "/realistic/local_traversability",
            "state_topic": "/realistic/navigation_state",
            "camera_frame": "camera_optical_frame",
            "base_frame": "base_link",
            "resolution_m": 0.02,
            "grid_width": 75,
            "grid_height": 75,
            "origin_x_m": 0.0,
            "origin_y_m": -0.75,
            "floor_z_in_base_m": -0.08,
            "min_free_confidence": 200,
            "max_image_wall_age_s": 0.45,
            "max_health_wall_age_s": 0.50,
            "max_camera_info_wall_age_s": 0.50,
            "max_odom_wall_age_s": 0.50,
            "max_odom_sim_age_s": 0.35,
            "max_clock_wall_age_s": 0.50,
            "max_sim_image_age_s": 0.35,
            "max_mask_sync_delta_s": 0.02,
            "max_camera_info_sync_delta_s": 0.10,
            "max_localization_xy_sigma_m": 0.50,
            "max_localization_yaw_sigma_rad": 0.50,
            "expected_camera_x_in_base_m": 0.10,
            "expected_camera_y_in_base_m": 0.0,
            "expected_camera_z_in_base_m": 0.10,
            "expected_camera_height_above_floor_m": 0.18,
            "camera_transform_tolerance_m": 0.02,
            "publish_rate_hz": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.mask = self.confidence = self.info = self.health = self.odom = None
        self.mask_records = []
        self.confidence_records = []
        self.health_records = []
        self.mask_stamp = self.confidence_stamp = self.info_stamp = None
        self.mask_frame = self.confidence_frame = None
        self.mask_wall = self.confidence_wall = self.info_wall = None
        self.health_wall = self.odom_wall = self.clock_wall = None
        self.clock_s = None
        self.last_metrics = {}
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self, spin_thread=False)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.p["grid_topic"], 10)
        self.state_pub = self.create_publisher(String, self.p["state_topic"], 10)
        self.create_subscription(Image, self.p["mask_topic"], self.on_mask, qos_profile_sensor_data)
        self.create_subscription(Image, self.p["confidence_topic"], self.on_confidence, qos_profile_sensor_data)
        self.create_subscription(String, self.p["health_topic"], self.on_health, 10)
        self.create_subscription(CameraInfo, self.p["camera_info_topic"], self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Clock, self.p["clock_topic"], self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p["odometry_topic"], self.on_odom, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/float(self.p["publish_rate_hz"]), self.publish_snapshot)
        self.get_logger().info(
            "Fresh-snapshot RGB local map; inputs are mask, confidence, perception "
            "health, CameraInfo, /clock timestamp, filtered odometry and TF only. "
            "No global map, Gazebo truth, obstacle geometry, depth or Nav2 path."
        )

    def synchronize_latest_inputs(self):
        pair = None
        tolerance = float(self.p["max_mask_sync_delta_s"])
        for mask_record in reversed(self.mask_records):
            for confidence_record in reversed(self.confidence_records):
                if abs(mask_record["stamp"]-confidence_record["stamp"]) <= tolerance:
                    pair = (mask_record, confidence_record)
                    break
            if pair:
                break
        if pair:
            mask_record, confidence_record = pair
            self.mask = mask_record["array"]
            self.mask_stamp = mask_record["stamp"]
            self.mask_frame = mask_record["frame"]
            self.mask_wall = mask_record["wall"]
            self.confidence = confidence_record["array"]
            self.confidence_stamp = confidence_record["stamp"]
            self.confidence_frame = confidence_record["frame"]
            self.confidence_wall = confidence_record["wall"]
        if self.health_records:
            selected = None
            if self.mask_stamp is not None:
                for record in reversed(self.health_records):
                    stamp = record["message"].get("image_stamp_s")
                    if stamp is not None:
                        try:
                            if abs(float(stamp)-self.mask_stamp) <= tolerance:
                                selected = record
                                break
                        except (TypeError, ValueError):
                            pass
            selected = selected or self.health_records[-1]
            self.health = selected["message"]
            self.health_wall = selected["wall"]

    def on_mask(self, msg):
        try:
            self.mask_records.append({"array": decode_mono8(msg),
                                      "stamp": stamp_seconds(msg.header.stamp),
                                      "frame": msg.header.frame_id,
                                      "wall": time.monotonic()})
            self.mask_records = self.mask_records[-8:]
            self.synchronize_latest_inputs()
        except ValueError as exc:
            self.mask = None
            self.mask_wall = time.monotonic()
            self.last_metrics = {"decode_error": str(exc)}

    def on_confidence(self, msg):
        try:
            self.confidence_records.append({"array": decode_mono8(msg),
                                            "stamp": stamp_seconds(msg.header.stamp),
                                            "frame": msg.header.frame_id,
                                            "wall": time.monotonic()})
            self.confidence_records = self.confidence_records[-8:]
            self.synchronize_latest_inputs()
        except ValueError:
            self.confidence = None
            self.confidence_wall = time.monotonic()

    def on_health(self, msg):
        try:
            health = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            health = {"valid": False, "status": "INVALID_UNKNOWN",
                      "reason": "health_json_invalid"}
        self.health_records.append({"message": health, "wall": time.monotonic()})
        self.health_records = self.health_records[-12:]
        self.synchronize_latest_inputs()

    def on_camera_info(self, msg):
        self.info = msg
        self.info_stamp = stamp_seconds(msg.header.stamp)
        self.info_wall = time.monotonic()

    def on_clock(self, msg):
        self.clock_s = stamp_seconds(msg.clock)
        self.clock_wall = time.monotonic()

    def on_odom(self, msg):
        self.odom = msg
        self.odom_wall = time.monotonic()

    def sensor_failures(self):
        now = time.monotonic()
        p = self.p
        failures = []
        if self.mask is None or self.mask_wall is None or now-self.mask_wall > float(p["max_image_wall_age_s"]):
            failures.append("mask_missing_or_stale")
        if self.confidence is None or self.confidence_wall is None or now-self.confidence_wall > float(p["max_image_wall_age_s"]):
            failures.append("confidence_missing_or_stale")
        if self.health is None or self.health_wall is None or now-self.health_wall > float(p["max_health_wall_age_s"]):
            failures.append("perception_health_missing_or_stale")
        elif self.health.get("valid") is not True or self.health.get("status") != "VALID":
            failures.append("perception_invalid_unknown:" + str(self.health.get("reason", "unknown")))
        if self.info is None or self.info_wall is None or now-self.info_wall > float(p["max_camera_info_wall_age_s"]):
            failures.append("camera_info_missing_or_stale")
        if self.clock_s is None or self.clock_wall is None or now-self.clock_wall > float(p["max_clock_wall_age_s"]):
            failures.append("simulation_clock_missing_or_stale")
        if self.odom is None or self.odom_wall is None or now-self.odom_wall > float(p["max_odom_wall_age_s"]):
            failures.append("filtered_localization_missing_or_stale")
        elif self.odom is not None:
            odom_age = None if self.clock_s is None else self.clock_s-stamp_seconds(self.odom.header.stamp)
            if odom_age is not None and (odom_age < -0.05 or odom_age > float(p["max_odom_sim_age_s"])):
                failures.append("filtered_localization_timestamp_stale_or_future")
            covariance = self.odom.pose.covariance
            xy_vars = (float(covariance[0]), float(covariance[7]))
            yaw_var = float(covariance[35])
            if (self.odom.header.frame_id.lstrip("/") != "odom" or
                    self.odom.child_frame_id.lstrip("/") not in ("base_footprint", "base_link")):
                failures.append("filtered_localization_frame_invalid")
            elif (not all(math.isfinite(v) and 0.0 <= v for v in (*xy_vars, yaw_var)) or
                  max(math.sqrt(max(v, 0.0)) for v in xy_vars) > float(p["max_localization_xy_sigma_m"]) or
                  math.sqrt(max(yaw_var, 0.0)) > float(p["max_localization_yaw_sigma_rad"])):
                failures.append("filtered_localization_uncertainty")
        if self.mask is not None and self.confidence is not None:
            if self.mask.shape != self.confidence.shape:
                failures.append("mask_confidence_shape_mismatch")
            if (self.mask_stamp is None or self.confidence_stamp is None or
                    abs(self.mask_stamp-self.confidence_stamp) > float(p["max_mask_sync_delta_s"])):
                failures.append("mask_confidence_timestamp_mismatch")
            if self.mask_frame != self.confidence_frame or self.mask_frame != p["camera_frame"]:
                failures.append("image_frame_mismatch")
            if self.mask_stamp is not None and self.clock_s is not None:
                age = self.clock_s-self.mask_stamp
                if age < -0.05 or age > float(p["max_sim_image_age_s"]):
                    failures.append("image_sim_timestamp_stale_or_future")
        if self.health is not None and self.mask_stamp is not None:
            health_stamp = self.health.get("image_stamp_s")
            if health_stamp is None or abs(float(health_stamp)-self.mask_stamp) > float(p["max_mask_sync_delta_s"]):
                failures.append("perception_health_image_stamp_mismatch")
        if self.info is not None and self.mask_stamp is not None:
            if self.info.header.frame_id != p["camera_frame"]:
                failures.append("camera_info_frame_mismatch")
            elif self.info_stamp is not None and abs(self.info_stamp-self.mask_stamp) > float(p["max_camera_info_sync_delta_s"]):
                failures.append("camera_info_timestamp_mismatch")
        return failures

    def get_transform(self):
        tf = self.buffer.lookup_transform(self.p["base_frame"], self.p["camera_frame"], Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        xyz = (float(t.x), float(t.y), float(t.z))
        tolerance = float(self.p["camera_transform_tolerance_m"])
        expected = (float(self.p["expected_camera_x_in_base_m"]),
                    float(self.p["expected_camera_y_in_base_m"]),
                    float(self.p["expected_camera_z_in_base_m"]))
        if any(abs(a-b) > tolerance for a, b in zip(xyz, expected)):
            raise ValueError("camera_transform_disagrees_with_documented_mount")
        if abs(xyz[2]+0.08-float(self.p["expected_camera_height_above_floor_m"])) > tolerance:
            raise ValueError("camera_height_disagrees_with_documented_mount")
        quaternion = (q.x, q.y, q.z, q.w)
        optical_forward = quaternion_matrix(quaternion)[:, 2]
        if not np.allclose(optical_forward, [1.0, 0.0, 0.0], atol=0.02):
            raise ValueError("camera_orientation_disagrees_with_level_forward_classifier")
        return xyz, quaternion

    def publish_snapshot(self):
        width, height = int(self.p["grid_width"]), int(self.p["grid_height"])
        size = width*height
        data = [-1]*size
        failures = self.sensor_failures()
        counts = {"free_image_pixels": 0, "high_confidence_free_pixels": 0,
                  "projected_free_pixels": 0, "free_grid_cells": 0,
                  "occupied_image_pixels_unlocalized": 0, "unknown_image_pixels": 0,
                  "grid_unknown_cells": size, "grid_occupied_cells": 0}
        camera_transform = None
        if not failures:
            try:
                camera_dict = {
                    "width": self.info.width, "height": self.info.height,
                    "frame_id": self.info.header.frame_id,
                    "distortion_model": self.info.distortion_model,
                    "d": list(self.info.d), "k": list(self.info.k),
                }
                xyz, quat = self.get_transform()
                camera_transform = {"translation_base_m": list(xyz), "quaternion_base": list(quat)}
                data, counts = project_free_pixels(
                    self.mask, self.confidence, camera_dict, xyz, quat,
                    expected_frame=str(self.p["camera_frame"]),
                    grid_width=width, grid_height=height,
                    resolution=float(self.p["resolution_m"]),
                    origin_x=float(self.p["origin_x_m"]),
                    origin_y=float(self.p["origin_y_m"]),
                    floor_z=float(self.p["floor_z_in_base_m"]),
                    min_confidence=int(self.p["min_free_confidence"]),
                )
            except (ValueError, TransformException, TypeError, IndexError) as exc:
                failures.append("geometry_invalid:" + str(exc))
                data = [-1]*size
                counts["grid_unknown_cells"] = size
        elif self.mask is not None:
            counts["free_image_pixels"] = int(np.count_nonzero(self.mask == 0))
            counts["occupied_image_pixels_unlocalized"] = int(np.count_nonzero(self.mask == 100))
            counts["unknown_image_pixels"] = int(np.count_nonzero(self.mask == 255))
        occupied_unlocalized = int(counts["occupied_image_pixels_unlocalized"])
        gate = "INVALID_UNKNOWN" if failures else "STOP"
        reason = failures[0] if failures else "phase8_motion_lock_stop_only"
        block_reasons = list(failures)
        if occupied_unlocalized > 0:
            block_reasons.append("unlocalized_occupied_evidence")
        block_reasons.extend(["forward_corridor_not_proven_free", "phase8_motion_lock_stop_only"])

        msg = OccupancyGrid()
        msg.header.frame_id = str(self.p["base_frame"])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self.p["resolution_m"])
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(self.p["origin_x_m"])
        msg.info.origin.position.y = float(self.p["origin_y_m"])
        msg.info.origin.position.z = float(self.p["floor_z_in_base_m"])
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        self.grid_pub.publish(msg)
        status = {
            "node": "anc_local_traversability",
            "profile": "REALISTIC_RGB_ONLY_STOP_ONLY",
            "status": gate, "reason": reason, "block_reasons": block_reasons,
            "map_frame": str(self.p["base_frame"]),
            "map_semantics": {"FREE": 0, "OCCUPIED": 100, "UNKNOWN": -1},
            "grid": {"resolution_m": float(self.p["resolution_m"]), "width": width,
                     "height": height, "origin_xy_m": [float(self.p["origin_x_m"]),
                     float(self.p["origin_y_m"])], "snapshot_only": True,
                     "stale_cells_fused": False},
            "counts": counts, "input_failures": failures,
            "input_contract": {
                "mask": str(self.p["mask_topic"]), "confidence": str(self.p["confidence_topic"]),
                "health": str(self.p["health_topic"]), "camera_info": str(self.p["camera_info_topic"]),
                "clock": str(self.p["clock_topic"]), "filtered_odometry": str(self.p["odometry_topic"]),
                "tf": f"{self.p['base_frame']} <- {self.p['camera_frame']}",
            },
            "camera_transform": camera_transform, "motion_authorized": False,
            "occupied_image_evidence_is_metric": False,
            "runtime_uses_world_map_or_truth": False,
        }
        state = String()
        state.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self.state_pub.publish(state)
        self.last_metrics = status

def main(args=None):
    rclpy.init(args=args)
    node = LocalTraversabilityNode()
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
