"""ROS port of object_detection_new_babu.py: YOLO obstacles + ArUco markers.

Subscribes /camera/image_raw and /camera/camera_info, publishes /vision/detections
(std_msgs/String JSON):
  {"aruco":   [{"id", "dist", "angle_deg", "zone"}],
   "objects": [{"cls", "conf", "box": [x1, y1, x2, y2], "dist", "zone", "status"}]}
Distances use the camera intrinsics from CameraInfo (no FOV guess). YOLO is optional:
without the ultralytics package the node still runs ArUco detection.
"""
import json
import math
import os
from collections import defaultdict, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from anc_autodrive.brain_node import image_to_bgr
from anc_autodrive.logic import FLAG_DISTANCE_M, WARN_DISTANCE_M

# Same priors as the original script (effective visible box heights, metres).
KNOWN_HEIGHTS_M = {
    "person": 1.55, "bicycle": 1.10, "car": 1.50, "motorcycle": 1.10, "bus": 3.20,
    "truck": 3.00, "traffic light": 0.75, "stop sign": 0.75, "fire hydrant": 0.75,
    "bench": 0.90, "dog": 0.55, "cat": 0.30, "chair": 0.95, "bottle": 0.25,
    "laptop": 0.25, "cell phone": 0.15, "backpack": 0.50,
}
DEFAULT_OBSTACLE_HEIGHT_M = 0.50


def zone(x, width):
    return "Left" if x < width / 3 else "Right" if x > 2 * width / 3 else "Center"


def status(d):
    if d is None:
        return "UNKNOWN"
    return "FLAGGED" if d <= FLAG_DISTANCE_M else "WARNING" if d <= WARN_DISTANCE_M else "SAFE"


class Vision(Node):
    def __init__(self):
        super().__init__("autodrive_vision")
        # Sim station uses DICT_4X4_50 (big cells read well at 320x240); the original
        # script's printed markers are DICT_7X7_1000 / 0.10 m - set these on the real car.
        self.marker_size = self.declare_parameter("aruco_marker_size_m", 0.375).value
        dict_name = self.declare_parameter("aruco_dictionary", "DICT_4X4_50").value
        self.use_yolo = self.declare_parameter("use_yolo", True).value
        self.model_name = self.declare_parameter("yolo_model", "yolo26s.pt").value
        self.imgsz = self.declare_parameter("yolo_imgsz", 320).value
        self.conf = self.declare_parameter("yolo_confidence", 0.45).value
        # Report only road-relevant classes (those with a height prior); COCO also has
        # "sports ball", "kite", ... which fire on scenery and mean nothing to the car.
        self.known_only = self.declare_parameter("known_classes_only", True).value
        # Size of the world relative to real life: 1.0 on the real car, 0.2 in the 1:5 sim town.
        self.object_scale = self.declare_parameter("object_scale", 1.0).value
        rate = self.declare_parameter("rate_hz", 3.0).value

        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
            self.detect_markers = cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters()).detectMarkers
        else:  # Ubuntu 24.04 / Raspberry Pi OS ship 4.6
            params = cv2.aruco.DetectorParameters_create()
            self.detect_markers = lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)
        self.model = self.load_yolo() if self.use_yolo else None
        self.K = None
        self.frame = None
        self.history = defaultdict(lambda: deque(maxlen=5))
        self.pub = self.create_publisher(String, "/vision/detections", 5)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 2)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 2)
        self.create_timer(1.0 / rate, self.process)

    def load_yolo(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().warn("ultralytics not installed: ArUco only (pip install ultralytics)")
            return None
        cv2.setNumThreads(1)
        cache = os.path.expanduser("~/.cache/anc_autodrive")
        os.makedirs(cache, exist_ok=True)
        os.chdir(cache)  # ultralytics downloads weights into the working directory
        import torch
        self.torch, self.threads = torch, self.declare_parameter("yolo_threads", 2).value
        self.get_logger().info(f"loading YOLO {self.model_name}")
        return YOLO(self.model_name)

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)

    def on_image(self, msg):
        self.frame = image_to_bgr(msg)
        self.frame_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def process(self):
        if self.frame is None or self.K is None:
            return
        frame, fx, stamp = self.frame, float(self.K[0, 0]), self.frame_stamp
        h, w = frame.shape[:2]
        out = {"stamp": stamp, "aruco": [], "objects": []}  # stamp = the frame these boxes belong to

        corners, ids, _ = self.detect_markers(frame)
        half = self.marker_size / 2.0
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
        for c, i in zip(corners, ids.flatten() if ids is not None else []):
            ok, _, t = cv2.solvePnP(obj, c[0], self.K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                tx, _, tz = (float(v) for v in t.flatten())
                out["aruco"].append({
                    "id": int(i), "dist": round(float(np.linalg.norm(t)), 3),
                    "angle_deg": round(math.degrees(math.atan2(tx, tz)), 1),
                    "zone": zone(float(c[0][:, 0].mean()), w)})

        if self.model is not None:
            # Ultralytics resets torch to 8 threads when its submodules load lazily; unlimited,
            # torch starves the simulator (seen: 500 % CPU for 0.5 Hz). Re-apply every call.
            self.torch.set_num_threads(self.threads)
            result = self.model(frame, imgsz=self.imgsz, conf=self.conf, iou=0.5,
                                max_det=20, verbose=False)[0]
            for box in result.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                x1, x2 = max(0, x1), min(w - 1, x2)
                y1, y2 = max(0, y1), min(h - 1, y2)
                cls = result.names[int(box.cls[0])]
                if self.known_only and cls not in KNOWN_HEIGHTS_M:
                    continue
                truncated = y1 <= 2 or y2 >= h - 2
                dist = None
                if y2 > y1:
                    real_h = KNOWN_HEIGHTS_M.get(cls, DEFAULT_OBSTACLE_HEIGHT_M) * self.object_scale
                    dist = real_h * fx / (y2 - y1)
                    if not truncated:
                        hist = self.history[f"{cls}:{(x1 + x2) // 80}"]
                        hist.append(dist)
                        dist = float(np.median(hist))
                out["objects"].append({
                    "cls": cls, "conf": round(float(box.conf[0]), 2), "box": [x1, y1, x2, y2],
                    "dist": None if dist is None else round(dist, 2),
                    "zone": zone((x1 + x2) / 2, w), "status": status(dist), "truncated": truncated})
        self.pub.publish(String(data=json.dumps(out)))


def main():
    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
