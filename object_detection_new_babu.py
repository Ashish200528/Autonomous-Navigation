"""Autonomous Navigation: Combined YOLO Obstacle Detection and ArUco Marker Tracker.

- Camera: cv2.VideoCapture(2)
- ArUco Detection: Displays Marker ID, 3D Pose, Distance, and Angle.
- Obstacle Detection: Detects generic obstacles and computes distance.
- Thresholds:
    * Distance <= 30 cm (0.30 m): FLAGGED / Emergency Stop (Red)
    * Distance <= 1.2 m: WARNING / Caution (Orange)
    * Distance > 1.2 m: SAFE / Clear (Green)
"""

import argparse
import math
import time
import cv2
import numpy as np
from ultralytics import YOLO
from collections import defaultdict, deque

# ==============================================================================
# 1. THRESHOLDS & GEOMETRY CONFIGURATION
# ==============================================================================

CAMERA_INDEX = 0

# Distance thresholds in metres
FLAG_DISTANCE_M = 0.30   # <= 30 cm -> FLAGGED / Critical Stop
WARN_DISTANCE_M = 1.20   # <= 1.20 m -> WARNING / Caution

CONFIDENCE_THRESHOLD = 0.45
# Most USB webcams are 75-95 degrees horizontally. 60 degrees causes a
# systematic over-estimate (often close to 3x). Override with --hfov or --focal.
CAMERA_HFOV_DEG = 90.0
FOCAL_LENGTH_PX = None

# Default height for generic obstacles (metres)
DEFAULT_OBSTACLE_HEIGHT_M = 0.50

# Known object heights in metres (COCO classes for more accurate distance prior)
KNOWN_HEIGHTS_M = {
    # COCO boxes usually include a small amount of background above/below a
    # person and frequently miss feet/limbs. This effective height is more
    # representative of the visible box used by monocular ranging.
    "person": 1.55,
    "bicycle": 1.10,
    "car": 1.50,
    "motorcycle": 1.10,
    "bus": 3.20,
    "truck": 3.00,
    "traffic light": 0.75,
    "stop sign": 0.75,
    "fire hydrant": 0.75,
    "bench": 0.90,
    "dog": 0.55,
    "cat": 0.30,
    "chair": 0.95,
    "bottle": 0.25,
    "laptop": 0.25,
    "cell phone": 0.15,
    "backpack": 0.50,
}

# ArUco marker configuration
MARKER_SIZE = 0.10  # 100 mm / 0.10 metres

ARUCO_DICTIONARIES = {
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_6X6_250":  cv2.aruco.DICT_6X6_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

aruco_params = cv2.aruco.DetectorParameters()
aruco_detectors = {
    name: cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dict_id), aruco_params)
    for name, dict_id in ARUCO_DICTIONARIES.items()
}


# ==============================================================================
# 2. CAMERA & POSE ESTIMATION HELPERS
# ==============================================================================

def open_webcam(index=CAMERA_INDEX):
    camera = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if camera.isOpened():
        # Prevent the driver from queuing old frames, which causes visible lag.
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FPS, 30)
        return camera
    return None


def get_camera_matrix(frame_width, frame_height, focal_length_px):
    """Constructs camera intrinsic matrix scaled to current capture resolution."""
    focal_length = focal_length_px
    cx = frame_width / 2.0
    cy = frame_height / 2.0
    cam_matrix = np.array([
        [focal_length, 0,            cx],
        [0,            focal_length, cy],
        [0,            0,             1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    return cam_matrix, dist_coeffs


def focal_length_from_fov(frame_width_px, hfov_deg):
    """Approximate focal length in pixels from horizontal FOV."""
    return (frame_width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def estimate_pose_single_markers(corners, marker_size, camera_matrix, dist_coeffs):
    """Estimate pose of single markers compatible with all OpenCV versions."""
    if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
        return cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
    
    half_size = marker_size / 2.0
    obj_points = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)
    
    rvecs, tvecs = [], []
    for c in corners:
        ret, rvec, tvec = cv2.solvePnP(obj_points, c[0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ret:
            ret, rvec, tvec = cv2.solvePnP(obj_points, c[0], camera_matrix, dist_coeffs)
        rvecs.append(rvec.reshape(1, 3))
        tvecs.append(tvec.reshape(1, 3))
    return np.array(rvecs), np.array(tvecs), None


def estimate_obstacle_distance(bbox_height_px, class_name, focal_length_px):
    """Return estimated distance in metres for general obstacles."""
    real_height = KNOWN_HEIGHTS_M.get(class_name, DEFAULT_OBSTACLE_HEIGHT_M)
    if real_height is None or bbox_height_px <= 0 or focal_length_px is None:
        return None
    return (real_height * focal_length_px) / bbox_height_px


def get_horizontal_zone(x_center, frame_width):
    """Determine relative position: Left, Center, or Right."""
    left_boundary = frame_width / 3.0
    right_boundary = 2.0 * frame_width / 3.0
    if x_center < left_boundary:
        return "Left"
    elif x_center > right_boundary:
        return "Right"
    else:
        return "Center"


def is_truncated(y1, y2, frame_height, margin=2):
    """True if bounding box is clipped by frame edges."""
    return y1 <= margin or y2 >= frame_height - margin


def get_obstacle_status(distance_m):
    """Return (status, color) based on 30cm flag and warning thresholds."""
    if distance_m is None:
        return "UNKNOWN", (200, 200, 200)
    if distance_m <= FLAG_DISTANCE_M:
        return "FLAGGED", (0, 0, 255)       # Red - Critical (<= 30 cm)
    if distance_m <= WARN_DISTANCE_M:
        return "WARNING", (0, 165, 255)     # Orange - Warning (> 30 cm & <= 1.2 m)
    return "SAFE", (0, 200, 0)              # Green - Safe (> 1.2 m)


# ==============================================================================
# 3. DRAWING HELPERS
# ==============================================================================

def draw_label(frame, x1, y1, label, color):
    """Draw high-contrast detection badge."""
    (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    text_top = max(y1 - text_h - baseline - 4, 0)
    cv2.rectangle(
        frame,
        (x1, text_top),
        (x1 + text_w + 6, text_top + text_h + baseline + 4),
        color,
        cv2.FILLED,
    )
    cv2.putText(
        frame,
        label,
        (x1 + 3, text_top + text_h + 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


# ==============================================================================
# 4. MAIN DETECTION PIPELINE
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calibrate",
        nargs=2,
        metavar=("CLASS", "DISTANCE_M"),
        help="Calibration mode: place known object at measured distance in metres.",
    )
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX,
                        help=f"Camera index (default: {CAMERA_INDEX}).")
    parser.add_argument("--hfov", type=float, default=CAMERA_HFOV_DEG,
                        help="Measured horizontal camera FOV in degrees (default: 90).")
    parser.add_argument("--focal", type=float, default=None,
                        help="Calibrated focal length in pixels; overrides --hfov.")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="YOLO inference size. 640 is a good accuracy/speed balance.")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="YOLO weights (yolov8n.pt for speed, yolov8s.pt for higher accuracy).")
    parser.add_argument("--yolo26n", action="store_true",
                        help="Use YOLO26n weights (yolo26n.pt). Ultralytics downloads them if absent.")
    args = parser.parse_args()

    calib_class = None
    calib_distance = None
    if args.calibrate:
        calib_class = args.calibrate[0]
        calib_distance = float(args.calibrate[1])
        if calib_class not in KNOWN_HEIGHTS_M:
            raise SystemExit(f"'{calib_class}' not in KNOWN_HEIGHTS_M.")

    model_path = "yolo26n.pt" if args.yolo26n else args.model
    print(f"[INFO] Loading YOLO model: {model_path}")
    model = YOLO(model_path)

    cap = open_webcam(args.camera)
    if cap is None:
        raise RuntimeError("No webcam could be opened. Please verify USB connection.")

    # Capture at the same 640-class workload used by inference. Capturing
    # 1280x720 and then downscaling wastes CPU time and increases latency.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    focal_length_px = args.focal or FOCAL_LENGTH_PX
    distance_history = defaultdict(lambda: deque(maxlen=5))
    frame_index = 0
    prev_time = time.time()

    print("\n" + "="*60)
    print("  AUTONOMOUS CAR - COMBINED OBSTACLE & ARUCO TRACKER")
    print("  Flag Threshold : <= 30 cm (Critical Stop)")
    print("  Warning Range  : 30 cm to 1.20 m")
    print("  Press 'Q' or ESC in window to exit.")
    print("="*60 + "\n")

    while True:
        try:
            ret, frame = cap.read()
        except cv2.error as error:
            print(f"Camera read error: {error}")
            break

        if not ret or frame is None:
            print("Failed to read a frame from the webcam.")
            break

        frame_height, frame_width = frame.shape[:2]
        frame_index += 1
        if focal_length_px is None:
            focal_length_px = focal_length_from_fov(frame_width, args.hfov)
        camera_matrix, dist_coeffs = get_camera_matrix(frame_width, frame_height, focal_length_px)

        closest_entity = None
        min_distance = float("inf")
        detected_aruco_list = []

        # ----------------------------------------------------------------------
        # A. ARUCO MARKER DETECTION
        # ----------------------------------------------------------------------
        aruco_corners = []
        aruco_ids = []

        # Scan dictionaries (primary 7x7 first)
        corners, ids, _ = aruco_detectors["DICT_7X7_1000"].detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            aruco_corners = corners
            aruco_ids = ids
        elif frame_index % 10 == 0:
            # Alternate dictionaries are expensive and only needed when the
            # primary dictionary does not find a marker. Probe periodically
            # so the normal object-detection loop stays responsive.
            for name, detector in aruco_detectors.items():
                if name == "DICT_7X7_1000":
                    continue
                c, i, _ = detector.detectMarkers(frame)
                if i is not None and len(i) > 0:
                    aruco_corners = c
                    aruco_ids = i
                    break

        if aruco_ids is not None and len(aruco_ids) > 0:
            ids_flat = np.array(aruco_ids).flatten()
            cv2.aruco.drawDetectedMarkers(frame, aruco_corners, aruco_ids)

            rvecs, tvecs, _ = estimate_pose_single_markers(
                aruco_corners,
                MARKER_SIZE,
                camera_matrix,
                dist_coeffs
            )

            for i in range(len(ids_flat)):
                m_id = int(ids_flat[i])
                tvec = np.array(tvecs[i]).flatten()
                rvec = np.array(rvecs[i]).flatten()

                tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])
                aruco_dist = math.sqrt(tx * tx + ty * ty + tz * tz)
                angle_deg = math.degrees(math.atan2(tx, tz)) if tz != 0 else 0.0

                center_x = int(np.mean(aruco_corners[i][0][:, 0]))
                center_y = int(np.mean(aruco_corners[i][0][:, 1]))
                zone = get_horizontal_zone(center_x, frame_width)
                status, color = get_obstacle_status(aruco_dist)

                detected_aruco_list.append((m_id, aruco_dist, zone, angle_deg))

                if aruco_dist < min_distance:
                    min_distance = aruco_dist
                    closest_entity = (f"ArUco #{m_id}", zone, aruco_dist, status)

                # Draw coordinate axes for 3D orientation
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE * 0.5)

                # ArUco Label
                if status == "FLAGGED":
                    label = f"FLAG ArUco #{m_id} [{zone}]: {aruco_dist*100:.0f}cm"
                elif status == "WARNING":
                    label = f"WARN ArUco #{m_id} [{zone}]: {aruco_dist:.2f}m"
                else:
                    label = f"ArUco #{m_id} [{zone}]: {aruco_dist:.2f}m ({angle_deg:+.0f}deg)"

                # Bounding box coordinates around marker
                c_pts = aruco_corners[i][0].astype(int)
                bx1, by1 = int(np.min(c_pts[:, 0])), int(np.min(c_pts[:, 1]))
                draw_label(frame, bx1, by1, label, color)

        # ----------------------------------------------------------------------
        # B. YOLO OBSTACLE DETECTION
        # ----------------------------------------------------------------------
        # Keep the default path CPU/Uno-friendly; Ultralytics will use an
        # accelerator automatically when available.  Lowering max_det avoids
        # spending time on irrelevant background boxes.
        results = model(frame, imgsz=args.imgsz, conf=CONFIDENCE_THRESHOLD,
                        iou=0.50, max_det=30, agnostic_nms=False, verbose=False)
        result = results[0]

        for box in result.boxes:
            confidence = float(box.conf[0])
            if confidence < CONFIDENCE_THRESHOLD:
                continue

            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            class_name = result.names[int(box.cls[0])]
            # Ultralytics normally returns image-space coordinates. Clamp them
            # to the captured frame so a malformed/out-of-scale prediction
            # cannot produce an impossible near distance.
            x1 = max(0, min(x1, frame_width - 1))
            x2 = max(0, min(x2, frame_width - 1))
            y1 = max(0, min(y1, frame_height - 1))
            y2 = max(0, min(y2, frame_height - 1))
            bbox_height = y2 - y1
            x_center = (x1 + x2) // 2

            raw_distance = estimate_obstacle_distance(bbox_height, class_name, focal_length_px)
            # Reject clipped boxes and stabilize monocular estimates over time.
            track_key = f"{class_name}:{x_center // 80}"
            if raw_distance is not None and not is_truncated(y1, y2, frame_height):
                history = distance_history[track_key]
                history.append(raw_distance)
                distance = float(np.median(history))
            else:
                distance = raw_distance
            truncated = is_truncated(y1, y2, frame_height)
            zone = get_horizontal_zone(x_center, frame_width)
            status, color = get_obstacle_status(distance)

            if calib_class is not None and class_name == calib_class and not truncated:
                implied_focal = (bbox_height * calib_distance) / KNOWN_HEIGHTS_M[class_name]
                print(f"Calib: height={bbox_height}px -> FOCAL_LENGTH_PX = {implied_focal:.1f}")

            # Obstacle label formatting
            if distance is None:
                label = f"Obstacle [{zone}]"
            elif truncated:
                label = f"Obstacle [{zone}]: >{distance:.2f}m?"
            elif status == "FLAGGED":
                label = f"FLAG [{zone}]: {distance * 100:.0f}cm"
            elif status == "WARNING":
                label = f"WARN [{zone}]: {distance:.2f}m"
            else:
                label = f"Obstacle [{zone}]: {distance:.2f}m"

            if distance is not None and distance < min_distance:
                min_distance = distance
                closest_entity = ("Obstacle", zone, distance, status)

            box_color = (200, 200, 200) if truncated else color
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            draw_label(frame, x1, y1, label, box_color)

        # ----------------------------------------------------------------------
        # C. AUTONOMOUS HUD & TELEMETRY OVERLAY
        # ----------------------------------------------------------------------
        # Visual zone lane boundaries
        cv2.line(frame, (frame_width // 3, 0), (frame_width // 3, frame_height), (70, 70, 70), 1)
        cv2.line(frame, (2 * frame_width // 3, 0), (2 * frame_width // 3, frame_height), (70, 70, 70), 1)

        # FPS
        now = time.time()
        fps = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now

        # Top Header Bar
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (frame_width, 70), (18, 18, 22), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.line(frame, (0, 70), (frame_width, 70), (60, 60, 75), 1)

        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (frame_width - 130, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        # Telemetry info
        if closest_entity:
            name, zone, dist, status = closest_entity
            if status == "FLAGGED":
                # Prominent RED Stop alert
                cv2.rectangle(frame, (8, 8), (470, 62), (0, 0, 220), cv2.FILLED)
                cv2.putText(
                    frame,
                    f"FLAGGED: {name} ({dist*100:.0f}cm) [{zone}] - STOP!",
                    (15, 42),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            elif status == "WARNING":
                cv2.putText(
                    frame,
                    f"WARNING: {name} at {dist:.2f}m [{zone}]",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    frame,
                    f"Safe: Nearest {name} at {dist:.2f}m [{zone}]",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
        else:
            cv2.putText(
                frame,
                "STATUS: PATH CLEAR",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        # ArUco Specific Info Line in Header
        if detected_aruco_list:
            aruco_info_str = " | ".join(
                [f"ArUco #{m[0]}: {m[1]:.2f}m ({m[2]})" for m in detected_aruco_list]
            )
            cv2.putText(
                frame,
                aruco_info_str,
                (15, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                "ArUco: None detected",
                (15, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (160, 160, 160),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow("Autonomous Car - Obstacle & ArUco Tracker", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q") or key == ord("Q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
