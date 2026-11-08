"""Autonomous driving brain: camera lane following + ultrasonic safety + servo scan.

Inputs  : /camera/image_raw, /ultrasonic/range, /odometry/filtered, /vision/detections
Outputs : /cmd_vel, /servo/pan_cmd (rad, + = left), /autodrive/hud, /autodrive/state

States
  DRIVE      follow the lane; slow when something is within WARN distance
  SCAN       obstacle ahead -> stop, pan camera left, right, centre and judge each side
  OVERTAKE   change to the free lane, pass, come back (odometry heading + distance)
  NUDGE      both sides blocked -> reverse 15 cm (heading held), rescan (max 3x)
  WAIT       still blocked after the nudges -> wait, then rescan (or resume if the path cleared)
  STOP_SIGN  YOLO "stop sign" close -> full stop for a few seconds, then go
  YIELD      YOLO "person" in the path -> wait until the road is clear (never overtake people)
  STATION    ArUco station marker close -> stop for a while, count a lap
  ARRIVED    target laps done -> parked
  HOLD       autostart:=false -> stand still until `ros2 param set /autodrive_brain go true`
Any stale sensor or an ultrasonic reading under FLAG distance forces zero speed.
"""
import json
from collections import deque
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float64, String

from anc_autodrive.logic import (FLAG_DISTANCE_M, WARN_DISTANCE_M, crossing_fraction, find_lane, lane_error,
                                 obstacle_fraction, overtake_plan, wrap)


def image_to_bgr(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return img[..., ::-1].copy() if msg.encoding == "rgb8" else img.copy()


# Ultralytics' default box palette (BGR), so the HUD reads like a YOLO output frame.
YOLO_COLOURS = [(56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207),
                (10, 249, 72), (23, 204, 146), (134, 219, 61), (52, 147, 26), (187, 212, 0),
                (168, 153, 44), (255, 194, 0), (147, 69, 52), (255, 115, 100), (236, 24, 0),
                (255, 56, 132), (133, 0, 82), (255, 56, 203), (200, 149, 255), (199, 55, 255)]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Brain(Node):
    def __init__(self):
        super().__init__("autodrive_brain")
        p = self.declare_parameter
        self.speed = p("cruise_speed", 0.216).value
        self.slow_speed = p("slow_speed", 0.096).value
        self.kp = p("steer_kp", 1.1).value
        self.kd = p("steer_kd", 0.15).value
        self.max_w = p("max_turn_rate", 0.9).value
        self.decide_m = p("decide_distance_m", 0.8).value
        self.flag_m = p("flag_distance_m", FLAG_DISTANCE_M).value
        self.warn_m = p("warn_distance_m", WARN_DISTANCE_M).value
        self.pan = p("scan_pan_rad", 1.2).value
        self.dwell = p("scan_dwell_s", 1.6).value
        self.blocked_frac = p("blocked_fraction", 0.01).value
        self.block_m = p("yolo_block_distance_m", 2.0).value  # farther objects don't block a lane
        # Image rows judged when looking sideways. Below ~62% the camera sees the car's own
        # body; recalibrate on the real car after mounting the camera.
        self.scan_roi = (p("scan_roi_top", 0.37).value, p("scan_roi_bottom", 0.62).value)
        self.allow_right = p("allow_right_overtake", False).value
        self.station_id = p("station_marker_id", 0).value
        self.station_m = p("station_detect_distance_m", 3.0).value
        self.station_offset = p("station_stop_before_m", 0.5).value
        self.station_goal = None  # (x, y, metres to travel) once the marker was seen
        self.station_wait = p("station_wait_s", 5.0).value
        self.target_laps = p("target_laps", 2).value  # 0 = drive forever
        self.stale_s = p("sensor_timeout_s", 0.6).value
        self.pass_m = p("overtake_pass_m", 2.0).value
        self.max_nudges = p("max_nudges", 3).value
        # No rear sensor: keep each back-up short.
        self.nudge_m = p("nudge_reverse_m", 0.15).value
        self.nudges = 0
        self.stop_sign_m = p("stop_sign_stop_distance_m", 1.0).value
        self.stop_sign_wait = p("stop_sign_wait_s", 3.0).value
        self.stop_sign_cooldown = 0.0
        self.yield_m = p("yield_distance_m", 1.3).value
        self.caution_m = p("pedestrian_caution_m", 1.6).value

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.servo_pub = self.create_publisher(Float64, "/servo/pan_cmd", 10)
        self.hud_pub = self.create_publisher(Image, "/autodrive/hud", 2)
        self.state_pub = self.create_publisher(String, "/autodrive/state", 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 2)
        self.create_subscription(Range, "/ultrasonic/range", self.on_range, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)
        self.create_subscription(String, "/vision/detections", self.on_vision, 5)

        self.frame = None
        self.frame_t = self.range_t = -1e9
        self.range = math.inf
        self.pose = None  # (x, y, yaw)
        self.lane = (None, None, None, None)
        self.lane_seen_t = -1e9
        self.on_crossing = False
        self.cross_yaw = None  # heading held while driving over a crossing
        self.last_err = 0.0
        self.vision = {"aruco": [], "objects": []}
        self.vision_t = -1e9
        self.recent = deque(maxlen=20)  # (stamp, frame): HUD draws YOLO boxes on their own frame

        self.state = "DRIVE" if p("autostart", True).value else "HOLD"
        self.declare_parameter("go", False)
        self.note = ""
        self.t0 = 0.0
        self.servo = 0.0
        self.scan = []          # [(pan, label)] still to visit
        self.scan_result = {}
        self.plan = []
        self.step_start = None
        self.laps = 0
        self.station_cooldown = 0.0
        self.v = self.w = 0.0
        self.create_timer(0.05, self.tick)
        self.create_timer(0.1, self.publish_hud)

    # ---------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, msg):
        img = image_to_bgr(msg)
        if img is None:
            return
        self.frame, self.frame_t = img, self.now()
        self.recent.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, img))
        if abs(self.servo) < 0.05:  # lane is only meaningful with the camera looking ahead
            self.lane = find_lane(img, self.lane[3])
            # Any zebra in the lane band: bars and the stop line can pass for lane lines, so
            # the lane estimate is not trusted until the crossing is behind us.
            self.on_crossing = crossing_fraction(img) > 0.08
            if self.lane[0] is not None or self.on_crossing:
                self.lane_seen_t = self.frame_t

    def on_range(self, msg):
        self.range = msg.range if math.isfinite(msg.range) else math.inf
        self.range_t = self.now()

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_vision(self, msg):
        try:
            self.vision, self.vision_t = json.loads(msg.data), self.now()
        except ValueError:
            pass

    # --------------------------------------------------------------- helpers
    def go(self, state, note=""):
        if state != self.state:
            self.get_logger().info(f"{self.state} -> {state} {note}")
        self.state, self.note, self.t0 = state, note, self.now()

    def set_servo(self, angle):
        self.servo = angle
        self.servo_pub.publish(Float64(data=float(angle)))

    def nearest(self, cls, zones=("Left", "Center", "Right")):
        """Closest fresh YOLO detection of a class (estimated metres), or None."""
        if self.now() - self.vision_t > 1.5:
            return None
        d = [o["dist"] for o in self.vision.get("objects", [])
             if o["cls"] == cls and o["zone"] in zones and o.get("dist")]
        return min(d) if d else None

    def side_is_clear(self, label):
        if self.frame is None:
            return False, 1.0, []
        frac = obstacle_fraction(self.frame, self.scan_roi)
        fresh = self.vision_t > self.t0 + 0.3  # YOLO result taken after the servo settled
        w = self.frame.shape[1]
        # Looking left, the image's right quarter is the road ahead - i.e. the obstacle we
        # want to pass (YOLO calls the barrier a "bench"). Mirror for the right look.
        ahead = (lambda cx: cx > 0.75 * w) if label == "left" else (lambda cx: cx < 0.25 * w)
        body_y = self.scan_roi[1] * self.frame.shape[0]  # below this the camera sees our own car
        seen = [o["cls"] for o in self.vision.get("objects", [])
                if o.get("dist") is not None and o["dist"] < self.block_m
                and not ahead((o["box"][0] + o["box"][2]) / 2)
                and (o["box"][1] + o["box"][3]) / 2 < body_y] if fresh else []
        return frac < self.blocked_frac and not seen, frac, seen

    # ------------------------------------------------------------------ loop
    def tick(self):
        now = self.now()
        self.v = self.w = 0.0
        stale = (now - self.frame_t > self.stale_s or now - self.range_t > self.stale_s
                 or self.pose is None)
        if stale:
            self.note = "waiting for camera / ultrasonic / odometry"
        else:
            getattr(self, "do_" + self.state.lower())(now)
        if self.range < self.flag_m and self.v > 0.0:  # hard rule, every state
            self.v, self.note = 0.0, f"EMERGENCY STOP {self.range * 100:.0f} cm"
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(self.v), float(self.w)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=json.dumps({
            "state": self.state, "note": self.note, "v": self.v, "w": self.w,
            "ultrasonic_m": None if math.isinf(self.range) else round(self.range, 3),
            "servo_rad": self.servo, "laps": self.laps})))

    def do_drive(self, now):
        self.set_servo(0.0)
        person = self.nearest("person", ("Center",))
        anyone = self.nearest("person")
        stop_sign = self.nearest("stop sign", ("Center", "Right"))
        if person is not None and person < self.yield_m:
            self.go("YIELD", f"pedestrian {person:.1f} m ahead")
            return
        if self.range < self.decide_m and anyone is not None and anyone < self.caution_m:
            # Something stepped in front while a person is close by: assume it is them.
            self.go("YIELD", f"pedestrian stepped out ({self.range:.2f} m)")
            return
        if stop_sign is not None and stop_sign < self.stop_sign_m and now > self.stop_sign_cooldown:
            self.go("STOP_SIGN", f"stop sign at {stop_sign:.1f} m")
            return
        if self.range < self.decide_m:
            self.start_scan(f"obstacle at {self.range:.2f} m")
            return
        # A roadside sign leaves a low camera's view well before the car reaches it, so the
        # stop point is measured once when the marker is seen, then reached by odometry.
        for m in self.vision.get("aruco", []) if now - self.vision_t < 1.0 else []:
            if (m["id"] == self.station_id and m["dist"] < self.station_m
                    and now > self.station_cooldown and self.station_goal is None):
                ahead = m["dist"] * math.cos(math.radians(m["angle_deg"])) - self.station_offset
                self.station_goal = (self.pose[0], self.pose[1], max(0.0, ahead))
                self.get_logger().info(f"station marker {m['id']} seen at {m['dist']:.2f} m, "
                                       f"stopping in {ahead:.2f} m")
        if self.station_goal is not None:
            x0, y0, ahead = self.station_goal
            if math.hypot(self.pose[0] - x0, self.pose[1] - y0) >= ahead:
                self.station_goal = None
                self.go("STATION", "arrived at station")
                return
        centre = self.lane[0]
        if self.on_crossing:
            if self.cross_yaw is None:
                self.cross_yaw = self.pose[2]
            self.v = self.slow_speed
            self.w = max(-0.4, min(0.4, 2.0 * wrap(self.cross_yaw - self.pose[2])))
            self.note = "zebra crossing - holding heading"
            return
        self.cross_yaw = None
        if centre is None:
            if now - self.lane_seen_t > 1.0:
                self.note = "LANE LOST - stopped"
                return
            err = self.last_err  # brief dropout: keep the last correction, go slow
            speed = self.slow_speed
        else:
            err = lane_error(centre, self.frame.shape[1])
            speed = self.speed
        if self.station_goal is not None:
            speed = min(speed, self.slow_speed)
        caution = ""
        if self.range < self.warn_m:
            caution = f"obstacle {self.range:.2f} m - slowing"
        elif anyone is not None and anyone < self.caution_m:
            caution = "pedestrians nearby - slowing"
        elif stop_sign is not None and now > self.stop_sign_cooldown:
            caution = "stop sign ahead - slowing"
        if caution:
            speed = min(speed, self.slow_speed)
        derr, self.last_err = err - self.last_err, err
        self.w = max(-self.max_w, min(self.max_w, -(self.kp * err + self.kd * derr / 0.05)))
        self.v = speed * (1.0 - 0.5 * min(1.0, abs(err)))  # slow in sharp corrections
        self.note = caution or f"lane error {err:+.2f}"
        if self.station_goal is not None:
            self.note = "station ahead - slowing to stop"

    def do_stop_sign(self, now):
        left = self.stop_sign_wait - (now - self.t0)
        self.note = f"STOP sign - full stop {max(0.0, left):.1f} s"
        if left <= 0:
            self.stop_sign_cooldown = now + 10.0  # the same sign is still in view when leaving
            self.go("DRIVE", "stop complete, proceeding")

    def do_yield(self, now):
        person = self.nearest("person")
        if person is not None and person < self.caution_m:
            self.t0 = now  # the clear-for-1.5 s timer restarts while someone is close
        clear_for = now - self.t0
        self.note = (f"yielding to pedestrian {person:.1f} m" if person is not None and person < self.caution_m
                     else f"road clear {clear_for:.1f}/1.5 s")
        if clear_for > 1.5:  # anything still in front is now handled by DRIVE (scan/overtake)
            self.go("DRIVE", "pedestrian has crossed")

    def backed_m(self):
        return self.nudges * self.nudge_m  # distance gained by backing up, not by the obstacle leaving

    def do_scan(self, now):
        if self.range > self.warn_m + self.backed_m():
            self.nudges = 0
            self.go("DRIVE", "path cleared")
            return
        if now - self.t0 < self.dwell:
            self.note = f"looking {self.scan[0][1]}"
            return
        pan, label = self.scan.pop(0)
        if label != "centre":
            self.scan_result[label] = self.side_is_clear(label)
            self.get_logger().info(f"scan {label}: clear={self.scan_result[label][0]} "
                                   f"colour={self.scan_result[label][1]:.3f} yolo={self.scan_result[label][2]}")
        if self.scan:
            self.set_servo(self.scan[0][0])
            self.t0 = now
            return
        left, right = self.scan_result["left"][0], self.scan_result["right"][0]
        side = 1 if left else (-1 if right and self.allow_right else 0)
        if side == 0 and self.nudges < self.max_nudges:
            # Backing up gives the camera a wider view past the obstacle before rescanning.
            self.nudges += 1
            yaw = self.pose[2]
            self.plan = [("reverse", self.nudge_m, yaw)]
            self.step_start = None
            self.go("NUDGE", f"both sides blocked - backing up ({self.nudges}/{self.max_nudges})")
            return
        if side == 0:
            self.go("WAIT", "both sides blocked")
            return
        self.plan = overtake_plan(self.pose[2], side, pass_m=self.pass_m)
        self.pass_half = None
        self.step_start = None
        self.go("OVERTAKE", "left lane free" if side > 0 else "right side free")

    def start_scan(self, note):
        self.scan = [(self.pan, "left"), (-self.pan, "right"), (0.0, "centre")]
        self.scan_result = {}
        self.set_servo(self.pan)
        self.go("SCAN", note)

    def do_wait(self, now):
        if self.range > self.decide_m + 0.2 + self.backed_m():
            self.nudges = 0
            self.go("DRIVE", "path cleared")
        elif now - self.t0 > 3.0:
            self.start_scan("rescan")

    def do_nudge(self, now):
        self.do_overtake(now)  # same step executor; the end of the plan triggers a rescan

    def do_overtake(self, now):
        if not self.plan:
            if self.state == "NUDGE":
                self.start_scan("rescan after backing up")
                return
            self.lane_seen_t = now
            self.nudges = 0
            self.go("DRIVE", "back in lane")
            return
        step = self.plan[0]
        x, y, yaw = self.pose
        if self.step_start is None:
            self.step_start = (x, y)
        if step[0] == "turn":
            err = wrap(step[1] - yaw)
            if abs(err) < 0.04:
                self.plan.pop(0)
                self.step_start = None
                return
            self.w = math.copysign(max(0.25, min(self.max_w, 2.0 * abs(err))), err)
            self.note = f"turning {math.degrees(err):+.0f} deg"
        else:
            done = math.hypot(x - self.step_start[0], y - self.step_start[1])
            if done >= step[1]:
                self.plan.pop(0)
                self.step_start = None
                return
            self.v = -self.slow_speed if step[0] == "reverse" else self.speed
            self.w = max(-self.max_w, min(self.max_w, 2.0 * wrap(step[2] - yaw)))
            self.note = (f"backing up {done:.2f}/{step[1]:.2f} m" if step[0] == "reverse"
                         else f"overtaking {done:.2f}/{step[1]:.2f} m")
            if step[0] == "pass" and self.frame is not None and abs(self.servo) < 0.05:
                # In the left lane the markings are mirrored (white edge left, yellow right):
                # flip the image and the normal lane finder follows the overtaking lane, so
                # a heading error at the start cannot make the car drift off the road.
                w_px = self.frame.shape[1]
                c, _, _, self.pass_half = find_lane(cv2.flip(self.frame, 1), self.pass_half)
                if c is not None:
                    err = lane_error(w_px - c, w_px)
                    self.w = max(-self.max_w, min(self.max_w, -self.kp * err))
                    self.note = f"overtaking in left lane {done:.2f}/{step[1]:.2f} m"

    def do_station(self, now):
        left = self.station_wait - (now - self.t0)
        self.note = f"station stop {max(0.0, left):.0f} s"
        if left > 0:
            return
        self.laps += 1
        if self.target_laps and self.laps >= self.target_laps:
            self.go("ARRIVED", f"{self.laps} laps done")
            return
        self.station_cooldown = now + 20.0
        self.go("DRIVE", f"lap {self.laps}")

    def do_hold(self, now):
        self.set_servo(0.0)
        self.note = "holding - set param go:=true to start"
        if self.get_parameter("go").value:
            self.go("DRIVE", "go")

    def do_arrived(self, now):
        self.note = f"mission complete: {self.laps} laps"

    # ------------------------------------------------------------------- HUD
    def publish_hud(self):
        if self.frame is None:
            return
        # YOLO runs at ~2 Hz: drawing its boxes on the newest frame makes them trail moving
        # objects. Show the frame the detections were computed on instead (~0.5 s behind).
        frame = self.frame
        stamp = self.vision.get("stamp")
        if stamp is not None and self.recent:
            frame = min(self.recent, key=lambda sf: abs(sf[0] - stamp))[1]
        k = max(1, 640 // frame.shape[1])  # draw at 640 px wide whatever the camera size
        img = cv2.resize(frame, None, fx=k, fy=k) if k > 1 else frame.copy()
        h, w = img.shape[:2]
        centre, yx, wx, _ = find_lane(frame, self.lane[3]) if frame is not self.frame else self.lane
        if abs(self.servo) < 0.05:
            band = int(h * 0.72)
            for x, col in ((yx, (0, 220, 255)), (wx, (255, 255, 255))):
                if x is not None:
                    cv2.circle(img, (int(k * x), band), 7, col, 2, cv2.LINE_AA)
            if centre is not None:
                cv2.line(img, (w // 2, h), (int(k * centre), band), (0, 255, 0), 3, cv2.LINE_AA)
        for o in self.vision.get("objects", []):  # Ultralytics-style box with a filled label
            x1, y1, x2, y2 = (k * v for v in o["box"])
            col = YOLO_COLOURS[sum(map(ord, o["cls"])) % len(YOLO_COLOURS)]
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 2, cv2.LINE_AA)
            label = f"{o['cls']} {o['conf']:.2f}" + (f" | {o['dist']:.1f} m" if o.get("dist") else "")
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = y1 - th - 8 if y1 - th - 8 > 58 else y1
            cv2.rectangle(img, (x1 - 1, ty), (x1 + tw + 8, ty + th + 8), col, -1)
            cv2.putText(img, label, (x1 + 4, ty + th + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
        for m in self.vision.get("aruco", []):
            cv2.putText(img, f"ArUco #{m['id']} {m['dist']:.2f} m", (10, h - 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"YOLO: {len(self.vision.get('objects', []))} objects", (w - 200, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 255), 1, cv2.LINE_AA)

        bar = img.copy()
        cv2.rectangle(bar, (0, 0), (w, 58), (18, 18, 22), -1)
        cv2.rectangle(bar, (0, h - 30), (w, h), (18, 18, 22), -1)
        cv2.addWeighted(bar, 0.75, img, 0.25, 0, img)
        state_col = {"DRIVE": (0, 220, 0), "OVERTAKE": (255, 200, 0), "ARRIVED": (255, 120, 255),
                     "STOP_SIGN": (60, 60, 255), "YIELD": (60, 60, 255), "NUDGE": (255, 200, 0)}
        cv2.putText(img, self.state, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    state_col.get(self.state, (0, 165, 255)), 2, cv2.LINE_AA)
        cv2.putText(img, self.note[:58], (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(img, f"{self.v:.2f} m/s", (w - 120, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        r = self.range
        r_col = (0, 0, 255) if r < self.flag_m else (0, 165, 255) if r < self.warn_m else (0, 200, 0)
        r_txt = "clear" if math.isinf(r) else f"{r * 100:.0f} cm"
        cv2.putText(img, f"ULTRASONIC {r_txt}", (10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.55, r_col, 2, cv2.LINE_AA)
        cv2.putText(img, f"SERVO {math.degrees(self.servo):+.0f} deg   LAPS {self.laps}",
                    (w - 300, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.height, msg.width, msg.encoding, msg.step = h, w, "rgb8", w * 3
        msg.data = img[..., ::-1].tobytes()
        self.hud_pub.publish(msg)


def main():
    rclpy.init()
    node = Brain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # after Ctrl+C the context is gone; the bridge/DiffDrive then stop anyway
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
