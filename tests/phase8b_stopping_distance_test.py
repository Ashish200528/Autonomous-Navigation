#!/usr/bin/env python3
"""Isolated low-speed S00 braking-distance measurements for Phase 8B.

This evaluator publishes directly to /cmd_vel only in a separate empty-world
Gazebo process. It is never part of the Phase 8 runtime or perception inputs.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from simulation_interfaces.srv import GetEntityState

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "simulation/scenarios/worlds/s00_empty_open_arena.sdf"
OUT_ROOT = ROOT / "docs/evidence/phase8b/stopping-distance"
DEFAULT_VELOCITIES = (0.02, 0.05, 0.10, 0.18)
DEFAULT_FRICTIONS = (1.2, 0.50)
DEFAULT_REPEATS = 3
ACCELERATION_WINDOW_S = 1.5
MAX_STOP_WINDOW_S = 8.0
SETTLE_SPEED_MPS = 0.005
SETTLE_ANGULAR_RADPS = 0.01
SETTLE_HOLD_S = 0.5


def yaw(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def pose_rate(previous, current):
    dt = float(current["sim_time_s"] - previous["sim_time_s"])
    if dt <= 1e-6:
        return None, None
    dx = current["x_m"] - previous["x_m"]
    dy = current["y_m"] - previous["y_m"]
    dyaw = math.atan2(math.sin(current["yaw_rad"] - previous["yaw_rad"]),
                      math.cos(current["yaw_rad"] - previous["yaw_rad"]))
    return math.hypot(dx, dy) / dt, abs(dyaw) / dt


class StopMonitor(Node):
    def __init__(self):
        super().__init__("anc_phase8b_stopping_distance_test")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Contacts, "/contacts", self.on_contacts, 20)
        self.state_client = self.create_client(GetEntityState, "/gzserver/get_entity_state")
        self.latest_odom = None
        self.odom_samples = []
        self.contact_events = []
        self.telemetry = []
        self.t0 = time.monotonic()

    def on_odom(self, msg):
        state = {
            "wall_elapsed_s": time.monotonic() - self.t0,
            "sim_time_s": msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
            "x_m": float(msg.pose.pose.position.x),
            "y_m": float(msg.pose.pose.position.y),
            "yaw_rad": yaw(msg.pose.pose.orientation),
            "linear_x_mps": float(msg.twist.twist.linear.x),
            "angular_z_radps": float(msg.twist.twist.angular.z),
        }
        self.latest_odom = state
        self.odom_samples.append(state)

    def on_contacts(self, msg):
        for contact in msg.contacts:
            event = {
                "wall_elapsed_s": time.monotonic() - self.t0,
                "collision1": contact.collision1.name,
                "collision2": contact.collision2.name,
                "max_depth_m": max(contact.depths, default=0.0),
            }
            self.contact_events.append(event)

    def publish(self, linear_x=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        self.cmd_pub.publish(msg)
        self.telemetry.append({
            "wall_elapsed_s": time.monotonic() - self.t0,
            "event": "command",
            "linear_x_mps": float(linear_x),
        })

    def spin_for_command(self, duration_s, linear_x):
        deadline = time.monotonic() + duration_s
        next_truth = 0.0
        truth_samples = []
        while time.monotonic() < deadline:
            self.publish(linear_x)
            rclpy.spin_once(self, timeout_sec=0.03)
            if time.monotonic() >= next_truth:
                truth_samples.append(self.get_truth())
                next_truth = time.monotonic() + 0.10
        return truth_samples

    def spin_zero(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.03)

    def get_truth(self, timeout_s=4.0):
        req = GetEntityState.Request()
        req.entity = "anc_robot"
        future = self.state_client.call_async(req)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
        if not future.done() or future.result() is None:
            raise RuntimeError("Gazebo evaluator truth query timed out")
        res = future.result()
        if res.result.result != 1:
            raise RuntimeError("Gazebo evaluator state failed: " + res.result.error_message)
        state = res.state
        q = state.pose.orientation
        return {
            "wall_elapsed_s": time.monotonic() - self.t0,
            "sim_time_s": state.header.stamp.sec + state.header.stamp.nanosec*1e-9,
            "x_m": float(state.pose.position.x),
            "y_m": float(state.pose.position.y),
            "z_m": float(state.pose.position.z),
            "yaw_rad": yaw(q),
            "linear_x_mps": float(state.twist.linear.x),
            "linear_y_mps": float(state.twist.linear.y),
            "angular_z_radps": float(state.twist.angular.z),
            "evaluator_only": True,
        }

    def wait_ready(self, timeout_s=45.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.latest_odom is not None and self.state_client.service_is_ready()
                    and self.count_publishers("/contacts") > 0):
                try:
                    truth = self.get_truth()
                    if abs(truth["z_m"]) < 0.5:
                        return truth
                except RuntimeError:
                    pass
            self.publish(0.0)
        raise RuntimeError("S00 robot, /odom, /contacts, or evaluator service not ready")

    def measure_trial(self, friction, target_mps, repeat):
        self.spin_zero(0.7)
        start_truth = self.get_truth()
        start_odom = dict(self.latest_odom)
        drive_truth_samples = self.spin_for_command(ACCELERATION_WINDOW_S, target_mps)
        before_zero_truth = self.get_truth()
        before_zero_odom = dict(self.latest_odom)
        contact_start = len(self.contact_events)
        drive_previous = drive_truth_samples[-1] if drive_truth_samples else start_truth
        pose_speed_at_zero, pose_yaw_rate_at_zero = pose_rate(
            drive_previous, before_zero_truth
        )
        if pose_speed_at_zero is None:
            pose_speed_at_zero = math.hypot(
                before_zero_truth["linear_x_mps"], before_zero_truth["linear_y_mps"]
            )
            pose_yaw_rate_at_zero = abs(before_zero_truth["angular_z_radps"])
        zero_wall = time.monotonic()
        self.publish(0.0)
        stop_samples = []
        slow_since = None
        first_below_threshold_s = None
        settled = False
        settle_previous = before_zero_truth
        deadline = zero_wall + MAX_STOP_WINDOW_S
        while time.monotonic() < deadline:
            self.publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.03)
            if time.monotonic() - getattr(self, "_last_truth_time", 0.0) >= 0.10:
                truth = self.get_truth()
                self._last_truth_time = time.monotonic()
                linear_pose_rate, angular_pose_rate = pose_rate(settle_previous, truth)
                truth["pose_derived_linear_speed_mps"] = linear_pose_rate
                truth["pose_derived_angular_speed_radps"] = angular_pose_rate
                stop_samples.append(truth)
                settle_previous = truth
                slow = (linear_pose_rate is not None
                        and linear_pose_rate <= SETTLE_SPEED_MPS
                        and angular_pose_rate is not None
                        and angular_pose_rate <= SETTLE_ANGULAR_RADPS)
                if slow:
                    if slow_since is None:
                        slow_since = time.monotonic()
                        first_below_threshold_s = truth["wall_elapsed_s"]
                    elif time.monotonic() - slow_since >= SETTLE_HOLD_S:
                        settled = True
                        break
                else:
                    slow_since = None
        after_truth = stop_samples[-1] if stop_samples else self.get_truth()
        after_odom = dict(self.latest_odom) if self.latest_odom else {}
        yaw0 = before_zero_truth["yaw_rad"]
        dx = after_truth["x_m"] - before_zero_truth["x_m"]
        dy = after_truth["y_m"] - before_zero_truth["y_m"]
        braking_distance = dx*math.cos(yaw0) + dy*math.sin(yaw0)
        odx = after_odom.get("x_m", before_zero_odom["x_m"]) - before_zero_odom["x_m"]
        ody = after_odom.get("y_m", before_zero_odom["y_m"]) - before_zero_odom["y_m"]
        odom_distance = odx*math.cos(yaw0) + ody*math.sin(yaw0)
        actual_before_zero_service_twist = math.hypot(
            before_zero_truth["linear_x_mps"], before_zero_truth["linear_y_mps"]
        )
        drive_dx = before_zero_truth["x_m"] - start_truth["x_m"]
        drive_dy = before_zero_truth["y_m"] - start_truth["y_m"]
        drive_distance = drive_dx*math.cos(start_truth["yaw_rad"]) + drive_dy*math.sin(
            start_truth["yaw_rad"]
        )
        drive_motion_confirmed = abs(drive_distance) >= 0.01
        stop_delay = max(0.0, after_truth["wall_elapsed_s"] -
                         (zero_wall - self.t0))
        contacts_during_trial = self.contact_events[contact_start:]
        record = {
            "friction_mu": friction,
            "target_velocity_mps": target_mps,
            "repeat": repeat,
            "settled": settled,
            "drive_motion_confirmed": drive_motion_confirmed,
            "drive_distance_truth_m": drive_distance,
            "settle_speed_source": "finite difference of consecutive Gazebo evaluator poses in simulation time",
            "settle_threshold_mps": SETTLE_SPEED_MPS,
            "settle_hold_s": SETTLE_HOLD_S,
            "zero_command_wall_elapsed_s": zero_wall - self.t0,
            "measured_speed_at_zero_mps": pose_speed_at_zero,
            "measured_speed_at_zero_service_twist_mps": actual_before_zero_service_twist,
            "pose_yaw_rate_at_zero_radps": pose_yaw_rate_at_zero,
            "braking_delay_wall_s": stop_delay,
            "delay_to_first_settle_threshold_s": (
                max(0.0, first_below_threshold_s - (zero_wall - self.t0))
                if first_below_threshold_s is not None else None
            ),
            "settle_hold_confirmation_s": (
                after_truth["wall_elapsed_s"] - first_below_threshold_s
                if first_below_threshold_s is not None and settled else None
            ),
            "max_measured_speed_during_drive_mps": max(
                (pose_rate(previous, current)[0] or 0.0
                 for previous, current in zip(
                     [start_truth, *drive_truth_samples[:-1]], drive_truth_samples
                 )),
                default=0.0,
            ),
            "braking_distance_truth_m": braking_distance,
            "braking_distance_odom_m": odom_distance,
            "truth_odom_braking_distance_error_m": odom_distance - braking_distance,
            "start_truth": start_truth,
            "before_zero_truth": before_zero_truth,
            "after_truth": after_truth,
            "start_odom": start_odom,
            "before_zero_odom": before_zero_odom,
            "after_odom": after_odom,
            "contact_events_during_trial": contacts_during_trial,
            "drive_truth_samples": drive_truth_samples,
            "stop_truth_samples": stop_samples,
            "no_non_ground_contacts_during_test": not any(
                "ground_plane" not in (c["collision1"] + " " + c["collision2"]).lower()
                for c in contacts_during_trial
            ),
        }
        self.telemetry.append({"event": "trial_result", **record})
        self.spin_zero(0.7)
        return record


def start_sim(friction, out_dir):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "74"
    env["GZ_PARTITION"] = f"anc_phase8b_stop_mu_{str(friction).replace('.', '_')}"
    env["ROS_LOG_DIR"] = str(out_dir / "ros_logs")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    cmd = [
        "ros2", "launch", "anc_bringup", "sim.launch.py",
        f"world_file:={WORLD}",
        "camera_enabled:=false",
        f"wheel_surface_mu:={friction}",
        "publish_known_start_tf:=false",
        "start_x:=0.0", "start_y:=0.0", "start_yaw:=0.0",
    ]
    log = (out_dir / f"gazebo-mu-{friction}.log").open("w")
    proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True, text=True)
    return proc, log, cmd


def _process_group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _wait_group_gone(pgid, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _process_group_alive(pgid):
            return True
        time.sleep(0.1)
    return not _process_group_alive(pgid)


def stop_sim(proc):
    pgid = proc.pid
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return {"signal": "SIGINT", "return_code": proc.poll(),
                "group_gone": True}
    try:
        proc.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        pass
    if _wait_group_gone(pgid, 4.0):
        return {"signal": "SIGINT", "return_code": proc.poll(),
                "group_gone": True}
    os.killpg(pgid, signal.SIGTERM)
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass
    if _wait_group_gone(pgid, 3.0):
        return {"signal": "SIGTERM", "return_code": proc.poll(),
                "group_gone": True}
    os.killpg(pgid, signal.SIGKILL)
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        pass
    return {"signal": "SIGKILL", "return_code": proc.poll(),
            "group_gone": not _process_group_alive(pgid)}


def summarize(records):
    grouped = {}
    for record in records:
        key = (str(record["friction_mu"]), str(record["target_velocity_mps"]))
        grouped.setdefault(key, []).append(record)
    output = []
    for (mu, speed), group in grouped.items():
        movers = [item for item in group if item["drive_motion_confirmed"]]
        valid = [item for item in movers if item["settled"]]
        distances = [item["braking_distance_truth_m"] for item in valid]
        delays = [item["braking_delay_wall_s"] for item in valid]
        actual = [item["measured_speed_at_zero_mps"] for item in movers]
        mean_distance = sum(distances)/len(distances) if distances else None
        output.append({
            "friction_mu": float(mu),
            "target_velocity_mps": float(speed),
            "repeat_count": len(group),
            "drive_motion_repeat_count": len(movers),
            "valid_settled_stop_repeat_count": len(valid),
            "all_moving_trials_settled": bool(movers) and len(valid) == len(movers),
            "no_motion_trials": len(group) - len(movers),
            "measured_speed_mean_mps": sum(actual)/len(actual) if actual else None,
            "measured_speed_max_mps": max(actual) if actual else None,
            "truth_braking_distance_mean_m": mean_distance,
            "truth_braking_distance_max_m": max(distances) if distances else None,
            "truth_braking_distance_min_m": min(distances) if distances else None,
            "truth_braking_distance_sample_sd_m": (
                math.sqrt(sum((d-mean_distance)**2 for d in distances)
                          / max(1, len(distances)-1))
                if len(distances) > 1 else (0.0 if distances else None)
            ),
            "braking_delay_max_s": max(delays) if delays else None,
        })
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default="run-20261102-pose-derived-mu050")
    parser.add_argument("--frictions", default=",".join(str(x) for x in DEFAULT_FRICTIONS))
    parser.add_argument("--velocities", default=",".join(str(x) for x in DEFAULT_VELOCITIES))
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS)
    args = parser.parse_args()
    frictions = tuple(float(x.strip()) for x in args.frictions.split(",") if x.strip())
    velocities = tuple(float(x.strip()) for x in args.velocities.split(",") if x.strip())
    repeats = int(args.repeats)
    if not frictions or any(x <= 0.0 for x in frictions):
        raise ValueError("frictions must be positive coefficients")
    if not velocities or any(x < 0.0 or x > 0.5 for x in velocities):
        raise ValueError("velocity targets must be in [0, 0.5] m/s")
    if repeats < 1 or repeats > 10:
        raise ValueError("repeats must be in [1, 10]")
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    run_dir = OUT_ROOT / args.run_id
    run_dir.mkdir(exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = "74"
    rclpy.init()
    records = []
    launches = []
    shutdowns = []
    telemetry_stream = []
    contact_events_all = []
    odom_sample_count = 0
    try:
        for friction in frictions:
            proc, log, command = start_sim(friction, run_dir)
            launches.append({"friction_mu": friction, "command": command})
            node = StopMonitor()
            try:
                node.wait_ready()
                node.spin_zero(1.0)
                for speed in velocities:
                    for repeat in range(1, repeats+1):
                        print(f"START mu={friction} speed={speed} repeat={repeat}", flush=True)
                        records.append(node.measure_trial(friction, speed, repeat))
            finally:
                try:
                    node.spin_zero(0.5)
                except Exception:
                    pass
                telemetry_stream.extend(
                    {"friction_mu": friction, **sample} for sample in node.telemetry
                )
                telemetry_stream.extend(
                    {"friction_mu": friction, "event": "odom", **sample}
                    for sample in node.odom_samples
                )
                telemetry_stream.extend(
                    {"friction_mu": friction, "event": "contact", **sample}
                    for sample in node.contact_events
                )
                contact_events_all.extend(node.contact_events)
                odom_sample_count += len(node.odom_samples)
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                shutdown = stop_sim(proc)
                log.close()
                shutdowns.append({"friction_mu": friction, **shutdown})
                rclpy.init()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    result = {
        "phase": "8B",
        "classification": "ISOLATED_SIMULATION_BRAKING_MEASUREMENT",
        "runtime_navigation_inputs": False,
        "physical_hardware_used": False,
        "world": str(WORLD.relative_to(ROOT)),
        "camera_enabled": False,
        "run_id": args.run_id,
        "friction_coefficients_mu": list(frictions),
        "velocity_targets_mps": list(velocities),
        "repeats_each": repeats,
        "settle_definition": {
            "linear_speed_mps_max": SETTLE_SPEED_MPS,
            "angular_speed_radps_max": SETTLE_ANGULAR_RADPS,
            "hold_wall_seconds": SETTLE_HOLD_S,
            "speed_source": "finite difference of consecutive Gazebo evaluator poses in simulation time",
            "stop_condition": "linear speed <= threshold and yaw rate <= threshold held for the configured duration",
            "braking_distance_definition": "evaluator pose displacement from zero-command pose to settled pose, projected along the initial heading",
        },
        "launches": launches,
        "shutdowns": shutdowns,
        "summary": summarize(records),
        "records": records,
        "contact_events": contact_events_all,
        "odom_sample_count": odom_sample_count,
        "telemetry_sample_count": len(telemetry_stream),
        "non_ground_contact_count": sum(
            "ground_plane" not in (event["collision1"] + " " + event["collision2"]).lower()
            for event in contact_events_all
        ),
        "evaluator_note": "Gazebo GetEntityState is test-only; this process is separate from the Phase 8 locked navigation runtime.",
    }
    result_path = run_dir / "stopping-distance-results.json"
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    csv_rows = []
    for row in telemetry_stream:
        csv_rows.append({
            key: (json.dumps(value, sort_keys=True) if isinstance(value, (dict, list))
                  else value)
            for key, value in row.items()
        })
    for row in records:
        csv_rows.append({
            "event": "trial_summary",
            "friction_mu": row["friction_mu"],
            "target_velocity_mps": row["target_velocity_mps"],
            "repeat": row["repeat"],
            "settled": row["settled"],
            "measured_speed_at_zero_mps": row["measured_speed_at_zero_mps"],
            "braking_delay_wall_s": row["braking_delay_wall_s"],
            "delay_to_first_settle_threshold_s": row["delay_to_first_settle_threshold_s"],
            "settle_hold_confirmation_s": row["settle_hold_confirmation_s"],
            "max_measured_speed_during_drive_mps": row["max_measured_speed_during_drive_mps"],
            "braking_distance_truth_m": row["braking_distance_truth_m"],
            "braking_distance_odom_m": row["braking_distance_odom_m"],
        })
    fields = sorted({key for row in csv_rows for key in row})
    with (run_dir / "stopping-distance-telemetry.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(csv_rows)
    print(json.dumps({"run_dir": str(run_dir), "shutdowns": shutdowns,
                      "summary": result["summary"]}, indent=2, sort_keys=True), flush=True)


if __name__ == "__main__":
    import csv
    main()
