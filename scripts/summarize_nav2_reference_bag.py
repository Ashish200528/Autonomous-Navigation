#!/usr/bin/env python3
"""Extract full goal feedback and measured motion from Nav2 trial evidence."""
import csv
import json
import math
import re
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs" / "evidence"
BAG = EVIDENCE / "nav2-reference-trial-bag"
GOAL_LOG = EVIDENCE / "nav2-reference-trial-goal.log"
PRE_LOG = EVIDENCE / "nav2-reference-trial-after-initialpose.log"
POST_LOG = EVIDENCE / "nav2-reference-trial-postflight.log"
MAP_CHECK = EVIDENCE / "nav2-reference-trial-map-check.json"
CSV_PATH = EVIDENCE / "nav2-reference-trial-telemetry.csv"
JSON_PATH = EVIDENCE / "nav2-reference-trial-summary.json"
TARGET = {"frame": "map", "x": 2.0, "y": -0.5, "yaw_rad": 0.0}
INITIAL = {"frame": "map", "x": -2.0, "y": -0.5, "yaw_rad": 0.0}

def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def section_xy(text, marker):
    part = text.split(marker, 1)[1].split("---", 1)[0]
    vals = {}
    for axis in ("x", "y", "z"):
        found = re.search(r"^\s*" + axis + r":\s*([-+0-9.eE]+)", part, re.M)
        if found:
            vals[axis] = float(found.group(1))
    return vals

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(BAG), storage_id="mcap"),
    rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                output_serialization_format="cdr"),
)
types = {item.name: item.type for item in reader.get_all_topics_and_types()}
counts, records, clocks = {}, [], []
odom, commands = [], {"/cmd_vel": [], "/cmd_vel_nav": []}
while reader.has_next():
    topic, raw, bag_ns = reader.read_next()
    counts[topic] = counts.get(topic, 0) + 1
    if topic not in ("/clock", "/odom", "/cmd_vel", "/cmd_vel_nav"):
        continue
    msg = deserialize_message(raw, get_message(types[topic]))
    row = {"topic": topic, "bag_timestamp_ns": int(bag_ns),
           "elapsed_s": None, "sim_time_s": None, "x": None, "y": None,
           "yaw_rad": None, "linear_x": None, "angular_z": None}
    if topic == "/clock":
        sim_s = msg.clock.sec + msg.clock.nanosec * 1e-9
        clocks.append((int(bag_ns), sim_s))
        row["sim_time_s"] = sim_s
    elif topic == "/odom":
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        sim_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        item = {"bag_ns": int(bag_ns), "sim_time_s": sim_s, "x": p.x,
                "y": p.y, "yaw_rad": yaw(q)}
        odom.append(item)
        row.update({"sim_time_s": sim_s, "x": p.x, "y": p.y, "yaw_rad": yaw(q),
                    "linear_x": msg.twist.twist.linear.x,
                    "angular_z": msg.twist.twist.angular.z})
    else:
        sample = {"bag_ns": int(bag_ns), "linear_x": msg.linear.x,
                  "angular_z": msg.angular.z}
        commands[topic].append(sample)
        row.update({"linear_x": msg.linear.x, "angular_z": msg.angular.z})
    records.append(row)

records.sort(key=lambda r: r["bag_timestamp_ns"])
if records:
    start_ns = records[0]["bag_timestamp_ns"]
    for row in records:
        row["elapsed_s"] = (row["bag_timestamp_ns"] - start_ns) * 1e-9
with CSV_PATH.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "topic", "bag_timestamp_ns", "elapsed_s", "sim_time_s",
        "x", "y", "yaw_rad", "linear_x", "angular_z"])
    writer.writeheader()
    writer.writerows(records)

odom.sort(key=lambda r: r["bag_ns"])
goal_text = GOAL_LOG.read_text(encoding="utf-8", errors="replace")
feedback = []
for block in goal_text.split("Feedback:")[1:]:
    p = re.search(r"position:\s*x:\s*([-+0-9.eE]+)\s*y:\s*([-+0-9.eE]+)", block)
    d = re.search(r"distance_remaining:\s*([-+0-9.eE]+)", block)
    stamp = re.search(r"stamp:\s*sec:\s*(\d+)\s*nanosec:\s*(\d+)", block)
    if p:
        feedback.append({
            "x": float(p.group(1)), "y": float(p.group(2)),
            "distance_remaining_m": float(d.group(1)) if d else None,
            "sim_time_s": (int(stamp.group(1)) + int(stamp.group(2)) * 1e-9)
                          if stamp else None,
        })
status = re.search(r"Goal finished with status:\s*(\w+)", goal_text)
error = re.search(r"error_code:\s*(-?\d+)", goal_text)
status = status.group(1) if status else "NOT_FOUND"
error = int(error.group(1)) if error else None
pre_text = PRE_LOG.read_text(encoding="utf-8", errors="replace")
post_text = POST_LOG.read_text(encoding="utf-8", errors="replace")
initial_amcl = section_xy(pre_text, "== amcl pose after initialpose ==")
final_amcl = section_xy(post_text, "== final AMCL xy ==")
final_odom = section_xy(post_text, "== final odometry xy ==")
goal_error = (math.dist((final_amcl["x"], final_amcl["y"]),
                        (TARGET["x"], TARGET["y"]))
              if "x" in final_amcl and "y" in final_amcl else None)
if feedback:
    feedback_distance = math.dist((feedback[0]["x"], feedback[0]["y"]),
                                  (feedback[-1]["x"], feedback[-1]["y"]))
    feedback_duration = (feedback[-1]["sim_time_s"] - feedback[0]["sim_time_s"]
                         if feedback[0]["sim_time_s"] is not None
                         and feedback[-1]["sim_time_s"] is not None else None)
else:
    feedback_distance = feedback_duration = None
if odom and "x" in final_odom and "y" in final_odom:
    odom_net = math.dist((odom[0]["x"], odom[0]["y"]),
                         (final_odom["x"], final_odom["y"]))
    odom_partial_net = math.dist((odom[0]["x"], odom[0]["y"]),
                                 (odom[-1]["x"], odom[-1]["y"]))
else:
    odom_net = odom_partial_net = None

def command_metrics(samples):
    moving = [s for s in samples if abs(s["linear_x"]) > 0.01
              or abs(s["angular_z"]) > 0.01]
    return {
        "sample_count": len(samples),
        "nonzero_sample_count": len(moving),
        "max_abs_linear_x_mps": max((abs(s["linear_x"]) for s in samples), default=0.0),
        "max_abs_angular_z_radps": max((abs(s["angular_z"]) for s in samples), default=0.0),
    }

bag_end_sim = max((clock[1] for clock in clocks), default=None)
feedback_end_sim = feedback[-1]["sim_time_s"] if feedback else None
bag_complete = (bag_end_sim is not None and feedback_end_sim is not None
                and bag_end_sim >= feedback_end_sim)
map_result = json.loads(MAP_CHECK.read_text(encoding="utf-8"))
action_ok = status == "SUCCEEDED" and error == 0
motion_ok = (feedback_distance is not None and feedback_distance >= 3.0
             and odom_net is not None and odom_net >= 3.0
             and command_metrics(commands["/cmd_vel"])["nonzero_sample_count"] > 0)
pose_ok = goal_error is not None and goal_error <= 0.30
telemetry_ok = all(counts.get(t, 0) > 0 for t in
                   ("/clock", "/odom", "/cmd_vel", "/cmd_vel_nav"))
overall = action_ok and motion_ok and pose_ok and telemetry_ok and map_result["all_samples_known_free"]
bag_duration = ((records[-1]["bag_timestamp_ns"] - records[0]["bag_timestamp_ns"]) * 1e-9
                if len(records) > 1 else 0.0)
summary = {
    "classification": "REFERENCE",
    "simulator": "Packaged TurtleBot3 Waffle, Nav2 Jazzy, Gazebo Harmonic headless",
    "navigation_inputs": "Static occupancy map and simulated laser scan; Gazebo ground-truth pose was not used for navigation",
    "initial_pose_map": INITIAL,
    "initial_amcl_pose_map": initial_amcl,
    "goal_map": TARGET,
    "action": {
        "name": "/navigate_to_pose", "status": status, "error_code": error,
        "feedback_pose_count": len(feedback),
        "first_feedback_xy": ({"x": feedback[0]["x"], "y": feedback[0]["y"]} if feedback else None),
        "last_feedback_xy": ({"x": feedback[-1]["x"], "y": feedback[-1]["y"]} if feedback else None),
        "feedback_net_map_displacement_m": feedback_distance,
        "feedback_duration_s": feedback_duration,
        "last_distance_remaining_m": (feedback[-1]["distance_remaining_m"] if feedback else None),
    },
    "final_amcl_pose_map": final_amcl,
    "final_amcl_xy_error_to_goal_m": goal_error,
    "final_odometry_pose": final_odom,
    "odometry_net_displacement_from_recorded_initial_to_postflight_m": odom_net,
    "odometry_partial_bag_displacement_m": odom_partial_net,
    "commands": {topic: command_metrics(samples) for topic, samples in commands.items()},
    "map_corridor": {
        "all_samples_known_free": map_result["all_samples_known_free"],
        "sample_count": map_result["line_samples"],
        "free_sample_count": map_result["free_sample_count"],
        "resolution_m": map_result["resolution_m_per_cell"],
    },
    "recording": {
        "format": "MCAP",
        "duration_s": bag_duration,
        "bag_end_sim_time_s": bag_end_sim,
        "last_goal_feedback_sim_time_s": feedback_end_sim,
        "covers_full_goal": bag_complete,
        "partial_capture_note": (None if bag_complete else
            "The MCAP stopped before the final action feedback. Full motion and result are taken from the action feedback log and live postflight pose snapshots."),
        "topic_message_counts": counts,
        "bag_directory": "docs/evidence/nav2-reference-trial-bag",
        "telemetry_csv": "docs/evidence/nav2-reference-trial-telemetry.csv",
    },
    "lifecycle_and_costmap_evidence": {
        "postflight_log": "docs/evidence/nav2-reference-trial-postflight.log",
        "after_initial_pose_log": "docs/evidence/nav2-reference-trial-after-initialpose.log",
        "map_corridor_log": "docs/evidence/nav2-reference-trial-map-check.log",
    },
    "checks": {
        "action_succeeded_and_error_code_zero": action_ok,
        "map_corridor_known_free": map_result["all_samples_known_free"],
        "odometry_and_velocity_motion_observed": motion_ok,
        "final_amcl_xy_error_at_most_0_30m": pose_ok,
        "telemetry_topics_recorded": telemetry_ok,
    },
    "overall_reference_goal_pass": overall,
}
JSON_PATH.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2))
