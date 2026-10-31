#!/usr/bin/env python3
"""Check the packaged static map cells along the S00 reference corridor."""
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "evidence" / "nav2-reference-trial-map-check.json"
START = (-2.0, -0.5)
GOAL = (2.0, -0.5)

class MapReader(Node):
    def __init__(self):
        super().__init__("anc_reference_map_check")
        self.message = None
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self.receive, qos)

    def receive(self, msg):
        self.message = msg

rclpy.init()
node = MapReader()
deadline = time.monotonic() + 12.0
while rclpy.ok() and node.message is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.message is None:
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit("Timed out waiting for transient-local /map")
msg = node.message
info = msg.info
resolution = float(info.resolution)
ox = float(info.origin.position.x)
oy = float(info.origin.position.y)

def sample(x, y):
    col = math.floor((x - ox) / resolution)
    row = math.floor((y - oy) / resolution)
    if col < 0 or row < 0 or col >= info.width or row >= info.height:
        return {"x": x, "y": y, "cell": None, "value": None}
    index = row * info.width + col
    return {"x": round(x, 4), "y": round(y, 4),
            "column": col, "row": row, "value": int(msg.data[index])}

distance = math.dist(START, GOAL)
steps = math.ceil(distance / (resolution / 2.0))
samples = [sample(START[0] + (GOAL[0] - START[0]) * i / steps,
                  START[1] + (GOAL[1] - START[1]) * i / steps)
           for i in range(steps + 1)]
values = [s["value"] for s in samples]
result = {
    "map_frame": "map",
    "map_dimensions_cells": {"width": int(info.width), "height": int(info.height)},
    "resolution_m_per_cell": resolution,
    "origin_xy": {"x": ox, "y": oy},
    "map_data_length": len(msg.data),
    "start_xy": {"x": START[0], "y": START[1]},
    "goal_xy": {"x": GOAL[0], "y": GOAL[1]},
    "line_sample_spacing_m": distance / steps,
    "line_samples": len(samples),
    "unique_occupancy_values": sorted(set(values)),
    "free_sample_count": sum(v == 0 for v in values),
    "unknown_sample_count": sum(v == -1 for v in values),
    "occupied_sample_count": sum(v is not None and v > 0 for v in values),
    "out_of_map_sample_count": sum(v is None for v in values),
    "all_samples_known_free": all(v == 0 for v in values),
}
OUTPUT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
if not result["all_samples_known_free"]:
    raise SystemExit(2)
