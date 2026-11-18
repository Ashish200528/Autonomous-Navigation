#!/usr/bin/env python3
import json
import statistics
import sys
import time
from collections import Counter, deque
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def stamp(msg):
    return float(msg.sec) + float(msg.nanosec) * 1e-9


class Probe(Node):
    def __init__(self):
        super().__init__('phase8c_camera_pair_probe')
        self.infos = deque(maxlen=64)
        self.masks = deque(maxlen=64)
        self.states = deque(maxlen=64)
        self.info_count = self.mask_count = self.state_count = 0
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/perception/image_mask', self.mask_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/realistic/navigation_state', self.state_cb, 10)

    def info_cb(self, msg):
        self.info_count += 1
        self.infos.append({'stamp': stamp(msg.header.stamp), 'wall': time.monotonic()})

    def mask_cb(self, msg):
        self.mask_count += 1
        self.masks.append({'stamp': stamp(msg.header.stamp), 'wall': time.monotonic()})

    def state_cb(self, msg):
        self.state_count += 1
        try:
            value = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            value = {'input_failures': ['json_invalid']}
        mask_stamp = self.masks[-1]['stamp'] if self.masks else None
        latest_info = self.infos[-1]['stamp'] if self.infos else None
        closest_info = None
        nearest_delta = None
        if mask_stamp is not None and self.infos:
            closest_info = min(self.infos, key=lambda item: abs(item['stamp']-float(mask_stamp)))['stamp']
            nearest_delta = abs(closest_info-float(mask_stamp))
        self.states.append({
            'wall': time.monotonic(), 'input_failures': value.get('input_failures', []),
            'mask_stamp_s': mask_stamp, 'latest_info_stamp_s': latest_info,
            'latest_delta_s': (None if latest_info is None or mask_stamp is None
                               else abs(latest_info-float(mask_stamp))),
            'closest_info_stamp_s': closest_info, 'nearest_delta_s': nearest_delta,
        })


def main():
    output = Path(sys.argv[1])
    rclpy.init()
    node = Probe()
    deadline = time.monotonic()+4.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        deltas = [v['latest_delta_s'] for v in node.states if v['latest_delta_s'] is not None]
        nearest = [v['nearest_delta_s'] for v in node.states if v['nearest_delta_s'] is not None]
        result = {
            'duration_s': 4.0, 'camera_info_count': node.info_count,
            'mask_count': node.mask_count, 'navigation_state_count': node.state_count,
            'latest_info_to_mask_delta_s': {
                'count': len(deltas), 'max': max(deltas, default=None),
                'median': statistics.median(deltas) if deltas else None,
                'under_0_10_s': sum(v <= 0.10 for v in deltas),
            },
            'nearest_info_to_mask_delta_s': {
                'count': len(nearest), 'max': max(nearest, default=None),
                'median': statistics.median(nearest) if nearest else None,
                'under_0_10_s': sum(v <= 0.10 for v in nearest),
            },
            'input_failure_counts': dict(Counter(
                failure for state in node.states
                for failure in state['input_failures'])),
            'samples': list(node.states),
            'runtime_uses_simulator_truth': False,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
        print(json.dumps({k: result[k] for k in result if k != 'samples'},
                         indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
