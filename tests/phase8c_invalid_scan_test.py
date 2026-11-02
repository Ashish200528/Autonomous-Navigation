#!/usr/bin/env python3
"""Inject one malformed raw LaserScan and verify fail-closed recovery."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


class InvalidScanTest(Node):
    def __init__(self):
        super().__init__('phase8c_invalid_scan_test')
        self.raw_pub = self.create_publisher(
            LaserScan, '/realistic/scan/raw', qos_profile_sensor_data)
        self.safety = None
        self.scan_health = None
        self.outputs = []
        self.sim_clock = None
        self.create_subscription(Clock, "/clock", self.on_clock,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/realistic/safety_state',
                                 self.on_safety, 10)
        self.create_subscription(String, '/realistic/scan/health',
                                 self.on_scan_health, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_output, 10)

    def on_clock(self, msg):
        self.sim_clock = msg.clock

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.safety = {'valid': False, 'failures': ['safety_json_invalid']}

    def on_scan_health(self, msg):
        try:
            self.scan_health = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.scan_health = {'valid': False, 'reason': 'scan_health_json_invalid'}

    def on_output(self, msg):
        self.outputs.append({'wall_s': time.monotonic(),
                             'linear_x_mps': float(msg.linear.x),
                             'angular_z_radps': float(msg.angular.z)})
        self.outputs = self.outputs[-600:]

    def spin_until(self, predicate, timeout_s):
        deadline = time.monotonic()+timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            if predicate():
                return True
        return bool(predicate())

    def wait_gate_ready(self):
        valid_since = None
        deadline = time.monotonic()+6.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            valid = (self.safety and self.safety.get('valid') is True
                     and float(self.safety.get('max_linear_speed_mps', 0.0)) >= 0.05)
            if valid:
                valid_since = valid_since or time.monotonic()
                if time.monotonic()-valid_since >= 0.30:
                    return dict(self.safety)
            else:
                valid_since = None
        reason = None if self.safety is None else self.safety.get('reason')
        raise RuntimeError('initial_gate_not_ready:'+str(reason))

    def inject_invalid_scan(self):
        scan = LaserScan()
        if self.sim_clock is None:
            raise RuntimeError('simulation_clock_not_ready_for_injection')
        scan.header.stamp = self.sim_clock
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0*math.pi)/359.0
        scan.time_increment = 0.0
        scan.scan_time = 0.2
        scan.range_min = 0.12
        scan.range_max = 6.0
        scan.ranges = [float("nan")]*360
        scan.intensities = []
        self.raw_pub.publish(scan)

    def run(self):
        initial = self.wait_gate_ready()
        raw_scan_subscriber_matched = self.spin_until(
            lambda: self.raw_pub.get_subscription_count() > 0, 4.0)
        if not raw_scan_subscriber_matched:
            raise RuntimeError("raw_scan_subscriber_unavailable")
        injection_wall = time.monotonic()
        injected_count = 0
        next_injection_wall = injection_wall
        invalid_health = None
        invalid_gate = None
        zero_after_gate_invalid = False
        recovered = False
        deadline = injection_wall+5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            now = time.monotonic()
            if invalid_health is None and now >= next_injection_wall:
                self.inject_invalid_scan()
                injected_count += 1
                next_injection_wall = now+0.05
            if (self.scan_health and self.scan_health.get('valid') is not True
                    and invalid_health is None):
                invalid_health = dict(self.scan_health)
            failures = [] if not self.safety else [str(x) for x in self.safety.get('failures', [])]
            scan_failure = any(x.startswith((
                'scan_invalid', 'scan_health', 'scan_sim_', 'measured_scan_rate_',
                'usable_beam_', 'lidar_local_map_', 'lidar_map_health_',
                'lidar_map_invalid')) for x in failures)
            if (self.safety and self.safety.get('valid') is not True
                    and scan_failure and invalid_gate is None):
                invalid_gate = {'wall_s': time.monotonic(), **dict(self.safety)}
            if (invalid_gate is not None and self.safety
                    and self.safety.get('valid') is True):
                recovered = True
            if invalid_gate is not None and any(
                    v['wall_s'] >= invalid_gate['wall_s']
                    and abs(v['linear_x_mps']) < 1e-8
                    and abs(v['angular_z_radps']) < 1e-8 for v in self.outputs):
                zero_after_gate_invalid = True
            if recovered and zero_after_gate_invalid:
                break
        all_zero = all(abs(v['linear_x_mps']) < 1e-8
                       and abs(v['angular_z_radps']) < 1e-8 for v in self.outputs)
        result = {
            'classification': 'STATIONARY_ONE_FRAME_INVALID_LIDAR_SCAN_AND_RECOVERY',
            'status': 'PASS' if (invalid_health is not None and invalid_gate is not None
                                 and zero_after_gate_invalid and recovered and all_zero)
                      else 'FAIL',
            'runtime_received_no_simulator_truth': True,
            'injection_wall_s': injection_wall,
            'raw_scan_subscriber_matched': raw_scan_subscriber_matched,
            'invalid_scan_injection_count': injected_count,
            'initial_gate': initial,
            'invalid_scan_health': invalid_health,
            'invalid_safety_state': invalid_gate,
            'gate_recovered_valid': recovered,
            'zero_output_after_invalid_gate': zero_after_gate_invalid,
            'all_authoritative_outputs_zero': all_zero,
            'output_sample_count': len(self.outputs),
            'output_samples': self.outputs,
        }
        return result


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: phase8c_invalid_scan_test.py OUTPUT_JSON')
    output = Path(sys.argv[1])
    rclpy.init()
    node = InvalidScanTest()
    result = {'status': 'FAIL'}
    try:
        result = node.run()
    except Exception as exc:
        result = {'status': 'FAIL', 'reason': str(exc),
                  'safety_state': node.safety,
                  'scan_health': node.scan_health,
                  'output_samples': node.outputs}
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
        print(json.dumps({k: v for k, v in result.items()
                          if k != 'output_samples'}, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
