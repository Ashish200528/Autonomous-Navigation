#!/usr/bin/env python3
"""Stationary EKF process pause: require stale-localization STOP and recovery."""
import json
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class LocalizationLossTest(Node):
    def __init__(self):
        super().__init__('phase8c_localization_loss_test')
        self.safety = None
        self.outputs = []
        self.create_subscription(String, '/realistic/safety_state',
                                 self.on_safety, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_output, 10)

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.safety = {'valid': False, 'failures': ['safety_json_invalid']}

    def on_output(self, msg):
        self.outputs.append({'wall_s': time.monotonic(),
                             'linear_x_mps': float(msg.linear.x),
                             'angular_z_radps': float(msg.angular.z)})
        self.outputs = self.outputs[-600:]

    def wait_ready(self, timeout_s=6.0):
        valid_since = None
        deadline = time.monotonic()+timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            ready = (self.safety and self.safety.get('valid') is True
                     and float(self.safety.get('max_linear_speed_mps', 0.0)) >= 0.05)
            if ready:
                valid_since = valid_since or time.monotonic()
                if time.monotonic()-valid_since >= 0.30:
                    return dict(self.safety)
            else:
                valid_since = None
        reason = None if self.safety is None else self.safety.get('reason')
        raise RuntimeError('initial_gate_not_ready:'+str(reason))

    def run(self, ekf_pid):
        initial = self.wait_ready()
        pause_wall = time.monotonic()
        stale_state = None
        zero_after_stale = False
        os.kill(ekf_pid, signal.SIGSTOP)
        try:
            deadline = pause_wall+1.25
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)
                failures = [] if not self.safety else [
                    str(x) for x in self.safety.get('failures', [])]
                if (stale_state is None and any(
                        x.startswith(('filtered_localization_stale',
                                      'filtered_localization_missing'))
                        for x in failures)):
                    stale_state = {'wall_s': time.monotonic(),
                                   **dict(self.safety)}
                if stale_state is not None and any(
                        output['wall_s'] >= stale_state['wall_s']
                        and abs(output['linear_x_mps']) < 1e-8
                        and abs(output['angular_z_radps']) < 1e-8
                        for output in self.outputs):
                    zero_after_stale = True
                    break
        finally:
            os.kill(ekf_pid, signal.SIGCONT)
        recovered = False
        deadline = time.monotonic()+6.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            if (self.safety and self.safety.get('valid') is True
                    and float(self.safety.get('max_linear_speed_mps', 0.0)) >= 0.05):
                recovered = True
                break
        all_zero = all(abs(x['linear_x_mps']) < 1e-8
                       and abs(x['angular_z_radps']) < 1e-8
                       for x in self.outputs)
        return {
            'classification': 'STATIONARY_FILTERED_LOCALIZATION_LOSS_AND_RECOVERY',
            'status': 'PASS' if stale_state and zero_after_stale and recovered and all_zero
                      else 'FAIL',
            'runtime_received_no_simulator_truth': True,
            'ekf_pid': ekf_pid,
            'initial_gate': initial,
            'loss_injection_wall_s': pause_wall,
            'first_localization_stale_state': stale_state,
            'zero_output_after_localization_stale': zero_after_stale,
            'gate_recovered_valid': recovered,
            'all_authoritative_outputs_zero': all_zero,
            'output_sample_count': len(self.outputs),
            'output_samples': self.outputs,
            'safety_state_at_end': self.safety,
        }


def main():
    if len(sys.argv) != 3:
        raise SystemExit('usage: phase8c_localization_loss_test.py EKF_PID OUTPUT_JSON')
    pid = int(sys.argv[1])
    output = Path(sys.argv[2])
    rclpy.init()
    node = LocalizationLossTest()
    result = {'status': 'FAIL'}
    try:
        result = node.run(pid)
    except Exception as exc:
        result = {'status': 'FAIL', 'reason': str(exc),
                  'safety_state': node.safety,
                  'output_samples': node.outputs}
    finally:
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
        print(json.dumps({k: v for k, v in result.items()
                          if k != 'output_samples'}, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
