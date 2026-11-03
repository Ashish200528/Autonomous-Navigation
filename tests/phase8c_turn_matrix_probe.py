#!/usr/bin/env python3
"""Measured preproduction matrix probe; never enables production authority."""
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data
from phase8c_rotation_trial import RotationTrial, angle_delta, pose_yaw


class MatrixProbe(RotationTrial):
    def __init__(self, obstacles):
        super().__init__()
        self.obstacles = obstacles
        self.local_map = None
        self.odom = None
        self.create_subscription(OccupancyGrid, '/realistic/local_map/lidar',
                                 lambda m: setattr(self, 'local_map', m), 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, '/realistic/local_map/lidar', 10)

    def clearance(self, pose):
        distances = []
        for x, y, yaw, sx, sy in self.obstacles:
            dx, dy = pose['x_m']-x, pose['y_m']-y
            lx = math.cos(yaw)*dx + math.sin(yaw)*dy
            ly = -math.sin(yaw)*dx + math.cos(yaw)*dy
            qx, qy = abs(lx)-sx/2, abs(ly)-sy/2
            distances.append(math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)-0.263592962728522)
        return min(distances)

    def execute(self, speed, duration, fault=None, fault_pid=None):
        self.wait_gate(0.10, timeout_s=20)
        start = self.get_truth()
        odom_start = None if self.odom is None else pose_yaw(self.odom.pose.pose.orientation)
        contact_start = self.non_ground_contact_records
        samples = [start]
        output_start_wall = time.monotonic()
        fault_wall = None
        observed_zero = None
        zero_pose = None
        min_clearance = self.clearance(start)
        max_drift = 0.0
        unsafe = None
        map_fault = None
        try:
            deadline = output_start_wall+duration
            while time.monotonic() < deadline:
                now = time.monotonic()
                if fault and fault_wall is None and now-output_start_wall >= 1.0:
                    fault_wall = now
                    if fault_pid:
                        os.kill(fault_pid, signal.SIGSTOP)
                    if fault == 'unknown_swept_region':
                        if self.local_map is None:
                            raise RuntimeError('missing_local_map_for_fault')
                        map_fault = self.local_map
                        map_fault.data = [-1] * len(map_fault.data)
                if map_fault is not None:
                    self.map_pub.publish(map_fault)
                if fault != 'command_timeout' or fault_wall is None:
                    self.publish_request(speed)
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.monotonic()-samples[-1]['wall_s'] >= 0.10:
                    pose = self.get_truth()
                    samples.append(pose)
                    min_clearance = min(min_clearance, self.clearance(pose))
                    max_drift = max(max_drift, math.hypot(pose['x_m']-start['x_m'], pose['y_m']-start['y_m']))
                    if min_clearance < 0.10 or max_drift > 0.10 or self.non_ground_contact_records > contact_start:
                        unsafe = 'clearance_drift_or_contact_limit'
                        break
                if fault_wall is not None:
                    candidates = [s for s in self.outputs if s['wall_s'] >= fault_wall
                                  and abs(s['angular_z_radps']) < 1e-8 and abs(s['linear_x_mps']) < 1e-8]
                    if candidates:
                        observed_zero = candidates[0]['wall_s']
                        zero_pose = self.get_truth()
                        break
                elif self.safety and not self.safety.get('authorized'):
                    unsafe = 'gate_revoked_during_motion:' + str(self.safety.get('reason'))
                    break
            stop_start = time.monotonic() if fault_wall is None else fault_wall
            self.publish_request(0.0)
            if zero_pose is None:
                deadline = time.monotonic()+0.5
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    candidates = [s for s in self.outputs if s['wall_s'] >= stop_start and abs(s['angular_z_radps']) < 1e-8]
                    if candidates:
                        observed_zero = candidates[0]['wall_s']
                        break
                zero_pose = self.get_truth()
            final, settled, stop_samples, _ = self.settle_from_pose(zero_pose)
            elapsed = time.monotonic()-stop_start
            residual = abs(angle_delta(final['yaw_rad'],zero_pose['yaw_rad']))
            drift = max(max_drift, math.hypot(final['x_m']-start['x_m'],final['y_m']-start['y_m']))
            min_clearance = min(min_clearance, *(self.clearance(p) for p in stop_samples))
            yaw_travel = angle_delta(zero_pose['yaw_rad'],start['yaw_rad'])
            sim_elapsed = zero_pose['sim_stamp_s']-start['sim_stamp_s']
            outputs = [s for s in self.outputs if s['wall_s'] >= output_start_wall]
            no_contacts = self.non_ground_contact_records-contact_start == 0
            safe_stop = observed_zero is not None and settled and elapsed <= 0.75 and residual <= 0.05
            return dict(trial_pass=bool(safe_stop and unsafe is None and (not fault or fault_wall is not None) and no_contacts and min_clearance >= .10
                                       and drift <= .10 and abs(yaw_travel)>.005 and yaw_travel*speed>0),
                        production_angular_authority_enabled=False, fault=fault, fault_injected=fault_wall is not None,
                        requested_angular_speed_radps=speed,
                        measured_average_yaw_rate_radps=yaw_travel/sim_elapsed,
                        yaw_travel_rad=yaw_travel, center_drift_m=drift,
                        minimum_clearance_m=min_clearance,
                        residual_yaw_after_zero_output_rad=residual,
                        zero_request_to_settled_wall_s=elapsed,
                        stop_time_reference='fault_injection' if fault_wall is not None else 'zero_request',
                        zero_output_response_s=None if observed_zero is None else observed_zero-stop_start,
                        safe_stop_observed=safe_stop, unsafe_reason=unsafe,
                        odom_truth_heading_delta_error_rad=None if self.odom is None or odom_start is None else
                        abs(angle_delta(angle_delta(pose_yaw(self.odom.pose.pose.orientation), odom_start), angle_delta(final['yaw_rad'],start['yaw_rad']))),
                        contact_records_non_ground=self.non_ground_contact_records-contact_start,
                        real_time_factor=(final['sim_stamp_s']-start['sim_stamp_s'])/(time.monotonic()-output_start_wall),
                        truth_samples=samples, stop_truth_samples=stop_samples, output_samples=outputs,
                        safety_state_at_stop=self.safety)
        finally:
            self.publish_request(0.0)
            if fault_pid:
                os.kill(fault_pid, signal.SIGCONT)


def main():
    spec = json.loads(Path(sys.argv[1]).read_text())
    rclpy.init()
    node = MatrixProbe(spec['obstacles'])
    result = dict(case_id=spec['case_id'], trial_pass=False, production_angular_authority_enabled=False)
    result['trials'] = trials = []
    try:
        if not 0 < spec['duration'] <= 8 or not spec['speeds'] or any(
                not .02 <= abs(speed) <= .10 for speed in spec['speeds']):
            raise ValueError('calibration_spec_outside_bounded_envelope')
        for speed in spec['speeds']:
            trials.append(node.execute(speed, spec['duration'],spec.get('fault'),spec.get('fault_pid')))
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        names = [p.node_name for p in publishers]
        result.update(trials=trials, cmd_vel_publisher_names=names,
                      trial_pass=(all(t['trial_pass'] for t in trials)
                                  and names == ['anc_safety_command_guard']))
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        node.spin_for(.5,0.0)
        Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
