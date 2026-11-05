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
from std_msgs.msg import String
from rclpy.node import Node
from simulation_interfaces.srv import GetEntityState
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from phase8c_turn_stop_contract import evaluate_stop
from phase8c_rotation_trial import RotationTrial, angle_delta, pose_yaw, pose_rate


def stop_timing_metrics(samples, trigger_wall_s, confirmation_wall_s):
    """Separate a verified slow interval from the observation hold, using response times.

    Invalid/duplicate-time rate samples and any rebound reset the candidate.
    This diagnostic never changes the original trial's acceptance decision.
    """
    candidate = None
    for sample in samples:
        v = sample.get('pose_derived_linear_speed_mps')
        w = sample.get('pose_derived_angular_speed_radps')
        slow = (v is not None and w is not None and math.isfinite(v)
                and math.isfinite(w) and 0 <= v <= .005 and 0 <= w <= .01)
        if not slow:
            candidate = None
        elif candidate is None:
            candidate = sample['wall_s']
    verified = (candidate is not None and samples
                and samples[-1]['wall_s']-candidate >= .5)
    return dict(first_verified_slow_sample_after_trigger_s=(
                    candidate-trigger_wall_s if verified else None),
                observation_hold_after_first_slow_s=(
                    confirmation_wall_s-candidate if verified else None),
                confirmed_settled_after_trigger_s=confirmation_wall_s-trigger_wall_s)


class MatrixProbe(RotationTrial):
    def __init__(self, obstacles, production=False, truth_source="service"):
        super().__init__()
        self.destroy_client(self.truth)
        self.truth_node=Node('anc_turn_truth_service_evaluator')
        self.truth=self.truth_node.create_client(GetEntityState,'/gzserver/get_entity_state')
        self.truth_source=truth_source
        self.stream_truth=None
        self.stream_clock=None
        self.stream_clock_wall=None
        self.last_truth_stamp=None
        if truth_source=='stream':
            from gz.transport13 import Node as GzNode
            from gz.msgs10.pose_v_pb2 import Pose_V
            self.gz_truth_node=GzNode()
            self.gz_truth_node.subscribe(Pose_V,'/world/anc_phase2_test/dynamic_pose/info',self.on_truth_stream)
            self.truth_node.create_subscription(Clock,'/clock',self.on_truth_clock,qos_profile_sensor_data)
        self.production=production
        self.safety_trace = []
        self.guard_trace = []
        self.create_subscription(String, "/realistic/safety_state",
                                 lambda m: self.record_state(m, self.safety_trace), 10)
        self.create_subscription(String, "/realistic/command_guard_state",
                                 lambda m: self.record_state(m, self.guard_trace), 10)
        self.obstacles = obstacles
        self.local_map = None
        self.odom = None
        self.create_subscription(OccupancyGrid, '/realistic/local_map/lidar',
                                 lambda m: setattr(self, 'local_map', m), 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, '/realistic/local_map/lidar', 10)

    def wait_gate(self, cap, timeout_s=20, stable_s=.4):
        if not self.production:
            return super().wait_gate(cap, timeout_s, stable_s)
        deadline=time.monotonic()+timeout_s
        stable=None
        while time.monotonic()<deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            s=self.safety or {};model=s.get('angular_stopping_model',{})
            ready=(s.get('valid') is True and s.get('authorized') is True
                   and s.get('motion_mode')=='production_turn_validation_only'
                   and model.get('measured') is True and model.get('production_validation_active') is True
                   and s.get('max_linear_speed_mps')==0
                   and s.get('max_angular_speed_radps',0)>=cap)
            if ready:
                stable=stable or time.monotonic()
                if time.monotonic()-stable>=stable_s:return dict(s)
            else:stable=None
        raise RuntimeError('evidence_bound_production_gate_not_ready:'+str(self.safety))

    def record_state(self, message, trace):
        try:
            trace.append(dict(wall_s=time.monotonic(), state=json.loads(message.data)))
        except (ValueError, TypeError):
            trace.append(dict(wall_s=time.monotonic(), state={'valid': False, 'reason':'malformed'}))

    def on_truth_clock(self, msg):
        self.stream_clock=msg.clock.sec+msg.clock.nanosec/1e9
        self.stream_clock_wall=time.monotonic()

    def on_truth_stream(self, msg):
        for pose in msg.pose:
            if pose.name=='anc_robot':
                self.stream_truth=dict(sim_stamp_s=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,
                                       x_m=pose.position.x,y_m=pose.position.y,yaw_rad=pose_yaw(pose.orientation),
                                       source_receive_wall_s=time.monotonic())

    def get_stream_truth(self, timeout_s):
        request_start=time.monotonic()
        deadline=request_start+timeout_s
        while time.monotonic()<deadline:
            rclpy.spin_once(self.truth_node,timeout_sec=.01)
            pose=self.stream_truth
            now=time.monotonic()
            if pose is None or self.stream_clock is None:continue
            age=self.stream_clock-pose['sim_stamp_s']
            clock_age=now-self.stream_clock_wall
            if (self.last_truth_stamp is None or pose['sim_stamp_s']>self.last_truth_stamp) and -.05<=age<=.05 and clock_age<=.05:
                self.last_truth_stamp=pose['sim_stamp_s']
                # Conservative local ingress bracket from clock/pose age. No runtime TF is published.
                uncertainty=max(0,age)+clock_age+max(0,now-pose["source_receive_wall_s"])
                return dict(pose,wall_s=now,query_start_wall_s=min(request_start,now-uncertainty),
                            evaluator_only=True,source='gz_transport_native_dynamic_pose_stream',ingress_uncertainty_s=uncertainty)
        raise RuntimeError('evaluator_pose_stream_missing_or_stale')

    def get_truth(self, timeout_s=2.5):
        if self.truth_source=='stream':return self.get_stream_truth(timeout_s)
        request_start=time.monotonic()
        if not self.truth.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('evaluator_truth_service_not_discovered')
        request=GetEntityState.Request();request.entity='anc_robot'
        future=self.truth.call_async(request)
        deadline=request_start+timeout_s
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(self.truth_node, timeout_sec=.01)
        if not future.done():
            future.cancel()
            raise RuntimeError('isolated_evaluator_truth_timeout')
        response=future.result()
        if response is None or response.result.result!=1:
            raise RuntimeError('evaluator_truth_query_failed')
        state=response.state
        return dict(wall_s=time.monotonic(),query_start_wall_s=request_start,
                    sim_stamp_s=state.header.stamp.sec+state.header.stamp.nanosec/1e9,
                    x_m=float(state.pose.position.x),y_m=float(state.pose.position.y),
                    yaw_rad=pose_yaw(state.pose.orientation),evaluator_only=True)

    def settle_from_pose(self, baseline, timeout_s=4.0):
        deadline=time.monotonic()+timeout_s
        previous=baseline
        samples=[]
        candidate=None
        final=baseline
        last_sample=0.0
        while time.monotonic()<deadline:
            self.publish_request(0.0)
            rclpy.spin_once(self,timeout_sec=.01)
            if time.monotonic()-last_sample<.04:continue
            current=self.get_truth()
            last_sample=current['wall_s']
            v,w=pose_rate(previous,current)
            current.update(pose_derived_linear_speed_mps=v,pose_derived_angular_speed_radps=w)
            samples.append(current);final=current
            if v is None or w is None or v>.005 or w>.01:
                candidate=None
            elif candidate is None:
                candidate=current['wall_s']
            if candidate is not None and current['query_start_wall_s']-candidate>=.5:
                return final,True,samples,candidate
            previous=current
        return final,False,samples,candidate

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
        gate_at_start=self.wait_gate(0.10, timeout_s=20)
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
                if time.monotonic()-samples[-1]['wall_s'] >= 0.04:
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
            confirmation_wall = time.monotonic()
            elapsed = confirmation_wall-stop_start
            timing = stop_timing_metrics(stop_samples, stop_start, confirmation_wall)
            residual = abs(angle_delta(final['yaw_rad'],zero_pose['yaw_rad']))
            drift = max(max_drift, math.hypot(final['x_m']-start['x_m'],final['y_m']-start['y_m']))
            min_clearance = min(min_clearance, *(self.clearance(p) for p in stop_samples))
            yaw_travel = angle_delta(zero_pose['yaw_rad'],start['yaw_rad'])
            sim_elapsed = zero_pose['sim_stamp_s']-start['sim_stamp_s']
            outputs = [s for s in self.outputs if s['wall_s'] >= output_start_wall]
            no_contacts = self.non_ground_contact_records-contact_start == 0
            stop_contract=evaluate_stop(samples+[zero_pose]+stop_samples, stop_start, observed_zero, confirmation_wall)
            detections=[s for s in self.safety_trace if s['wall_s']>=stop_start
                        and (s['state'].get('authorized') is False or s['state'].get('valid') is False)]
            guard_denials=[s for s in self.guard_trace if s['wall_s']>=stop_start
                          and s['state'].get('authorized') is False]
            detection=(guard_denials[0]['wall_s'] if fault=='command_timeout' and guard_denials
                       else detections[0]['wall_s'] if detections else None)
            origin_zero=[s['state']['decision_monotonic_s'] for s in guard_denials
                         if s['state'].get('output',{}).get('is_zero') is True
                         and 'decision_monotonic_s' in s['state']]
            origin_detection=(origin_zero[0] if fault=='command_timeout' and origin_zero else
                              detections[0]['state'].get('decision_monotonic_s') if detections else None)
            legacy_safe_stop=observed_zero is not None and settled and elapsed <= .75 and residual <= .05
            safe_stop = settled and stop_contract['status']=='PASS'

            return dict(trial_pass=bool(safe_stop and unsafe is None and (not fault or fault_wall is not None) and no_contacts and min_clearance >= .10
                                       and drift <= .10 and abs(yaw_travel)>.005 and yaw_travel*speed>0),
                        production_angular_authority_enabled=(gate_at_start.get("motion_mode")=="production_turn_validation_only"),
                        evaluator_truth_source=self.truth_source,
                        tested_stopping_model=gate_at_start.get("angular_stopping_model"), fault=fault, fault_injected=fault_wall is not None,
                        requested_angular_speed_radps=speed,
                        measured_average_yaw_rate_radps=yaw_travel/sim_elapsed,
                        yaw_travel_rad=yaw_travel, center_drift_m=drift,
                        minimum_clearance_m=min_clearance,
                        residual_yaw_after_zero_output_rad=residual,
                        zero_request_to_settled_wall_s=elapsed,
                        stop_timing_diagnostic=timing,
                        legacy_combined_timing_pass=legacy_safe_stop,
                        stop_contract=stop_contract,
                        fault_detection_origin_s=None if origin_detection is None else origin_detection-stop_start,
                        guard_decision_suppression_s=(None if not origin_zero or origin_detection is None else origin_zero[0]-origin_detection),
                        output_delivery_after_guard_decision_s=(None if not origin_zero or observed_zero is None else observed_zero-origin_zero[0]),
                        fault_detection_observed_s=None if detection is None else detection-stop_start,
                        guard_suppression_after_detection_s=(None if detection is None or observed_zero is None
                                                            else observed_zero-detection),
                        safety_trace=self.safety_trace, guard_trace=self.guard_trace,
                        first_zero_output_truth=zero_pose,
                        stop_trigger_wall_s=stop_start,
                        final_truth=final,
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
    node = MatrixProbe(spec['obstacles'],spec.get('production_turning',False),spec.get('truth_source','service'))
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
                      classification=('PHASE8C_PRODUCTION_TURNING_VALIDATION' if node.production else 'PREPRODUCTION_TURN_CONTRACT_VALIDATION'),
                      production_angular_authority_enabled=bool(trials) and all(t['production_angular_authority_enabled'] for t in trials),
                      trial_pass=(all(t['trial_pass'] for t in trials)
                                  and names == ['anc_safety_command_guard']))
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        result['partial_observations'] = dict(output_samples=node.outputs, safety_trace=node.safety_trace, guard_trace=node.guard_trace,
                                             observed_non_ground_contacts=node.non_ground_contact_records)
    finally:
        node.spin_for(.5,0.0)
        Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n')
        if hasattr(node, "gz_truth_node"):
            node.gz_truth_node.unsubscribe("/world/anc_phase2_test/dynamic_pose/info")
        node.truth_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
