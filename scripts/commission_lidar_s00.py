#!/usr/bin/env python3
"""First-goal commissioning: validated turning + moving SLAM, then live guard audit.

This does not require a previous Nav2 goal (no circular readiness dependency).
Normal demonstration readiness remains unchanged. No reference map is created.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from anc_lidar.turn_envelope import load_envelope
from phase8c_readiness import _validate_moving
from phase8c_navigation_runner import Eval,quat_yaw,wrap
from run_native_slam_trials import stop

ROOT=Path(__file__).resolve().parents[1]


class CommissionEval(Eval):
    def __init__(self):
        super().__init__()
        self.set_parameters([Parameter('use_sim_time',value=True)])
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.gz=Node();self.latest=None;self.previous_stamp=None;self.origin=None
        self.gz.subscribe(Pose_V,'/world/anc_phase2_test/dynamic_pose/info',self.on_pose)

    def on_pose(self,msg):
        for p in msg.pose:
            if p.name=='anc_robot':
                self.latest=dict(sim_s=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,
                                 x_m=p.position.x,y_m=p.position.y,yaw_rad=quat_yaw(p.orientation))

    def truth_ready(self,timeout=.1):
        return self.latest is not None

    def get_truth(self):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.01)
            p=self.latest
            if p is None or not self.clock:continue
            age=self.clock[-1][1]-p['sim_s']
            if -.05<=age<=.05 and (self.previous_stamp is None or p['sim_s']>self.previous_stamp):
                p=dict(p,wall_s=time.monotonic(),linear_x_mps=0.0,angular_z_radps=0.0)
                if self.truth:
                    old=self.truth[-1];dt=p['sim_s']-old['sim_s']
                    if dt>0:
                        p['linear_x_mps']=math.hypot(p['x_m']-old['x_m'],p['y_m']-old['y_m'])/dt
                        p['angular_z_radps']=wrap(p['yaw_rad']-old['yaw_rad'])/dt
                self.previous_stamp=p['sim_s'];self.truth.append(p)
                self.origin=self.origin or p
                if math.hypot(p['x_m']-self.origin['x_m'],p['y_m']-self.origin['y_m'])>1.5:
                    if self.active_goal:self.active_goal.cancel_goal_async()
                    raise RuntimeError('commission_1_5m_travel_abort')
                return p
        raise RuntimeError('commission_truth_stream_missing_or_stale')

    def run(self,layout,timeout_s):
        self.ready()
        names=[p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')]
        if names!=['anc_safety_command_guard']:raise RuntimeError('pre_goal_guard_publisher_audit_failed')
        report=super().run(layout,timeout_s)
        candidate=None;stop_samples=[];deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            p=self.get_truth();stop_samples.append(p)
            if p['linear_x_mps']>.005 or abs(p['angular_z_radps'])>.01:candidate=None
            elif candidate is None:candidate=p['wall_s']
            if candidate and p['wall_s']-candidate>=.5:break
            end=time.monotonic()+.08
            while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)
        stopped=candidate is not None and stop_samples[-1]['wall_s']-candidate>=.5
        final=self.truth[-1];map_final=self.pose_map();ms=report['start_map_pose'];sw=layout['start_xyyaw']
        d=wrap(sw[2]-ms[2]);dx,dy=map_final[0]-ms[0],map_final[1]-ms[1]
        estimated=[sw[0]+math.cos(d)*dx-math.sin(d)*dy,sw[1]+math.sin(d)*dx+math.cos(d)*dy,wrap(map_final[2]+d)]
        loc_error=math.hypot(estimated[0]-final['x_m'],estimated[1]-final['y_m'])
        yaw_error=abs(wrap(estimated[2]-final['yaw_rad']))
        report.update(pre_goal_cmd_vel_publishers=names,post_goal_stop_confirmed=stopped,
                      stop_samples=stop_samples,final_localization_error_m=loc_error,
                      final_localization_heading_error_rad=yaw_error,final_stopped_truth=final,
                      commissioning_goal_distance_m=.75)
        if not stopped or report['truth_path_length_m']<.1 or loc_error>.1 or yaw_error>.1:report['status']='FAIL'
        return report


def prerequisites(envelope):
    model=load_envelope(envelope,validation_only=False)
    refs=[]
    for index in range(3):
        p=ROOT/f'docs/evidence/native-slam-map1s-20261103/trial{index}-summary.json'
        data=json.loads(p.read_text())
        if _validate_moving(data):raise ValueError('moving_SLAM_prerequisite_invalid:'+str(p))
        refs.append(dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),trial_id=data['trial_id']))
    if len({r['trial_id'] for r in refs})!=3:raise ValueError('three_distinct_SLAM_trials_required')
    return dict(turn_envelope=model,moving_slam=refs)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('envelope',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();proof=prerequisites(args.envelope)
    available=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))
    if available<2048*1024:raise SystemExit('BLOCKED: commissioning requires 2048 MiB available before startup')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'prerequisites.json').write_text(json.dumps(proof,indent=2)+'\n')
    with (out/'scenario.log').open('w') as f:
        subprocess.run(['python3',str(ROOT/'scripts/scenario_generator.py'),'--scenario','S00','--seed','101','--output-dir',str(out/'scenario')],stdout=f,stderr=subprocess.STDOUT,check=True)
    layout=json.loads((out/'scenario/manifest.json').read_text());layout['goal_xyyaw']=[.75,0,0]
    (out/'commission-layout.json').write_text(json.dumps(layout,indent=2)+'\n')
    os.environ.update(ROS_DOMAIN_ID='180',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    rclpy.init();node=CommissionEval();launch=None
    report=dict(status='FAIL',classification='REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL',scenario_id='S00')
    try:
        with (out/'launch.log').open('w') as f:
            launch=subprocess.Popen(['ros2','launch','anc_lidar','realistic_rgb_lidar.launch.py',
                'world_file:='+layout['world_file'],'production_turning:=true','turn_validation_only:=false',
                'production_turn_evidence_file:='+str(args.envelope.resolve()),'rotation_calibration_max_radps:=0.0',
                'enable_slam:=true','enable_nav2:=true'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            report=node.run(layout,60)
    except Exception as exc:
        report.update(reason=f'{type(exc).__name__}: {exc}',safety_state=node.safety,guard_state=node.guard,
                      commands={k:len(v) for k,v in node.cmd.items()},contacts=node.collisions,
                      map_updates=node.maps,plan_messages=node.plans,rgb_frames=node.rgb,scans=len(node.scans))
    finally:
        if node.active_goal:
            future=node.active_goal.cancel_goal_async();end=time.monotonic()+1
            while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        if launch:stop(launch)
        report.update(runtime_navigation_receives_no_gazebo_truth=True,predefined_map_used=False,
                      commissioning_prerequisites=proof)
        (out/'navigation-result.json').write_text(json.dumps(report,indent=2)+'\n')
        with (out/'truth-trajectory.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['wall_s','sim_s','x_m','y_m','yaw_rad','linear_x_mps','angular_z_radps'])
            writer.writeheader();writer.writerows(node.truth)
        node.gz.unsubscribe('/world/anc_phase2_test/dynamic_pose/info');node.destroy_node();rclpy.shutdown()
    print(report['status'],report.get('reason',''))
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
