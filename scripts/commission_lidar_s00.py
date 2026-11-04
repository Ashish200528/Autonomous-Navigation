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
from phase8c_navigation_runner import Eval,quat_yaw,wrap,map_goal
from run_native_slam_trials import stop

ROOT=Path(__file__).resolve().parents[1]


def spawn_arguments(layout):
    values=layout['start_xyyaw']
    if len(values)!=3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('invalid_scenario_spawn_pose')
    return ['start_x:='+str(values[0]),'start_y:='+str(values[1]),'start_yaw:='+str(values[2])]


class CommissionEval(Eval):
    def __init__(self):
        self.active_intervention_counts={}
        self.interventions=[]
        self.localization_trace=[]
        self.reference_map_start=None
        self.reference_world_start=None
        self.travel_limit_m=1.5
        self.goal_distance_m=.75
        self.artifact_dir=None
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
                if math.hypot(p['x_m']-self.origin['x_m'],p['y_m']-self.origin['y_m'])>self.travel_limit_m:
                    if self.active_goal:self.active_goal.cancel_goal_async()
                    raise RuntimeError('commission_travel_limit_abort')
                if self.reference_map_start is not None:
                    try:
                        tf=self.tf.lookup_transform('map','base_footprint',rclpy.time.Time())
                        est=tf.transform;ms=self.reference_map_start;sw=self.reference_world_start
                        d=wrap(sw[2]-ms[2]);dx,dy=est.translation.x-ms[0],est.translation.y-ms[1]
                        ex=sw[0]+math.cos(d)*dx-math.sin(d)*dy;ey=sw[1]+math.sin(d)*dx+math.cos(d)*dy
                        self.localization_trace.append(dict(wall_s=p['wall_s'],sim_s=p['sim_s'],
                            xy_error_m=math.hypot(ex-p['x_m'],ey-p['y_m']),
                            heading_error_rad=abs(wrap(quat_yaw(est.rotation)+d-p['yaw_rad'])),
                            tf_age_sim_s=p['sim_s']-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)))
                    except Exception:pass
                return p
        raise RuntimeError('commission_truth_stream_missing_or_stale')

    def guard_cb(self,msg):
        super().guard_cb(msg)
        if self.active_goal is not None and self.guard:
            reason=self.guard.get('reason','unknown')
            self.active_intervention_counts[reason]=self.active_intervention_counts.get(reason,0)+1
            if not self.interventions or self.interventions[-1]['reason']!=reason:
                self.interventions.append(dict(wall_s=time.monotonic(),reason=reason,
                    requested=self.guard.get('requested'),output=self.guard.get('output')))

    def save_map(self,label,goal=None):
        if self.map is None or self.artifact_dir is None:return
        m=self.map;counts={str(v):sum(x==v for x in m.data) for v in (-1,0,100)}
        metadata=dict(frame=m.header.frame_id,width=m.info.width,height=m.info.height,
                      resolution_m=m.info.resolution,counts=counts,stamp_sim_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                      origin_xy=[m.info.origin.position.x,m.info.origin.position.y])
        if goal:
            yaw=quat_yaw(m.info.origin.orientation);dx=goal[0]-m.info.origin.position.x;dy=goal[1]-m.info.origin.position.y
            ix=int(math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/m.info.resolution))
            iy=int(math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/m.info.resolution))
            metadata['goal_cell']=int(m.data[iy*m.info.width+ix]) if 0<=ix<m.info.width and 0<=iy<m.info.height else 'OUTSIDE_MAP'
        (self.artifact_dir/(label+'-map.json')).write_text(json.dumps(metadata,indent=2)+'\n')
        rows=[]
        for y in range(m.info.height-1,-1,-1):
            rows.append(' '.join('205' if v<0 else '0' if v>=50 else '254' for v in m.data[y*m.info.width:(y+1)*m.info.width]))
        (self.artifact_dir/(label+'-map.pgm')).write_text(f'P2\n{m.info.width} {m.info.height}\n255\n'+'\n'.join(rows)+'\n')

    def run(self,layout,timeout_s):
        self.reference_map_start,_=self.ready()
        self.reference_world_start=layout['start_xyyaw']
        initial_truth=self.get_truth();sw=layout['start_xyyaw']
        if math.hypot(initial_truth['x_m']-sw[0],initial_truth['y_m']-sw[1])>.05 or abs(wrap(initial_truth['yaw_rad']-sw[2]))>.05:
            raise RuntimeError('scenario_spawn_pose_mismatch')
        self.save_map('initial',map_goal(self.reference_map_start,sw,layout['goal_xyyaw']))
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
        transitions=[];previous=None
        mixed=0
        for wall,v,w in self.cmd['/cmd_vel']:
            if abs(v)>1e-9 and abs(w)>1e-9:mixed+=1
            primitive=('forward',1) if abs(v)>1e-9 else ('yaw',1 if w>0 else -1) if abs(w)>1e-9 else None
            if primitive is None:continue
            if previous and primitive!=previous[1]:
                transitions.append(dict(from_primitive=previous[1],to_primitive=primitive,observed_zero_interval_s=wall-previous[0]))
            previous=(wall,primitive)
        report.update(guarded_primitive_transitions=transitions,guarded_mixed_output_samples=mixed,
                      pre_goal_cmd_vel_publishers=names,post_goal_stop_confirmed=stopped,
                      stop_samples=stop_samples,final_localization_error_m=loc_error,
                      final_localization_heading_error_rad=yaw_error,final_stopped_truth=final,
                      commissioning_goal_distance_m=self.goal_distance_m,planned_path_points=self.plan,
                      localization_trace=self.localization_trace,active_goal_guard_reason_sample_counts=self.active_intervention_counts,
                      guard_interventions=self.interventions)
        if not stopped or mixed or report['truth_path_length_m']<.1 or loc_error>.1 or yaw_error>.1:report['status']='FAIL'
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
    parser.add_argument('--scenario',choices=['S00','S01'],default='S00')
    parser.add_argument('--distance-m',type=float,default=.75,help='S00 goal distance; S01 uses original goal')
    parser.add_argument('--timeout-s',type=float,default=60)
    parser.add_argument('--domain-id',type=int,default=42)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--rviz',action='store_true')
    args=parser.parse_args()
    if not .75<=args.distance_m<=3.0:raise SystemExit('S00 distance must be .75..3 metres')
    extended=args.scenario=='S01' or args.distance_m>.75
    if not 5<=args.timeout_s<=(300 if extended else 60):raise SystemExit('Timeout exceeds bounded route profile')
    if not 0<=args.domain_id<=100 or (args.rviz and not args.gui):raise SystemExit('Use domain 0..100 to avoid native Linux ephemeral ports; RViz also requires GUI')
    proof=prerequisites(args.envelope)
    available=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))
    if available<(2560 if args.gui else 2048)*1024:raise SystemExit('BLOCKED: insufficient available memory for commissioning profile')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'prerequisites.json').write_text(json.dumps(proof,indent=2)+'\n')
    with (out/'scenario.log').open('w') as f:
        subprocess.run(['python3',str(ROOT/'scripts/scenario_generator.py'),'--scenario',args.scenario,'--seed','101','--output-dir',str(out/'scenario')],stdout=f,stderr=subprocess.STDOUT,check=True)
    layout=json.loads((out/'scenario/manifest.json').read_text())
    if args.scenario=='S00':layout['goal_xyyaw']=[args.distance_m,0,0]
    (out/'commission-layout.json').write_text(json.dumps(layout,indent=2)+'\n')
    (out/'run-settings.json').write_text(json.dumps(dict(scenario=args.scenario,domain_id=args.domain_id,
        gui=args.gui,rviz=args.rviz,goal=layout['goal_xyyaw'],spawn=layout['start_xyyaw'],timeout_s=args.timeout_s,
        initial_available_mib=available/1024),indent=2)+'\n')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    rclpy.init();node=CommissionEval();launch=None
    node.artifact_dir=out
    node.goal_distance_m=math.hypot(layout['goal_xyyaw'][0]-layout['start_xyyaw'][0],layout['goal_xyyaw'][1]-layout['start_xyyaw'][1])
    node.travel_limit_m=(node.goal_distance_m+2 if extended else 1.5)
    report=dict(status='FAIL',classification='REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL',scenario_id=args.scenario)
    try:
        with (out/'launch.log').open('w') as f:
            launch=subprocess.Popen(['ros2','launch','anc_lidar','realistic_rgb_lidar.launch.py',
                'world_file:='+layout['world_file'],
                *spawn_arguments(layout),'production_turning:=true','turn_validation_only:=false',
                'production_turn_evidence_file:='+str(args.envelope.resolve()),'rotation_calibration_max_radps:=0.0',
                'enable_slam:=true','enable_nav2:=true','nav2_startup_delay_s:='+('12.0' if args.gui else '6.0'),'gui:='+str(args.gui).lower(),'rviz:='+str(args.rviz).lower()],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            report=node.run(layout,args.timeout_s)
    except Exception as exc:
        report.update(reason=f'{type(exc).__name__}: {exc}',safety_state=node.safety,guard_state=node.guard,
                      commands={k:len(v) for k,v in node.cmd.items()},contacts=node.collisions,
                      map_updates=node.maps,plan_messages=node.plans,rgb_frames=node.rgb,scans=len(node.scans))
    finally:
        if node.active_goal:
            future=node.active_goal.cancel_goal_async();end=time.monotonic()+1
            while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        node.save_map('final')
        report.update(localization_trace=node.localization_trace,active_goal_guard_reason_sample_counts=node.active_intervention_counts,
                      guard_interventions=node.interventions,scenario_id=args.scenario,
                      configured_goal_world=layout['goal_xyyaw'],configured_travel_limit_m=node.travel_limit_m)
        if launch:stop(launch)
        report.update(runtime_navigation_receives_no_gazebo_truth=True,predefined_map_used=False,
                      commissioning_prerequisites=proof)
        (out/'navigation-result.json').write_text(json.dumps(report,indent=2)+'\n')
        with (out/'truth-trajectory.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['wall_s','sim_s','x_m','y_m','yaw_rad','linear_x_mps','angular_z_radps'])
            writer.writeheader();writer.writerows(node.truth)
        with (out/'guarded-commands.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['wall_s','topic','linear_x_mps','angular_z_radps'])
            for topic,values in node.cmd.items():
                for wall,v,w in values:writer.writerow([wall,topic,v,w])
        node.gz.unsubscribe('/world/anc_phase2_test/dynamic_pose/info');node.destroy_node();rclpy.shutdown()
    print(report['status'],report.get('reason',''))
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
