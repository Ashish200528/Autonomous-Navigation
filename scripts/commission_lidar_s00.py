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
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from anc_lidar.turn_envelope import load_envelope
from phase8c_readiness import _validate_moving
from phase8c_navigation_runner import Eval,quat_yaw,wrap,map_goal,clearance
from run_native_slam_trials import stop

ROOT=Path(__file__).resolve().parents[1]


def spawn_arguments(layout):
    values=layout['start_xyyaw']
    if len(values)!=3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('invalid_scenario_spawn_pose')
    return ['start_x:='+str(values[0]),'start_y:='+str(values[1]),'start_yaw:='+str(values[2])]


def snapshot_configuration(source,out,label):
    """Launch the saved bytes so later edits cannot change trial provenance."""
    source=source.resolve();payload=source.read_bytes();saved=out/(label+'-config.yaml')
    saved.write_bytes(payload)
    return saved,dict(source=str(source),snapshot=saved.name,sha256=hashlib.sha256(payload).hexdigest())


def timeout_diagnostic_eligible(report):
    """A timeout may motivate a separate experiment, never a mission PASS."""
    numeric=('final_localization_error_m','final_localization_heading_error_rad','min_evaluator_clearance_m')
    if not all(isinstance(report.get(k),(int,float)) and math.isfinite(report[k]) for k in numeric):return False
    sensors=report.get('sensor_integrity',{})
    return (report.get('status')=='FAIL' and report.get('nav2_action_status')=='TIMEOUT'
            and report.get('post_goal_stop_confirmed') is True
            and report['final_localization_error_m']<=.1 and report['final_localization_heading_error_rad']<=.1
            and report['min_evaluator_clearance_m']>=.10
            and report.get('non_ground_contact_events')==[] and report.get('guarded_mixed_output_samples')==0
            and report.get('sole_authoritative_cmd_vel_publisher') is True
            and report.get('travel_limit_exceeded') is False
            and all(sensors.get(k) for k in ('rgb_fresh','scan_fresh','map_fresh','filtered_odom_received')))


class CommissionEval(Eval):
    def __init__(self):
        self.active_intervention_counts={}
        self.interventions=[]
        self.localization_trace=[]
        self.reference_map_start=None
        self.reference_world_start=None
        self.travel_limit_m=1.5
        self.goal_distance_m=.75
        self.evaluation_layout=None
        self.artifact_dir=None
        self.raw_odom=None
        self.slam_diagnostics=None
        self.local_map=None;self.local_map_wall=None;self.run_number=0
        super().__init__()
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.create_subscription(OccupancyGrid,'/realistic/local_map/lidar',self.on_local_map,qos_profile_sensor_data)
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'raw_odom',m),qos_profile_sensor_data)
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.gz=Node();self.latest=None;self.previous_stamp=None;self.origin=None
        self.gz.subscribe(Pose_V,'/world/anc_phase2_test/dynamic_pose/info',self.on_pose)

    def stationary_diagnostic(self,duration):
        trace=[];deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            p=self.get_truth()
            row=dict(truth=p,localization=self.localization_trace[-1] if self.localization_trace else None,
                     guard_output=None if self.guard is None else self.guard.get('output'))
            for name,msg in [('raw_odom',self.raw_odom),('filtered_odom',self.odom)]:
                if msg:
                    row[name]=dict(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,yaw=quat_yaw(msg.pose.pose.orientation),
                                   vx=msg.twist.twist.linear.x,wz=msg.twist.twist.angular.z,
                                   stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
            try:
                t=self.tf.lookup_transform('map','odom',rclpy.time.Time()).transform
                row['map_to_odom']=dict(x=t.translation.x,y=t.translation.y,yaw=quat_yaw(t.rotation))
            except Exception:pass
            trace.append(row)
            output=row['guard_output'] or {}
            if abs(output.get('linear_x_mps',0))>1e-9 or abs(output.get('angular_z_radps',0))>1e-9:
                raise RuntimeError('stationary_diagnostic_nonzero_command')
            end=time.monotonic()+.10
            while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)
        return trace

    def on_local_map(self,msg):
        self.local_map=msg;self.local_map_wall=time.monotonic()

    @staticmethod
    def grid_view(msg):
        return dict(data=msg.data,width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,
                    origin=[msg.info.origin.position.x,msg.info.origin.position.y],yaw=quat_yaw(msg.info.origin.orientation))

    def on_pose(self,msg):
        for p in msg.pose:
            if p.name=='anc_robot':
                if self.slam_diagnostics is not None:self.slam_diagnostics.truth(msg,p)
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
                if self.evaluation_layout is not None and clearance(p,self.evaluation_layout)<.10:
                    if self.active_goal:self.active_goal.cancel_goal_async()
                    raise RuntimeError('evaluator_minimum_clearance_abort')
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
        self.evaluation_layout=layout
        current_map,_=self.ready()
        if self.slam_diagnostics is not None:self.slam_diagnostics.record_parameters()
        if self.reference_map_start is None:
            self.reference_map_start=current_map
            self.reference_world_start=layout['start_xyyaw']
        self.fixed_goal_map_reference=self.reference_map_start
        verify_spawn=self.origin is None
        initial_truth=self.get_truth();sw=layout['start_xyyaw']
        if verify_spawn and (math.hypot(initial_truth['x_m']-sw[0],initial_truth['y_m']-sw[1])>.05 or abs(wrap(initial_truth['yaw_rad']-sw[2]))>.05):
            raise RuntimeError('scenario_spawn_pose_mismatch')
        self.save_map('initial' if self.run_number==0 else f'leg-{self.run_number}-initial',map_goal(self.reference_map_start,sw,layout['goal_xyyaw']))
        self.run_number+=1
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
        final=self.truth[-1];map_final=self.pose_map();ms=self.reference_map_start;sw=layout['start_xyyaw']
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
        if not stopped or mixed or report['truth_path_length_m']<.1 or loc_error>.1 or yaw_error>.1 or report.get('min_evaluator_clearance_m',0)<.10:report['status']='FAIL'
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
    parser.add_argument('--slam-config',type=Path)
    parser.add_argument('--nav2-config',type=Path,help='Explicit bounded commissioning profile; default unchanged')
    parser.add_argument('--trace-slam',action='store_true',help='Passive evaluator source trace; no runtime publications')
    parser.add_argument('--slam-hits-only',action='store_true',help='Experimental Karto endpoint encoding; safety scan unchanged')
    parser.add_argument('--nav2-clearing-scan',action='store_true')
    parser.add_argument('--stationary-hold-s',type=float,default=0)
    parser.add_argument('--observation-steps',type=int,default=0,choices=range(4))
    parser.add_argument('--observation-max-step-m',type=float,default=2.)
    parser.add_argument('--observation-heading',choices=['destination','arrival'],default='destination')
    parser.add_argument('--observation-clear-corridor',action='store_true',help='Require observed .60 m radius capsule before original goal')
    parser.add_argument('--post-timeout-diagnostic-s',type=float,default=0,
                        help='Separate S01 experiment after a safe timeout; original mission remains FAIL')
    parser.add_argument('--scenario',choices=['S00','S01'],default='S00')
    parser.add_argument('--distance-m',type=float,default=.75,help='S00 goal distance; S01 uses original goal')
    parser.add_argument('--goal-yaw-rad',type=float,default=0,help='S00 turn/stop qualification; original S01 yaw is fixed')
    parser.add_argument('--timeout-s',type=float,default=60)
    parser.add_argument('--domain-id',type=int,default=42)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--rviz',action='store_true')
    args=parser.parse_args()
    if not math.isfinite(args.goal_yaw_rad) or abs(args.goal_yaw_rad)>math.pi or (args.scenario=='S01' and abs(args.goal_yaw_rad)>1e-9):
        raise SystemExit('S00 goal yaw must be finite within +/-pi; preserve original S01 yaw')
    if not 0<=args.post_timeout_diagnostic_s<=300 or (args.post_timeout_diagnostic_s and args.scenario!='S01'):
        raise SystemExit('Separate timeout diagnostic is S01 only, bounded to 300 s')
    if not .35<=args.observation_max_step_m<=2.:raise SystemExit('Observation step must be .35..2 m')
    if not 0<=args.stationary_hold_s<=60:raise SystemExit('stationary diagnostic hold must be 0..60 s')
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
    if args.scenario=='S00':layout['goal_xyyaw']=[args.distance_m,0,args.goal_yaw_rad]
    (out/'commission-layout.json').write_text(json.dumps(layout,indent=2)+'\n')
    config_dir=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
    slam_snapshot,slam_provenance=snapshot_configuration(args.slam_config or config_dir/'slam_toolbox.yaml',out,'slam')
    nav_snapshot,nav_provenance=snapshot_configuration(args.nav2_config or config_dir/('nav2_s01_turn_room.yaml' if args.scenario=='S01' and args.observation_steps else 'nav2_lidar.yaml'),out,'nav2')
    (out/'run-settings.json').write_text(json.dumps(dict(scenario=args.scenario,domain_id=args.domain_id,
        gui=args.gui,rviz=args.rviz,goal=layout['goal_xyyaw'],spawn=layout['start_xyyaw'],timeout_s=args.timeout_s,
        observation_steps=args.observation_steps,stationary_hold_s=args.stationary_hold_s,trace_slam=args.trace_slam,
        observation_max_step_m=args.observation_max_step_m,observation_heading=args.observation_heading,
        observation_clear_corridor=args.observation_clear_corridor,
        post_timeout_diagnostic_s=args.post_timeout_diagnostic_s,
        slam_hits_only=args.slam_hits_only, nav2_clearing_scan=args.nav2_clearing_scan,
        initial_available_mib=available/1024,slam_config=str(args.slam_config) if args.slam_config else 'baseline',
        configuration_snapshots=dict(slam=slam_provenance,nav2=nav_provenance)),indent=2)+'\n')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    rclpy.init();node=CommissionEval();launch=None
    node.artifact_dir=out
    if args.trace_slam:
        from slam_source_diagnostics import SlamSourceDiagnostics
        node.slam_diagnostics=SlamSourceDiagnostics(node,out)
    node.goal_distance_m=math.hypot(layout['goal_xyyaw'][0]-layout['start_xyyaw'][0],layout['goal_xyyaw'][1]-layout['start_xyyaw'][1])
    node.travel_limit_m=(node.goal_distance_m+2 if extended else 1.5)
    waypoint_reports=[];original_goal_attempted=False
    report=dict(status='FAIL',classification='REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL',scenario_id=args.scenario)
    try:
        with (out/'launch.log').open('w') as f:
            launch=subprocess.Popen(['ros2','launch','anc_lidar','realistic_rgb_lidar.launch.py',
                'world_file:='+layout['world_file'],
                'slam_params_file:='+str(slam_snapshot),
                'slam_hits_only:='+str(args.slam_hits_only).lower(),
                'nav2_clearing_scan:='+str(args.nav2_clearing_scan).lower(),
                'nav2_params_file:='+str(nav_snapshot),
                *spawn_arguments(layout),'production_turning:=true','turn_validation_only:=false',
                'production_turn_evidence_file:='+str(args.envelope.resolve()),'rotation_calibration_max_radps:=0.0',
                'enable_slam:=true','enable_nav2:=true','nav2_startup_delay_s:='+('12.0' if args.gui else '6.0'),'gui:='+str(args.gui).lower(),'rviz:='+str(args.rviz).lower()],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            mission_start=time.monotonic()
            if args.observation_steps:
                from lidar_observation_goals import select_observation_goal,target_observed,target_corridor_observed
                node.reference_map_start,_=node.ready();node.reference_world_start=layout['start_xyyaw']
                target=map_goal(node.reference_map_start,layout['start_xyyaw'],layout['goal_xyyaw'])
                node.save_map('pre-observation',target)
                visited=[]
                for index in range(args.observation_steps):
                    pose=node.pose_map()
                    observed=(target_corridor_observed if args.observation_clear_corridor else target_observed)
                    if observed(node.grid_view(node.map),pose,target):break
                    if node.local_map is None or time.monotonic()-node.local_map_wall>.35:
                        raise RuntimeError('observation_policy_local_map_stale')
                    choice=select_observation_goal(node.grid_view(node.map),node.grid_view(node.local_map),pose,target,visited,
                                                   max_step_m=args.observation_max_step_m,heading_policy=args.observation_heading)
                    (out/f'observation-{index}-selection.json').write_text(json.dumps(choice,indent=2)+'\n')
                    if choice['status']!='SELECTED':raise RuntimeError('observation_goal_blocked:'+choice['reason'])
                    goal=choice['goal_map'];visited.append(goal)
                    ms=node.reference_map_start;sw=layout['start_xyyaw'];d=wrap(sw[2]-ms[2]);dx,dy=goal[0]-ms[0],goal[1]-ms[1]
                    interim=dict(layout,goal_xyyaw=[sw[0]+math.cos(d)*dx-math.sin(d)*dy,sw[1]+math.sin(d)*dx+math.cos(d)*dy,wrap(goal[2]+d)])
                    remaining=args.timeout_s-(time.monotonic()-mission_start)
                    if remaining<5:raise RuntimeError('observation_mission_deadline')
                    leg=node.run(interim,min(remaining,120));leg['metrics_scope']='cumulative_since_original_spawn'
                    filename=f'observation-{index}-result.json';waypoint_reports.append(filename)
                    (out/filename).write_text(json.dumps(leg,indent=2)+'\n')
                    if leg['status']!='PASS':raise RuntimeError('observation_goal_failed')
            if args.observation_clear_corridor and args.observation_steps:
                if not target_corridor_observed(node.grid_view(node.map),node.pose_map(),target):
                    raise RuntimeError('original_goal_corridor_not_fully_observed')
            remaining=args.timeout_s-(time.monotonic()-mission_start)
            if remaining<5:raise RuntimeError('commission_mission_deadline')
            original_goal_attempted=True
            report=node.run(layout,remaining)
            report['mission_elapsed_wall_s']=time.monotonic()-mission_start
            if args.post_timeout_diagnostic_s and timeout_diagnostic_eligible(report):
                from action_msgs.msg import GoalStatus
                (out/'original-mission-result.json').write_text(json.dumps(report,indent=2)+'\n')
                previous_result=node.active_goal.get_result_async();deadline=time.monotonic()+2
                while not previous_result.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
                if not previous_result.done() or previous_result.result().status!=GoalStatus.STATUS_CANCELED:
                    report['post_timeout_diagnostic_blocked']='previous_goal_cancellation_unconfirmed'
                else:
                    continuation=node.run(layout,args.post_timeout_diagnostic_s)
                    continuation.update(classification='POST_TIMEOUT_S01_DIAGNOSTIC',diagnostic_only=True,
                                        qualifies_original_mission=False,metrics_scope='cumulative_since_original_spawn',
                                        previous_goal_cancellation_confirmed=True)
                    (out/'post-timeout-diagnostic-result.json').write_text(json.dumps(continuation,indent=2)+'\n')
                    report=dict(status='FAIL',classification='MISSION_FAILED_WITH_SEPARATE_TIMEOUT_DIAGNOSTIC',
                                reason='original_mission_timeout_retained',primary_mission_result='original-mission-result.json',
                                post_timeout_diagnostic_result='post-timeout-diagnostic-result.json',
                                diagnostic_original_goal_reached=continuation['status']=='PASS',
                                mission_elapsed_wall_s=time.monotonic()-mission_start)
    except Exception as exc:
        report.update(status='FAIL',reason=f'{type(exc).__name__}: {exc}',safety_state=node.safety,guard_state=node.guard,
                      commands={k:len(v) for k,v in node.cmd.items()},contacts=node.collisions,
                      map_updates=node.maps,plan_messages=node.plans,rgb_frames=node.rgb,scans=len(node.scans))
    finally:
        if node.active_goal:
            future=node.active_goal.cancel_goal_async();end=time.monotonic()+1
            while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        if args.stationary_hold_s and node.origin is not None:
            end=time.monotonic()+.5
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
            try:
                diagnostic=node.stationary_diagnostic(args.stationary_hold_s)
                (out/'stationary-diagnostic.json').write_text(json.dumps(diagnostic,indent=2)+'\n')
            except Exception as exc:
                report['stationary_diagnostic_error']=str(exc)
        node.save_map('final')
        report.update(observation_goal_reports=waypoint_reports,original_goal_attempted=original_goal_attempted,localization_trace=node.localization_trace,active_goal_guard_reason_sample_counts=node.active_intervention_counts,
                      guard_interventions=node.interventions,scenario_id=args.scenario,
                      configured_goal_world=layout['goal_xyyaw'],configured_travel_limit_m=node.travel_limit_m)
        if node.slam_diagnostics is not None:report['slam_source_diagnostics']=node.slam_diagnostics.close()
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
