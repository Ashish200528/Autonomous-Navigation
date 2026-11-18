"""Passive, evaluator-only source trace. Publishes nothing to the ROS graph."""
import gzip
import json
import math
import threading
import time
from collections import Counter

from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String


def stamp(s):
    return s.sec+s.nanosec/1e9


def xyz(p):
    return [p.x,p.y,p.z]


def quaternion(q):
    return [q.x,q.y,q.z,q.w]


def ranges(values):
    return [round(v,6) if math.isfinite(v) else ('+inf' if v>0 else '-inf' if v<0 else None) for v in values]


def json_parameter(value):
    return value.tolist() if hasattr(value,'tolist') else value


class SlamSourceDiagnostics:
    """Stream to disk, keeping sensor/SLAM timestamps and 3D evaluator truth.

    No TF lookup at callback receipt time is mistaken for the scan-time pose:
    offline analysis interpolates the recorded timestamped streams instead.
    """
    def __init__(self,node,out):
        self.node=node;self.out=out;self.lock=threading.Lock();self.counts=Counter()
        self.file=gzip.open(out/'slam-source.jsonl.gz','wt',compresslevel=1)
        self.bytes=0;self.truncated=False;self.closed=False;self.last_truth=-1;self.last_imu=-1
        self.parameters_recorded=False
        self.local_costmap=None;self.last_costmap_dump=-1
        self.subscriptions=[]
        def sub(kind,topic,callback,qos=qos_profile_sensor_data):
            self.subscriptions.append(node.create_subscription(kind,topic,callback,qos))
        sub(LaserScan,'/realistic/scan/raw',lambda m:self.scan('raw_scan',m))
        sub(LaserScan,'/realistic/scan',lambda m:self.scan('conditioned_scan',m))
        sub(LaserScan,'/realistic/scan/slam_hits',lambda m:self.scan('slam_hit_scan',m))
        sub(LaserScan,'/realistic/scan/nav2',lambda m:self.scan('nav2_clearing_scan',m))
        sub(PoseWithCovarianceStamped,'/pose',self.slam_pose,10)
        sub(PointStamped,'/lookahead_point',lambda m:self.write('controller_carrot',stamp(m.header.stamp),
            frame=m.header.frame_id,position=xyz(m.point)),10)
        sub(Odometry,'/odom',lambda m:self.odom('wheel_odom',m))
        sub(Odometry,'/odometry/filtered',lambda m:self.odom('filtered_odom',m))
        sub(Imu,'/imu/data',self.imu)
        sub(TFMessage,'/tf',self.transforms,100)
        staticq=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        sub(TFMessage,'/tf_static',lambda m:self.transforms(m,True),staticq)
        sub(MarkerArray,'/slam_toolbox/graph_visualization',self.graph,10)
        sub(String,'/sequential_controller/state',self.controller_state,10)
        sub(String,'/realistic/command_guard_state',self.guard_decision,10)
        costq=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        sub(OccupancyGrid,'/local_costmap/costmap',lambda m:setattr(self,'local_costmap',m),costq)

    def guard_decision(self,m):
        try:state=json.loads(m.data)
        except (ValueError,TypeError):return
        data={k:state.get(k) for k in ('decision_monotonic_s','reason','output','requested','output_count')}
        self.write('guard_decision',self.node.clock[-1][1] if self.node.clock else None,data=data)

    def controller_state(self,m):
        try:data=json.loads(m.data)
        except (ValueError,TypeError):data={'malformed':True}
        self.write('sequential_controller',self.node.clock[-1][1] if self.node.clock else None,data=data)
        if data.get('reason')=='footprint_rejected' and self.local_costmap is not None and time.monotonic()-self.last_costmap_dump>1:
            self.last_costmap_dump=time.monotonic();grid=self.local_costmap
            self.write('rejected_local_costmap',stamp(grid.header.stamp),frame=grid.header.frame_id,
                       width=grid.info.width,height=grid.info.height,resolution=grid.info.resolution,
                       origin=xyz(grid.info.origin.position),data=list(grid.data))

    def record_parameters(self):
        """Query live services after Nav2 activation, before the first goal."""
        if self.parameters_recorded:return
        import rclpy
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rcl_interfaces.msg import Parameter as ParameterMessage
        from rclpy.parameter import Parameter
        groups={
            'slam_toolbox':['scan_topic','use_scan_matching','do_loop_closing','distance_variance_penalty',
                            'minimum_travel_distance','minimum_time_interval','max_laser_range','base_frame','odom_frame'],
            'controller_server':['FollowPath.plugin','FollowPath.linear_speed_mps','FollowPath.angular_speed_radps',
                                 'FollowPath.path_heading_limit_rad','FollowPath.turn_stop_error_rad','FollowPath.settle_wall_s',
                                 'FollowPath.max_angular_accel','FollowPath.rotate_to_heading_min_angle',
                                 'FollowPath.rotate_to_heading_angular_vel','FollowPath.lookahead_dist',
                                 'FollowPath.min_lookahead_dist','progress_checker.required_movement_angle',
                                 'progress_checker.movement_time_allowance','goal_checker.xy_goal_tolerance','goal_checker.yaw_goal_tolerance'],
            'velocity_smoother':['max_accel','max_decel','max_velocity','deadband_velocity'],
            'anc_lidar_scan_conditioner':['publish_slam_hits_only','publish_nav2_clearing_scan','input_topic','output_topic','range_min_m','range_max_m'],
        }
        audit={}
        for name,names in groups.items():
            client=self.node.create_client(GetParameters,'/'+name+'/get_parameters')
            lister=self.node.create_client(ListParameters,'/'+name+'/list_parameters')
            try:
                if not client.wait_for_service(timeout_sec=2) or not lister.wait_for_service(timeout_sec=2):
                    audit[name]={'error':'parameter_service_missing'};continue
                listing=lister.call_async(ListParameters.Request(depth=0));deadline=time.monotonic()+2
                while not listing.done() and time.monotonic()<deadline:rclpy.spin_once(self.node,timeout_sec=.02)
                if not listing.done() or listing.result() is None:
                    audit[name]={'error':'parameter_listing_timeout'};continue
                declared=set(listing.result().result.names);query=[key for key in names if key in declared]
                audit[name]={key:None for key in names}
                audit[name]['__not_declared']=[key for key in names if key not in declared]
                if not query:continue
                future=client.call_async(GetParameters.Request(names=query));deadline=time.monotonic()+2
                while not future.done() and time.monotonic()<deadline:rclpy.spin_once(self.node,timeout_sec=.02)
                if not future.done() or future.result() is None:
                    audit[name]={'error':'parameter_response_timeout'};continue
                if len(future.result().values)!=len(query):
                    audit[name]={'error':'parameter_response_count_mismatch'};continue
                audit[name].update({key:json_parameter(Parameter.from_parameter_msg(ParameterMessage(name=key,value=value)).value)
                                    for key,value in zip(query,future.result().values)})
            finally:
                self.node.destroy_client(client);self.node.destroy_client(lister)
        (self.out/'actual-runtime-parameters.json').write_text(json.dumps(audit,indent=2)+'\n')
        self.parameters_recorded=True

    def write(self,kind,sim_s,**values):
        row=dict(kind=kind,stamp_s=sim_s,receipt_wall_s=time.monotonic(),**values)
        line=json.dumps(row,separators=(',',':'),allow_nan=False)+'\n'
        with self.lock:
            if self.closed:return
            if self.bytes+len(line)>64*1024*1024:
                self.truncated=True;return
            self.file.write(line);self.bytes+=len(line);self.counts[kind]+=1

    def scan(self,kind,m):
        self.write(kind,stamp(m.header.stamp),frame=m.header.frame_id,
                   angle_min=m.angle_min,angle_max=m.angle_max,angle_increment=m.angle_increment,
                   range_min=m.range_min,range_max=m.range_max,time_increment=m.time_increment,
                   scan_time=m.scan_time,ranges=ranges(m.ranges),
                   receipt_clock_s=self.node.clock[-1][1] if self.node.clock else None)

    def slam_pose(self,m):
        p=m.pose.pose
        self.write('slam_pose',stamp(m.header.stamp),frame=m.header.frame_id,
                   position=xyz(p.position),orientation=quaternion(p.orientation),covariance=list(m.pose.covariance))

    def odom(self,kind,m):
        p=m.pose.pose
        self.write(kind,stamp(m.header.stamp),frame=m.header.frame_id,child=m.child_frame_id,
                   position=xyz(p.position),orientation=quaternion(p.orientation),
                   linear=xyz(m.twist.twist.linear),angular=xyz(m.twist.twist.angular))

    def imu(self,m):
        t=stamp(m.header.stamp)
        if t-self.last_imu<.09:return
        self.last_imu=t
        self.write('imu',t,frame=m.header.frame_id,angular=xyz(m.angular_velocity),
                   linear_acceleration=xyz(m.linear_acceleration))

    def transforms(self,m,static=False):
        for t in m.transforms:
            if t.child_frame_id not in ('odom','base_footprint','base_link','lidar_link'):continue
            self.write('tf_static' if static else 'tf',stamp(t.header.stamp),parent=t.header.frame_id,
                       child=t.child_frame_id,position=xyz(t.transform.translation),orientation=quaternion(t.transform.rotation))

    def truth(self,msg,p):
        t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
        if t-self.last_truth<.019:return
        self.last_truth=t
        clock=self.node.clock[-1] if self.node.clock else None
        self.write('truth_evaluator_only',t,position=xyz(p.position),orientation=quaternion(p.orientation),
                   clock_s=clock[1] if clock else None,clock_receipt_wall_s=clock[0] if clock else None)

    def graph(self,m):
        markers=[dict(ns=a.ns,id=a.id,type=a.type,action=a.action,
                      position=xyz(a.pose.position),orientation=quaternion(a.pose.orientation),
                      points=[xyz(p) for p in a.points]) for a in m.markers]
        self.write('graph',stamp(m.markers[0].header.stamp) if m.markers else None,markers=markers)

    def close(self):
        audit={topic:[p.node_name for p in self.node.get_publishers_info_by_topic(topic)]
               for topic in ('/tf','/tf_static','/pose','/realistic/scan','/realistic/scan/raw','/odom','/odometry/filtered')}
        with self.lock:
            self.closed=True;self.file.close()
        result=dict(evaluator_only=True,publishes_no_messages=True,counts=dict(self.counts),
                    uncompressed_bytes=self.bytes,truncated=self.truncated,publishers=audit,
                    ranges_encoding='finite values rounded to 1 micrometre; +inf/-inf strings, NaN null')
        (self.out/'slam-source-summary.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
