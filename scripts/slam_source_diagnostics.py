"""Passive, evaluator-only source trace. Publishes nothing to the ROS graph."""
import gzip
import json
import math
import threading
import time
from collections import Counter

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


def stamp(s):
    return s.sec+s.nanosec/1e9


def xyz(p):
    return [p.x,p.y,p.z]


def quaternion(q):
    return [q.x,q.y,q.z,q.w]


def ranges(values):
    return [round(v,6) if math.isfinite(v) else ('+inf' if v>0 else '-inf' if v<0 else None) for v in values]


class SlamSourceDiagnostics:
    """Stream to disk, keeping sensor/SLAM timestamps and 3D evaluator truth.

    No TF lookup at callback receipt time is mistaken for the scan-time pose:
    offline analysis interpolates the recorded timestamped streams instead.
    """
    def __init__(self,node,out):
        self.node=node;self.out=out;self.lock=threading.Lock();self.counts=Counter()
        self.file=gzip.open(out/'slam-source.jsonl.gz','wt',compresslevel=1)
        self.bytes=0;self.truncated=False;self.closed=False;self.last_truth=-1;self.last_imu=-1
        self.subscriptions=[]
        def sub(kind,topic,callback,qos=qos_profile_sensor_data):
            self.subscriptions.append(node.create_subscription(kind,topic,callback,qos))
        sub(LaserScan,'/realistic/scan/raw',lambda m:self.scan('raw_scan',m))
        sub(LaserScan,'/realistic/scan',lambda m:self.scan('conditioned_scan',m))
        sub(LaserScan,'/realistic/scan/slam_hits',lambda m:self.scan('slam_hit_scan',m))
        sub(PoseWithCovarianceStamped,'/pose',self.slam_pose,10)
        sub(Odometry,'/odom',lambda m:self.odom('wheel_odom',m))
        sub(Odometry,'/odometry/filtered',lambda m:self.odom('filtered_odom',m))
        sub(Imu,'/imu/data',self.imu)
        sub(TFMessage,'/tf',self.transforms,100)
        staticq=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        sub(TFMessage,'/tf_static',lambda m:self.transforms(m,True),staticq)
        sub(MarkerArray,'/slam_toolbox/graph_visualization',self.graph,10)

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
        self.write('truth_evaluator_only',t,position=xyz(p.position),orientation=quaternion(p.orientation))

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
