"""Actual Gazebo/ROS marker perception and isolated camera-loss fault test."""
import argparse,json,os,signal,subprocess,time,sys
from pathlib import Path
import rclpy,numpy as np,psutil
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
ROOT=Path(__file__).resolve().parents[2]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('world',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args()
 if psutil.virtual_memory().available<2048*2**20:raise RuntimeError('memory preflight')
 a.output.mkdir(parents=True,exist_ok=False);procs=[];files=[];states=[];poses=[];images=[]
 commands=[['gz','sim','-s','-r','--headless-rendering',str(a.world.resolve())],['ros2','run','ros_gz_bridge','parameter_bridge','/parking/rgb@sensor_msgs/msg/Image[gz.msgs.Image','/phase9/v1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','--ros-args','-r','/parking/rgb:=/camera/image_raw','-r','/phase9/v1/camera_info:=/camera/camera_info'],[sys.executable,str(ROOT/'scripts/phase9/aruco_node.py'),'--ros-args','-p','use_sim_time:=true']]
 try:
  for i,cmd in enumerate(commands):
   f=(a.output/f'process-{i}.log').open('w');files.append(f);procs.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
  rclpy.init();n=Node('parking_perception_evaluator');n.create_subscription(String,'/parking/perception_health',lambda m:states.append(json.loads(m.data)),10);n.create_subscription(PoseStamped,'/parking/marker_pose',lambda m:poses.append(m),10);n.create_subscription(Image,'/camera/image_raw',lambda m:images.append(m),qos_profile_sensor_data)
  blankpub=n.create_publisher(Image,'/camera/image_raw',qos_profile_sensor_data)
  def spin_until(predicate,seconds):
   end=time.monotonic()+seconds
   while not predicate() and time.monotonic()<end:
    if psutil.virtual_memory().available<768*2**20:raise RuntimeError('memory floor')
    rclpy.spin_once(n,timeout_sec=.02)
   return predicate()
  good=spin_until(lambda:sum(s.get('valid',False) for s in states)>=5 and len(poses)>=5,30)
  inputs={(m.header.stamp.sec,m.header.stamp.nanosec) for m in images};matched=all((p.header.stamp.sec,p.header.stamp.nanosec) in inputs for p in poses)
  if good:
   # Freeze only the owned static scene. No robot commands are published.
   os.killpg(procs[0].pid,signal.SIGSTOP);count=len(poses);cut=len(states)
   last=images[-1];blank=Image();blank.header=last.header;blank.width=last.width;blank.height=last.height;blank.encoding='rgb8';blank.step=blank.width*3;blank.data=bytes([255])*(blank.step*blank.height);blankpub.publish(blank)
   missing=spin_until(lambda:any(s.get('reason')=='marker_missing' for s in states[cut:]),.4)
   stale=spin_until(lambda:any(s.get('reason')=='camera_or_marker_stale' for s in states[cut:]),1.2);no_pose=len(poses)==count
  else:missing=stale=no_pose=False
  pubs=[p[0] for p in n.get_publisher_names_and_types_by_node('anc_aruco_observer','/')]
  report=dict(status='SIMULATION_PASS' if good and matched and missing and stale and no_pose and '/cmd_vel' not in pubs else 'FAIL',valid_observations=sum(s.get('valid',False) for s in states),pose_messages=len(poses),source_timestamps_preserved=matched,blank_image_rejected=missing,stale_camera_rejected=stale,no_pose_after_loss=no_pose,publishers=pubs,health=states,truth_used_at_runtime=False,motor_authority=False)
  (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n');n.destroy_node();rclpy.shutdown();print(report['status'])
 finally:
  for p in reversed(procs):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGCONT);os.killpg(p.pid,signal.SIGINT)
    try:p.wait(10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in files:f.close()
if __name__=='__main__':main()
