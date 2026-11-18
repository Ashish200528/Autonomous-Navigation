"""Optional ArUco camera observer: source-frame pose, overlay and fail-closed health."""
import json,time
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from anc_perception.perception_node import _decode_rgb,_image_message
from aruco_pose import detect,overlay,heading_in_marker
class ArucoNode(Node):
 def __init__(self):
  super().__init__('anc_aruco_observer');self.declare_parameter('camera_pitch_down_rad',.8);self.info=None;self.last=None;self.info_wall=None
  self.pose=self.create_publisher(PoseStamped,'/parking/marker_pose',10)
  self.health=self.create_publisher(String,'/parking/perception_health',10)
  self.overlay=self.create_publisher(Image,'/parking/marker_overlay',qos_profile_sensor_data)
  self.create_subscription(CameraInfo,'/camera/camera_info',self.calibration,qos_profile_sensor_data)
  self.create_subscription(Image,'/camera/image_raw',self.image,qos_profile_sensor_data)
  self.create_timer(.1,self.stale,clock=Clock(clock_type=ClockType.STEADY_TIME))
 def calibration(self,m):self.info=m;self.info_wall=time.monotonic()
 def status(self,result):
  m=String();m.data=json.dumps(result,allow_nan=False);self.health.publish(m)
 def stale(self):
  if self.last is None or time.monotonic()-self.last>.5:self.status(dict(valid=False,reason='camera_or_marker_stale',motor_authority=False))
 def image(self,m):
  self.last=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  try:
   if self.info is None or time.monotonic()-self.info_wall>.5:raise ValueError('calibration_missing_or_stale')
   info=self.info
   if (info.width,info.height,info.header.frame_id)!=(m.width,m.height,m.header.frame_id):raise ValueError('calibration_image_mismatch')
   it=info.header.stamp.sec+info.header.stamp.nanosec/1e9
   if abs(it-stamp)>.1:raise ValueError('calibration_stamp_mismatch')
   if info.distortion_model not in ['plumb_bob','']:raise ValueError('unsupported_distortion')
   rgb=_decode_rgb(m);result=detect(rgb,np.asarray(info.k).reshape(3,3),info.d,stamp,self.get_clock().now().nanoseconds/1e9)
   image=cv2.cvtColor(overlay(rgb,result),cv2.COLOR_BGR2RGB);self.overlay.publish(_image_message(image,'rgb8',m.header))
   if result['valid']:
    result['robot_heading_in_marker_rad']=heading_in_marker(result['rotation_vector'],self.get_parameter('camera_pitch_down_rad').value)
    p=PoseStamped();p.header=m.header;p.pose.position.x,p.pose.position.y,p.pose.position.z=result['translation_optical_m'];quat=Rotation.from_rotvec(result['rotation_vector']).as_quat();p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,quat);self.pose.publish(p)
   self.status(result)
  except Exception as exc:self.status(dict(valid=False,reason=str(exc),source_stamp_s=stamp,motor_authority=False))
def main():
 rclpy.init();n=ArucoNode()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
