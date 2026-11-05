"""Optional trained CNN observer. Has no command publisher or safety authority."""
import argparse,json,time,copy
import cv2,numpy as np,torch,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock,ClockType
from sensor_msgs.msg import Image
from std_msgs.msg import String
from anc_perception.perception_node import _decode_rgb,_image_message
from network import load
class Shadow(Node):
 def __init__(self,path):
  super().__init__('anc_cnn_shadow');torch.set_num_threads(2);self.model=None;self.last=None
  self.pubs={k:self.create_publisher(Image,'/perception/cnn/'+k,qos_profile_sensor_data) for k in ['mask','confidence','overlay']};self.health=self.create_publisher(String,'/perception/cnn/health',10)
  try:self.model=load(path)
  except Exception as exc:self.status(False,'model_load_failed:'+type(exc).__name__)
  self.create_subscription(Image,'/camera/image_raw',self.image,qos_profile_sensor_data)
  self.create_timer(.2,self.stale,clock=Clock(clock_type=ClockType.STEADY_TIME))
 def status(self,valid,reason,**extra):
  m=String();m.data=json.dumps(dict(valid=valid,reason=reason,mode='shadow',motor_authority=False,**extra));self.health.publish(m)
 def stale(self):
  if self.last is None or time.monotonic()-self.last>.5:self.status(False,'input_stale_or_missing')
 def image(self,msg):
  self.last=time.monotonic()
  try:
   if self.model is None:raise ValueError('model_missing')
   stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;age=self.get_clock().now().nanoseconds/1e9-stamp
   if age<-.05 or age>.35:raise ValueError('image_timestamp_invalid')
   rgb=_decode_rgb(msg);x=torch.tensor(cv2.resize(rgb,(160,120)).transpose(2,0,1)[None]/255.,dtype=torch.float32)
   start=time.perf_counter()
   with torch.inference_mode():prob=self.model(x).softmax(1);conf,pred=prob.max(1)
   elapsed=(time.perf_counter()-start)*1000
   mask=cv2.resize(pred[0].numpy().astype(np.uint8),(msg.width,msg.height),interpolation=cv2.INTER_NEAREST);confidence=cv2.resize(conf[0].numpy(),(msg.width,msg.height));mask[confidence<.6]=255
   colors=np.array([[30,190,60],[220,50,50],[70,100,220]]+[[100,100,100]]*253,np.uint8)
   for key,array,encoding in [('mask',mask,'mono8'),('confidence',(confidence*255).astype(np.uint8),'mono8'),('overlay',cv2.addWeighted(rgb,.55,colors[mask],.45,0),'rgb8')]:self.pubs[key].publish(_image_message(array,encoding,msg.header))
   self.status(True,'prediction',inference_ms=elapsed,source_stamp_s=stamp,input_age_s=age,wall_s=time.time())
  except Exception as exc:self.status(False,str(exc))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('checkpoint');args,ros=ap.parse_known_args();rclpy.init(args=ros);node=Shadow(args.checkpoint)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
