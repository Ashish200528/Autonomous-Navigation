"""Bounded sensing-only live CNN check, launching and cleaning only owned processes."""
import json,os,signal,subprocess,sys,time,argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
ROOT=Path(__file__).resolve().parents[2]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('checkpoint');ap.add_argument('output',type=Path);a=ap.parse_args();a.output.mkdir(exist_ok=False,parents=True);procs=[];logs=[];rows=[];inputs={};outputs=[]
 try:
  commands=[['ros2','launch','anc_bringup','sim.launch.py','gui:=false','camera_mount_z_m:=0.16','camera_pitch_down_rad:=0.8','camera_update_rate_hz:=2'],[str(ROOT/'.venv/phase9/bin/python'),str(ROOT/'scripts/phase9/shadow_node.py'),a.checkpoint,'--ros-args','-p','use_sim_time:=true']]
  for i,cmd in enumerate(commands):
   f=(a.output/f'process-{i}.log').open('w');logs.append(f);procs.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
  rclpy.init();n=Node('phase9_live_evaluator')
  n.create_subscription(String,'/perception/cnn/health',lambda m:rows.append(json.loads(m.data)),10)
  n.create_subscription(Image,'/camera/image_raw',lambda m:inputs.update({(m.header.stamp.sec,m.header.stamp.nanosec):m}),qos_profile_sensor_data)
  n.create_subscription(Image,'/perception/cnn/mask',lambda m:outputs.append((m.header.stamp.sec,m.header.stamp.nanosec)),qos_profile_sensor_data)
  from anc_perception.perception_node import _decode_rgb
  import cv2,numpy as np
  def overlay(m):
   key=(m.header.stamp.sec,m.header.stamp.nanosec)
   if key in inputs:
    pair=np.hstack([_decode_rgb(inputs[key]),_decode_rgb(m)])
    cv2.imwrite(str(a.output/'live-rgb-and-cnn.png'),cv2.cvtColor(pair,cv2.COLOR_RGB2BGR))
  n.create_subscription(Image,'/perception/cnn/overlay',overlay,qos_profile_sensor_data)
  end=time.monotonic()+35
  while time.monotonic()<end and sum(r.get('valid',False) for r in rows)<8:rclpy.spin_once(n,timeout_sec=.1)
  valid=[r for r in rows if r.get('valid')];matched=all(t in inputs for t in outputs)
  report=dict(status='SIMULATION_PASS' if len(valid)>=8 and outputs and matched else 'FAIL',valid_predictions=len(valid),mask_messages=len(outputs),source_stamp_preserved=matched,health=rows,node_publishers=[p[0] for p in n.get_publisher_names_and_types_by_node('anc_cnn_shadow','/')])
  # Publisher list is queried separately below; graph tuples carry topic names.
  (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n');n.destroy_node();rclpy.shutdown();print(report['status'])
 finally:
  for p in reversed(procs):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(12)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in logs:f.close()
if __name__=='__main__':main()
