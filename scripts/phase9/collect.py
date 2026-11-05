#!/usr/bin/env python3
"""Offline-only stationary RGB/semantic capture; no ROS or command publishers."""
import argparse,copy,hashlib,json,math,os,signal,subprocess,time,threading
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np,psutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
ROOT=Path(__file__).resolve().parents[2]
def put(parent,name,text):ET.SubElement(parent,name).text=str(text)
def label(model,value):
 p=ET.SubElement(model,'plugin',filename='gz-sim-label-system',name='gz::sim::systems::Label');put(p,'label',value)
def stamp(m):return (m.header.stamp.sec,m.header.stamp.nsec)
def array(m):
 if m.step!=m.width*3:raise ValueError('unsupported image stride')
 return np.frombuffer(m.data,np.uint8).reshape(m.height,m.width,3).copy()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('--scenes',type=int,default=1);args=ap.parse_args()
 if not 1<=args.scenes<=40:raise SystemExit('scenes 1..40')
 if args.output.exists():raise SystemExit('new output directory required')
 if psutil.virtual_memory().available<2048*2**20:raise SystemExit('insufficient RAM')
 args.output.mkdir(parents=True)
 for d in ['images','masks','metadata','splits','worlds']: (args.output/d).mkdir()
 # Convert the original camera definition without editing any robotics sources.
 urdf=subprocess.check_output(['ros2','run','xacro','xacro',str(ROOT/'simulation/ros2_ws/src/anc_description/urdf/anc_robot.urdf.xacro'),'camera_mount_z_m:=0.16','camera_pitch_down_rad:=0.8','camera_update_rate_hz:=2'])
 (args.output/'robot.urdf').write_bytes(urdf)
 sdf=subprocess.check_output(['gz','sdf','-p',str(args.output/'robot.urdf')]);robot=ET.fromstring(sdf).find('model')
 for p in list(robot.findall('plugin')):robot.remove(p)
 put(robot,'static','true')
 for link in robot.findall('link'):
  for sensor in list(link.findall('sensor')):
   if sensor.get('type')!='camera':link.remove(sensor)
 camera=robot.find('.//sensor[@type="camera"]')
 records=[];peak=0;minimum=psutil.virtual_memory().available
 for scene in range(args.scenes):
  root=ET.parse(ROOT/'simulation/ros2_ws/src/anc_gazebo/worlds/phase2_test_world.sdf').getroot();world=root.find('world')
  for m in world.findall('model'):label(m,10 if m.get('name')=='ground_plane' else 20)
  wall=world.find("model[@name='phase2_wall']/pose");wall.text=f'{.15+.07*(scene%5)} {-.45+.15*(scene%7)} .30 0 0 {scene*.17}'
  # Reuse the existing wall geometry for a second labelled box in half the scenes.
  if scene%2:
   box=copy.deepcopy(world.find("model[@name='phase2_wall']"));box.set('name','dataset_box');box.find('pose').text='.7 .5 .2 0 0 .3'
   for size in box.findall('.//box/size'):size.text='.35 .35 .4'
   world.append(box)
  topics=[]
  for view in range(3):
   model=copy.deepcopy(robot);model.set('name',f'camera_robot_{view}');label(model,30)
   put(model,'pose',f'{-.2-.1*(scene%3)} {-.4+.4*view} 0 0 0 {-.3+.3*view+.04*(scene%3)}')
   rgb=model.find('.//sensor[@type="camera"]');rgb.find('topic').text=f'/phase9/v{view}/rgb'
   rgb.find('camera/camera_info_topic').text=f'/phase9/v{view}/camera_info'
   sem=copy.deepcopy(rgb);sem.set('name','offline_semantic');sem.set('type','segmentation');sem.find('topic').text=f'/phase9/v{view}/semantic';put(sem.find('camera'),'segmentation_type','semantic')
   model.find('.//sensor[@type="camera"]/..').append(sem);world.append(model)
   topics.append((rgb.find('topic').text,sem.find('topic').text+'/labels_map'))
  path=(args.output/'worlds'/f'scene-{scene:03d}.sdf').resolve();ET.ElementTree(root).write(path)
  node=Node();frames={};lock=threading.Lock()
  def receive(key):
   def cb(m):
    with lock:frames.setdefault(key,{})[stamp(m)]=m
   return cb
  for v,(rgb,sem) in enumerate(topics):
   node.subscribe(Image,rgb,receive((v,'rgb')));node.subscribe(Image,sem,receive((v,'sem')))
  with (args.output/'worlds'/f'scene-{scene:03d}.log').open('w') as log:
   proc=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   try:
    deadline=time.monotonic()+35;captured=set()
    while time.monotonic()<deadline and len(captured)<3:
     mem=psutil.virtual_memory().available;minimum=min(minimum,mem)
     if mem<768*2**20:raise RuntimeError('memory floor')
     peak=max(peak,sum(p.memory_info().rss for p in [psutil.Process(proc.pid)]+psutil.Process(proc.pid).children(recursive=True) if p.is_running()))
     with lock:
      for v in range(3):
       if v in captured:continue
       common=set(frames.get((v,'rgb'),{}))&set(frames.get((v,'sem'),{}))
       if not common:continue
       t=max(common);rgb=frames[v,'rgb'][t];sem=frames[v,'sem'][t];im=array(rgb);raw=array(sem)
       print('semantic values',np.unique(raw.reshape(-1,3),axis=0)[:12].tolist(),flush=True)
       if not np.array_equal(raw[:,:,0],raw[:,:,1]) or not np.array_equal(raw[:,:,0],raw[:,:,2]):raise ValueError('semantic channel disagreement')
       mask=np.full(raw.shape[:2],255,np.uint8)
       for value,cls in [(10,0),(20,1),(30,2)]:mask[raw[:,:,0]==value]=cls
       # Ignore antialiased class boundaries rather than inventing precise labels.
       edge=cv2.morphologyEx(mask,cv2.MORPH_GRADIENT,np.ones((3,3),np.uint8))>0;mask[edge]=255
       name=f's{scene:03d}-v{v}';cv2.imwrite(str(args.output/'images'/f'{name}.png'),cv2.cvtColor(im,cv2.COLOR_RGB2BGR));cv2.imwrite(str(args.output/'masks'/f'{name}.png'),mask)
       records.append(dict(id=name,scene=scene,view=v,stamp=list(t),rgb_stamp=list(stamp(rgb)),semantic_stamp=list(stamp(sem)),width=rgb.width,height=rgb.height,camera_pose=camera.find('pose').text,intrinsics=[320/(2*math.tan(1.0471975512/2)),320/(2*math.tan(1.0471975512/2)),160,120],label_source='Gazebo Label system and semantic segmentation sensor',semantic_mapping={'10':0,'20':1,'30':2,'0':255},static_capture=True,semantic_pixel_format=sem.pixel_format_type));captured.add(v)
     time.sleep(.1)
    if len(captured)!=3:raise RuntimeError(f'synchronized capture missing: topics={topics}, received={list(frames)}')
   finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    for rgb,sem in topics:node.unsubscribe(rgb);node.unsubscribe(sem)
  (args.output/'metadata'/'capture.json').write_text(json.dumps(dict(records=records,peak_child_rss_bytes=peak,minimum_available_bytes=minimum),indent=2)+'\n')
 print('captured',len(records))
if __name__=='__main__':main()
