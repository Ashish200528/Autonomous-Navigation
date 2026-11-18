"""Static parking-camera evaluator. Ground truth exists only in this script."""
import argparse,copy,json,math,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as E
import cv2,numpy as np,psutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from collect import array,put
from aruco_pose import marker_image,detect,overlay,MARKER_SIZE_M,heading_in_marker
ROOT=Path(__file__).resolve().parents[2]
def ry(a):return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
def rz(a):return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
def main():
 ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('--views',type=int,default=9,choices=range(1,10));a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 if psutil.virtual_memory().available<2048*2**20:raise RuntimeError('memory preflight')
 panel=np.full((500,500),255,np.uint8);panel[50:450,50:450]=marker_image(400);cv2.imwrite(str(a.output/'marker23.png'),panel)
 # Explicit world XY / marker XY vertex and UV correspondence, +Z front face.
 h=.225/2;(a.output/'panel.obj').write_text(f'v {-h} {h} 0\nv {h} {h} 0\nv {h} {-h} 0\nv {-h} {-h} 0\nvt 0 1\nvt 1 1\nvt 1 0\nvt 0 0\nvn 0 0 1\nf 1/1/1 3/3/1 2/2/1\nf 1/1/1 4/4/1 3/3/1\n')
 prototype=E.parse(ROOT/'docs/academic/phase9-data/worlds/scene-000.sdf').getroot();robot=prototype.find(".//model[@name='camera_robot_1']")
 positions=[(0,0,0),(-.08,0,0),(.06,0,0),(0,.05,0),(0,-.05,0),(0,0,.15),(0,0,-.15),(-.18,0,0),(-.32,0,0)]
 f=320/(2*math.tan(1.0471975512/2));k=np.array([[f,0,160],[0,f,120],[0,0,1]]);rows=[];peak=0;minmem=psutil.virtual_memory().available
 for i,(x,y,yaw) in enumerate(positions[:a.views]):
  root=E.parse(ROOT/'simulation/ros2_ws/src/anc_gazebo/worlds/phase2_test_world.sdf').getroot();world=root.find('world');world.remove(world.find("model[@name='phase2_wall']"))
  car=copy.deepcopy(robot);car.set('name','parking_robot');car.find('pose').text=f'{x} {y} 0 0 0 {yaw}'
  link=car.find('link')
  for sem in car.findall('.//sensor[@type="segmentation"]'):link.remove(sem)
  rgb=car.find('.//sensor[@type="camera"]');rgb.find('topic').text='/parking/rgb';world.append(car)
  model=E.SubElement(world,'model',name='parking_marker');put(model,'static','true');put(model,'pose','.35 0 .004 0 0 0');l=E.SubElement(model,'link',name='panel');v=E.SubElement(l,'visual',name='printed_marker');g=E.SubElement(v,'geometry');mesh=E.SubElement(g,'mesh');put(mesh,'uri',(a.output/'panel.obj').resolve().as_uri());mat=E.SubElement(v,'material');put(mat,'ambient','1 1 1 1');put(mat,'diffuse','1 1 1 1');metal=E.SubElement(E.SubElement(mat,'pbr'),'metal');put(metal,'albedo_map',(a.output/'marker23.png').resolve().as_uri());put(metal,'roughness',1)
  # Thin bay borders are separate from the marker and do not command the robot.
  for j,side in enumerate([-.25,.25]):
   m=E.SubElement(world,'model',name=f'bay_border_{j}');put(m,'static','true');put(m,'pose',f'0 {side} .002 0 0 0');l=E.SubElement(m,'link',name='border');v=E.SubElement(l,'visual',name='border');put(E.SubElement(E.SubElement(v,'geometry'),'box'),'size','.8 .01 .004');mat=E.SubElement(v,'material');put(mat,'ambient','0 1 1 1');put(mat,'diffuse','0 1 1 1')
  path=(a.output/f'view-{i:02d}.sdf').resolve();E.ElementTree(root).write(path);node=Node();frames={}
  node.subscribe(Image,'/parking/rgb',lambda m:frames.update({(m.header.stamp.sec,m.header.stamp.nsec):m}))
  with (a.output/f'view-{i:02d}.log').open('w') as log:
   proc=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   try:
    end=time.monotonic()+25
    while len(frames)<4 and time.monotonic()<end:
     mem=psutil.virtual_memory().available;minmem=min(minmem,mem)
     if mem<768*2**20:raise RuntimeError('memory floor')
     peak=max(peak,psutil.Process(proc.pid).memory_info().rss);time.sleep(.1)
    if not frames:raise RuntimeError('no Gazebo camera frames')
    Rwo=rz(yaw)@ry(.8)@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    C=np.array([x,y,0])+rz(yaw)@np.array([.1,0,.24]);gt=Rwo.T@(np.array([.35,0,.004])-C);up=Rwo@np.array([0.,-1.,0.]);gt_heading=math.atan2(up[0],up[1])
    samples=[]
    for t,m in sorted(frames.items()):
     image=array(m);result=detect(image,k,[0]*5,t[0]+t[1]/1e9,t[0]+t[1]/1e9)
     if result['valid']:
      result['robot_heading_in_marker_rad']=heading_in_marker(result['rotation_vector'],.8);result['robot_heading_error_rad']=math.atan2(math.sin(result['robot_heading_in_marker_rad']-yaw),math.cos(result['robot_heading_in_marker_rad']-yaw));result['distance_error_m']=result['distance_m']-float(np.linalg.norm(gt));result['translation_error_m']=float(np.linalg.norm(np.array(result['translation_optical_m'])-gt));result['heading_error_rad']=math.atan2(math.sin(result['image_up_heading_rad']-gt_heading),math.cos(result['image_up_heading_rad']-gt_heading))
     samples.append(result)
    cv2.imwrite(str(a.output/f'view-{i:02d}-rgb.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR));cv2.imwrite(str(a.output/f'view-{i:02d}-detection.png'),overlay(image,result))
    rows.append(dict(view=i,robot_pose=[x,y,yaw],truth_distance_m=float(np.linalg.norm(gt)),truth_translation_optical_m=gt.tolist(),truth_image_up_heading_rad=gt_heading,samples=samples))
   finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    node.unsubscribe('/parking/rgb')
  (a.output/'result.json').write_text(json.dumps(dict(classification='STATIONARY_GAZEBO_PARKING_PERCEPTION',marker_id=23,marker_size_m=.18,intrinsics=k.tolist(),parking_target_world=[-.05,0,0],staging_pose_world=[-.32,0,0],samples=rows,peak_server_rss_bytes=peak,min_available_bytes=minmem,runtime_truth_used=False,motion_authorized=False),indent=2)+'\n')
  print(i,[(s['valid'],s['reason']) for s in samples],flush=True)
if __name__=='__main__':main()
