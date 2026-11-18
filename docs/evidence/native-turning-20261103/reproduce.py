import os,signal,subprocess,time,pathlib,sys
sys.path.insert(0,str(pathlib.Path.cwd()/'scripts'))
from run_native_slam_trials import stop
out=pathlib.Path('docs/evidence/native-turning-20261103')
with (out/'launch.log').open('w') as log:
 p=subprocess.Popen(['ros2','launch','anc_lidar','realistic_rgb_lidar.launch.py',f'world_file:={pathlib.Path.cwd()}/simulation/scenarios/worlds/s00_empty_open_arena.sdf','enable_slam:=true','enable_nav2:=false','rotation_calibration_max_radps:=0.10'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  time.sleep(15)
  with (out/'console.log').open('w') as f: subprocess.run(['python3','tests/phase8c_rotation_trial.py',str(out/'rotation.json'),'0.10','2.0'],stdout=f,stderr=subprocess.STDOUT,timeout=90)
 finally:stop(p)
