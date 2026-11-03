#!/usr/bin/env python3
"""Sequential headless preproduction probes. Runtime authority stays calibration-only."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from run_native_slam_trials import stop

ROOT=Path(__file__).resolve().parents[1]
CASES = [
    ('clockwise',[-.10],2,None,'S00',(0,0)),
    ('anticlockwise',[.10],2,None,'S00',(0,0)),
    ('multi_speed',[.02,-.05,.10],2,None,'S00',(0,0)),
    ('repeated_turns',[.10,-.10,.10,-.10],2,None,'S00',(0,0)),
    ('extended_rotation',[.10],8,None,'S00',(0,0)),
    ('near_wall',[.05],2,None,'S00',(0,2.05)),
    ('near_box',[.05],2,None,'S01',(-1.2,0)),
    ('narrow_passage',[.05],2,None,'S03',(0,0)),
    ('low_friction',[.10,-.10],2,None,'S00',(0,0)),
    ('lidar_loss',[.10],4,'scan_conditioner','S00',(0,0)),
    ('localization_loss',[.10],4,'ekf_node','S00',(0,0)),
    ('command_timeout',[.10],4,None,'S00',(0,0)),
    ('unknown_swept_region',[.10],4,'local_scan_mapper','S00',(0,0)),
]


def main():
    out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
    any_failed = False
    selected = sys.argv[2:]
    if set(selected)-{case[0] for case in CASES}:
        raise SystemExit('Unknown case selection')
    for index,(name,speeds,duration,fault,scenario,xy) in enumerate(CASES):
        if selected and name not in selected:
            continue
        b=out/name;b.mkdir()
        world=next((ROOT/'simulation/scenarios/worlds').glob(scenario.lower()+'_*'))
        tree=ET.parse(world)
        if name=='low_friction':
            for e in tree.findall('.//model[@name="ground_plane"]//friction/ode/*'):e.text='0.5'
        tree.write(b/'world.sdf')
        obstacles=[]
        for model in tree.findall('.//world/model'):
            box=model.find('link/collision/geometry/box/size')
            pose=model.findtext('pose','0 0 0 0 0 0')
            if box is not None:
                p=list(map(float,pose.split()));s=list(map(float,box.text.split()))
                obstacles.append([p[0],p[1],p[5],s[0],s[1]])
        env=dict(os.environ,ROS_DOMAIN_ID=str(150+index),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        with (b/'launch.log').open('w') as log:
            p=subprocess.Popen(['ros2','launch','anc_lidar','realistic_rgb_lidar.launch.py',
                                f'world_file:={b}/world.sdf',f'start_x:={xy[0]}',f'start_y:={xy[1]}',
                                'enable_slam:=true','enable_nav2:=false','rotation_calibration_max_radps:=0.10'],
                               stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            try:
                time.sleep(12)
                fault_pid=None
                if fault:
                    rows=subprocess.check_output(['ps','-eo','pid=,pgid=,args='],text=True)
                    found=[int(r.split()[0]) for r in rows.splitlines() if int(r.split()[1])==p.pid
                           and ('/lib/'+('robot_localization/ekf_node' if fault=='ekf_node' else 'anc_lidar/'+fault)) in r]
                    if len(found)!=1:raise RuntimeError(f'fault process count {len(found)} for {fault}')
                    fault_pid=found[0]
                spec=dict(case_id=name,obstacles=obstacles,speeds=speeds,duration=duration,
                          fault=name if name in ['lidar_loss','localization_loss','command_timeout','unknown_swept_region'] else None,
                          fault_pid=fault_pid)
                (b/'spec.json').write_text(json.dumps(spec,indent=2)+'\n')
                with (b/'probe.log').open('w') as plog:
                    subprocess.run(['python3',str(ROOT/'tests/phase8c_turn_matrix_probe.py'),str(b/'spec.json'),str(b/'result.json')],
                                   env=env,stdout=plog,stderr=subprocess.STDOUT,timeout=80)
            except Exception as exc:
                (b/'runner-error.txt').write_text(str(exc)+'\n')
            finally:
                stop(p)
        result=json.loads((b/'result.json').read_text()) if (b/'result.json').exists() else {}
        any_failed = any_failed or result.get('trial_pass') is not True
        print(name,result.get('trial_pass'),result.get('error'),flush=True)
    raise SystemExit(1 if any_failed else 0)


if __name__=='__main__':main()
