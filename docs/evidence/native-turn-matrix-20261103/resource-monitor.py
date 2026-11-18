import json,pathlib,time,subprocess
out=pathlib.Path('docs/evidence/native-turn-matrix-20261103/resources.jsonl')
with out.open('w') as f:
 for _ in range(1000):
  rows=subprocess.check_output(['ps','-eo','pid=,comm=,rss=,pcpu=,args='],text=True).splitlines()
  if not any('python3 scripts/run_turn_matrix.py' in r for r in rows):break
  mem={l.split(':')[0]:int(l.split()[1]) for l in pathlib.Path('/proc/meminfo').read_text().splitlines() if l.startswith(('MemAvailable:','MemTotal:','SwapFree:'))}
  selected=[r.split(None,4) for r in rows if ('/opt/ros/jazzy/lib/' in r or '/simulation/ros2_ws/install/' in r or 'python3 /home/ashish/AutonomousNavigationCar/tests/phase8c_turn_matrix_probe.py' in r)]
  mem.update(wall_s=time.time(),project_rss_kib=sum(int(x[2]) for x in selected),project_lifetime_cpu_percent=sum(float(x[3]) for x in selected))
  f.write(json.dumps(mem)+'\n');f.flush();time.sleep(.5)
