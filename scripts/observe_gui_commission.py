#!/usr/bin/env python3
"""Observe an existing commissioning GUI run; capture only Gazebo/RViz windows."""
import argparse,json,os,re,signal,subprocess,sys,time
from pathlib import Path
import psutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('--domain-id',default=43,type=int)
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():raise SystemExit('Output directory must be new')
    monitor=out.with_name(out.name+'-observer');monitor.mkdir(parents=True,exist_ok=False)
    display=None
    command=[sys.executable,'scripts/commission_lidar_s00.py',
             'docs/evidence/native-turn-production-matrix-20261103/envelope.json',str(out),
             '--gui','--rviz','--timeout-s','60','--domain-id',str(args.domain_id)]
    samples=[];captures=[];last_capture=0;processes={};abort=None
    with (monitor/'console.log').open('w') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while child.poll() is None:
                now=time.monotonic();vm=psutil.virtual_memory();swap=psutil.swap_memory()
                selected=[psutil.Process(child.pid)]+psutil.Process(child.pid).children(recursive=True)
                rss=cpu=0
                for proc in selected:
                    try:
                        cached=processes.setdefault(proc.pid,proc)
                        rss+=cached.memory_info().rss;cpu+=cached.cpu_percent(None)
                    except (psutil.NoSuchProcess,psutil.AccessDenied):pass
                samples.append(dict(wall_s=time.time(),available_mib=vm.available/2**20,
                                    aggregate_process_rss_mib=rss/2**20,process_cpu_percent=cpu,
                                    system_cpu_percent=psutil.cpu_percent(None),swap_used_mib=swap.used/2**20))
                if vm.available<512*2**20:
                    abort='observer_memory_floor_512MiB';os.killpg(child.pid,signal.SIGINT);break
                if now-last_capture>=10 and len(captures)<10:
                    tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True).stdout
                    for line in tree.splitlines():
                        if not re.search(r'"[^"]*(?:Gazebo|RViz)[^"]*"',line,re.I):continue
                        match=re.search(r'(0x[0-9a-fA-F]+).*?"([^"]+)".*?(\d+)x(\d+)\+',line)
                        if not match:continue
                        pid_info=subprocess.run(['xprop','-id',match[1],'_NET_WM_PID'],capture_output=True,text=True).stdout
                        pid_match=re.search(r'=\s*(\d+)',pid_info)
                        if not pid_match or int(pid_match[1]) not in {proc.pid for proc in selected}:continue
                        if display is None:
                            import gi
                            gi.require_version('Gdk','3.0');gi.require_version('GdkX11','3.0')
                            from gi.repository import Gdk,GdkX11
                            Gdk.init([]);display=Gdk.Display.get_default()
                        window=GdkX11.X11Window.foreign_new_for_display(display,int(match[1],16))
                        if window is None:continue
                        width,height=window.get_width(),window.get_height()
                        if width<500 or height<400:continue
                        if width<1100:
                            window.move_resize(0,0,1200,740);display.sync();time.sleep(.1)
                            width,height=window.get_width(),window.get_height()
                        pixbuf=Gdk.pixbuf_get_from_window(window,0,0,width,height)
                        if pixbuf:
                            name=f'window-{len(captures):02d}.png';pixbuf.savev(str(monitor/name),'png',[],[])
                            captures.append(dict(file=name,title=match[2],wall_s=time.time(),monotonic_s=time.monotonic(),width=width,height=height))
                    last_capture=now
                time.sleep(.5)
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    # Terminate only descendants of this observer's owned commissioning process.
                    for proc in psutil.Process(child.pid).children(recursive=True):
                        try:proc.terminate()
                        except psutil.NoSuchProcess:pass
                    child.terminate();child.wait(timeout=10)
    report=dict(command=command,returncode=child.returncode,abort_reason=abort,captures=captures,
                minimum_available_mib=min((s['available_mib'] for s in samples),default=None),
                peak_aggregate_process_rss_mib=max((s['aggregate_process_rss_mib'] for s in samples),default=None),
                samples=samples)
    (monitor/'observer.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Observer saved',monitor,'commission exit',child.returncode,'screenshots',len(captures))
    return child.returncode if child.returncode else (1 if abort else 0)


if __name__=='__main__':raise SystemExit(main())
