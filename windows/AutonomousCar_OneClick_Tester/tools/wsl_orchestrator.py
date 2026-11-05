#!/usr/bin/env python3
"""Safe, evidence-preserving WSL coordinator for the Desktop one-click tester."""
from __future__ import annotations
import argparse, csv, datetime as dt, json, os, re, signal, subprocess, sys, time, traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import collect_results

STOP = False
SIM = None
ORPHANS = []


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def manifest(run):
    return read_json(Path(run) / "RUN_MANIFEST.json")


def mutate(run, fn):
    path = Path(run) / "RUN_MANIFEST.json"
    data = read_json(path)
    fn(data)
    atomic(path, data)


def record(run, name, status, reason, payload=None, log="", exit_code=None, trial=None):
    root = Path(run).resolve()
    folder = root / "tests" / name
    folder.mkdir(parents=True, exist_ok=True)
    result = folder / "result.json"
    logfile = folder / "output.log"
    if payload is not None:
        atomic(result, payload)
    logfile.write_text(log or "", encoding="utf-8", errors="replace")
    def update(data):
        item = data.setdefault("tests", {}).setdefault(name, {"artifacts": []})
        item.update(status=status, reason=reason, exit_code=exit_code,
                    result_file=str(result.relative_to(root)) if payload is not None else None,
                    log_file=str(logfile.relative_to(root)), trial_id=trial, updated_utc=utc(),
                    artifacts=([str(result.relative_to(root))] if payload is not None else [])
                              + [str(logfile.relative_to(root))])
    mutate(root, update)
    return result


def command(argv, cwd, timeout=30, env=None, grace=8):
    if STOP:
        raise KeyboardInterrupt("run stop has already been requested")
    start = time.monotonic()
    p = subprocess.Popen(argv, cwd=str(cwd), env=env, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                         errors="replace", start_new_session=True)
    try:
        out, _ = p.communicate(timeout=timeout)
        timed = False
        clean = True
    except subprocess.TimeoutExpired:
        timed = True
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            out, _ = p.communicate(timeout=grace)
            clean = True
        except subprocess.TimeoutExpired:
            out, clean = "", False
            ORPHANS.append(p.pid)
            globals()["STOP"] = True
    except KeyboardInterrupt:
        timed = True
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            out, _ = p.communicate(timeout=grace)
        except subprocess.TimeoutExpired:
            out = ""
            clean = False
            ORPHANS.append(p.pid)
            globals()["STOP"] = True
        raise
    state = "TIMEOUT" if timed else ("PASS" if p.returncode == 0 else "FAIL")
    return {"status": state, "exit_code": p.returncode, "timed_out": timed,
            "cleanup_confirmed": clean, "elapsed_s": round(time.monotonic()-start, 2),
            "command": argv, "output": out or ""}


def saved_command(run, name, result, reason):
    return record(run, name, result["status"], reason,
                  {k: v for k, v in result.items() if k != "output"},
                  result["output"], result["exit_code"])


def wsl_memory(run):
    vals = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        m = re.match(r"^(MemTotal|MemAvailable|SwapTotal|SwapFree):\s+(\d+)", line)
        if m:
            vals[m.group(1)] = int(m.group(2)) // 1024
    import shutil
    return {"timestamp_utc": utc(), "wsl_total_mib": vals.get("MemTotal"),
            "wsl_available_mib": vals.get("MemAvailable"),
            "wsl_swap_total_mib": vals.get("SwapTotal"),
            "wsl_swap_used_mib": vals.get("SwapTotal", 0)-vals.get("SwapFree", 0),
            "disk_free_mib": shutil.disk_usage(run).free // (1024*1024)}


def host_snapshot(run):
    try:
        p = Path(run) / "environment" / "current_host_snapshot.json"
        data = read_json(p)
        stamp = dt.datetime.fromisoformat(data["timestampUtc"].replace("Z", "+00:00"))
        data["_age_s"] = max(0, (dt.datetime.now(dt.timezone.utc)-stamp).total_seconds())
        return data
    except Exception as exc:
        return {"_age_s": None, "_error": f"{type(exc).__name__}: {exc}"}


def environment_check(args, cfg):
    root, run = Path(args.project), Path(args.run_dir)
    env = {"windows": host_snapshot(run), "wsl": wsl_memory(run)}
    checks = [("branch", ["git","branch","--show-current"]),
              ("commit", ["git","rev-parse","HEAD"]),
              ("status", ["git","status","--short"]),
              ("ros", ["printenv","ROS_DISTRO"]), ("gazebo",["gz","sim","--versions"]),
              ("pytest",["pytest","--version"])]
    packages = ("anc_lidar","anc_navigation","anc_gazebo","ros_gz_sim",
                "slam_toolbox","nav2_bringup","nav2_map_server")
    checks += [("pkg:"+name,["ros2","pkg","prefix",name]) for name in packages]
    logs=[]
    for name, argv in checks:
        r=command(argv,root,15)
        logs.append({"name":name,"exit_code":r["exit_code"],"output":r["output"][-3000:]})
        if name == "branch": env["git_branch"]=r["output"].strip() if r["exit_code"]==0 else None
        elif name == "commit": env["git_commit"]=r["output"].strip() if r["exit_code"]==0 else None
        elif name == "status":
            lines=[x for x in r["output"].splitlines() if x.strip()]
            env["git_tracked_changes"]=[x for x in lines if not x.startswith("??")]
            env["git_untracked_count"]=sum(x.startswith("??") for x in lines)
        elif name.startswith("pkg:"):
            env.setdefault("ros_packages",{})[name[4:]]={"present":r["exit_code"]==0,
                "prefix":r["output"].strip() if r["exit_code"]==0 else None}
        elif r["exit_code"]==0: env[name]=r["output"].strip()
    env["commands"]=logs
    gpu_path=Path(args.run_dir)/"environment"/"windows_gpu.txt"
    env["graphics"]={"capture_file":"environment/windows_gpu.txt",
                     "output":gpu_path.read_text(encoding="utf-8-sig").strip()
                     if gpu_path.is_file() else "GPU details were not captured"}
    atomic(run/"environment"/"system.json",env)
    packages_ok=all(v["present"] for v in env.get("ros_packages",{}).values())
    ok=(packages_ok and env.get("git_branch")=="feature/phase8c-lidar"
        and env.get("git_commit") and env.get("gazebo"))
    reason="ROS 2 Jazzy, Gazebo, project workspace and required packages verified"
    if not ok: reason="one or more ROS/project prerequisites are missing or unexpected"
    if env["windows"].get("_age_s") is None: reason += "; Windows host snapshot is missing/stale"
    payload={"status":"PASS" if ok else "FAIL","reason":reason,**env}
    record(run,"environment",payload["status"],reason,payload,json.dumps(env,indent=2),
           0 if ok else 1)
    return env,ok


def offline_tests(args,cfg):
    root=Path(args.project)
    targets=["tests/phase8c_lidar_model_test.py",
             "tests/phase8c_common_infrastructure_test.py",
             "tests/phase8c_nav2_config_test.py",
             "tests/phase8c_nav2_runner_test.py",
             "tests/phase8c_readiness_test.py"]
    outcomes=[]
    for path in targets:
        name="offline_"+Path(path).stem.replace("phase8c_","")
        r=command([sys.executable,"-m","pytest","-q",path],root,
                  int(cfg["tests"]["offlineTestTimeoutSeconds"]))
        outcomes.append({"test":path,"status":r["status"],"exit_code":r["exit_code"]})
        saved_command(args.run_dir,name,r,"existing offline pytest (no simulator/motion)")
    ok=all(x["status"]=="PASS" for x in outcomes)
    record(args.run_dir,"offline_tests","PASS" if ok else "FAIL",
           "all selected offline tests passed" if ok else "one or more offline tests failed/timed out",
           {"tests":outcomes,"scope":"offline only"})
    return ok


def readiness(args,name="readiness"):
    root=Path(args.project)
    r=command([sys.executable,str(root/"scripts/phase8c_readiness.py"),
               str(root/"config/phase8c_demo_readiness.json"),str(root),"--brief"],root,30)
    out=r["output"].strip()
    state=out.split("\t",1)[0].upper() if out else "ERROR"
    status="PASS" if r["exit_code"]==0 and state=="READY" else (
        "BLOCKED" if r["exit_code"]==0 and state=="BLOCKED" else r["status"])
    payload={"validator_status":state,"validator_output":out,"exit_code":r["exit_code"]}
    r["status"]=status
    saved_command(args.run_dir,name,r,"readiness validator: "+out)
    return status,payload


def session_check(args):
    root=Path(args.project)
    state=Path.home()/".cache/anc_demo_launcher/current.json"
    if state.exists():
        try:
            contents=read_json(state)
        except Exception as exc:
            contents={"error":f"{type(exc).__name__}: {exc}"}
        record(args.run_dir,"session_safety","BLOCKED",
               "project launcher state exists and was left untouched",
               {"state_file":str(state),"state":contents})
        return False,"an existing project session record exists"
    p=command(["ps","-eo","pid,pgid,args","--no-headers"],root,10)
    matches=[x.strip() for x in p["output"].splitlines() if
             re.search(r"(^|\s)gz\s+sim(\s|$)",x) or
             re.search(r"(^|\s)gzserver(\s|$)",x) or
             "ros2 launch anc_lidar realistic_rgb_lidar.launch.py" in x]
    if matches:
        record(args.run_dir,"session_safety","BLOCKED",
               "Gazebo/realistic LiDAR process already active; no process touched",
               {"processes":matches},p["output"])
        return False,"an existing Gazebo or LiDAR process is active"
    record(args.run_dir,"session_safety","PASS","no existing matching simulation session",
           {"launcher_state":"absent","processes":[]},p["output"],0)
    return True,""


def resource_gate(args,cfg,dry):
    minimum=int(cfg["resource"]["minimumWindowsAvailableMiBHeadless"])
    minwsl=int(cfg["resource"]["minimumWslAvailableMiB"])
    first=host_snapshot(args.run_dir); started=time.monotonic(); checks=[]
    while True:
        host=host_snapshot(args.run_dir); wsl=wsl_memory(args.run_dir)
        checks.append({"utc":utc(),"windows_available_mib":host.get("windowsAvailableMiB"),
          "windows_load_percent":host.get("windowsMemoryLoadPercent"),
          "snapshot_age_s":host.get("_age_s"),"wsl_available_mib":wsl["wsl_available_mib"]})
        enough=(host.get("_age_s") is not None and host["_age_s"]<=20 and
                (host.get("windowsAvailableMiB") or 0)>=minimum and
                (wsl.get("wsl_available_mib") or 0)>=minwsl)
        if enough:
            result={"status":"PASS","checks":checks,"minimum_windows_mib":minimum,
                    "minimum_wsl_mib":minwsl,"first_snapshot":first}
            record(args.run_dir,"resource_gate","PASS","RAM thresholds met",result)
            return True,result
        if dry or time.monotonic()-started>=int(cfg["resource"]["recheckSeconds"]):
            result={"status":"BLOCKED","checks":checks,"minimum_windows_mib":minimum,
                    "minimum_wsl_mib":minwsl,"first_snapshot":first,
                    "waited_s":round(time.monotonic()-started,1)}
            last=checks[-1]
            why=(f"DRY_RUN: Gazebo withheld; current Windows free RAM is "
                 f"{last['windows_available_mib']} MiB (minimum {minimum}), WSL available "
                 f"RAM is {last['wsl_available_mib']} MiB (minimum {minwsl})."
                 if dry else
                 f"headless RAM thresholds were not met within bounded recheck; last sample: "
                 f"Windows {last['windows_available_mib']} MiB, WSL {last['wsl_available_mib']} MiB")
            record(args.run_dir,"resource_gate","BLOCKED",why,result)
            return False,result
        time.sleep(max(1,int(cfg["resource"]["recheckIntervalSeconds"])))


def sim_state(run,state,**extra):
    atomic(Path(run)/"environment"/"simulation_state.json",{"state":state,"updated_utc":utc(),**extra})


def stop_group(proc,seconds=30):
    if proc is None or proc.poll() is not None: return True
    try: os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError: return True
    try: proc.wait(timeout=seconds); return True
    except subprocess.TimeoutExpired: ORPHANS.append(proc.pid); return False


def classify_sensor(data,duration):
    scan=data.get("scan") or {}; cam=data.get("camera") or {}; fail=[]
    if int(scan.get("messages") or 0)<max(2,int(duration*.5)): fail.append("too_few_scan_messages")
    if not (scan.get("measured_wall_rate_hz") or 0)>0: fail.append("scan_rate_missing")
    if int(cam.get("frames") or 0)<1: fail.append("camera_frames_missing")
    if int(cam.get("camera_info_messages") or 0)<1: fail.append("camera_info_missing")
    if int(data.get("imu_messages") or 0)<1: fail.append("imu_missing")
    if int(data.get("filtered_odom_messages") or 0)<1: fail.append("filtered_odom_missing")
    if not isinstance(data.get("lidar_local_map"),dict): fail.append("lidar_map_missing")
    if not isinstance(data.get("rgb_fused_map"),dict): fail.append("rgb_map_missing")
    if not isinstance(data.get("lidar_static_tf"),dict): fail.append("lidar_static_tf_missing")
    if (data.get("safety_state") or {}).get("valid") is not True: fail.append("safety_invalid")
    return {"status":"PASS" if not fail else "FAIL","failures":fail,
            "scan_messages":scan.get("messages"),"scan_rate_hz":scan.get("measured_wall_rate_hz"),
            "camera_frames":cam.get("frames"),"camera_info_messages":cam.get("camera_info_messages"),
            "imu_messages":data.get("imu_messages"),
            "filtered_odom_messages":data.get("filtered_odom_messages"),
            "safety":data.get("safety_state"),"guard":data.get("command_guard_state"),
            "probe":data}


def write_csvs(run,name,data):
    dest=Path(run)/"trajectories"; dest.mkdir(parents=True,exist_ok=True)
    for key in ("truth_samples","output_samples","guard_samples","stop_truth_samples"):
        rows=data.get(key)
        if not isinstance(rows,list) or not rows: continue
        fields=sorted({k for row in rows if isinstance(row,dict) for k in row})
        with (dest/f"{name}_{key}.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore"); w.writeheader()
            w.writerows(x for x in rows if isinstance(x,dict))


def run_live(args,cfg):
    global SIM
    root,run=Path(args.project),Path(args.run_dir)
    log=run/"tests/live_demo/launcher.log"; log.parent.mkdir(parents=True,exist_ok=True)
    (run/"ros_logs").mkdir(exist_ok=True); (run/"gazebo_logs").mkdir(exist_ok=True)
    env=os.environ.copy(); env["ROS_LOG_DIR"]=str(run/"ros_logs")
    env["GZ_SIM_LOG_PATH"]=str(run/"gazebo_logs")
    before={p.name for p in (root/"docs/evidence/phase8c/lidar/demos").glob("*") if p.is_dir()}
    dur=min(180,int(cfg["tests"]["simulationDurationSeconds"]))
    argv=["bash",str(root/"scripts/run_lidar_demo.sh"),"--scenario","S00",
          "--seed",str(cfg["seed"]),"--duration",str(dur)]
    stream=log.open("w",encoding="utf-8",errors="replace")
    proc=subprocess.Popen(argv,cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,
                          start_new_session=True); SIM=proc
    sim_state(run,"RUNNING",process_group=proc.pid)
    try:
        deadline=time.monotonic()+48; topics=False
        while time.monotonic()<deadline and proc.poll() is None and not STOP:
            r=command(["ros2","topic","list"],root,8)
            if r["exit_code"]==0 and {"/realistic/scan","/camera/image_raw","/map"}.issubset(set(r["output"].splitlines())):
                topics=True; break
            time.sleep(2)
        if not topics:
            record(run,"lidar_sensors","FAIL","required sensor/map topics not ready within 48 s",
                   {"topics_ready":False},log.read_text(encoding="utf-8",errors="replace"))
            return
        sensor=run/"tests/lidar_sensors/probe.json"; sensor.parent.mkdir(parents=True,exist_ok=True)
        secs=int(cfg["tests"]["sensorProbeSeconds"])
        pr=command([sys.executable,str(root/"tests/phase8c_live_probe.py"),str(sensor),str(secs)],
                   root,secs+15)
        data=read_json(sensor) if sensor.exists() else {}
        cls=classify_sensor(data,secs) if data else {"status":"FAIL","failures":["probe_output_missing"]}
        record(run,"lidar_sensors",cls["status"],"; ".join(cls["failures"]) or "live sensor checks passed",
               cls,pr["output"],pr["exit_code"])
        cp=run/"tests/rgb_camera_pair/probe.json"; cp.parent.mkdir(parents=True,exist_ok=True)
        cr=command([sys.executable,str(root/"tests/phase8c_camera_pair_probe.py"),str(cp)],root,15)
        cdata=read_json(cp) if cp.exists() else {}
        delta=cdata.get("latest_info_to_mask_delta_s",{})
        cpass=(cr["status"]=="PASS" and cdata.get("camera_info_count",0)>0
               and cdata.get("mask_count",0)>0 and delta.get("count",0)>0
               and (delta.get("max") or 0)<=.10)
        record(run,"rgb_camera_pair","PASS" if cpass else "FAIL",
               "camera-info/mask timing <=100 ms" if cpass else "camera pair evidence missing or >100 ms",
               cdata or {"exit_code":cr["exit_code"]},cr["output"],cr["exit_code"])
        sp=run/"tests/stationary_slam/probe.json"; sp.parent.mkdir(parents=True,exist_ok=True)
        sd=int(cfg["tests"]["stationarySlamSeconds"])
        sr=command([sys.executable,str(root/"tests/phase8c_slam_stationary_test.py"),str(sp),str(sd)],
                   root,sd+20)
        sdata=read_json(sp) if sp.exists() else {}; spass=sr["status"]=="PASS" and sdata.get("diagnostic_pass") is True
        record(run,"stationary_slam","PASS" if spass else ("TIMEOUT" if sr["status"]=="TIMEOUT" else "FAIL"),
               "stationary no-command SLAM diagnostic passed" if spass else sdata.get("error","diagnostic failed"),
               sdata or {"exit_code":sr["exit_code"]},sr["output"],sr["exit_code"])
        if spass:
            prefix=str(run/"maps/s00_online_slam")
            mr=command(["ros2","run","nav2_map_server","map_saver_cli","-f",prefix],root,
                       int(cfg["tests"]["mapSaveTimeoutSeconds"]))
            maps=Path(prefix+".yaml").is_file() and Path(prefix+".pgm").is_file()
            record(run,"map_save","PASS" if mr["status"]=="PASS" and maps else
                   ("TIMEOUT" if mr["status"]=="TIMEOUT" else "FAIL"),
                   "live map saved" if mr["status"]=="PASS" and maps else "YAML/PGM map evidence missing",
                   {"yaml":Path(prefix+".yaml").is_file(),"pgm":Path(prefix+".pgm").is_file()},
                   mr["output"],mr["exit_code"])
        else:
            record(run,"map_save","BLOCKED","stationary SLAM did not pass; map save withheld",
                   {"map_save_attempted":False})
        rp=run/"tests/turning_clearance_probe/probe.json"; rp.parent.mkdir(parents=True,exist_ok=True)
        rr=command([sys.executable,str(root/"tests/phase8c_rotation_gate_probe.py"),str(rp),"S00"],root,20)
        rdata=read_json(rp) if rp.exists() else {}; rpass=rr["status"]=="PASS" and rdata.get("probe_pass") is True
        record(run,"turning_clearance_probe","PASS" if rpass else "FAIL",
               "no-command rotation gate denial verified" if rpass else "expected denial evidence missing",
               rdata or {"exit_code":rr["exit_code"]},rr["output"],rr["exit_code"])
        record(run,"production_turning","SKIPPED","production turning remains disabled; no angular request",
               {"trials_run":False})
        safety=(data.get("safety_state") or {}) if data else {}
        # The guard is expected to be unauthorized before an upstream command exists.
        # Require the live sensor safety gate here; the bounded probe then requests
        # motion through /cmd_vel_nav and verifies the guard's /cmd_vel response.
        safety_forward_ok=(safety.get("valid") is True
                           and safety.get("authorized") is True
                           and float(safety.get("max_linear_speed_mps") or 0.0)>=0.05
                           and abs(float(safety.get("max_angular_speed_radps") or 0.0))<=1e-9)
        allowed=cls["status"]=="PASS" and safety_forward_ok
        n=min(3,int(cfg["tests"]["movingSlamRepeatTrials"]))
        for i in range(1,n+1):
            name=f"moving_slam_trial_{i}"; trial=f"{args.run_id}-S00-moving-{i}"
            if not allowed:
                record(run,name,"BLOCKED","LiDAR sensor validation or explicit forward safety authorization was not ready",
                       {"trial_executed":False,"sensor_status":cls["status"],
                        "safety_valid":safety.get("valid"),
                        "safety_authorized":safety.get("authorized"),
                        "max_linear_speed_mps":safety.get("max_linear_speed_mps"),
                        "max_angular_speed_radps":safety.get("max_angular_speed_radps")},
                       "no velocity request issued",trial=trial); continue
            out=run/"tests"/name/"probe.json"; out.parent.mkdir(parents=True,exist_ok=True)
            tr=command([sys.executable,str(root/"tests/phase8c_slam_timeout_probe.py"),str(out)],
                       root,int(cfg["tests"]["movingTrialTimeoutSeconds"]))
            result=read_json(out) if out.exists() else {}
            result.update(run_id=args.run_id,trial_id=trial,test_id=name)
            atomic(out,result); write_csvs(run,name,result)
            passed=tr["status"]=="PASS" and result.get("status")=="PASS"
            record(run,name,"PASS" if passed else ("TIMEOUT" if tr["status"]=="TIMEOUT" else "FAIL"),
                   "bounded guarded forward trial passed" if passed else result.get("reason","trial did not pass"),
                   result,tr["output"],tr["exit_code"],trial)
            if not passed:
                for j in range(i+1,n+1):
                    record(run,f"moving_slam_trial_{j}","SKIPPED",
                           "prior trial did not pass; repeat withheld",{"trial_executed":False},
                           "no additional motion command",trial=f"{args.run_id}-S00-moving-{j}")
                break
        record(run,"additional_fault_injection","SKIPPED","disabled by safe defaults",
               {"additional_faults_run":False})
        state,payload=readiness(args,"readiness_after_trials")
        record(run,"readiness",state,payload["validator_output"],payload)
        if state!="PASS":
            record(run,"nav2","BLOCKED","evidence-backed readiness is not READY",
                   {"nav2_launched":False},"no Nav2 goal request")
        else:
            record(run,"nav2","NOT TESTED","waiting for the post-SLAM resource and live-safety gate",
                   {"nav2_launched":False})
        return state
    finally:
        stream.close()
        clean=stop_group(proc)
        sim_state(run,"STOPPED" if clean else "CLEANUP_BLOCKED",process_group=proc.pid)
        record(run,"simulation_cleanup","PASS" if clean else "BLOCKED",
               "test-owned launcher stopped by SIGINT" if clean else
               "test process group remained after 30 s; no stronger signal sent",
               {"process_group":proc.pid,"cleanup_confirmed":clean})
        demo=root/"docs/evidence/phase8c/lidar/demos"
        new=[p for p in demo.glob("*") if p.is_dir() and p.name not in before] if demo.is_dir() else []
        for src in new:
            import shutil
            dst=run/"project_demo_evidence"/src.name
            if not dst.exists(): shutil.copytree(src,dst,symlinks=True)


def run_nav2(args,cfg):
    root,run=Path(args.project),Path(args.run_dir)
    demo=root/"docs/evidence/phase8c/lidar/demos"
    before={p.name for p in demo.glob("*") if p.is_dir()} if demo.is_dir() else set()
    env=os.environ.copy(); env["ROS_LOG_DIR"]=str(run/"ros_logs")
    env["GZ_SIM_LOG_PATH"]=str(run/"gazebo_logs")
    log=run/"tests/nav2/launcher.log"; log.parent.mkdir(parents=True,exist_ok=True)
    duration=min(90,int(cfg["tests"].get("navigationTimeoutSeconds",90)))
    argv=["bash",str(root/"scripts/run_lidar_demo.sh"),"--scenario","S00",
          "--seed",str(cfg["seed"]),"--goal","--duration",str(duration)]
    sim_state(run,"RUNNING",nav2=True)
    result=command(argv,root,150,env,grace=30)
    sim_state(run,"STOPPED" if result["cleanup_confirmed"] else "CLEANUP_BLOCKED",nav2=True)
    candidates=[p for p in demo.glob("*") if p.is_dir() and p.name not in before] if demo.is_dir() else []
    reports=[]
    for src in candidates:
        report=src/"navigation-result.json"
        if report.is_file():
            try: reports.append((report,read_json(report)))
            except Exception: pass
        dst=run/"project_demo_evidence"/src.name
        if not dst.exists():
            import shutil
            shutil.copytree(src,dst,symlinks=True)
    evidence=reports[-1][1] if reports else None
    log.write_text(result["output"],encoding="utf-8",errors="replace")
    if result["status"]=="TIMEOUT":
        status="TIMEOUT"; reason="bounded Nav2 goal test exceeded its time limit"
    elif result["status"]!="PASS":
        status="FAIL"; reason="Nav2 launcher returned a nonzero result"
    elif not evidence:
        status="FAIL"; reason="navigation-result.json evidence was not produced"
    elif evidence.get("status")!="PASS":
        status="FAIL"; reason="navigation evaluator did not pass its motion, clearance, safety and completion checks"
    else:
        status="PASS"; reason="evidence-backed S00 Nav2 goal trial passed"
    payload={"nav2_launched":True,"launcher_exit_code":result["exit_code"],
             "launcher_status":result["status"],"navigation_evidence":evidence,
             "evidence_directory":str(reports[-1][0].parent) if reports else None,
             "cleanup_confirmed":result["cleanup_confirmed"]}
    record(run,"nav2",status,reason,payload,result["output"],result["exit_code"])
    record(run,"nav2_cleanup","PASS" if result["cleanup_confirmed"] else "BLOCKED",
           "test-owned Nav2 launcher stopped gracefully" if result["cleanup_confirmed"]
           else "Nav2 process group remained after SIGINT; no stronger signal sent",
           {"cleanup_confirmed":result["cleanup_confirmed"]},log.read_text(errors="replace")
           if log.exists() else "")


def skip_live(args,reason,status):
    for name in ("lidar_sensors","rgb_camera_pair","stationary_slam","map_save",
                 "turning_clearance_probe","production_turning","moving_slam_trial_1",
                 "moving_slam_trial_2","moving_slam_trial_3","additional_fault_injection",
                 "simulation_cleanup"):
        item_status=status
        item_reason=reason
        if name=="production_turning":
            item_status="SKIPPED"; item_reason="production turning trials are disabled; no angular request"
        elif name=="additional_fault_injection":
            item_status="SKIPPED"; item_reason="additional fault injection is disabled by safe defaults"
        elif name=="simulation_cleanup":
            item_status="PASS"; item_reason="no simulator was started; no cleanup was required"
        record(args.run_dir,name,item_status,item_reason,{"simulator_started":False},
               "No Gazebo process was started.")
    if manifest(args.run_dir)["tests"].get("nav2",{}).get("status")!="BLOCKED":
        record(args.run_dir,"nav2",status,reason,{"nav2_launched":False})
    sim_state(args.run_dir,"NOT_STARTED",reason=reason)


def on_signal(_sig,_frame):
    global STOP
    STOP=True
    raise KeyboardInterrupt("graceful stop requested")


def main_run(args):
    global SIM
    signal.signal(signal.SIGINT,on_signal); signal.signal(signal.SIGTERM,on_signal)
    run=Path(args.run_dir).resolve(); root=Path(args.project).resolve()
    cfg=read_json(args.settings)
    if not (run/"RUN_MANIFEST.json").exists():
        collect_results.init_run(argparse.Namespace(run_dir=str(run),run_id=args.run_id,
            mode="DRY_RUN" if args.dry_run=="true" else "FULL",scenario=cfg["scenario"],seed=cfg["seed"]))
    sim_state(run,"PREFLIGHT")
    env,envok=environment_check(args,cfg)
    offline=offline_tests(args,cfg)
    ready,readydata=readiness(args)
    if ready!="PASS":
        record(run,"nav2","BLOCKED","readiness validator did not return READY",
               {"nav2_launched":False,"readiness":readydata},readydata["validator_output"])
    sessions,why=session_check(args)
    enough,gate=resource_gate(args,cfg,args.dry_run=="true")
    if args.dry_run=="true":
        skip_live(args,"DRY_RUN: live simulation deliberately not launched","SKIPPED")
    elif not envok or not offline:
        skip_live(args,"environment/offline prerequisite failed; live tests withheld","BLOCKED")
    elif not sessions:
        skip_live(args,why,"BLOCKED")
    elif not enough:
        skip_live(args,"headless RAM threshold not met; Gazebo was not started","BLOCKED")
    else:
        nav_ready=run_live(args,cfg)
        if cfg["tests"].get("enableNav2OnlyWhenReadinessPasses",True) and nav_ready=="PASS":
            sensor=manifest(run)["tests"].get("lidar_sensors",{})
            try:
                sensor_data=read_json(run/"tests/lidar_sensors/result.json").get("probe",{})
            except Exception:
                sensor_data={}
            guard=sensor_data.get("command_guard_state") or {}
            safety=sensor_data.get("safety_state") or {}
            live_ok=(sensor.get("status")=="PASS" and safety.get("valid") is True
                     and guard.get("authorized") is True
                     and (guard.get("max_linear_speed_mps") or 0)>0)
            if not live_ok:
                record(run,"nav2","BLOCKED","live sensors/safety/command guard did not independently authorize navigation",
                       {"nav2_launched":False,"live_safety_passed":False})
            else:
                sessions2,why2=session_check(args)
                enough2,gate2=resource_gate(args,cfg,False)
                if sessions2 and enough2:
                    run_nav2(args,cfg)
                else:
                    record(run,"nav2","BLOCKED",why2 or
                           "headless resource gate failed before Nav2",
                           {"nav2_launched":False,"resource_gate":gate2})
    pending=[]
    for pgid in ORPHANS:
        try: os.killpg(pgid,signal.SIGINT)
        except ProcessLookupError: continue
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            try: os.killpg(pgid,0)
            except ProcessLookupError: break
            time.sleep(.2)
        else: pending.append(pgid)
    if pending:
        record(run,"cleanup_orphan_groups","BLOCKED","child groups remained after SIGINT; no stronger signal sent",
               {"process_groups":pending})
    winpath=command(["wslpath","-w",str(run)],run,10)
    winpath_value=winpath["output"].strip() if winpath["exit_code"]==0 else None
    mutate(run,lambda d:d.update(windows_run_path=winpath_value,environment={
        "windows":env.get("windows",{}),"wsl":env.get("wsl",{}),
        "git_branch":env.get("git_branch"),"git_commit":env.get("git_commit"),
        "ros_packages":env.get("ros_packages",{}),"gazebo_version":env.get("gazebo"),
        "pytest_version":env.get("pytest"),"graphics":env.get("graphics"),
        "system_file":"environment/system.json","resource_gate":gate},
        branch=env.get("git_branch"),commit=env.get("git_commit"),
        cleanup={"status":"BLOCKED" if pending else "PASS",
        "details":"SIGINT only; no WSL shutdown or broad process termination"}))
    collect_results.finalize(argparse.Namespace(run_dir=str(run),latest_dir=args.latest_dir))
    return 0 if envok and offline else 1


def stop(args):
    run=Path(args.run_dir).resolve(); path=run/".active_wsl_session.json"
    if not path.is_file():
        print("No active standalone WSL session record; no process was signaled."); return 0
    try: s=read_json(path)
    except Exception as e:
        print(f"BLOCKED: unreadable session record: {e}",file=sys.stderr); return 3
    pid=int(s.get("pid",-1)); pgid=int(s.get("pgid",-1)); rid=s.get("run_id")
    if pid<=1 or pgid!=pid or s.get("run_dir")!=str(run) or s.get("runner")!=str(Path(__file__).resolve()):
        print("BLOCKED: run identity mismatch; no signal sent.",file=sys.stderr); return 3
    try:
        raw=Path(f"/proc/{pid}/stat").read_text(); fields=raw[raw.rfind(")")+2:].split()
        live_pgid=int(fields[2]); ticks=fields[19]
        cmd=Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace")
        env=Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
    except FileNotFoundError:
        path.unlink(missing_ok=True); print("Recorded runner is gone; removed only its stale run record."); return 0
    if live_pgid!=pgid or ticks!=str(s.get("start_ticks")) or str(run) not in cmd or f"ANC_TEST_RUN_ID={rid}".encode() not in env:
        print("BLOCKED: live PID/group/start-time/run-marker mismatch; no signal sent.",file=sys.stderr); return 3
    try: os.killpg(pgid,signal.SIGINT)
    except ProcessLookupError: path.unlink(missing_ok=True); return 0
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        try: os.killpg(pgid,0)
        except ProcessLookupError:
            path.unlink(missing_ok=True); print("Verified runner stopped after SIGINT."); return 0
        time.sleep(.2)
    print("BLOCKED: runner remains after 30 s; no stronger signal sent.",file=sys.stderr); return 3


def register(args):
    run=Path(args.run_dir).resolve(); pid=os.getpid(); pgid=os.getpgrp()
    if pgid!=pid: raise RuntimeError(f"runner is not isolated: PID={pid}, PGID={pgid}")
    raw=Path(f"/proc/{pid}/stat").read_text(); ticks=raw[raw.rfind(")")+2:].split()[19]
    atomic(run/".active_wsl_session.json",{"pid":pid,"pgid":pgid,"start_ticks":ticks,
        "runner":str(Path(__file__).resolve()),"run_id":args.run_id,"run_dir":str(run),
        "controller_pid":int(args.controller_pid),"started_utc":utc()})


def main():
    p=argparse.ArgumentParser(); sub=p.add_subparsers(dest="mode",required=True)
    r=sub.add_parser("run")
    for key in ("run_id","run_dir","settings","project","controller_pid"):
        r.add_argument(key)
    r.add_argument("dry_run",choices=("true","false")); r.add_argument("controller_start"); r.add_argument("latest_dir")
    s=sub.add_parser("stop"); s.add_argument("run_dir")
    a=p.parse_args()
    if a.mode=="stop": return stop(a)
    Path(a.run_dir).mkdir(parents=True,exist_ok=True); register(a)
    exit_path=Path(a.run_dir)/"environment"/"worker_exit_code.txt"
    try:
        result=main_run(a)
        exit_path.write_text(str(result)+"\n",encoding="utf-8")
        return result
    except KeyboardInterrupt:
        run=Path(a.run_dir)
        if SIM is not None:
            clean=stop_group(SIM); sim_state(run,"STOPPED" if clean else "CLEANUP_BLOCKED")
        data=manifest(run)
        for item in data["tests"].values():
            if item.get("status")=="NOT TESTED":
                item.update(status="BLOCKED",reason="interrupted before this test started")
        data.setdefault("notes",[]).append("Interrupted; graceful SIGINT cleanup requested.")
        data["cleanup"]={"status":"BLOCKED" if ORPHANS else "PASS","details":"no stronger signal sent"}
        atomic(run/"RUN_MANIFEST.json",data)
        collect_results.finalize(argparse.Namespace(run_dir=str(run),latest_dir=a.latest_dir))
        exit_path.write_text("130\n",encoding="utf-8")
        return 130
    except Exception as e:
        (Path(a.run_dir)/"environment"/"orchestrator-error.log").write_text(traceback.format_exc(),encoding="utf-8")
        print(f"WSL orchestrator ERROR: {type(e).__name__}: {e}",file=sys.stderr)
        exit_path.write_text("2\n",encoding="utf-8")
        return 2
    finally:
        (Path(a.run_dir)/".active_wsl_session.json").unlink(missing_ok=True)


if __name__=="__main__": raise SystemExit(main())
