#!/usr/bin/env python3
"""Aggregate saved Autonomous Navigation Car evidence without conflating profiles."""
import csv, json, subprocess
from datetime import datetime, timezone
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/evidence/phase8c/lidar"
def load(path):
    return json.loads((ROOT / path).read_text())

p6 = load("docs/evidence/phase6/phase6-summary.json")
p7 = load("docs/evidence/phase7/dataset/accepted-phase7-final-verified/dataset-results.json")
p8 = load("docs/evidence/phase8/phase8-summary.json")
p8c = load("docs/evidence/phase8c/lidar/phase8c-summary.json")
rot = load("docs/evidence/phase8c/lidar/stage1-rotation-trials-retry.json")
slam = load("docs/evidence/phase8c/lidar/stage2-s00-stationary-slam-tuned.json")
reachable = [x for x in p6["scenarios"] if x["scenario_id"] in ["S00","S01","S02","S03","S04"]]
metrics = [x["metrics"] for x in reachable]
trials = rot["trials"]
p7m = p7["aggregate_metrics"]
p8a = p8["aggregate"]
free_output = subprocess.check_output(["free","-b"], text=True).splitlines()
free = free_output[1].split()
swap = free_output[2].split()
mem_available = int(free[6])
disk_available = int(subprocess.check_output(["df","-B1","--output=avail",str(ROOT)],text=True).splitlines()[1].strip())
idle = {
  "captured_at_utc": datetime.now(timezone.utc).isoformat(),
  "state": "idle_after_simulator_shutdown",
  "memory_total_bytes": int(free[1]),
  "memory_used_bytes": int(free[2]),
  "memory_available_bytes": mem_available,
  "memory_available_gib": round(mem_available / 1024**3,3),
  "swap_total_bytes": int(swap[1]),
  "swap_used_bytes": int(swap[2]),
  "repository_filesystem_available_bytes": disk_available,
  "simulator_processes_running": False,
  "source_command": "free -b and df -B1"
}
(OUT/"stage6-idle-resource-observation.json").write_text(json.dumps(idle,indent=2)+"\n")
mean = lambda field: sum(x[field] for x in metrics)/len(metrics)
rot_stop = [x["zero_request_to_zero_output_wall_s"] for x in trials]
rot_slow = [x["first_slow_pose_rate_wall_s_after_zero_request"] for x in trials]
rot_settle = [x["zero_request_to_settled_wall_s"] for x in trials]
probes = {}
for n in ["stage1-s00-near-wall-gate","stage1-s01-near-box-gate","stage1-s03-constriction-gate"]:
    d=load("docs/evidence/phase8c/lidar/"+n+".json")
    g=d["gate_snapshot"]
    probes[n]={"authorized":g["authorized"],"gate_reason":g["reason"],"clearance_reason":g["rotation_clearance"]["reason"],"occupied_cells":g["rotation_clearance"]["occupied_cells"],"unknown_cells":g["rotation_clearance"]["unknown_cells"],"truth_xy_drift_m":d["truth_xy_drift_m"],"truth_yaw_change_rad":d["truth_yaw_change_rad"]}
parking_log=(OUT/"stage5-parking-controller-tests.log").read_text()
parking_passes=parking_log.count(" ... ok")
p6s09=next(x for x in p6["scenarios"] if x["scenario_id"]=="S09")
summary={
 "generated_at_utc":datetime.now(timezone.utc).isoformat(),
 "branch":"feature/phase8c-lidar",
 "classification":"PARTIAL_WITH_SAFETY_BLOCKERS",
 "interpretation":"These are separate profile-specific experiments, not a head-to-head navigation benchmark.",
 "stages":[
  {"stage":1,"status":"PASS_WITH_LIMITS_CALIBRATION_ONLY","production_yaw_enabled":False,"combined_motion_tested":False},
  {"stage":2,"status":"PASS_WITH_LIMITS_STATIONARY_MAPPING_ONLY","moving_localization_validated":False,"drift_improvement_demonstrated":False},
  {"stage":3,"status":"BLOCKED_NOT_RUN"},
  {"stage":4,"status":"BLOCKED_NOT_RUN"},
  {"stage":5,"status":"BLOCKED_LIVE","parking_unit_tests_passed":parking_passes,"runtime_wired":False},
  {"stage":6,"status":"PARTIAL_WITH_LIMITS"},
  {"stage":7,"status":"PARTIAL_DESIGN_ONLY","physical_validation":"PENDING_HARDWARE"},
  {"stage":8,"status":"PASS_WITH_LIMITS_HEADLESS_RECORDING_AND_GUIDE"}
 ],
 "capabilities":[
  {"name":"LiDAR sensing","status":"PASS_WITH_LIMITS"},
  {"name":"local mapping","status":"PASS_WITH_LIMITS"},
  {"name":"safe rotation","status":"PASS_WITH_LIMITS_CALIBRATION_ONLY"},
  {"name":"SLAM and moving localization","status":"BLOCKED"},
  {"name":"autonomous goal navigation","status":"BLOCKED"},
  {"name":"obstacle avoidance","status":"BLOCKED"},
  {"name":"replanning","status":"BLOCKED"},
  {"name":"autonomous parking","status":"BLOCKED_LIVE"},
  {"name":"fault handling","status":"PASS_WITH_LIMITS"},
  {"name":"full simulation evaluation","status":"BLOCKED_INCOMPLETE"},
  {"name":"hardware deployment readiness","status":"PARTIAL"},
  {"name":"physical hardware validation","status":"PENDING_HARDWARE"}
 ],
 "comparisons":{
  "phase6_oracle_reference":{"status":p6["overall_status"],"S00_S04_goal_successes":sum(x["action"]["status"]=="SUCCEEDED" for x in reachable),"reachable_scenarios":len(reachable),"S09_action_status":p6s09["action"]["status"],"S09_expected_no_route":True,"mean_truth_path_m_S00_S04":round(mean("truth_path_length_m"),5),"mean_final_xy_error_m_S00_S04":round(mean("final_xy_goal_error_m"),5),"mean_localization_drift_m_S00_S04":round(mean("final_localization_drift_xy_m"),5),"collisions_S00_S04":sum(x["contact_collision_events"] for x in metrics),"not_realistic_LiDAR_navigation":True,"source":"docs/evidence/phase6/phase6-summary.json"},
  "phase7_rgb_perception":{"status":p7["assessment"],"views":p7["sample_count"],"false_free_rate":p7m["false_free_rate_on_known_occupied"],"known_floor_free_coverage":p7m["useful_free_coverage_on_known_floor"],"occupied_precision":p7m["occupied_precision"],"occupied_recall":p7m["occupied_recall"],"unknown_coverage":p7m["unknown_coverage_total"],"median_fps":p7m["measured_wall_fps"]["median"],"p95_latency_ms":p7m["processing_latency_ms"]["p95"],"source":"docs/evidence/phase7/dataset/accepted-phase7-final-verified/dataset-results.json"},
  "phase8_rgb_stop_only":{"status":p8["classification"],"runs_passed":p8a["live_runs_passed"],"runs":p8a["live_runs"],"cmd_vel_samples":p8a["actual_cmd_vel_samples"],"nonzero_samples":p8a["actual_nonzero_cmd_vel_samples"],"max_truth_displacement_m":p8a["maximum_evaluator_displacement_m"],"max_sampled_process_rss_kib":p8a["max_sampled_process_rss_kib"],"min_available_memory_kib":p8a["min_mem_available_kib"],"source":"docs/evidence/phase8/phase8-summary.json"},
  "phase8c_lidar":{"status":p8c["assessments"]["actual_movement"],"scan_hz":p8c["sensor"]["measured_rate_hz"],"usable_beams":p8c["sensor"]["usable_beams"],"total_beams":p8c["sensor"]["beams"],"usable_fraction":p8c["sensor"]["usable_fraction"],"forward_corridor_m":p8c["safety"]["observed_forward_corridor_m"],"required_corridor_m_at_0_18_mps":p8c["safety"]["required_corridor_at_0_18_mps"],"production_angular_speed_radps":p8c["safety"]["max_angular_speed_radps"],"local_map":p8c["lidar_map"],"source":"docs/evidence/phase8c/lidar/phase8c-summary.json"},
  "fusion_effect":{"tested_scene":"S00 clear corridor","fused_equals_LiDAR_only":p8c["fused_map_matches_lidar_in_tested_s00_corridor"],"RGB_added_metric_cells":0,"interpretation":"No measurable RGB map contribution in this S00 snapshot.","source":"docs/evidence/phase8c/lidar/phase8c-summary.json"},
  "rotation_calibration":{"trials_passed":sum(bool(x["trial_pass"]) for x in trials),"trials":len(trials),"requested_radps":rot["requested_speeds_radps"],"max_center_drift_m":max(x["center_drift_m"] for x in trials),"max_residual_yaw_rad":max(abs(x["residual_yaw_after_zero_output_rad"]) for x in trials),"command_to_zero_output_s_range":[min(rot_stop),max(rot_stop)],"first_slow_pose_rate_after_stop_s_range":[min(rot_slow),max(rot_slow)],"settle_s_range":[min(rot_settle),max(rot_settle)],"max_abs_yaw_change_rad":max(abs(x["yaw_change_until_zero_rad"]) for x in trials),"production_yaw_enabled":rot["production_angular_authority_enabled"],"swept_disk_radius_m":rot["preflight_gate"]["rotation_clearance"]["required_radius_m"],"combined_motion_authorized":rot["preflight_gate"]["combined_motion_authorized"],"clearance_probes":probes,"source":"docs/evidence/phase8c/lidar/stage1-rotation-trials-retry.json"},
  "stationary_slam":{"status":"PASS_WITH_LIMITS_STATIONARY_ONLY","wall_s":slam["duration_observed_wall_s"],"sim_s":slam["duration_observed_sim_s"],"scan_messages":slam["scan_messages"],"map_messages":slam["map_messages"],"map_to_odom_messages":slam["map_to_odom_tf_messages"],"slam_pose_messages":slam["slam_pose_messages"],"map":slam["map_snapshot"],"stationary_truth_drift_m":slam["truth_xy_drift_m"],"moving_localization_validated":False,"drift_improvement_demonstrated":False,"map_saved":True,"limits":["Startup TF-cache message-filter drop logged.","QoS warnings from diagnostic subscriptions logged.","No queue-full or EKF missed-rate warning appeared in this 25-second tuned log only.","No SLAM pose messages arrived; moving accuracy remains untested."],"source":"docs/evidence/phase8c/lidar/stage2-s00-stationary-slam-tuned.json"},
  "parking":{"unit_tests_passed":parking_passes,"unit_tests_total":7,"runtime_wired":False,"live_goal_executed":False,"source":"tests/parking_controller_test.py"}
 },
 "resources":{
  "host_ram_gib":8,"WSL_assigned_ram_gib":3.5,
  "phase8_max_sampled_process_rss_kib":p8a["max_sampled_process_rss_kib"],
  "phase8_min_available_memory_kib":p8a["min_mem_available_kib"],
  "stationary_SLAM_single_sample":{"available_memory_gib_approx":2.2,"process_rss_kib":{"gzserver":508616,"EKF":43008,"RGB_perception":187392,"scan_conditioner":79872,"local_mapper":82944,"safety_gate":78848,"slam_toolbox":54272},"CPU_percent_sample":{"gzserver":112.0,"EKF":14.5,"RGB_perception":51.8,"scan_conditioner":34.5,"local_mapper":50.1,"safety_gate":49.7,"slam_toolbox":8.9},"scope":"Single ps/free sample, not time-series peak or normalized total; raw sample not retained separately."},
  "idle_after_shutdown":idle,
  "comparison_limit":"Run durations and resource sampling differ. Treat values as capacity observations, not a controlled benchmark ranking."
 },
 "historical_failures_preserved":["docs/evidence/phase8c/lidar/stage1-rotation-trials.json","docs/evidence/phase8c/lidar/stage2-s00-stationary-slam-tuned.log","docs/evidence/phase8c/lidar/stage2-s00-stationary-slam-tuned-launch.log"],
 "github_issue_review":{"result":"gh issue list returned no issues","new_issue_created":False}
}
(OUT/"stage6-comparative-summary.json").write_text(json.dumps(summary,indent=2)+"\n")
rows=[]
def row(profile,metric,value,unit,limit,source):
 rows.append({"profile_or_stage":profile,"metric":metric,"value":value,"unit":unit,"interpretation_or_limit":limit,"source":source})
for name, obj in summary["comparisons"].items():
 for key,value in obj.items():
  if isinstance(value,(str,int,float)) and key not in ["status","source","interpretation","tested_scene"]:
   row(name,key,value,"as_named","Profile-specific; denominator and context in JSON.",obj.get("source",""))
with (OUT/"stage6-comparison.csv").open("w",newline="") as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
plt.rcParams.update({"font.size":9,"axes.titlesize":11})
fig,axs=plt.subplots(2,2,figsize=(12,8.6),constrained_layout=True)
fig.suptitle("Autonomous Navigation Car evidence snapshot (profiles are not equivalent)",fontsize=14,weight="bold")
ax=axs[0,0]; ax.axis("off"); ax.set_title("Phase 6 ORACLE_REFERENCE only",loc="left")
ax.text(.03,.78,"S00-S04 goals: 5 / 5 succeeded",transform=ax.transAxes,fontsize=14,weight="bold")
ax.text(.03,.59,"S09 no-route case: ABORTED as expected",transform=ax.transAxes,fontsize=11)
ax.text(.03,.40,f"Mean final XY error: {mean('final_xy_goal_error_m'):.3f} m",transform=ax.transAxes,fontsize=11)
ax.text(.03,.21,f"Collision events in S00-S04: {sum(x['contact_collision_events'] for x in metrics)}",transform=ax.transAxes,fontsize=11)
ax.text(.03,.04,"This profile used an oracle reference map.",transform=ax.transAxes,fontsize=9,color="#555555")
ax=axs[0,1]; labs=["False-FREE","Known-floor FREE","Occupied precision","Occupied recall","UNKNOWN"]; vals=[100*p7m["false_free_rate_on_known_occupied"],100*p7m["useful_free_coverage_on_known_floor"],100*p7m["occupied_precision"],100*p7m["occupied_recall"],100*p7m["unknown_coverage_total"]]; colors=["#bc4749","#6a994e","#386641","#f2cc8f","#8d99ae"]; bars=ax.barh(labs,vals,color=colors); ax.set_xlim(0,105); ax.set_xlabel("percent"); ax.set_title("Phase 7 RGB classifier (12 views)"); ax.invert_yaxis(); ax.grid(axis="x",alpha=.25)
for b,v in zip(bars,vals): ax.text(max(v+1,2),b.get_y()+b.get_height()/2,f"{v:.1f}%",va="center",fontsize=8)
ax=axs[1,0]; ax.axis("off"); ax.set_title("Phase 8 RGB-only stop baseline",loc="left")
ax.text(.03,.78,f"{p8a['actual_zero_samples']}/{p8a['actual_cmd_vel_samples']} zero /cmd_vel samples",transform=ax.transAxes,fontsize=14,weight="bold")
ax.text(.03,.58,f"{p8a['live_runs_passed']}/{p8a['live_runs']} bounded live runs passed",transform=ax.transAxes,fontsize=11)
ax.text(.03,.38,f"Maximum truth displacement: {p8a['maximum_evaluator_displacement_m']:.8f} m",transform=ax.transAxes,fontsize=11)
ax.text(.03,.18,f"Maximum sampled process RSS: {p8a['max_sampled_process_rss_kib']/1024:.0f} MiB",transform=ax.transAxes,fontsize=11)
ax.text(.03,.01,"Motion was intentionally not attempted.",transform=ax.transAxes,fontsize=9,color="#555555")
ax=axs[1,1]; ax.barh(["usable LiDAR beams"],[100*p8c["sensor"]["usable_fraction"]],color="#277da1"); ax.set_xlim(0,105); ax.set_xlabel("usable beams (%)"); ax.set_title(f"Phase 8C LiDAR + stationary SLAM\\n{p8c['sensor']['usable_beams']}/{p8c['sensor']['beams']} beams at {p8c['sensor']['measured_rate_hz']:.2f} Hz; {slam['scan_messages']} scans, {slam['map_messages']} maps; no pose messages"); ax.text(95,0,f"{100*p8c['sensor']['usable_fraction']:.1f}%",ha="right",va="center",color="white",weight="bold"); ax.grid(axis="x",alpha=.25)
fig.text(.01,.005,"Phase 6, RGB-only stop tests, and LiDAR sensing are different profiles. This is a status snapshot, not a head-to-head navigation benchmark.",fontsize=8)
fig.savefig(OUT/"stage6-comparison.png",dpi=160,bbox_inches="tight"); plt.close(fig)
fig,ax=plt.subplots(figsize=(8.5,4.5),constrained_layout=True)
p8min=p8a["min_mem_available_kib"]/1024**2
labels=["Phase 7 after dataset","Phase 8 minimum","Current idle after shutdown"]; vals=[2.8,p8min,idle["memory_available_gib"]]
bars=ax.bar(labels,vals,color=["#90be6d","#f9c74f","#577590"]); ax.set_ylabel("available WSL memory (GiB)"); ax.set_title("Memory observations on WSL assigned about 3.5 GiB"); ax.set_ylim(0,3.6)
for b,v in zip(bars,vals): ax.text(b.get_x()+b.get_width()/2,v+.06,f"{v:.2f}",ha="center")
ax.grid(axis="y",alpha=.25); fig.text(.02,-.045,"Sampling windows differ: Phase 7 after-run, Phase 8 minimum during trials, current idle. Not a controlled benchmark.",fontsize=8); fig.savefig(OUT/"stage6-resource-observations.png",dpi=160,bbox_inches="tight"); plt.close(fig)
print("Wrote stage6-comparative-summary.json, stage6-comparison.csv, stage6-comparison.png, stage6-resource-observations.png, stage6-idle-resource-observation.json")
print(f"Parking unit tests parsed: {parking_passes}/7; current idle available memory: {idle['memory_available_gib']:.3f} GiB")
