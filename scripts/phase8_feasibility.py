#!/usr/bin/env python3
"""Quantify whether Phase 7 RGB evidence can justify Phase 8 motion."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]

def load_json(path):
    return json.loads(Path(path).read_text())

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='docs/evidence/phase7/dataset/phase8-rgb-s00-20261102/dataset-results.json')
    parser.add_argument('--output', default='docs/evidence/phase8/phase8-feasibility.json')
    args = parser.parse_args()
    dataset_path = (ROOT / args.dataset).resolve()
    result = load_json(dataset_path)
    robot = yaml.safe_load((ROOT / 'config/robot_parameters.yaml').read_text())
    perception = yaml.safe_load((ROOT / 'simulation/ros2_ws/src/anc_perception/config/realistic.yaml').read_text())
    sim = robot['robot']['simulation_model']
    cam = robot['sensors']['camera']
    footprint = sim['footprint_xy_m']
    front_x = max(float(p[0]) for p in footprint)
    side_extent = max(abs(float(p[1])) for p in footprint)
    health_samples = [s['health'] for c in result['scenario_cases'] for s in c['samples'] if s.get('health',{}).get('status') == 'VALID']
    metrics = result['aggregate_metrics']
    cam_age = float(metrics['camera_sim_timestamp_age_s']['p95'])
    proc_latency = float(metrics['processing_latency_ms']['p95']) / 1000.0
    supervisor_period = 0.05
    delay = cam_age + proc_latency + supervisor_period
    safety_margin = 0.20
    speed_limit = float(sim['max_linear_velocity_mps'])
    speeds = [0.05, 0.10, 0.18, speed_limit]
    speed_table = []
    for speed in speeds:
        rd = speed * delay
        speed_table.append({'speed_mps': speed, 'reaction_only_distance_m': rd,
                            'margin_m': safety_margin,
                            'lower_bound_before_braking_m': rd+safety_margin,
                            'braking_distance_measured_m': None,
                            'required_clear_distance_m': None,
                            'status': 'BLOCKED_BRAKING_DISTANCE_UNMEASURED'})
    fx = float(health_samples[0]['intrinsics']['fx_px'])
    fy = float(health_samples[0]['intrinsics']['fy_px'])
    cx = float(health_samples[0]['intrinsics']['cx_px'])
    cy = float(health_samples[0]['intrinsics']['cy_px'])
    height = float(perception['/**']['ros__parameters']['camera_height_m'])
    camera_x = float(perception['/**']['ros__parameters']['camera_forward_offset_m'])
    pitch = float(perception['/**']['ros__parameters']['camera_pitch_down_rad'])
    views=[]
    for case in result['scenario_cases']:
        for sample in case['samples']:
            sp=(ROOT / sample['sample_dir']).resolve()
            mask=np.asarray(Image.open(sp/'prediction_mask.png').convert('L'))
            ys,xs=np.where(mask==0)
            if len(xs):
                theta=np.arctan2(ys.astype(float)-cy, fy)+pitch
                valid=np.tan(theta)>0
                forward=np.full(theta.shape, np.nan)
                forward[valid]=height/np.tan(theta[valid])
                xb=camera_x+forward
                yb=-forward*(xs.astype(float)-cx)/fx
                ok=valid & np.isfinite(xb) & np.isfinite(yb)
                xmin=float(np.min(xb[ok]));xmax=float(np.max(xb[ok]))
                ymin=float(np.min(yb[ok]));ymax=float(np.max(yb[ok]))
                res=0.02
                cells={(int(math.floor(x/res)),int(math.floor(y/res))) for x,y in zip(xb[ok],yb[ok])}
                bounds={'base_link_x_m':[xmin,xmax], 'base_link_y_m':[ymin,ymax],
                        'longitudinal_extent_m':xmax-xmin,'lateral_extent_m':ymax-ymin,
                        'free_grid_cells_at_0_02m':len(cells),
                        'unknown_gap_from_robot_front_m':max(0.0,xmin-front_x),
                        'connected_free_path_from_footprint':xmin <= front_x + res}
            else:
                bounds=None
            occ=int(sample['metrics']['prediction_counts_px']['occupied'])
            views.append({'case_id':case['case_id'],'viewpoint_id':sample['viewpoint_id'],
                          'status':sample['health']['status'],'reason':sample['health']['reason'],
                          'predicted_free_pixels':len(xs),'predicted_occupied_pixels':occ,
                          'unlocalized_occupied_evidence_blocks_motion':occ>0,
                          'free_patch_projection':bounds})
    phase7_full=load_json(ROOT/'docs/evidence/phase7/dataset/accepted-phase7-final-verified/dataset-results.json')
    phase4=load_json(ROOT/'docs/evidence/phase4-localization-normal-final7.json')
    slip=load_json(ROOT/'docs/evidence/phase4-localization-slip-final4.json')
    phase6=load_json(ROOT/'docs/evidence/phase6/runs/phase8-baseline-s00-20261102/result.json')
    p7m=phase7_full['aggregate_metrics']
    first=next(v for v in views if v['status']=='VALID' and v['free_patch_projection'])
    proj=first['free_patch_projection']
    reasons=[]
    if not proj['connected_free_path_from_footprint']:
        reasons.append('FREE patch is disconnected from robot footprint by an UNKNOWN region.')
    if proj['longitudinal_extent_m'] < (speed_table[2]['lower_bound_before_braking_m']):
        reasons.append('Observed FREE patch is shorter than reaction distance plus the explicit 0.20 m margin at 0.18 m/s, even before braking distance.')
    if p7m['unknown_coverage_total'] > 0.5 or p7m['occupied_recall'] < 0.5:
        reasons.append('Phase 7 challenge aggregate retains high UNKNOWN coverage and low occupied recall.')
    reasons.append('RGB OCCUPIED pixels have no defensible metric range; they remain UNKNOWN in metric cells and trigger a stop reason.')
    reasons.append('No dedicated free-space emergency-braking distance measurement exists; Phase 6 goal settling is not a hard-stop test.')
    out={
      'phase':8,'assessment':'BLOCKED_FOR_AUTONOMOUS_MOTION_STOP_ONLY_BASELINE_REQUIRED',
      'dataset':str(dataset_path.relative_to(ROOT)),
      'camera_assumptions':{'resolution_px':cam['resolution_px'],'fx_fy_px':[fx,fy],'principal_point_px':[cx,cy],
       'horizontal_fov_rad':cam['horizontal_fov_rad'],'distortion':'zero simulated pinhole',
       'camera_height_above_floor_m':height,'camera_forward_offset_from_base_link_m':camera_x,
       'camera_pitch_down_rad':pitch,'map_frame':'base_link'},
      'robot_assumptions':{'source':'config/robot_parameters.yaml simulation_model; placeholder, not hardware measured',
       'front_envelope_x_m':front_x,'half_width_envelope_y_m':side_extent,
       'max_linear_speed_mps':speed_limit},
      'observed_free_projection':{'view_count':len(views),'views':views},
      'timing':{'camera_sim_age_p95_s':cam_age,'processing_latency_p95_s':proc_latency,
       'supervisor_period_s':supervisor_period,'combined_reaction_delay_p95_s':delay,
       'speed_reaction_distance_table':speed_table},
      'braking':{'hard_stop_distance_measured':False,'hard_stop_distance_m':None,
       'phase6_reference_proxy':{'max_command_speed_mps':phase6['metrics']['max_abs_cmd_vel_linear_x_mps'],
        'post_action_truth_delta_m':phase6['metrics']['post_action_truth_pose_window']['xy_delta_m'],
        'window_s':phase6['metrics']['post_action_truth_pose_window']['duration_s'],
        'settle_time_s':phase6['metrics']['post_action_settle_time_s'],
        'interpretation':'goal-controller settling only; not an abrupt sensor-loss braking test'},
       'required_clear_distance_formula':'v*(camera_age_p95 + processing_p95 + supervisor_period) + braking_distance + 0.20 m margin'},
      'localization':{'phase4_normal_filtered_xy_rmse_m':phase4['filtered_vs_evaluator']['xy_rmse_m'],
       'phase4_normal_filtered_xy_max_error_m':phase4['filtered_vs_evaluator']['xy_max_m'],
       'phase4_normal_mean_sigma_xy_m':phase4['filtered_vs_evaluator']['mean_estimator_sigma'],
       'phase4_slip_filtered_endpoint_error_m':slip['filtered_vs_evaluator']['endpoint_xy_error_m'],
       'phase6_s00_reference_drift_m':phase6['metrics']['final_localization_drift_xy_m'],
       'interpretation':'known-start single-run values do not bound slip or unknown-start error'},
      'phase7_challenge_metrics':{'unknown_coverage':p7m['unknown_coverage_total'],
       'occupied_recall':p7m['occupied_recall'],'false_free_rate':p7m['false_free_rate_on_known_occupied'],
       'useful_free_coverage_on_known_floor':p7m['useful_free_coverage_on_known_floor']},
      'gate':'STOP_ONLY','blockers':reasons,
      'decision':'Do not authorize realistic movement. Publish only justified local FREE cells, leave all other metric cells UNKNOWN, and require zero motor command until a continuous footprint-clear stopping corridor is demonstrable.'}
    output=(ROOT/args.output).resolve();output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
    print(json.dumps(out,indent=2,sort_keys=True))
    return 0
if __name__=='__main__': raise SystemExit(main())
