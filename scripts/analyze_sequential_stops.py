#!/usr/bin/env python3
"""Evaluator-only guard-zero stopping measurements, using the unchanged V2 contract."""
import argparse
import csv
import gzip
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tests'))
from phase8c_turn_stop_contract import evaluate_stop


def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def analyze(folder):
    source=folder/'slam-source.jsonl.gz'
    if not source.exists():source=folder/'controller-source.jsonl.gz'
    truth=[];decisions=[];proposals=[]
    with gzip.open(source,'rt') as f:
        for line in f:
            r=json.loads(line)
            if r['kind']=='truth_evaluator_only' and r.get('clock_s') is not None:
                age=r['clock_s']-r['stamp_s'];clock_age=r['receipt_wall_s']-r['clock_receipt_wall_s']
                if not -.05<=age<=.05 or not 0<=clock_age<=.05:continue
                ingress=max(0,age)+clock_age
                truth.append(dict(wall_s=r['receipt_wall_s'],query_start_wall_s=r['receipt_wall_s']-ingress,
                                  sim_stamp_s=r['stamp_s'],x_m=r['position'][0],y_m=r['position'][1],yaw_rad=yaw(r['orientation'])))
            elif r['kind']=='guard_decision':decisions.append(r['data'])
            elif r['kind']=='sequential_controller':proposals.append(r)
    truth.sort(key=lambda r:r['wall_s']);decisions.sort(key=lambda r:r['decision_monotonic_s'])
    raw_stops=[];prior_raw_w=0
    for row in csv.DictReader((folder/'guarded-commands.csv').open()):
        if row['topic']!='/cmd_vel_nav_raw':continue
        v,w=float(row['linear_x_mps']),float(row['angular_z_radps'])
        if abs(prior_raw_w)>1e-6 and abs(v)<1e-9 and abs(w)<1e-9:raw_stops.append(float(row['wall_s']))
        prior_raw_w=w
    stops=[];previous=None
    for i,state in enumerate(decisions):
        out=state.get('output') or {};v=out.get('linear_x_mps',0);w=out.get('angular_z_radps',0)
        if previous and abs(previous.get('angular_z_radps',0))>1e-6 and abs(v)<1e-9 and abs(w)<1e-9:
            trigger=state['decision_monotonic_s']
            next_move=next((d['decision_monotonic_s'] for d in decisions[i+1:]
                            if not (d.get('output') or {}).get('is_zero',True)),math.inf)
            segment=[s for s in truth if trigger-.15<=s['wall_s']<min(trigger+3,next_move)]
            candidate=None;confirmation=None;used=None
            for j,(a,b) in enumerate(zip(segment,segment[1:]),start=1):
                if b['wall_s']<trigger:continue
                dt=b['sim_stamp_s']-a['sim_stamp_s']
                if dt<=0: candidate=None;continue
                linear=math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m'])/dt
                angular=abs(wrap(b['yaw_rad']-a['yaw_rad']))/dt
                if linear>.005 or angular>.01:candidate=None
                elif candidate is None:candidate=b['wall_s']
                if candidate is not None and b['query_start_wall_s']-candidate>=.5:
                    used=segment[:j+1];confirmation=b['wall_s'];break
            result=evaluate_stop(used or segment,trigger,trigger,confirmation or trigger+3,angular_cap=.05)
            if confirmation is not None:
                later=[s for s in segment if s['wall_s']>confirmation]
                follow=[(used or segment)[-1]]+later
                rebound=False
                for a,b in zip(follow,follow[1:]):
                    dt=b['sim_stamp_s']-a['sim_stamp_s']
                    if dt>0 and (abs(wrap(b['yaw_rad']-a['yaw_rad']))/dt>.01 or math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m'])/dt>.005):rebound=True
                if rebound:result['status']='FAIL';result['failures'].append('post_confirmation_rebound')
            result.update(trigger_reference='authoritative_guard_zero_decision',trigger_wall_s=trigger,
                          prior_guard_w_radps=previous['angular_z_radps'],next_motion_wall_s=None if math.isinf(next_move) else next_move)
            upstream=next((t for t in reversed(raw_stops) if t<=trigger),None)
            if upstream is not None and trigger-upstream<2:
                upstream_segment=[s for s in truth if upstream-.15<=s['wall_s']<=(confirmation or trigger+3)]
                observed=evaluate_stop(upstream_segment,upstream,trigger,confirmation or trigger+3,angular_cap=.05)
                observed['trigger_reference']='upstream_zero_subscription_receipt_not_publisher_origin'
                result['upstream_zero_observed_to_guard_zero_s']=trigger-upstream
                result['upstream_observation']=observed
            stops.append(result)
        previous=out
    nav=json.loads((folder/'navigation-result.json').read_text())
    observed=nav.get('command_summary',{}).get('/cmd_vel',{})
    caps_ok=observed.get('max_v',math.inf)<=.08+1e-9 and observed.get('max_w',math.inf)<=.05+1e-9
    report=dict(classification='SEQUENTIAL_NAV2_GUARD_ZERO_STOP_MEASUREMENTS',
                status='PASS' if stops and all(x['status']=='PASS' and x.get('upstream_observation',{}).get('status')=='PASS' for x in stops) and nav['status']=='PASS' and caps_ok else 'FAIL',
                runtime_truth_used=False,unchanged_contract='TURN_STOP_V2',stops=stops,
                observed_command_caps_within_profile=caps_ok,
                requires_both_guard_and_upstream_observation=True,
                note='Measures physical response from authoritative guard zero. Upstream smoothing latency is separate; no fault matrix category is relabelled.')
    (folder/'normal-turn-stop-measurements.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    report=analyze(parser.parse_args().folder)
    print(json.dumps(report,indent=2))
