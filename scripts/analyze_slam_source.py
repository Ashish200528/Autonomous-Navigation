#!/usr/bin/env python3
"""Offline timestamp-aligned source attribution; never imported by navigation."""
import argparse
import bisect
import csv
import gzip
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def pose(r):return [r['position'][0],r['position'][1],yaw(r['orientation'])]
def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return [a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],wrap(a[2]+b[2])]
def inverse(a):
    c,s=math.cos(a[2]),math.sin(a[2])
    return [-c*a[0]-s*a[1],s*a[0]-c*a[1],-a[2]]


class Stream:
    def __init__(self,rows):
        self.rows=sorted({r['stamp_s']:r for r in rows}.values(),key=lambda r:r['stamp_s'])
        self.times=[r['stamp_s'] for r in self.rows]

    def at(self,t,max_gap=.15):
        i=bisect.bisect_left(self.times,t)
        if i<len(self.times) and abs(self.times[i]-t)<1e-8:return pose(self.rows[i])
        if not 0<i<len(self.times) or self.times[i]-self.times[i-1]>max_gap:return None
        a,b=self.rows[i-1],self.rows[i];f=(t-a['stamp_s'])/(b['stamp_s']-a['stamp_s'])
        pa,pb=pose(a),pose(b)
        return [pa[0]+f*(pb[0]-pa[0]),pa[1]+f*(pb[1]-pa[1]),wrap(pa[2]+f*wrap(pb[2]-pa[2]))]


def expected_planar_ranges(p,angles,layout):
    """Evaluator-only intersections with vertical arena walls / scenario boxes."""
    dx=np.cos(angles+p[2]);dy=np.sin(angles+p[2]);x,y=p[:2]
    xmin,xmax,ymin,ymax=layout['arena']['inner_bounds_xy']
    with np.errstate(divide='ignore',invalid='ignore'):
        tx=np.where(dx>0,(xmax-x)/dx,(xmin-x)/dx)
        ty=np.where(dy>0,(ymax-y)/dy,(ymin-y)/dy)
        result=np.minimum(tx,ty)
        for o in layout['obstacles']:
            if o['shape']!='box':raise ValueError('unsupported_evaluation_geometry')
            ox,oy,a=o['pose_xyyaw'];c,s=math.cos(a),math.sin(a)
            px=c*(x-ox)+s*(y-oy);py=-s*(x-ox)+c*(y-oy)
            vx=c*dx+s*dy;vy=-s*dx+c*dy;hx,hy=np.asarray(o['size_xyz'][:2])/2
            ex1,ex2=(-hx-px)/vx,(hx-px)/vx;ey1,ey2=(-hy-py)/vy,(hy-py)/vy
            enter=np.maximum(np.minimum(ex1,ex2),np.minimum(ey1,ey2));leave=np.minimum(np.maximum(ex1,ex2),np.maximum(ey1,ey2))
            result=np.minimum(result,np.where((enter>=0)&(leave>=enter),enter,np.inf))
    return result


def analyze(folder,source=None):
    streams=defaultdict(list)
    source=source or (folder/'slam-source.jsonl.gz')
    if not source.exists():source=folder/'controller-source.jsonl.gz'
    with gzip.open(source,'rt') as f:
        for line in f:
            row=json.loads(line);streams[row['kind']].append(row)
    truth=Stream(streams['truth_evaluator_only']);wheel=Stream(streams['wheel_odom']);fused=Stream(streams['filtered_odom'])
    odomtf=Stream([r for r in streams['tf'] if (r['parent'],r['child'])==('odom','base_footprint')])
    root=json.loads((folder/'navigation-result.json').read_text())
    legname=root.get('observation_goal_reports',[None])[0] if root.get('observation_goal_reports') else None
    report=json.loads((folder/legname).read_text()) if legname else root
    layout=json.loads((folder/'commission-layout.json').read_text())
    reference=report['start_map_pose'];sw=layout['start_xyyaw'];map_world=compose(sw,inverse(reference))
    rows=[];previous=None
    for r in streams['slam_pose']:
        t=r['stamp_s'];gt=truth.at(t);od=odomtf.at(t);wh=wheel.at(t);ek=fused.at(t)
        if gt is None or od is None:continue
        sp=pose(r);wp=compose(map_world,sp);correction=compose(sp,inverse(od))
        row=dict(scan_stamp_s=t,truth_x=gt[0],truth_y=gt[1],slam_x=wp[0],slam_y=wp[1],
                 slam_error_m=math.hypot(wp[0]-gt[0],wp[1]-gt[1]),slam_heading_error_rad=abs(wrap(wp[2]-gt[2])),
                 map_odom_x=correction[0],map_odom_y=correction[1],map_odom_yaw=correction[2])
        for label,p in [('wheel',wh),('fused',ek)]:
            w=compose(sw,p) if p is not None else None
            row[label+'_error_m']=None if w is None else math.hypot(w[0]-gt[0],w[1]-gt[1])
        if previous:
            prediction=compose(compose(previous[0],inverse(previous[1])),od)
            row['scan_correction_step_m']=math.hypot(sp[0]-prediction[0],sp[1]-prediction[1])
        else:row['scan_correction_step_m']=0
        previous=(sp,od);rows.append(row)
    raw={r['stamp_s']:r for r in streams['raw_scan']};conditioned={r['stamp_s']:r for r in streams['conditioned_scan']}
    normalized=0;paired=0;geometry=[];ages=[];unchanged=0;faulted=0;finite_hits=0
    for t,r in conditioned.items():
        if r.get('receipt_clock_s') is not None:ages.append(r['receipt_clock_s']-t)
        if t not in raw:continue
        paired+=1;a=raw[t]
        normalized+=sum(v=='+inf' and w==r['range_max'] for v,w in zip(a['ranges'],r['ranges']))
        for v,w in zip(a['ranges'],r['ranges']):
            if isinstance(v,(int,float)) and r['range_min']<v<r['range_max']:
                finite_hits+=1
                if v==w:unchanged+=1
                else:faulted+=1
        gt=truth.at(t)
        if gt is not None:
            angles=a['angle_min']+np.arange(len(a['ranges']))*a['angle_increment']
            predicted=expected_planar_ranges(gt,angles,layout)
            values=np.array([v if isinstance(v,(int,float)) else np.nan for v in a['ranges']])
            valid=np.isfinite(values)&(values>a['range_min'])&(values<a['range_max']-.02)
            geometry.extend(np.abs(predicted[valid]-values[valid]).tolist())
    graph_moves=[];last={};graph_counts=[]
    for graph in streams['graph']:
        vertices={m['id']:m['position'] for m in graph['markers'] if m['ns']=='slam_toolbox' and m['type']==2}
        changes=[math.hypot(p[0]-last[i][0],p[1]-last[i][1]) for i,p in vertices.items() if i in last]
        graph_moves.extend(changes);last=vertices
        edges=sum(len(m['points'])//2 for m in graph['markers'] if m['type']==5)
        graph_counts.append(dict(stamp_s=graph['stamp_s'],vertices=len(vertices),edges=edges))
    tilts=[]
    for r in streams['truth_evaluator_only']:
        x,y,z,w=r['orientation']
        tilts.append([abs(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),abs(math.asin(max(-1,min(1,2*(w*y-z*x)))))])
    encoding_pairs=encoding_mismatches=masked_endpoints=0
    for hit_scan in streams['slam_hit_scan']:
        original=conditioned.get(hit_scan['stamp_s'])
        if original is None:continue
        encoding_pairs+=1
        if len(original['ranges'])!=len(hit_scan['ranges']):
            encoding_mismatches+=1;continue
        for value,encoded in zip(original['ranges'],hit_scan['ranges']):
            real=isinstance(value,(int,float)) and original['range_min']<value<original['range_max']
            if (real and encoded!=value) or (not real and encoded is not None):encoding_mismatches+=1
            if not real and encoded is None:masked_endpoints+=1
        for field in ('frame','angle_min','angle_max','angle_increment','time_increment','scan_time','range_min','range_max'):
            if original[field]!=hit_scan[field]:encoding_mismatches+=1
    summary=dict(evaluation_only=True,source_trace=source.name,
                 source_trace_scope='complete' if streams['raw_scan'] else 'bounded_controller_events',
                 accepted_slam_poses=len(streams['slam_pose']),aligned_slam_pose_samples=len(rows),
                 maximum_scan_stamped_slam_error_m=max((r['slam_error_m'] for r in rows),default=None),
                 final_scan_stamped_sample=rows[-1] if rows else None,
                 first_slam_error_above_10cm=next((r for r in rows if r['slam_error_m']>.1),None),
                 largest_scan_correction_step=max(rows,key=lambda r:r['scan_correction_step_m']) if rows else None,
                 paired_scans=paired,raw_no_return_beams_normalized_to_finite_max=normalized,
                 real_hit_beams=finite_hits,real_hit_beams_unchanged=unchanged,real_hit_beams_fault_injected=faulted,
                 scan_receipt_sim_age_s_percentiles=np.percentile(ages,[0,50,95,100]).tolist() if ages else None,
                 raw_scan_vs_independent_planar_geometry_abs_error_m_percentiles=np.percentile(geometry,[50,95,99,100]).tolist() if geometry else None,
                 geometry_beam_count=len(geometry),maximum_truth_abs_roll_pitch_rad=np.max(tilts,axis=0).tolist() if tilts else None,
                 largest_existing_graph_vertex_revision_m=max(graph_moves,default=0),graph_counts=graph_counts,
                 static_transforms=streams['tf_static'],
                 slam_endpoint_encoding=dict(paired_scans=encoding_pairs,mismatches=encoding_mismatches,masked_non_hit_endpoints=masked_endpoints))
    filename='slam-source-analysis.json' if streams['raw_scan'] else 'controller-source-analysis.json'
    (folder/filename).write_text(json.dumps(summary,indent=2)+'\n')
    with (folder/'slam-scan-corrections.csv').open('w',newline='') as f:
        if rows:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    parser.add_argument('--source',type=Path,help='Optional full locally archived range trace')
    args=parser.parse_args();result=analyze(args.folder,args.source)
    print(json.dumps({k:v for k,v in result.items() if k not in ('graph_counts','static_transforms')},indent=2))
