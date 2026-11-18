"""Sample-based preproduction stopping contract, independent of ROS/runtime authority.

Physical time <=0.75 s, angle <=0.05 rad, stationary observation >=0.5 s.
The observation window is additional to physical stopping, not part of it.
All bounds include measured truth-query and intersample uncertainty.
"""
import math


def evaluate_stop(samples, trigger, zero_wall, confirmation_wall, angular_cap=.1):
    failures=[]
    result={'contract':'TURN_STOP_V2', 'status':'FAIL', 'failures':failures,
            'physical_stop_limit_s':.75, 'angle_limit_rad':.05,
            'confirmation_window_s':.5, 'maximum_sample_gap_s':.15}
    if len(samples)<3 or zero_wall is None:
        failures.append('missing_pose_or_zero_output');return result
    if not all(all(math.isfinite(s[k]) for k in ('wall_s','sim_stamp_s','yaw_rad','x_m','y_m','query_start_wall_s')) for s in samples):
        failures.append('nonfinite_pose');return result
    intervals=[]
    candidate=None
    for a,b in zip(samples,samples[1:]):
        wall_dt=b['wall_s']-a['wall_s'];dt=b['sim_stamp_s']-a['sim_stamp_s']
        if wall_dt<=0 or dt<0:
            failures.append('time_reversal');return result
        intervals.append(wall_dt)
        if b['wall_s']<zero_wall:continue
        if dt<=1e-6:
            # Duplicate simulation timestamps cannot establish a stationary interval.
            candidate=None
            continue
        w=abs(math.atan2(math.sin(b['yaw_rad']-a['yaw_rad']),math.cos(b['yaw_rad']-a['yaw_rad'])))/dt
        v=math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m'])/dt
        if w>.01 or v>.005:
            candidate=None
        elif candidate is None:
            candidate=b['wall_s']
    before=[s for s in samples if s['query_start_wall_s']<=trigger]
    if not before or candidate is None:
        failures.append('missing_trigger_bracket_or_stable_interval');return result
    anchor=before[-1]
    # Accumulated absolute angular travel catches rebound; endpoint difference does not.
    segment=[anchor]+[s for s in samples if s['wall_s']>anchor['wall_s']]
    gap=max(b['wall_s']-a['wall_s'] for a,b in zip(segment,segment[1:]))
    rtt=max(s['wall_s']-s['query_start_wall_s'] for s in segment)
    if rtt<0 or gap>.15:
        failures.append('sampling_uncertainty_exceeds_budget')
    travel=sum(abs(math.atan2(math.sin(b['yaw_rad']-a['yaw_rad']),math.cos(b['yaw_rad']-a['yaw_rad']))) for a,b in zip(segment,segment[1:]))
    uncertainty=gap+rtt
    angle_upper=travel+angular_cap*uncertainty
    physical_upper=candidate-trigger+rtt
    postzero=[s for s in samples if s['query_start_wall_s']<=zero_wall]
    zero_anchor=postzero[-1] if postzero else anchor
    zero_segment=[zero_anchor]+[s for s in samples if s['wall_s']>zero_anchor['wall_s']]
    postzero_travel=sum(abs(math.atan2(math.sin(b['yaw_rad']-a['yaw_rad']),math.cos(b['yaw_rad']-a['yaw_rad']))) for a,b in zip(zero_segment,zero_segment[1:]))
    observation=samples[-1]['query_start_wall_s']-candidate
    # 0.75 physical deadline + 0.5 hold + one 0.15 s bounded sampling interval.
    confirmation_deadline=.75+.5+.15
    if physical_upper>.75:failures.append('physical_stop_time_exceeded')
    if angle_upper>.05:failures.append('stopping_angle_exceeded')
    if observation<.5:failures.append('stationary_confirmation_incomplete')
    if confirmation_wall-trigger>confirmation_deadline:failures.append('confirmation_deadline_exceeded')
    result.update(physical_stop_observed_s=candidate-trigger,
                  physical_stop_time_upper_s=physical_upper,
                  stationary_observation_s=observation,
                  fault_to_confirmation_s=confirmation_wall-trigger,
                  confirmation_deadline_s=confirmation_deadline,
                  measured_total_stopping_angle_rad=travel,
                  stopping_angle_upper_rad=angle_upper,
                  post_zero_angle_upper_rad=postzero_travel+angular_cap*uncertainty,
                  zero_to_physical_stop_upper_s=max(0,candidate-zero_wall)+rtt,
                  max_sample_gap_s=gap, max_query_round_trip_s=rtt,
                  status='PASS' if not failures else 'FAIL')
    return result
