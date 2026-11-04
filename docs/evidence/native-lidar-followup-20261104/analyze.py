"""Reproduce the follow-up summary from saved measurements; no runtime inputs."""
from pathlib import Path
import json
import math

OUT = Path(__file__).resolve().parent
EVIDENCE = OUT.parent
NAMES = [
    'native-s00-3m-repeat-20261104',
    'native-s00-observation-qualification-20261104',
    'native-s01-observation-mission-20261104',
    'native-s01-localization-diagnostic-20261104',
    'native-s00-prior-qualification-20261104',
    'native-s00-prior-direct2m-20261104',
    'native-s01-prior-observation-20261104',
    'native-s01-no-loop-diagnostic-20261104',
]
KEYS = [
    'status', 'nav2_action_status', 'truth_path_length_m', 'final_xy_error_m',
    'final_localization_error_m', 'final_localization_heading_error_rad',
    'min_evaluator_clearance_m', 'post_goal_stop_confirmed',
    'guarded_mixed_output_samples', 'non_ground_contact_events',
    'runtime_rtf', 'resource_summary', 'sensor_summary',
    'active_goal_guard_reason_sample_counts',
]
summary = {}
for name in NAMES:
    folder = EVIDENCE / name
    mission = json.loads((folder / 'navigation-result.json').read_text())
    reports = mission.get('observation_goal_reports', [])
    leg = (json.loads((folder / reports[-1]).read_text())
           if 'truth_path_length_m' not in mission and reports else mission)
    trace = mission.get('localization_trace', [])
    summary[name] = dict(
        mission_status=mission['status'], reason=mission.get('reason'),
        original_goal_attempted=mission.get('original_goal_attempted', True),
        measured_result_file=reports[-1] if leg is not mission else 'navigation-result.json',
        metrics_scope=leg.get('metrics_scope', 'single_goal'),
        measurements={key: leg.get(key) for key in KEYS},
        maximum_sampled_localization_error_m=max((x['xy_error_m'] for x in trace), default=None),
        maximum_absolute_tf_age_s=max((abs(x['tf_age_sim_s']) for x in trace), default=None),
    )
    diagnostic = folder / 'stationary-diagnostic.json'
    if diagnostic.exists():
        rows = json.loads(diagnostic.read_text())
        a, b = rows[0], rows[-1]
        # This original S01 start is (-3,0,0). Odometry starts at its origin.
        raw_error = math.hypot(a['raw_odom']['x'] - (a['truth']['x_m'] + 3),
                               a['raw_odom']['y'] - a['truth']['y_m'])
        fused_error = math.hypot(a['filtered_odom']['x'] - (a['truth']['x_m'] + 3),
                                 a['filtered_odom']['y'] - a['truth']['y_m'])
        summary[name]['stationary_hold'] = dict(
            sample_count=len(rows), wall_duration_s=b['truth']['wall_s'] - a['truth']['wall_s'],
            truth_displacement_m=math.hypot(b['truth']['x_m'] - a['truth']['x_m'], b['truth']['y_m'] - a['truth']['y_m']),
            raw_odom_xy_error_at_hold_start_m=raw_error,
            fused_odom_xy_error_at_hold_start_m=fused_error,
            first_map_to_odom=a['map_to_odom'], last_map_to_odom=b['map_to_odom'],
            first_localization_error_m=a['localization']['xy_error_m'],
            last_localization_error_m=b['localization']['xy_error_m'],
            all_recorded_guard_outputs_zero=all(r['guard_output'] and r['guard_output'].get('is_zero') is True for r in rows),
        )
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({name: row['mission_status'] for name, row in summary.items()}, indent=2))
