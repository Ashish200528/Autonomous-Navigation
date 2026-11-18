"""Evidence-bound angular envelope. Empty/invalid evidence never grants authority."""
import hashlib
import json
import math
from pathlib import Path

CASES = {'clockwise','anticlockwise','multi_speed','repeated_turns','extended_rotation',
         'near_wall','near_box','narrow_passage','low_friction','lidar_loss',
         'localization_loss','command_timeout','unknown_swept_region'}
FAULTS={'lidar_loss','localization_loss','command_timeout','unknown_swept_region'}


def number(value, low, high):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
        raise ValueError('turn_evidence_measurement_invalid')
    return value


def load_envelope(filename, validation_only=True):
    """Manifest references hashed measurements, not boolean readiness declarations."""
    manifest_path=Path(filename).resolve()
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('schema')!='TURN_ENVELOPE_V2':raise ValueError('turn_evidence_schema_invalid')
    cap=number(manifest.get('angular_cap_radps'),.02,.10)
    entries=manifest.get('cases',{})
    if set(entries)!=CASES:raise ValueError('turn_evidence_case_matrix_incomplete')
    for case, entry in entries.items():
        path=(manifest_path.parent/entry['path']).resolve()
        # Evidence is kept in the same run tree; no arbitrary external references.
        if not path.is_relative_to(manifest_path.parent):raise ValueError('turn_evidence_path_escape')
        data=path.read_bytes()
        if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('turn_evidence_hash_mismatch')
        result=json.loads(data)
        if result.get('case_id')!=case or result.get('trial_pass') is not True:
            raise ValueError('turn_evidence_case_failed')
        if result.get('cmd_vel_publisher_names')!=['anc_safety_command_guard']:
            raise ValueError('turn_evidence_guard_unverified')
        trials=result.get('trials',[])
        if not trials:raise ValueError('turn_evidence_trials_missing')
        if not validation_only and result.get('production_angular_authority_enabled') is not True:
            raise ValueError('turn_production_matrix_unverified')
        speeds=[]
        for trial in trials:
            if trial.get('trial_pass') is not True or trial.get('contact_records_non_ground')!=0:
                raise ValueError('turn_evidence_contact_or_failed_trial')
            if not validation_only and trial.get('production_angular_authority_enabled') is not True:
                raise ValueError('turn_production_trial_unverified')
            contract=trial.get('stop_contract',{})
            if contract.get('contract')!='TURN_STOP_V2' or contract.get('status')!='PASS' or contract.get('failures')!=[]:
                raise ValueError('turn_stopping_contract_failed')
            number(contract.get('physical_stop_time_upper_s'),0,.75)
            number(contract.get('stopping_angle_upper_rad'),0,.05)
            number(contract.get('post_zero_angle_upper_rad'),0,.05)
            number(contract.get('stationary_observation_s'),.5,10)
            number(contract.get('max_sample_gap_s'),0,.15)
            number(contract.get('max_query_round_trip_s'),0,.15)
            number(contract.get('fault_to_confirmation_s'),0,1.4)
            number(trial.get('minimum_clearance_m'),.1,100)
            number(trial.get('center_drift_m'),0,.1)
            requested=number(trial.get('requested_angular_speed_radps'),-cap,cap)
            measured=number(trial.get('measured_average_yaw_rate_radps'),-1,1)
            if requested*measured<=0:raise ValueError('turn_direction_invalid')
            if case in FAULTS and trial.get('fault_injected') is not True:
                raise ValueError('turn_fault_not_injected')
            speeds.append(requested)
        if case=='extended_rotation':
            number(trials[0].get('observed_rotation_sim_duration_s'),7.5,10)
        if case=='clockwise' and not all(s<0 for s in speeds):raise ValueError('turn_clockwise_invalid')
        if case=='anticlockwise' and not all(s>0 for s in speeds):raise ValueError('turn_anticlockwise_invalid')
        if case=='multi_speed' and len(set(abs(s) for s in speeds))<3:raise ValueError('turn_speeds_incomplete')
        if case=='repeated_turns' and (len(speeds)<4 or not any(s<0 for s in speeds) or not any(s>0 for s in speeds)):
            raise ValueError('turn_repetition_incomplete')
    return dict(angular_cap_radps=cap,physical_stop_time_bound_s=.75,
                stopping_angle_bound_rad=.05,confirmation_window_s=.5,
                manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                validated_for='production_test' if validation_only else 'production')
