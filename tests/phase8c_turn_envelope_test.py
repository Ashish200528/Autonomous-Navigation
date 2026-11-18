import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from anc_lidar.turn_envelope import CASES,load_envelope


class EnvelopeTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.manifest=dict(schema='TURN_ENVELOPE_V2',angular_cap_radps=.1,cases={})
        for name in CASES:
            speeds=[-.1] if name=='clockwise' else [.1]
            if name=='multi_speed':speeds=[.02,.05,.1]
            if name=='repeated_turns':speeds=[.1,-.1,.1,-.1]
            d=dict(case_id=name,trial_pass=True,production_angular_authority_enabled=False,
                   cmd_vel_publisher_names=['anc_safety_command_guard'],trials=[])
            for speed in speeds:
                d['trials'].append(dict(trial_pass=True,production_angular_authority_enabled=False,
                    contact_records_non_ground=0,requested_angular_speed_radps=speed,
                    measured_average_yaw_rate_radps=speed*.9,minimum_clearance_m=.5,center_drift_m=.01,
                    fault_injected=True,observed_rotation_sim_duration_s=8,
                    stop_contract=dict(contract='TURN_STOP_V2',status='PASS',failures=[],
                        physical_stop_time_upper_s=.5,stopping_angle_upper_rad=.04,post_zero_angle_upper_rad=.02,
                        stationary_observation_s=.55,max_sample_gap_s=.11,max_query_round_trip_s=.01,
                        fault_to_confirmation_s=1.1)))
            (self.root/(name+'.json')).write_text(json.dumps(d))
            self.manifest['cases'][name]=dict(path=name+'.json',sha256=hashlib.sha256((self.root/(name+'.json')).read_bytes()).hexdigest())
        self.path=self.root/'envelope.json';self.save()

    def save(self):self.path.write_text(json.dumps(self.manifest))
    def tearDown(self):self.temp.cleanup()

    def test_complete_preproduction_evidence_grants_only_validation_envelope(self):
        self.assertEqual(load_envelope(self.path)['validated_for'],'production_test')
        with self.assertRaises(ValueError):load_envelope(self.path,False)

    def test_missing_case_cannot_open_authority(self):
        self.manifest['cases'].pop('lidar_loss');self.save()
        with self.assertRaises(ValueError):load_envelope(self.path)

    def test_changed_measurement_rejected(self):
        p=self.root/'lidar_loss.json';p.write_text(p.read_text()+' ')
        with self.assertRaises(ValueError):load_envelope(self.path)

    def test_absolute_path_escape_rejected(self):
        self.manifest['cases']['lidar_loss']['path']='/tmp/outside-turn-evidence.json';self.save()
        with self.assertRaises(ValueError):load_envelope(self.path)

    def test_boolean_or_nonfinite_cap_rejected(self):
        for cap in (True,float('nan'),.11):
            self.manifest['angular_cap_radps']=cap;self.save()
            with self.assertRaises(ValueError):load_envelope(self.path)

    def test_pass_boolean_cannot_hide_incomplete_confirmation(self):
        p=self.root/'lidar_loss.json';d=json.loads(p.read_text());d['trials'][0]['stop_contract']['stationary_observation_s']=.1
        p.write_text(json.dumps(d));self.manifest['cases']['lidar_loss']['sha256']=hashlib.sha256(p.read_bytes()).hexdigest();self.save()
        with self.assertRaises(ValueError):load_envelope(self.path)
