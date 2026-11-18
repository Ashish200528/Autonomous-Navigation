import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import commission_lidar_s00 as commission


class CommissionPrerequisitesTest(unittest.TestCase):
    def test_s01_original_goal_orientation_cannot_be_overridden(self):
        with patch.object(sys,'argv',['commission','missing','unused','--scenario','S01','--goal-yaw-rad','1']), \
                patch.object(commission,'prerequisites',side_effect=AssertionError('must reject before authority check')):
            with self.assertRaises(SystemExit):commission.main()

    def test_timeout_diagnostic_requires_every_existing_safety_observation(self):
        report=dict(status='FAIL',nav2_action_status='TIMEOUT',post_goal_stop_confirmed=True,
                    final_localization_error_m=.02,final_localization_heading_error_rad=.01,
                    min_evaluator_clearance_m=.5,non_ground_contact_events=[],guarded_mixed_output_samples=0,
                    sole_authoritative_cmd_vel_publisher=True,travel_limit_exceeded=False,
                    sensor_integrity=dict(rgb_fresh=True,scan_fresh=True,map_fresh=True,filtered_odom_received=True))
        self.assertTrue(commission.timeout_diagnostic_eligible(report))
        for field,value in [('post_goal_stop_confirmed',False),('final_localization_error_m',.11),
                            ('final_localization_heading_error_rad',float('nan')),('min_evaluator_clearance_m',.09),
                            ('non_ground_contact_events',[['robot','box']]),('guarded_mixed_output_samples',1),
                            ('sole_authoritative_cmd_vel_publisher',False),('travel_limit_exceeded',True),
                            ('nav2_action_status','ABORTED')]:
            with self.subTest(field=field):
                self.assertFalse(commission.timeout_diagnostic_eligible(dict(report,**{field:value})))
        for field in report['sensor_integrity']:
            self.assertFalse(commission.timeout_diagnostic_eligible(dict(report,sensor_integrity={field:False})))

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.base=self.root/'docs/evidence/native-slam-map1s-20261103';self.base.mkdir(parents=True)
        for i in range(3):(self.base/f'trial{i}-summary.json').write_text(json.dumps(dict(trial_id=str(i))))

    def tearDown(self):self.temp.cleanup()

    def test_production_evidence_failure_prevents_commissioning(self):
        with patch.object(commission,'load_envelope',side_effect=ValueError('not production')):
            with self.assertRaises(ValueError):commission.prerequisites('missing')

    def test_three_distinct_trials_required(self):
        for i in range(3):(self.base/f'trial{i}-summary.json').write_text('{"trial_id":"same"}')
        with patch.object(commission,'ROOT',self.root),patch.object(commission,'load_envelope',return_value={}),patch.object(commission,'_validate_moving',return_value=[]):
            with self.assertRaises(ValueError):commission.prerequisites('fixture')

    def test_failed_moving_localization_prevents_goal(self):
        with patch.object(commission,'ROOT',self.root),patch.object(commission,'load_envelope',return_value={}),patch.object(commission,'_validate_moving',return_value=['failed']):
            with self.assertRaises(ValueError):commission.prerequisites('fixture')

    def test_first_goal_does_not_require_a_previous_goal(self):
        with patch.object(commission,'ROOT',self.root),patch.object(commission,'load_envelope',return_value={'validated_for':'production'}),patch.object(commission,'_validate_moving',return_value=[]):
            result=commission.prerequisites('fixture')
            self.assertEqual(len(result['moving_slam']),3)

    def test_unsafe_linux_ephemeral_domain_is_rejected_before_authority_checks(self):
        with patch.object(sys,'argv',['commission','missing','unused','--domain-id','180']), \
                patch.object(commission,'prerequisites',side_effect=AssertionError('must reject domain first')):
            with self.assertRaises(SystemExit):commission.main()

    def test_s01_spawn_is_preserved_in_launch_arguments(self):
        from scenario_generator import find_scenario,registry_data
        layout=find_scenario(registry_data(),'S01')
        arguments=commission.spawn_arguments(layout)
        self.assertEqual(float(arguments[0].split(':=')[1]),-3.0)
        self.assertEqual(float(arguments[1].split(':=')[1]),0.0)

    def test_nonfinite_spawn_is_rejected(self):
        with self.assertRaises(ValueError):commission.spawn_arguments(dict(start_xyyaw=[float('nan'),0,0]))
