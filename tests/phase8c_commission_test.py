import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import commission_lidar_s00 as commission


class CommissionPrerequisitesTest(unittest.TestCase):
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
