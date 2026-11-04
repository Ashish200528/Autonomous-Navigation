import math
from pathlib import Path
import sys
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_slam_source import Stream,compose,inverse
from anc_lidar.scan_model import slam_hit_ranges


class SlamSourceTests(unittest.TestCase):
    def test_endpoint_mask_retains_indices_and_real_hits(self):
        source=[float('inf'),6.,7.,float('nan'),.12,.5,5.999,-1.]
        output=slam_hit_ranges(source,.12,6.)
        self.assertEqual(len(output),len(source))
        self.assertEqual(output[5:7],[.5,5.999])
        self.assertTrue(all(math.isnan(output[i]) for i in (0,1,2,3,4,7)))
        self.assertEqual(source[1],6.)  # Local safety clearing representation is untouched.

    def test_candidate_does_not_change_matching_weights_or_odometry_model(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        baseline=yaml.safe_load((config/'slam_toolbox.yaml').read_text())
        candidate=yaml.safe_load((config/'slam_toolbox_hit_endpoints.yaml').read_text())
        self.assertEqual(candidate['slam_toolbox']['ros__parameters'].pop('scan_topic'),'/realistic/scan/slam_hits')
        baseline['slam_toolbox']['ros__parameters'].pop('scan_topic')
        self.assertEqual(baseline,candidate)

    def test_scan_time_interpolation_handles_yaw_wrap(self):
        def record(t,x,a):return dict(stamp_s=t,position=[x,0,0],orientation=[0,0,math.sin(a/2),math.cos(a/2)])
        stream=Stream([record(1,0,math.pi-.02),record(1.1,1,-math.pi+.02)])
        p=stream.at(1.05)
        self.assertAlmostEqual(p[0],.5)
        self.assertAlmostEqual(abs(p[2]),math.pi)

    def test_missing_temporal_bracket_is_not_extrapolated(self):
        stream=Stream([dict(stamp_s=1,position=[0,0,0],orientation=[0,0,0,1]),
                       dict(stamp_s=2,position=[1,0,0],orientation=[0,0,0,1])])
        self.assertIsNone(stream.at(1.5))
        self.assertIsNone(stream.at(2.1))

    def test_pose_correction_decomposition_respects_rotated_odometry(self):
        correction=[.2,-.1,.5];odom=[1,2,.3]
        recovered=compose(compose(correction,odom),inverse(odom))
        for a,b in zip(recovered,correction):self.assertAlmostEqual(a,b)
