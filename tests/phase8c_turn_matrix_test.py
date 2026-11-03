"""Independent geometry checks for the turning evaluator's clearance metric."""
import math
import unittest
from phase8c_turn_matrix_probe import MatrixProbe, stop_timing_metrics


class TurningClearanceTest(unittest.TestCase):
    def setUp(self):
        self.probe = MatrixProbe.__new__(MatrixProbe)
        self.radius = 0.263592962728522

    def test_wall_clearance_includes_robot_radius(self):
        self.probe.obstacles = [(0, 2, 0, 8, .2)]
        self.assertAlmostEqual(self.probe.clearance({'x_m':0, 'y_m':0}),1.9-self.radius)

    def test_rotated_box_corner(self):
        self.probe.obstacles = [(0, 0, math.pi/2, 2, 4)]
        self.assertAlmostEqual(self.probe.clearance({'x_m':3, 'y_m':2}),math.sqrt(2)-self.radius)

    def test_center_inside_box_is_negative(self):
        self.probe.obstacles = [(0,0,0,2,4)]
        self.assertLess(self.probe.clearance({'x_m':0,'y_m':0}),-1)

    def test_nearest_obstacle_selected(self):
        self.probe.obstacles = [(0,2,0,8,.2),(0,-1,0,8,.2)]
        self.assertAlmostEqual(self.probe.clearance({'x_m':0,'y_m':0}),.9-self.radius)


class StopTimingTest(unittest.TestCase):
    def sample(self, wall, v=0.0, w=0.0):
        return dict(wall_s=wall, pose_derived_linear_speed_mps=v,
                    pose_derived_angular_speed_radps=w)

    def test_confirmation_hold_is_separate_from_first_slow_sample(self):
        result = stop_timing_metrics([self.sample(10.3),self.sample(10.9)],10,10.91)
        self.assertAlmostEqual(result['first_verified_slow_sample_after_trigger_s'],.3)
        self.assertAlmostEqual(result['observation_hold_after_first_slow_s'],.61)
        self.assertAlmostEqual(result['confirmed_settled_after_trigger_s'],.91)

    def test_rebound_resets_candidate(self):
        result=stop_timing_metrics([self.sample(10.1),self.sample(10.2,w=.1),
                                  self.sample(10.3),self.sample(10.9)],10,10.91)
        self.assertAlmostEqual(result['first_verified_slow_sample_after_trigger_s'],.3)

    def test_missing_duplicate_time_rate_cannot_count_as_stopped(self):
        result=stop_timing_metrics([self.sample(10.1),self.sample(10.2,None,None),
                                  self.sample(10.3)],10,10.9)
        self.assertIsNone(result['first_verified_slow_sample_after_trigger_s'])

    def test_nonfinite_rate_cannot_count_as_stopped(self):
        result=stop_timing_metrics([self.sample(10.1,w=float('nan'))],10,11)
        self.assertIsNone(result['first_verified_slow_sample_after_trigger_s'])
