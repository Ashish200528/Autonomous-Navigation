"""Independent geometry checks for the turning evaluator's clearance metric."""
import math
import unittest
from phase8c_turn_matrix_probe import MatrixProbe


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
