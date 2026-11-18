import unittest
from anc_navigation_common.safety import SequentialMotionInterlock


class SequentialInterlockTests(unittest.TestCase):
    def setUp(self):self.lock=SequentialMotionInterlock()
    def decide(self,t,v,w,odom=(0,0,0),age=0):
        return self.lock.apply(dict(authorized=True,linear_x_mps=v,angular_z_radps=w,reason='bounded'),t,odom,age)

    def test_initial_motion_requires_full_stationary_hold(self):
        self.assertFalse(self.decide(0,.1,0)['authorized'])
        self.assertFalse(self.decide(.49,.1,0)['authorized'])
        self.assertTrue(self.decide(.5,.1,0)['authorized'])

    def test_forward_to_yaw_requires_additional_stop_interval(self):
        self.decide(0,.1,0);self.decide(.5,.1,0)
        self.assertFalse(self.decide(.6,0,.1,(.1,0,0))['authorized'])
        self.assertFalse(self.decide(.7,0,.1)['authorized'])
        self.assertFalse(self.decide(1.3,0,.1)['authorized'])
        self.assertTrue(self.decide(1.76,0,.1)['authorized'])

    def test_yaw_reversal_requires_stop(self):
        self.decide(0,0,.1);self.decide(.5,0,.1)
        self.assertFalse(self.decide(.6,0,-.1,(0,0,.1))['authorized'])
        self.decide(.7,0,-.1)
        self.assertTrue(self.decide(1.76,0,-.1)['authorized'])

    def test_stale_odometry_fails_closed_and_resets_confirmation(self):
        self.decide(0,.1,0)
        self.assertFalse(self.decide(.6,.1,0,age=.3)['authorized'])
        self.assertFalse(self.decide(.7,.1,0)['authorized'])
        self.assertTrue(self.decide(1.21,.1,0)['authorized'])

    def test_rebound_resets_stationary_hold(self):
        self.decide(0,.1,0)
        self.decide(.4,.1,0,(.02,0,0))
        self.assertFalse(self.decide(.5,.1,0)['authorized'])
        self.assertFalse(self.decide(.9,.1,0)['authorized'])
        self.assertTrue(self.decide(1.01,.1,0)['authorized'])
