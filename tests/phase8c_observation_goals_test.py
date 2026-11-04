import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from lidar_observation_goals import select_observation_goal,target_observed,target_corridor_observed


def grid(a):return dict(data=a.ravel().tolist(),width=a.shape[1],height=a.shape[0],resolution=.1,origin=[-5,-5],yaw=0)


class ObservationGoalsTests(unittest.TestCase):
    def test_connected_goal_behind_obstacle_is_not_a_clear_corridor(self):
        a=np.zeros((100,100),dtype=int);a[48:52,60:64]=100
        self.assertTrue(target_observed(grid(a),(0,0,0),(3,0,0)))
        self.assertFalse(target_corridor_observed(grid(a),(0,0,0),(3,0,0)))

    def test_unknown_intersecting_capsule_boundary_denies_goal(self):
        a=np.zeros((100,100),dtype=int);a[56,60]=-1
        self.assertFalse(target_corridor_observed(grid(a),(0,0,0),(3,0,0)))

    def test_unknown_far_from_corridor_does_not_block_observed_route(self):
        a=np.zeros((100,100),dtype=int);a[70,60]=-1
        self.assertTrue(target_corridor_observed(grid(a),(0,0,0),(3,0,0)))

    def test_corridor_cannot_extend_outside_observed_grid(self):
        a=np.zeros((100,100),dtype=int)
        self.assertFalse(target_corridor_observed(grid(a),(0,0,0),(4.8,0,0)))

    def test_arrival_heading_and_shorter_step_preserve_measured_geometry(self):
        a=np.zeros((100,100),dtype=int)
        r=select_observation_goal(grid(a),grid(a),(0,0,0),(4,1,0),max_step_m=1.5,heading_policy='arrival')
        self.assertEqual(r['status'],'SELECTED')
        self.assertLessEqual(r['step_m'],1.5)
        self.assertAlmostEqual(r['goal_map'][2],np.arctan2(r['goal_map'][1],r['goal_map'][0]))

    def test_invalid_heading_policy_is_rejected(self):
        a=np.zeros((100,100),dtype=int)
        with self.assertRaises(ValueError):select_observation_goal(grid(a),grid(a),(0,0,0),(4,0,0),heading_policy='invalid')

    def test_unknown_destination_does_not_become_observed(self):
        a=np.zeros((100,100),dtype=int);a[:,70:]=-1
        self.assertFalse(target_observed(grid(a),(0,0,0),(4,0,0)))
        result=select_observation_goal(grid(a),grid(a),(0,0,0),(4,0,0))
        self.assertEqual(result['status'],'SELECTED')
        self.assertLess(result['goal_map'][0],1.2)
        self.assertFalse(result['unknown_treated_as_free'])

    def test_disconnected_free_island_is_not_reachable(self):
        a=np.zeros((100,100),dtype=int);a[:,55:57]=100
        self.assertFalse(target_observed(grid(a),(0,0,0),(3,0,0)))
        r=select_observation_goal(grid(a),grid(a),(0,0,0),(4,0,0))
        self.assertEqual(r['status'],'BLOCKED')

    def test_all_unknown_cannot_authorize_a_goal(self):
        a=np.full((100,100),-1)
        self.assertEqual(select_observation_goal(grid(a),grid(a),(0,0,0),(4,0,0))['status'],'BLOCKED')

    def test_observation_step_is_bounded_and_retreat_is_not_selected(self):
        a=np.zeros((100,100),dtype=int)
        r=select_observation_goal(grid(a),grid(a),(0,0,0),(4,0,0))
        self.assertLessEqual(r['step_m'],2)
        self.assertGreater(r['goal_map'][0],0)
        self.assertGreaterEqual(r['local_clear_disk_radius_m'],.95)

    def test_local_unknown_overrides_global_free(self):
        a=np.zeros((100,100),dtype=int);local=np.full((100,100),-1)
        self.assertEqual(select_observation_goal(grid(a),grid(local),(0,0,0),(4,0,0))['status'],'BLOCKED')

    def test_local_body_frame_rotation_is_applied(self):
        a=np.zeros((100,100),dtype=int);local=np.zeros_like(a);local[:,:50]=-1
        r=select_observation_goal(grid(a),grid(local),(0,0,np.pi/2),(0,4,0))
        self.assertEqual(r['status'],'SELECTED')
        self.assertGreater(r['goal_map'][1],1)
