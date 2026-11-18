import unittest
from phase8c_turn_stop_contract import evaluate_stop


class StopContractTests(unittest.TestCase):
    def trace(self, stop=.3):
        return [dict(query_start_wall_s=i*.05, wall_s=i*.05+.002,
                     sim_stamp_s=i*.05, yaw_rad=.1*max(0,min(i*.05,stop)),x_m=0,y_m=0)
                for i in range(-2,27)]

    def test_safe_physical_stop_with_full_confirmation(self):
        r=evaluate_stop(self.trace(),0,.3,1.31)
        self.assertEqual(r['status'],'PASS',r)
        self.assertGreater(r['stationary_observation_s'],.5)
        self.assertLess(r['stopping_angle_upper_rad'],.05)

    def test_late_physical_stop_not_rescued_by_confirmation(self):
        self.assertIn('physical_stop_time_exceeded',evaluate_stop(self.trace(.8),0,.3,1.31)['failures'])

    def test_confirmation_is_not_shortened(self):
        self.assertIn('stationary_confirmation_incomplete',evaluate_stop(self.trace()[:12],0,.3,.46)['failures'])

    def test_large_sampling_gap_fails_even_if_endpoints_stationary(self):
        s=self.trace();del s[12:18]
        self.assertIn('sampling_uncertainty_exceeds_budget',evaluate_stop(s,0,.3,1.31)['failures'])

    def test_rebound_near_end_resets_hold(self):
        s=self.trace();s[-3]['yaw_rad']+=.02
        self.assertIn('stationary_confirmation_incomplete',evaluate_stop(s,0,.3,1.31)['failures'])

    def test_reversing_yaw_cannot_cancel_stopping_angle(self):
        s=self.trace()
        for j in range(8,18):s[j]['yaw_rad']+=.04
        self.assertIn('stopping_angle_exceeded',evaluate_stop(s,0,.3,1.31)['failures'])

    def test_missing_zero_fails_closed(self):
        self.assertEqual(evaluate_stop(self.trace(),0,None,1.31)['status'],'FAIL')

    def test_frozen_simulation_cannot_confirm_stationary(self):
        s=self.trace()
        for p in s[8:]:p['sim_stamp_s']=.3
        self.assertEqual(evaluate_stop(s,0,.3,1.31)['status'],'FAIL')

    def test_query_uncertainty_is_part_of_time_bound(self):
        s=self.trace()
        for p in s:p['query_start_wall_s']-=.5
        self.assertIn('physical_stop_time_exceeded',evaluate_stop(s,0,.3,1.31)['failures'])

    def test_unrelated_startup_service_delay_is_not_fault_uncertainty(self):
        s=self.trace();s[0]['query_start_wall_s']-=1
        self.assertEqual(evaluate_stop(s,0,.3,1.31)['status'],'PASS')

    def test_source_age_cannot_shorten_confirmation_window(self):
        s=self.trace()[:19]
        # Stop candidate near .35, final response near .8 but source bracket is earlier.
        result=evaluate_stop(s,0,.3,.81)
        self.assertIn('stationary_confirmation_incomplete',result['failures'])
