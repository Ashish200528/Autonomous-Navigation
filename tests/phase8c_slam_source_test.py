import math
from array import array
import json
import tempfile
from types import SimpleNamespace
from pathlib import Path
import sys
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_slam_source import Stream,compose,inverse
from anc_lidar.scan_model import slam_hit_ranges
from slam_source_diagnostics import json_parameter,SlamSourceDiagnostics


class SlamSourceTests(unittest.TestCase):
    def test_parameter_audit_filters_undeclared_plugin_parameters(self):
        from rcl_interfaces.srv import ListParameters
        from rcl_interfaces.msg import ParameterValue
        known={'FollowPath.plugin':ParameterValue(type=4,string_value='sequential'),
               'FollowPath.linear_speed_mps':ParameterValue(type=3,double_value=.08)}
        class Client:
            def __init__(self,listing):self.listing=listing
            def wait_for_service(self,**_):return True
            def call_async(self,request):
                if self.listing:result=SimpleNamespace(result=SimpleNamespace(names=list(known)))
                else:
                    assert all(key in known for key in request.names)
                    result=SimpleNamespace(values=[known[key] for key in request.names])
                return SimpleNamespace(done=lambda:True,result=lambda:result)
        node=SimpleNamespace(create_client=lambda kind,name:Client(kind==ListParameters),destroy_client=lambda _:None)
        with tempfile.TemporaryDirectory() as path:
            diagnostic=SlamSourceDiagnostics.__new__(SlamSourceDiagnostics)
            diagnostic.node=node;diagnostic.out=Path(path);diagnostic.parameters_recorded=False
            diagnostic.record_parameters()
            audit=json.loads((Path(path)/'actual-runtime-parameters.json').read_text())
            self.assertEqual(audit['controller_server']['FollowPath.plugin'],'sequential')
            self.assertEqual(audit['controller_server']['FollowPath.linear_speed_mps'],.08)
            self.assertIn('FollowPath.max_angular_accel',audit['controller_server']['__not_declared'])

    def test_ros_array_parameter_can_be_saved_in_actual_configuration_audit(self):
        value=json_parameter(array('d',[.12,0,.1]))
        self.assertEqual(json.loads(json.dumps(value)),[.12,0,.1])
        self.assertIs(json_parameter(True),True)

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

    def test_s01_pose_progress_changes_only_progress_definition(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        baseline=yaml.safe_load((config/'nav2_s01_turn_room.yaml').read_text())
        candidate=yaml.safe_load((config/'nav2_s01_pose_progress.yaml').read_text())
        b=baseline['controller_server']['ros__parameters']['progress_checker']
        c=candidate['controller_server']['ros__parameters']['progress_checker']
        self.assertEqual(c.pop('plugin'),'nav2_controller::PoseProgressChecker')
        self.assertEqual(c.pop('required_movement_angle'),.5)
        b.pop('plugin')
        self.assertEqual(c['movement_time_allowance'],12.)
        self.assertEqual(c['required_movement_radius'],.10)
        self.assertEqual(baseline,candidate)

    def test_longer_lookahead_preserves_every_other_control_limit(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        baseline=yaml.safe_load((config/'nav2_s01_pose_progress.yaml').read_text())
        candidate=yaml.safe_load((config/'nav2_s01_sequential_lookahead.yaml').read_text())
        b=baseline['controller_server']['ros__parameters']['FollowPath']
        c=candidate['controller_server']['ros__parameters']['FollowPath']
        for field in ('lookahead_dist','min_lookahead_dist'):
            self.assertEqual(c.pop(field),.60);b.pop(field)
        self.assertEqual(baseline,candidate)

    def test_measured_slew_keeps_all_other_profile_parameters(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        baseline=yaml.safe_load((config/'nav2_s01_pose_progress.yaml').read_text())
        candidate=yaml.safe_load((config/'nav2_s01_measured_slew.yaml').read_text())
        accel=candidate['controller_server']['ros__parameters']['FollowPath'].pop('max_angular_accel')
        self.assertEqual(accel,candidate['velocity_smoother']['ros__parameters']['max_accel'][2])
        self.assertEqual(baseline,candidate)

    def test_optional_sequential_profile_preserves_goal_safety_and_upstream_limits(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        baseline=yaml.safe_load((config/'nav2_s01_pose_progress.yaml').read_text())
        candidate=yaml.safe_load((config/'nav2_sequential_commission.yaml').read_text())
        params=candidate['controller_server']['ros__parameters']
        self.assertEqual(params['FollowPath']['plugin'],'anc_nav2_sequential_controller::SequentialController')
        self.assertEqual(params['FollowPath']['angular_speed_radps'],.05)
        self.assertEqual(params['FollowPath']['linear_speed_mps'],.08)
        baseline['controller_server']['ros__parameters'].pop('FollowPath')
        params.pop('FollowPath')
        self.assertEqual(params['progress_checker']['required_movement_angle'],.4)
        params['progress_checker']['required_movement_angle']=.5
        self.assertEqual(baseline,candidate)

    def test_live_map_fusion_retains_unknown_rejection_and_maximum_obstacles(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        original=yaml.safe_load((config/'nav2_sequential_commission.yaml').read_text())
        fused=yaml.safe_load((config/'nav2_sequential_live_map.yaml').read_text())
        local=fused['local_costmap']['local_costmap']['ros__parameters']
        self.assertTrue(local['track_unknown_space'])
        self.assertEqual(local['static_layer']['map_topic'],'/map')
        self.assertFalse(local['static_layer']['footprint_clearing_enabled'])
        self.assertEqual(local['obstacle_layer'].pop('combination_method'),1)
        local.pop('static_layer');local['plugins'].remove('static_layer')
        self.assertEqual(original,fused)

    def test_nav2_no_return_encoding_never_clears_invalid_beams(self):
        from anc_lidar.scan_model import nav2_clearing_ranges
        values=[.5,5.999,6.,7.,0.,.12,float('nan'),float('-inf'),float('inf')]
        out=nav2_clearing_ranges(values,.12,6.)
        self.assertEqual(out[:2],values[:2]);self.assertEqual(out[2],float('inf'))
        self.assertTrue(all(math.isnan(v) for v in out[3:]))
        self.assertEqual(values[2],6.)

    def test_nav2_clearing_profile_changes_only_scan_endpoint_encoding(self):
        config=ROOT/'simulation/ros2_ws/src/anc_lidar/config'
        baseline=yaml.safe_load((config/'nav2_sequential_commission.yaml').read_text())
        candidate=yaml.safe_load((config/'nav2_sequential_clearing.yaml').read_text())
        for name in ['local_costmap','global_costmap']:
            scan=candidate[name][name]['ros__parameters']['obstacle_layer']['scan']
            self.assertTrue(scan.pop('inf_is_valid'));self.assertEqual(scan['topic'],'/realistic/scan/nav2')
            scan['topic']='/realistic/scan'
        self.assertEqual(baseline,candidate)
