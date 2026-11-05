import unittest,numpy as np,cv2,math
from aruco_pose import detect,marker_image,MARKER_SIZE_M
class ArucoTests(unittest.TestCase):
 def image(self):
  im=np.full((480,640,3),255,np.uint8);im[140:340,220:420]=cv2.cvtColor(marker_image(200),cv2.COLOR_GRAY2RGB);return im
 def test_known_front_view_distance(self):
  k=np.array([[600,0,320],[0,600,240],[0,0,1]],float);r=detect(self.image(),k,[0]*5,1,1)
  self.assertTrue(r['valid'],r);self.assertLess(abs(r['distance_m']-600*MARKER_SIZE_M/200),.01)
 def test_stale_wrong_id_and_missing(self):
  k=np.array([[600,0,320],[0,600,240],[0,0,1]],float)
  self.assertFalse(detect(self.image(),k,[0]*5,1,2)['valid'])
  self.assertFalse(detect(self.image(),k,[0]*5,1,1,marker_id=22)['valid'])
  self.assertFalse(detect(np.full((480,640,3),255,np.uint8),k,[0]*5,1,1)['valid'])
 def test_invalid_calibration(self):
  self.assertFalse(detect(self.image(),np.zeros((3,3)),[0]*5,1,1)['valid'])

class ExtendedArucoTests(unittest.TestCase):
 def test_duplicate_ids_are_rejected(self):
  im=np.full((480,800,3),255,np.uint8);marker=cv2.cvtColor(marker_image(150),cv2.COLOR_GRAY2RGB);im[150:300,100:250]=marker;im[150:300,500:650]=marker
  r=detect(im,[[600,0,400],[0,600,240],[0,0,1]],[0]*5,1,1)
  self.assertEqual(r['reason'],'wrong_or_ambiguous_ids')
 def test_heading_extrinsic_convention(self):
  from scipy.spatial.transform import Rotation
  from aruco_pose import heading_in_marker
  # Independent rotation library: intrinsic robot yaw then camera downward pitch.
  for yaw in [-.2,0,.2]:
   sensor=Rotation.from_euler('ZY',[yaw,.8]).as_matrix();optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
   rvec=Rotation.from_matrix((sensor@optical).T).as_rotvec()
   self.assertAlmostEqual(heading_in_marker(rvec,.8),yaw)
 def test_marker_translation_from_projected_corners(self):
  # A frontal square of physical width .18 m occupies 180 px at .60 m, fx=600.
  im=np.full((480,640,3),255,np.uint8);im[150:330,230:410]=cv2.cvtColor(marker_image(180),cv2.COLOR_GRAY2RGB)
  r=detect(im,[[600,0,320],[0,600,240],[0,0,1]],[0]*5,3,3)
  self.assertTrue(r['valid'],r);self.assertLess(np.linalg.norm(np.array(r['translation_optical_m'])-[0,0,.6]),.01)

class ArucoROSTests(unittest.TestCase):
 def test_missing_calibration_invalidates_health(self):
  from types import SimpleNamespace
  from sensor_msgs.msg import Image
  from aruco_node import ArucoNode
  states=[];fake=SimpleNamespace(info=None,last=None,status=lambda r:states.append(r))
  ArucoNode.image(fake,Image());self.assertFalse(states[-1]['valid']);self.assertEqual(states[-1]['reason'],'calibration_missing_or_stale')
 def test_stale_input_invalidates_health(self):
  from types import SimpleNamespace
  import time
  from aruco_node import ArucoNode
  states=[];ArucoNode.stale(SimpleNamespace(last=time.monotonic()-1,status=lambda r:states.append(r)))
  self.assertFalse(states[-1]['valid']);self.assertFalse(states[-1]['motor_authority'])

class ResourceTests(unittest.TestCase):
 def test_low_memory_prevents_simulator_launch(self):
  from unittest.mock import patch
  from types import SimpleNamespace
  import aruco_live_probe,sys
  with patch.object(sys,'argv',['probe','unused-world','unused-output']),patch.object(aruco_live_probe.psutil,'virtual_memory',return_value=SimpleNamespace(available=0)),patch.object(aruco_live_probe.subprocess,'Popen',side_effect=AssertionError('must not start simulator')):
   with self.assertRaisesRegex(RuntimeError,'memory preflight'):aruco_live_probe.main()

if __name__=='__main__':unittest.main()
