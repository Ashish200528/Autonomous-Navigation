import json,unittest,tempfile
from pathlib import Path
import xml.etree.ElementTree as E
import torch,numpy as np
from network import Segmenter,load
from train import metrics
ROOT=Path(__file__).resolve().parents[2]
class Phase9Tests(unittest.TestCase):
 def test_confusion_metrics_independent_fixture(self):
  r=metrics(np.array([[8,2,0],[1,9,0],[0,0,10]]));self.assertAlmostEqual(r['iou'][0],8/11);self.assertEqual(r['iou'][2],1)
 def test_tensor_and_checkpoint_schema(self):
  self.assertEqual(tuple(Segmenter()(torch.zeros(1,3,120,160)).shape),(1,3,120,160))
  with tempfile.TemporaryDirectory() as p:
   path=Path(p)/'bad.pt';torch.save(dict(schema=2,classes=[]),path)
   with self.assertRaises(ValueError):load(path)
 def test_camera_extrinsic_and_intrinsic_alignment(self):
  for path in (ROOT/'docs/academic/phase9-data/worlds').glob('*.sdf'):
   for m in E.parse(path).findall('.//model'):
    a=m.find('.//sensor[@type="camera"]');b=m.find('.//sensor[@type="segmentation"]')
    if a is None:continue
    self.assertIsNotNone(b)
    for key in ['pose','camera/horizontal_fov','camera/image/width','camera/image/height','update_rate']:
     self.assertEqual(a.find(key).text,b.find(key).text)
 def test_splits_have_no_pixel_duplicates_or_scene_overlap(self):
  import cv2,hashlib
  root=ROOT/'docs/academic/phase9-data';prior=set();scenes=set()
  for split in ['train','validation','test']:
   ids=json.loads((root/'splits'/f'{split}.json').read_text());these={n.split('-')[0] for n in ids};self.assertFalse(scenes&these);scenes|=these
   for n in ids:
    d=hashlib.sha256(cv2.imread(str(root/'images'/f'{n}.png')).tobytes()).hexdigest();self.assertNotIn(d,prior);prior.add(d)
class ROSImageTests(unittest.TestCase):
 def test_conversion_and_invalid_encoding(self):
  from sensor_msgs.msg import Image
  from anc_perception.perception_node import _decode_rgb,_image_message
  m=Image();m.header.stamp.sec=42
  rgb=np.zeros((3,4,3),np.uint8);rgb[:,:,0]=200
  converted=_image_message(rgb,'rgb8',m.header)
  self.assertTrue(np.array_equal(_decode_rgb(converted),rgb));self.assertEqual(converted.header.stamp.sec,42)
  converted.encoding='unsupported'
  with self.assertRaises(ValueError):_decode_rgb(converted)
 def test_shadow_missing_model_and_stale_input_fail_closed(self):
  import time
  from types import SimpleNamespace
  from sensor_msgs.msg import Image
  from shadow_node import Shadow
  states=[];fake=SimpleNamespace(model=None,last=None,status=lambda valid,reason,**extra:states.append((valid,reason)))
  Shadow.image(fake,Image());self.assertEqual(states[-1],(False,'model_missing'))
  fake.last=time.monotonic()-1;Shadow.stale(fake);self.assertEqual(states[-1],(False,'input_stale_or_missing'))
if __name__=='__main__':unittest.main()
