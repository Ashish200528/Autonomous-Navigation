import copy,hashlib,json,unittest
from pathlib import Path
import xml.etree.ElementTree as E
import cv2,numpy as np
from scene_variants import apply
ROOT=Path(__file__).resolve().parents[2]
class GeneralisationTests(unittest.TestCase):
 def test_variants_preserve_semantic_annotations(self):
  w=E.parse(ROOT/'docs/academic/phase9-data/worlds/scene-000.sdf').find('world');before=[p.text for p in w.findall('.//plugin/label')]
  apply(w,1000);self.assertEqual(before,[p.text for p in w.findall('.//plugin/label')])
  self.assertTrue(all(np.isfinite(np.fromstring(p.text,sep=' ')).all() for p in w.findall('.//material/diffuse')))
 def test_baseline_checkpoint_is_unchanged(self):
  self.assertEqual(hashlib.sha256((ROOT/'docs/academic/phase9-results/best.pt').read_bytes()).hexdigest(),'fa59636f29a1c7f048361839f64811416a5bdfba4fdfce99f3943f361e24a4fe')
 def test_diverse_scene_split_and_label_integrity(self):
  root=ROOT/'docs/academic/phase9d-data';r=json.loads((root/'metadata/capture.json').read_text());rows={v['id']:v for v in r['records']};seen=set();hashes=set()
  for split in ['train','validation','test']:
   ids=json.loads((root/'splits'/f'{split}.json').read_text());these={rows[n]['scene'] for n in ids};self.assertFalse(seen&these);seen|=these
   for n in ids:
    v=rows[n];self.assertEqual(v['appearance'],'diverse');self.assertGreaterEqual(v['variant_scene'],1000);self.assertEqual(v['rgb_stamp'],v['semantic_stamp'])
    im=cv2.imread(str(root/'images'/f'{n}.png'));mask=cv2.imread(str(root/'masks'/f'{n}.png'),0);self.assertEqual(im.shape[:2],mask.shape);self.assertTrue(set(np.unique(mask))<={0,1,2,255})
    digest=hashlib.sha256(im.tobytes()).hexdigest();self.assertNotIn(digest,hashes);hashes.add(digest)
if __name__=='__main__':unittest.main()
