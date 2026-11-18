"""Validate exact frame provenance; create scene-separated splits and review sheets."""
import argparse,json,hashlib
from pathlib import Path
import cv2,numpy as np
COLORS=np.array([[30,190,60],[220,50,50],[70,100,220]]+[[100,100,100]]*253,np.uint8)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('dataset',type=Path);a=ap.parse_args();root=a.dataset
 records=json.loads((root/'metadata/capture.json').read_text())['records'];counts=np.zeros(3,np.int64);seen={};duplicates=[];splits={k:[] for k in ['train','validation','test']};tiles=[]
 assert len(records)==len(list((root/'images').glob('*.png')))==len(list((root/'masks').glob('*.png')))
 for r in records:
  name=r['id'];im=cv2.imread(str(root/'images'/f'{name}.png'));mask=cv2.imread(str(root/'masks'/f'{name}.png'),0)
  assert im is not None and mask is not None and im.shape[:2]==mask.shape==(r['height'],r['width'])
  assert set(np.unique(mask))<={0,1,2,255} and np.any(mask!=255)
  assert r['rgb_stamp']==r['semantic_stamp'] and r['static_capture'] and r['label_source'].startswith('Gazebo')
  assert len(r['intrinsics'])==4 and min(r['intrinsics'])>0 and len(r['camera_pose'].split())==6
  digest=hashlib.sha256(im.tobytes()).hexdigest()
  if digest in seen:
   duplicates.append([seen[digest],name]);continue
  seen[digest]=name
  split='validation' if r['scene']%10==8 else 'test' if r['scene']%10==9 else 'train';splits[split].append(name)
  counts+=np.bincount(mask[mask<3],minlength=3)
  if len(tiles)<12:
   color=COLORS[mask][:,:,::-1];tile=np.hstack([im,color,cv2.addWeighted(im,.55,color,.45,0)])
   cv2.putText(tile,name,(5,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1);tiles.append(tile)
 assert all(splits.values()),'all scene splits required'
 for k,v in splits.items():(root/'splits'/f'{k}.json').write_text(json.dumps(v,indent=2)+'\n')
 for i,t in enumerate(tiles):cv2.imwrite(str(root/'metadata'/f'review-{i:02d}.png'),t)
 report=dict(status='OFFLINE_PASS',images=len(records),masks=len(records),resolution=[320,240],classes={'0':'traversable floor','1':'obstacle','2':'robot body/other','255':'unlabelled or class boundary: ignore'},class_pixels=counts.tolist(),split_counts={k:len(v) for k,v in splits.items()},duplicate_images_excluded_from_all_splits=duplicates,disk_bytes=sum(p.stat().st_size for p in root.rglob('*') if p.is_file()),alignment='exact matching simulator timestamp, cloned camera intrinsics and pose; static scene',annotation_provenance='Gazebo Label plugin: floor=10, obstacle=20, robot=30; unlabelled=ignore')
 (root/'metadata/validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
