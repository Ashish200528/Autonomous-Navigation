"""Frozen checkpoint evaluation: every frame in a declared scene-held-out split."""
import argparse,json,time,hashlib
from pathlib import Path
import cv2,numpy as np,torch,yaml
from network import load
from train import metrics
from anc_perception.classifier import classify_rgb

def evaluate(checkpoint,dataset,output,split='test'):
 output.mkdir(parents=True,exist_ok=False);torch.set_num_threads(2);model=load(checkpoint)
 ids=json.loads((dataset/'splits'/f'{split}.json').read_text());palette=np.array([[30,190,60],[220,50,50],[70,100,220]]+[[100,100,100]]*253,np.uint8)
 config=Path(__file__).resolve().parents[2]/'simulation/ros2_ws/src/anc_perception/config/realistic.yaml'
 cfg=yaml.safe_load(config.read_text())['/**']['ros__parameters'];cfg.update(camera_height_m=.24,camera_pitch_down_rad=.8,min_ground_forward_m=.1,max_ground_forward_m=.9,floor_seed_row_fraction=.75)
 cms={k:np.zeros((3,3),np.int64) for k in ['cnn','classical']};times={k:[] for k in cms};rows=[]
 with torch.inference_mode():
  model(torch.zeros(1,3,120,160))
  failures=[]
  for name in ids:
   rgb=cv2.cvtColor(cv2.imread(str(dataset/'images'/f'{name}.png')),cv2.COLOR_BGR2RGB);mask=cv2.imread(str(dataset/'masks'/f'{name}.png'),0);truth=cv2.resize(mask,(160,120),interpolation=cv2.INTER_NEAREST);valid=truth<3
   x=torch.tensor(cv2.resize(rgb,(160,120)).transpose(2,0,1)[None]/255.,dtype=torch.float32)
   t=time.perf_counter();prob=model(x).softmax(1);confidence,p=prob.max(1);times['cnn'].append((time.perf_counter()-t)*1000);p=p[0].numpy();confidence=confidence[0].numpy()
   f=320/(2*np.tan(1.0471975512/2));info=dict(width=320,height=240,k=[f,0,160,0,f,120,0,0,1],d=[0]*5)
   t=time.perf_counter()
   try:raw,_,_=classify_rgb(rgb,info,cfg)
   except ValueError as exc:
    failures.append(dict(id=name,reason=str(exc)));raw=np.full(rgb.shape[:2],255,np.uint8)
   times['classical'].append((time.perf_counter()-t)*1000)
   q=np.full(raw.shape,2,np.uint8);q[raw==0]=0;q[raw==100]=1;q=cv2.resize(q,(160,120),interpolation=cv2.INTER_NEAREST)
   for key,pred in [('cnn',p),('classical',q)]:cms[key]+=np.bincount(3*truth[valid]+pred[valid],minlength=9).reshape(3,3)
   wrong=valid&(truth!=p)
   rows.append(dict(id=name,accuracy=float((p[valid]==truth[valid]).mean()),unknown_fraction_at_06=float((confidence[valid]<.6).mean()),mean_confidence_wrong=float(confidence[wrong].mean()) if wrong.any() else None,obstacle_false_negative_pixels=int(((truth==1)&(p!=1)).sum())))
   image=np.hstack([cv2.resize(rgb,(160,120)),palette[truth],palette[q],palette[p]])
   cv2.imwrite(str(output/f'{name}.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
 report=dict(classical_invalid_frames=failures,checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),split=split,images=len(ids),dataset=str(dataset),samples=rows,convention='classical UNKNOWN maps to other; CNN 160x120 and classical native 320x240',results={k:dict(**metrics(cms[k]),latency_ms_mean=float(np.mean(times[k])),latency_ms_p95=float(np.percentile(times[k],95)),obstacle_false_negative_pixels=int(cms[k][1].sum()-cms[k][1,1])) for k in cms})
 (output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['results'],indent=2));return report
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('checkpoint',type=Path);ap.add_argument('dataset',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args();evaluate(a.checkpoint,a.dataset,a.output)
