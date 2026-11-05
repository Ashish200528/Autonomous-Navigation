"""Bounded CPU training with scene-held-out evaluation and classical CV comparison."""
import argparse,json,random,time,hashlib
from pathlib import Path
import cv2,numpy as np,torch,psutil,yaml
from torch import nn
from network import Segmenter,load

def metrics(cm):
 a=cm.astype(float);tp=a.diagonal();union=a.sum(1)+a.sum(0)-tp
 return dict(confusion_matrix=cm.tolist(),iou=np.divide(tp,union,out=np.zeros(3),where=union>0).tolist(),mean_iou=float(np.divide(tp,union,out=np.zeros(3),where=union>0).mean()),precision=np.divide(tp,a.sum(0),out=np.zeros(3),where=a.sum(0)>0).tolist(),recall=np.divide(tp,a.sum(1),out=np.zeros(3),where=a.sum(1)>0).tolist())
def main():
 ap=argparse.ArgumentParser();ap.add_argument('dataset',type=Path);ap.add_argument('output',type=Path);ap.add_argument('--epochs',type=int,default=25);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
 torch.set_num_threads(2);torch.manual_seed(91);np.random.seed(91);random.seed(91)
 def data(split):
  ids=json.loads((args.dataset/'splits'/f'{split}.json').read_text());xs=[];ys=[]
  for n in ids:
   im=cv2.cvtColor(cv2.imread(str(args.dataset/'images'/f'{n}.png')),cv2.COLOR_BGR2RGB);mask=cv2.imread(str(args.dataset/'masks'/f'{n}.png'),0)
   xs.append(cv2.resize(im,(160,120)).transpose(2,0,1)/255.);ys.append(cv2.resize(mask,(160,120),interpolation=cv2.INTER_NEAREST))
  return ids,torch.tensor(np.array(xs),dtype=torch.float32),torch.tensor(np.array(ys),dtype=torch.long)
 train,x,y=data('train');val,vx,vy=data('validation');test,tx,ty=data('test');model=Segmenter();optimizer=torch.optim.Adam(model.parameters(),lr=.003)
 counts=torch.bincount(y[y<3],minlength=3).float();weights=(counts.sum()/counts.clamp_min(1)).sqrt();weights/=weights.mean();loss_fn=nn.CrossEntropyLoss(weight=weights,ignore_index=255)
 history=[];best=float('inf');bad=0;t0=time.perf_counter();peak=0;initial=next(model.parameters()).detach().clone()
 for epoch in range(args.epochs):
  model.train();total=0
  for idx in torch.randperm(len(x)).split(4):
   optimizer.zero_grad();loss=loss_fn(model(x[idx]),y[idx]);assert torch.isfinite(loss);loss.backward();optimizer.step();total+=loss.item()*len(idx)
  model.eval()
  with torch.no_grad():vl=loss_fn(model(vx),vy).item()
  history.append(dict(epoch=epoch+1,training_loss=total/len(x),validation_loss=vl));peak=max(peak,psutil.Process().memory_info().rss)
  if vl<best:
   best=vl;bad=0;torch.save(dict(schema=1,classes=['floor','obstacle','other'],state=model.state_dict()),args.output/'best.pt')
  else:bad+=1
  print(history[-1],flush=True)
  if bad>=5:break
 assert not torch.equal(initial,next(model.parameters()).detach()),'weights did not update'
 model=load(args.output/'best.pt');cm=np.zeros((3,3),np.int64);times=[];preds=[]
 with torch.inference_mode():
  model(tx[:1])
  for image,truth in zip(tx,ty):
   t=time.perf_counter();p=model(image[None]).argmax(1)[0].numpy();times.append((time.perf_counter()-t)*1000);valid=truth.numpy()<3;cm+=np.bincount(3*truth.numpy()[valid]+p[valid],minlength=9).reshape(3,3);preds.append(p)
 report=dict(status='TRAINED_AND_TESTED_OFFLINE',architecture='4 convolution encoder/decoder + 1x1 classifier; width 8/16; bilinear upsampling',parameters=sum(p.numel() for p in model.parameters()),seed=91,epochs=len(history),history=history,training_seconds=time.perf_counter()-t0,peak_process_rss_bytes=peak,device='cpu',torch_version=torch.__version__,class_training_pixels=counts.tolist(),checkpoint_bytes=(args.output/'best.pt').stat().st_size,checkpoint_sha256=hashlib.sha256((args.output/'best.pt').read_bytes()).hexdigest(),test=metrics(cm),inference_ms_mean=float(np.mean(times)),inference_ms_p95=float(np.percentile(times,95)),split_counts=dict(train=len(train),validation=len(val),test=len(test)))
 from anc_perception.classifier import classify_rgb
 root=Path(__file__).resolve().parents[2];cfg=yaml.safe_load((root/'simulation/ros2_ws/src/anc_perception/config/realistic.yaml').read_text())['/**']['ros__parameters'];cfg.update(camera_height_m=.24,camera_pitch_down_rad=.8,min_ground_forward_m=.1,max_ground_forward_m=.9,floor_seed_row_fraction=.75)
 cm_cv=np.zeros((3,3),np.int64);cv_times=[]
 palette=np.array([[30,190,60],[220,50,50],[70,100,220]]+[[100,100,100]]*253,np.uint8)
 for n,truth,p in zip(test,ty,preds):
  image=cv2.cvtColor(cv2.imread(str(args.dataset/'images'/f'{n}.png')),cv2.COLOR_BGR2RGB);f=320/(2*np.tan(1.0471975512/2));info=dict(width=320,height=240,k=[f,0,160,0,f,120,0,0,1],d=[0]*5)
  t=time.perf_counter();raw,_,_=classify_rgb(image,info,cfg);cv_times.append((time.perf_counter()-t)*1000)
  # Classical UNKNOWN (-1/255) is an abstention, mapped to other for the 3-class comparison; report this limitation.
  q=np.full(raw.shape,2,np.uint8);q[raw==0]=0;q[raw==100]=1;q=cv2.resize(q,(160,120),interpolation=cv2.INTER_NEAREST);valid=truth.numpy()<3
  cm_cv+=np.bincount(3*truth.numpy()[valid]+q[valid],minlength=9).reshape(3,3)
  tiles=[cv2.resize(image,(160,120)),palette[truth.numpy()],palette[q],palette[p]];cv2.imwrite(str(args.output/f'{n}-comparison.png'),cv2.cvtColor(np.hstack(tiles),cv2.COLOR_RGB2BGR))
 report['classical_cv']=dict(**metrics(cm_cv),latency_ms_mean=float(np.mean(cv_times)),unknown_policy='abstention mapped to other for 3-class comparison; not semantic other recognition')
 (args.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 plt.plot([r['training_loss'] for r in history],label='train');plt.plot([r['validation_loss'] for r in history],label='validation');plt.legend();plt.xlabel('Epoch');plt.ylabel('Cross entropy');plt.savefig(args.output/'loss.png');plt.close()
 print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2))
if __name__=='__main__':main()
