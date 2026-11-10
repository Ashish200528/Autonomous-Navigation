"""Small encoder/decoder CNN; no pretrained weights or motor interface."""
import torch
from torch import nn
class Segmenter(nn.Module):
 def __init__(self):
  super().__init__();self.layers=nn.Sequential(nn.Conv2d(3,8,3,padding=1),nn.ReLU(),nn.Conv2d(8,16,3,stride=2,padding=1),nn.ReLU(),nn.Conv2d(16,16,3,padding=1),nn.ReLU(),nn.Upsample(scale_factor=2,mode='bilinear',align_corners=False),nn.Conv2d(16,8,3,padding=1),nn.ReLU(),nn.Conv2d(8,3,1))
 def forward(self,x):return self.layers(x)
def load(path):
 checkpoint=torch.load(path,map_location='cpu',weights_only=True)
 if checkpoint['schema']!=1 or checkpoint['classes']!=['floor','obstacle','other']:raise ValueError('checkpoint schema/class mismatch')
 model=Segmenter();model.load_state_dict(checkpoint['state']);return model.eval()
