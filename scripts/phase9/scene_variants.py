"""Deterministic offline scene appearance variation; never used by navigation."""
import numpy as np

def apply(world,seed):
 rng=np.random.default_rng(17000+seed)
 for model in world.findall('model'):
  # Broad reflectance variation includes low contrast and reversed floor/wall hues.
  color=rng.uniform(.08,.9,3)
  for material in model.findall('.//visual/material'):
   for key in ['ambient','diffuse']:
    node=material.find(key)
    if node is not None:node.text=' '.join(map(str,color))+' 1'
  if model.get('name')!='ground_plane':
   for size in model.findall('.//box/size'):
    old=np.fromstring(size.text,sep=' ');old*=rng.uniform(.65,1.35,3);size.text=' '.join(map(str,old))
 light=world.find('light')
 if light is not None:
  light.find('diffuse').text=' '.join([str(float(rng.uniform(.35,.95)))]*3)+' 1'
  light.find('direction').text=f'{rng.uniform(-.8,.8)} {rng.uniform(-.8,.8)} -.9'
