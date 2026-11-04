"""Conservative observation goals from live grids, with no world/obstacle oracle."""
import math
import cv2
import numpy as np


def cells(grid):
    return np.asarray(grid['data'],dtype=np.int16).reshape(grid['height'],grid['width'])


def indices(grid,x,y):
    yaw=grid.get('yaw',0);c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=x-grid['origin'][0],y-grid['origin'][1]
    return np.floor((c*dx+s*dy)/grid['resolution']).astype(int),np.floor((-s*dx+c*dy)/grid['resolution']).astype(int)


def component(grid,pose):
    free=(cells(grid)==0).astype(np.uint8)
    _,labels=cv2.connectedComponents(free,connectivity=8)
    x,y=indices(grid,np.asarray(pose[0]),np.asarray(pose[1]))
    if not (0<=x<grid['width'] and 0<=y<grid['height']) or labels[y,x]==0:return None
    return labels==labels[y,x]


def target_observed(grid,pose,target):
    connected=component(grid,pose)
    x,y=indices(grid,np.asarray(target[0]),np.asarray(target[1]))
    return bool(connected is not None and 0<=x<grid['width'] and 0<=y<grid['height'] and connected[y,x])


def target_corridor_observed(grid,pose,target,radius_m=.60):
    """Require a wholly observed FREE capsule, including intersecting cell corners.

    This stricter observation condition does not authorize motion. Nav2 and the
    runtime guard still check every command. No scenario geometry is consulted.
    """
    if not all(math.isfinite(float(v)) for v in (*pose[:2],*target[:2],radius_m)) or radius_m<=0:return False
    yaw=grid.get('yaw',0);c,s=math.cos(yaw),math.sin(yaw)
    def local(p):
        dx,dy=p[0]-grid['origin'][0],p[1]-grid['origin'][1]
        return np.array([c*dx+s*dy,-s*dx+c*dy])
    a,b=local(pose),local(target);resolution=grid['resolution']
    bounds=np.array([grid['width'],grid['height']])*resolution
    if np.any(np.minimum(a,b)-radius_m<0) or np.any(np.maximum(a,b)+radius_m>bounds):return False
    ys,xs=np.indices((grid['height'],grid['width']))
    px,py=(xs+.5)*resolution,(ys+.5)*resolution;d=b-a;length2=float(np.dot(d,d))
    t=np.clip(((px-a[0])*d[0]+(py-a[1])*d[1])/length2,0,1) if length2 else 0
    distance=np.hypot(px-a[0]-t*d[0],py-a[1]-t*d[1])
    touched=distance<=radius_m+math.sqrt(2)*resolution/2
    return bool(np.all(cells(grid)[touched]==0))


def select_observation_goal(global_grid,local_grid,pose,target,visited=(),max_step_m=2.0,heading_policy='destination'):
    """Known connected global cell AND a locally observed >=.95 m clear disk.

    UNKNOWN and occupied cells are both obstacles. Goals are chosen for progress
    toward the destination, bounded to 2 m; Nav2 still plans and the guard decides
    whether each requested primitive is permitted. No route is invented here.
    """
    if heading_policy not in ('destination','arrival'):raise ValueError('invalid_observation_heading_policy')
    connected=component(global_grid,pose)
    if connected is None:return {'status':'BLOCKED','reason':'start_not_in_known_free_component'}
    radius_cells=int(math.ceil(.95/local_grid['resolution']))
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius_cells+1,2*radius_cells+1))
    safe=cv2.erode((cells(local_grid)==0).astype(np.uint8),kernel,
                   borderType=cv2.BORDER_CONSTANT,borderValue=0)
    ys,xs=np.where(connected)
    yaw=global_grid.get('yaw',0);c,s=math.cos(yaw),math.sin(yaw)
    gx=(xs+.5)*global_grid['resolution'];gy=(ys+.5)*global_grid['resolution']
    wx=global_grid['origin'][0]+c*gx-s*gy;wy=global_grid['origin'][1]+s*gx+c*gy
    dx,dy=wx-pose[0],wy-pose[1];travel=np.hypot(dx,dy)
    c,s=math.cos(pose[2]),math.sin(pose[2]);bx=c*dx+s*dy;by=-s*dx+c*dy
    lx,ly=indices(local_grid,bx,by)
    valid=(travel>=.35)&(travel<=max_step_m)&(lx>=0)&(ly>=0)&(lx<local_grid['width'])&(ly<local_grid['height'])
    li=np.where(valid)[0]
    valid[li]&=safe[ly[li],lx[li]]>0
    remaining=np.hypot(wx-target[0],wy-target[1])
    valid&=remaining<=math.hypot(target[0]-pose[0],target[1]-pose[1])-.2
    for old in visited:valid&=np.hypot(wx-old[0],wy-old[1])>=.35
    choices=np.where(valid)[0]
    if not len(choices):return {'status':'BLOCKED','reason':'no_known_connected_clear_observation_goal'}
    best=choices[np.argmin((remaining+.1*travel)[choices])]
    heading=(math.atan2(target[1]-wy[best],target[0]-wx[best]) if heading_policy=='destination'
             else math.atan2(dy[best],dx[best]))
    return dict(status='SELECTED',goal_map=[float(wx[best]),float(wy[best]),heading],heading_policy=heading_policy,
                local_clear_disk_radius_m=radius_cells*local_grid['resolution'],
                step_m=float(travel[best]),remaining_m=float(remaining[best]),
                selected_from='connected_global_FREE_and_local_FREE_disk',unknown_treated_as_free=False)
