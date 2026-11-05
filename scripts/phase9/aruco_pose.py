"""Camera-pixel ArUco perception only; never imports Gazebo or publishes commands."""
import math,time
import cv2,numpy as np
MARKER_ID=23
MARKER_SIZE_M=.18

def marker_image(pixels=400):
 dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
 return cv2.aruco.drawMarker(dictionary,MARKER_ID,pixels)

def heading_in_marker(rvec,camera_pitch_down_rad):
 R,_=cv2.Rodrigues(np.asarray(rvec,float))
 p=camera_pitch_down_rad
 robot_from_sensor=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
 sensor_from_optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
 forward=R.T@(robot_from_sensor@sensor_from_optical).T@np.array([1.,0.,0.])
 return math.atan2(forward[1],forward[0])

def detect(rgb,k,distortion,stamp_s,now_s,marker_id=MARKER_ID,size_m=MARKER_SIZE_M):
 start=time.perf_counter()
 def invalid(reason):return dict(valid=False,reason=reason,source_stamp_s=stamp_s,motor_authority=False)
 if not all(math.isfinite(v) for v in [stamp_s,now_s,size_m]) or size_m<=0:return invalid('invalid_time_or_size')
 if not -.05<=now_s-stamp_s<=.35:return invalid('stale_or_future_image')
 if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:return invalid('invalid_rgb')
 try:k=np.asarray(k,float);d=np.asarray(distortion,float)
 except (TypeError,ValueError):return invalid('invalid_calibration')
 if k.shape!=(3,3) or d.size not in (0,4,5,8,12,14) or not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0,0],k[1,1])<=0:return invalid('invalid_calibration')
 gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
 params=cv2.aruco.DetectorParameters_create();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
 corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary,parameters=params)
 if ids is None:
  # Global Otsu fallback helps printed panels whose quiet border lies in shadow.
  _,binary=cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
  corners,ids,_=cv2.aruco.detectMarkers(binary,dictionary,parameters=params)
 if ids is None:return invalid('marker_missing')
 if len(ids)!=1 or int(ids[0,0])!=marker_id:return invalid('wrong_or_ambiguous_ids')
 c=corners[0].reshape(4,2).astype(np.float64)
 if min(np.linalg.norm(c-np.roll(c,1,axis=0),axis=1))<12:return invalid('marker_too_small')
 s=size_m/2;object_points=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float64)
 ok,rs,ts,errors=cv2.solvePnPGeneric(object_points,c,k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
 choices=[]
 for r,t in zip(rs,ts):
  if not np.isfinite(t).all() or t[2,0]<=0:continue
  projected,_=cv2.projectPoints(object_points,r,t,k,d);rms=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-c)**2,axis=1))))
  choices.append((rms,r,t))
 if not choices:return invalid('pose_unsolved')
 choices.sort(key=lambda a:a[0]);rms,r,t=choices[0]
 if rms>2:return invalid('reprojection_error')
 if len(choices)>1 and choices[1][0]-rms<.1:
  rotation,_=cv2.Rodrigues(r);other,_=cv2.Rodrigues(choices[1][1]);angle=math.acos(float(np.clip((np.trace(rotation.T@other)-1)/2,-1,1)))
  if angle>math.radians(10):return invalid('ambiguous_planar_pose')
 R,_=cv2.Rodrigues(r);up=R.T@np.array([0.,-1.,0.]);heading=math.atan2(up[0],up[1])
 return dict(valid=True,reason='marker_observed',marker_id=marker_id,marker_size_m=size_m,corners=c.tolist(),translation_optical_m=t.ravel().tolist(),rotation_vector=r.ravel().tolist(),distance_m=float(np.linalg.norm(t)),image_up_heading_rad=heading,reprojection_rms_px=rms,source_stamp_s=stamp_s,input_age_s=now_s-stamp_s,latency_ms=(time.perf_counter()-start)*1000,motor_authority=False)

def overlay(rgb,result):
 out=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
 if result['valid']:
  cv2.polylines(out,[np.round(result['corners']).astype(np.int32)],True,(0,255,0),2)
  text=f"ID23 {result['distance_m']:.3f}m RMS {result['reprojection_rms_px']:.2f}px"
 else:text=result['reason']
 cv2.putText(out,text,(4,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,255),1)
 return out
