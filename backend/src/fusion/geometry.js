// Camera / robot / world geometry shared by distance, depth scale and the 3D map.
// Camera: x right, y down, z forward (OpenCV), tilted down by tilt_deg, panned (+ = left) by the servo.
// Robot:  X forward, Y left, Z up, origin on the floor under the pan axis.
// World:  odometry frame (pose x, y, yaw).
import { focalFromFov } from '../perception/mono_distance.js';

export function intrinsics(cfg) {
  const c = cfg.camera, k = c.intrinsics;
  if (k) return { f: k.fy, fx: k.fx, fy: k.fy, cx: k.cx, cy: k.cy };
  const f = c.focal_px ?? focalFromFov(c.width, c.hfov_deg);
  return { f, fx: f, fy: f, cx: c.width / 2, cy: c.height / 2 };
}

const tilt = (cfg) => (cfg.camera.tilt_deg * Math.PI) / 180;

// camera point -> robot frame
export function camToRobot(xc, yc, zc, pan, cfg) {
  const th = tilt(cfg), down = yc * Math.cos(th) + zc * Math.sin(th), fwd = zc * Math.cos(th) - yc * Math.sin(th);
  const c = Math.cos(pan), s = Math.sin(pan);
  return [cfg.mount.camera_x_m + fwd * c + xc * s, fwd * s - xc * c, cfg.camera.height_m - down];
}

// robot point -> camera point (inverse of camToRobot)
export function robotToCam(X, Y, Z, pan, cfg) {
  const th = tilt(cfg), c = Math.cos(pan), s = Math.sin(pan);
  const dx = X - cfg.mount.camera_x_m, fwd = dx * c + Y * s, xc = dx * s - Y * c, down = cfg.camera.height_m - Z;
  return [xc, down * Math.cos(th) - fwd * Math.sin(th), down * Math.sin(th) + fwd * Math.cos(th)];
}

export function robotToWorld(X, Y, Z, pose) {
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  return [pose.x + X * c - Y * s, pose.y + X * s + Y * c, Z];
}

// Optical-axis depth of a floor point seen at image row v (null above the horizon).
export function floorDepthAt(v, cfg) {
  const { f, cy } = intrinsics(cfg), th = tilt(cfg), ang = th + Math.atan((v - cy) / f);
  if (ang < 0.03) return null;
  const D = cfg.camera.height_m / Math.tan(ang);
  return cfg.camera.height_m * Math.sin(th) + D * Math.cos(th);
}

// Pixel + optical depth of a robot-frame point (camera facing forward unless pan given).
export function project(X, Y, Z, pan, cfg) {
  const [xc, yc, zc] = robotToCam(X, Y, Z, pan, cfg), { fx, fy, cx, cy } = intrinsics(cfg);
  return zc > 0.01 ? { u: cx + (fx * xc) / zc, v: cy + (fy * yc) / zc, z: zc } : null;
}
