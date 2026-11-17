// Pseudo-LiDAR (plan §8): metric depth pixels back-projected into a 5 cm voxel map in the
// odometry frame, plus measured Sharp rays and YOLO objects. Old voxels fade so moving things
// clear and odometry drift does not pile up. scan() gives a LiDAR-like 1° virtual laser scan.
import { camToRobot, intrinsics, robotToWorld } from './geometry.js';

export const KIND = { CAMERA: 0, SHARP: 1, OBJECT: 2 };

export class VoxelMap {
  constructor(cfg) { this.cfg = cfg; this.vox = new Map(); }

  key(x, y, z) { const s = this.cfg.map.voxel_m; return `${Math.floor(x / s)},${Math.floor(y / s)},${Math.floor(z / s)}`; }

  // robot-frame point seen from pose at time t
  addRobotPoint(X, Y, Z, pose, t, kind) {
    const m = this.cfg.map;
    if (Z < m.floor_z_m || Z > m.max_z_m || Math.hypot(X, Y) > m.max_range_m) return false;
    const [x, y, z] = robotToWorld(X, Y, Z, pose), k = this.key(x, y, z);
    const v = this.vox.get(k);
    if (v) { v.hits++; v.t = t; if (kind > v.kind) v.kind = kind; } else this.vox.set(k, { x, y, z, hits: 1, t, kind });
    return true;
  }

  // depth: DepthScale with a good fit; every `step`-th map pixel is back-projected
  addDepth(depth, step = 4) {
    if (!depth.fit?.ok) return 0;
    const { fx, fy, cx, cy } = intrinsics(this.cfg), { n, W, H, meta } = depth;
    let added = 0;
    for (let i = step >> 1; i < n; i += step) {
      for (let j = step >> 1; j < n; j += step) {
        const Z = depth.depthOf(depth.disp[i * n + j]);
        if (!(Z > 0.1 && Z < this.cfg.map.max_range_m)) continue;
        const u = ((j + 0.5) * W) / n, v = ((i + 0.5) * H) / n;
        const [X, Y, Zr] = camToRobot(((u - cx) * Z) / fx, ((v - cy) * Z) / fy, Z, meta.pan, this.cfg);
        if (this.addRobotPoint(X, Y, Zr, meta.pose, meta.t, KIND.CAMERA)) added++;
      }
    }
    return added;
  }

  prune(nowMs) {
    const maxAge = this.cfg.map.fade_s * 1000;
    for (const [k, v] of this.vox) if (nowMs - v.t > maxAge) this.vox.delete(k);
  }

  // Float32Array [x, y, z, kind]*, newest first, capped at map.max_points
  cloud() {
    const vs = [...this.vox.values()].sort((a, b) => b.t - a.t).slice(0, this.cfg.map.max_points);
    const out = new Float32Array(vs.length * 4);
    vs.forEach((v, i) => out.set([v.x, v.y, v.z, v.kind], i * 4));
    return out;
  }

  // 360 bins (index = bearing in degrees, 0 = straight ahead, + = left, wraps), nearest obstacle
  // between floor and robot height. Infinity = nothing seen in that direction.
  scan(pose) {
    const out = new Float32Array(360).fill(Infinity), m = this.cfg.map;
    for (const v of this.vox.values()) {
      if (v.z > m.robot_height_m) continue;
      const dx = v.x - pose.x, dy = v.y - pose.y, r = Math.hypot(dx, dy);
      const deg = Math.round(((Math.atan2(dy, dx) - pose.yaw) * 180) / Math.PI);
      const b = ((deg % 360) + 360) % 360;
      if (r < out[b]) out[b] = r;
    }
    return out;
  }
}

// Nearest range in a bearing sector [a, b] degrees (a < b, may be negative).
export function nearestIn(scan, a, b) {
  let r = Infinity;
  for (let d = a; d <= b; d++) r = Math.min(r, scan[((d % 360) + 360) % 360]);
  return r;
}
