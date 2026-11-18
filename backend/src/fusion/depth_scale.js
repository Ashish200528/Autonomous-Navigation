// Relative MiDaS output -> metres (plan §7.3). MiDaS gives inverse depth up to an unknown
// scale and shift, so 1/Z = s·d + t is fitted with RANSAC against points whose real depth
// is known: floor pixels (camera height/tilt), the Sharp beam, and objects' cue A/B distances.
import { floorDepthAt, project } from './geometry.js';
import { median } from '../perception/mono_distance.js';

export class DepthScale {
  // disp: Float32Array n*n (MiDaS output), meta: {t, pan, pose} of the frame it came from
  constructor(disp, n, meta, cfg) {
    Object.assign(this, { disp, n, meta, cfg, fit: null });
    this.W = cfg.camera.width; this.H = cfg.camera.height;
  }

  idx(u, v) {
    const j = Math.min(this.n - 1, Math.max(0, Math.floor((u * this.n) / this.W)));
    const i = Math.min(this.n - 1, Math.max(0, Math.floor((v * this.n) / this.H)));
    return i * this.n + j;
  }

  // bilinear: anchors sit at exact pixels, the map is coarser than the image
  dispAt(u, v) {
    const n = this.n, x = Math.min(n - 1.001, Math.max(0, (u * n) / this.W - 0.5)), y = Math.min(n - 1.001, Math.max(0, (v * n) / this.H - 0.5));
    const j = Math.floor(x), i = Math.floor(y), ax = x - j, ay = y - i, d = this.disp, k = i * n + j;
    return (d[k] * (1 - ax) + d[k + 1] * ax) * (1 - ay) + (d[k + n] * (1 - ax) + d[k + n + 1] * ax) * ay;
  }

  depthOf(d) { const y = this.fit.s * d + this.fit.t; return y > 1e-3 ? 1 / y : Infinity; }

  // anchors: [{u, v, z, w?}] (z = optical-axis metres, w = repeat weight)
  solve(anchors) {
    const c = this.cfg.depth;
    const pts = [];
    for (const a of anchors) for (let k = 0; k < (a.w ?? 1); k++) pts.push({ d: this.dispAt(a.u, a.v), y: 1 / a.z, z: a.z });
    if (pts.length < c.min_inliers) return (this.fit = { ok: false, reason: 'too few anchors', n: pts.length });
    let seed = 12345;
    const rnd = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
    const inliersOf = (s, t) => pts.filter((p) => { const y = s * p.d + t; return y > 0 && Math.abs(1 / y - p.z) / p.z < c.inlier_rel_err; });
    let best = [];
    for (let k = 0; k < c.ransac_iters; k++) {
      const a = pts[Math.floor(rnd() * pts.length)], b = pts[Math.floor(rnd() * pts.length)];
      if (Math.abs(a.d - b.d) < 1e-6) continue;
      const s = (a.y - b.y) / (a.d - b.d), t = a.y - s * a.d;
      if (!(s > 0)) continue; // larger disparity must mean nearer
      const inl = inliersOf(s, t);
      if (inl.length > best.length) best = inl;
    }
    if (best.length < c.min_inliers) return (this.fit = { ok: false, reason: 'no consistent scale', n: pts.length });
    // least squares on the inliers
    const m = best.length, sd = best.reduce((x, p) => x + p.d, 0), sy = best.reduce((x, p) => x + p.y, 0);
    const sdd = best.reduce((x, p) => x + p.d * p.d, 0), sdy = best.reduce((x, p) => x + p.d * p.y, 0);
    const den = m * sdd - sd * sd;
    const s = den ? (m * sdy - sd * sy) / den : 0, t = (sy - s * sd) / m;
    const errs = best.map((p) => Math.abs(1 / (s * p.d + t) - p.z) / p.z);
    this.fit = { ok: s > 0, s, t, inliers: m, n: pts.length, frac: m / pts.length, medErr: median(errs) };
    return this.fit;
  }

  // Median metric depth in the lower-centre of a box (where the object meets its own base).
  boxDepth([x1, y1, x2, y2]) {
    if (!this.fit?.ok) return null;
    const zs = [];
    const h = y2 - y1, w = x2 - x1;
    for (let v = y2 - 0.3 * h; v <= y2 - 0.05 * h; v += Math.max(1, (0.25 * h) / 4))
      for (let u = x1 + 0.25 * w; u <= x2 - 0.25 * w; u += Math.max(1, (0.5 * w) / 4)) zs.push(this.depthOf(this.dispAt(u, v)));
    const z = zs.length ? median(zs) : null;
    return z && Number.isFinite(z) ? z : null;
  }

  // Fresh enough, same servo angle, robot hardly moved since the frame was taken.
  validFor(pan, pose, nowMs) {
    const m = this.meta;
    return !!this.fit?.ok && nowMs - m.t < this.cfg.depth.max_age_ms && Math.abs(pan - m.pan) < 0.05
      && Math.hypot(pose.x - m.pose.x, pose.y - m.pose.y) < 0.05 && Math.abs(pose.yaw - m.pose.yaw) < 0.05;
  }
}

// Points of known depth in the frame: floor grid, Sharp spot, YOLO objects' bottoms.
export function buildAnchors(cfg, { pan, sharpM, objects }) {
  const W = cfg.camera.width, H = cfg.camera.height, out = [];
  const boxes = objects.map((o) => o.box);
  const inBox = (u, v) => boxes.some(([x1, y1, x2, y2]) => u >= x1 && u <= x2 && v >= y1 && v <= y2);
  for (let v = H * 0.7; v < H; v += H * 0.06) {
    const z = floorDepthAt(v, cfg);
    if (z) for (let u = W * 0.25; u <= W * 0.75; u += W * 0.083) if (!inBox(u, v)) out.push({ u, v, z });
  }
  if (Math.abs(pan) < 0.05 && Number.isFinite(sharpM) && sharpM >= cfg.range.sharp_min_m && sharpM <= cfg.range.sharp_max_m) {
    const p = project(cfg.mount.sharp_x_m + sharpM, 0, cfg.mount.sharp_height_m, 0, cfg);
    if (p && p.v >= 0 && p.v < H) out.push({ u: p.u, v: p.v, z: p.z, w: 5 }); // measured: weigh it more
  }
  for (const o of objects) {
    const ab = [o.cues?.A, o.cues?.B].filter(Boolean);
    if (!ab.length || o.truncated || o.inconsistent) continue;
    const D = Math.min(...ab), th = (cfg.camera.tilt_deg * Math.PI) / 180;
    out.push({ u: (o.box[0] + o.box[2]) / 2, v: o.box[3] - 3, z: cfg.camera.height_m * Math.sin(th) + D * Math.cos(th) });
  }
  return out;
}
