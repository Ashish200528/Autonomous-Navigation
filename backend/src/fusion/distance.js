// Camera distance per YOLO object ("depth optimization", plan §7.2):
//   B  known height  — your method from object_detection_new_babu.py
//   A  floor contact — size-independent, from camera height and tilt
//   C  dense depth   — MiDaS map scaled to metres (depth_scale.js), when fresh
//   fused by inverse variance, smoothed by the script's 5-sample median per track,
//   and self-calibrated against the front Sharp sensor (per-class real heights).
import { horizontalZone, isTruncated, knownHeightDistance, median, obstacleStatus } from '../perception/mono_distance.js';
import { intrinsics } from './geometry.js';

export class DistanceEstimator {
  constructor(cfg) {
    this.cfg = cfg;
    this.heightCorr = {}; // class -> factor learned from the Sharp
    this.samples = {};
  }

  focal() { return intrinsics(this.cfg).f; }

  floorContact(vBottom) {
    const c = this.cfg.camera, { f, cy } = intrinsics(this.cfg);
    const ang = (c.tilt_deg * Math.PI) / 180 + Math.atan((vBottom - cy) / f);
    return ang > 0.01 ? c.height_m / Math.tan(ang) : null;
  }

  // det: {name, score, box, track}; pan: servo angle (rad) at capture; sharpM: front range or Infinity;
  // depth: a fresh DepthScale result valid for this pan/pose, or null
  estimate(det, pan = 0, sharpM = Infinity, depth = null) {
    const { camera: cam, distance: dc, range: rc } = this.cfg;
    const [x1, y1, x2, y2] = det.box;
    const hPx = y2 - y1, f = this.focal();
    const truncated = isTruncated(y1, y2, cam.height);
    const known = det.name in dc.known_heights_m;
    const heights = known ? { [det.name]: dc.known_heights_m[det.name] * (this.heightCorr[det.name] ?? 1) } : {};
    const B = knownHeightDistance(hPx, det.name, f, heights, dc.default_height_m);
    const A = y2 >= cam.height - 2 ? null : this.floorContact(y2);
    const C = depth ? depth.boxDepth(det.box) : null;

    const cues = [];
    if (B) cues.push([B, (known ? 0.15 : 0.4) * B * (truncated ? 3 : 1)]);
    if (A) cues.push([A, 0.05 * A + 0.03 * A * A]);
    if (C) cues.push([C, C * (0.1 + depth.fit.medErr)]);
    let dist = null, sigma = null, inconsistent = false;
    const zs = cues.map((c) => c[0]);
    if (cues.length >= 2 && cues.some((a) => cues.some((b) => Math.abs(a[0] - b[0]) > 3 * Math.hypot(a[1], b[1])))) {
      inconsistent = true; // disagree: take the nearer one (safe side)
      [dist, sigma] = cues[zs.indexOf(Math.min(...zs))];
    } else if (cues.length) {
      const wsum = cues.reduce((s, [, sg]) => s + 1 / sg ** 2, 0);
      dist = cues.reduce((s, [z, sg]) => s + z / sg ** 2, 0) / wsum;
      sigma = Math.sqrt(1 / wsum);
    }

    const tr = det.track;
    if (tr && dist != null) {
      tr.hist.push(dist);
      if (tr.hist.length > dc.history) tr.hist.shift();
      const smooth = median(tr.hist), now = tr.t;
      if (tr.dist != null && now > tr.distT) {
        const closing = (tr.dist - smooth) / ((now - tr.distT) / 1000);
        tr.ttc = closing > 0.02 ? smooth / closing : null;
      }
      tr.dist = smooth; tr.distT = now;
      dist = smooth;
    }

    // Self-calibration: Sharp beam (image centre column, camera facing forward) hits this object.
    const sc = dc.selfcal;
    if (sc.enabled && known && B && !truncated && Math.abs(pan) < 0.05 && x1 < cam.width / 2 && x2 > cam.width / 2
      && sharpM >= rc.sharp_min_m && sharpM <= rc.sharp_max_m && Math.abs(B - sharpM) / sharpM < 0.5) {
      const s = (this.samples[det.name] ??= []);
      s.push((sharpM * hPx) / f / dc.known_heights_m[det.name]);
      if (s.length > sc.samples) s.shift();
      this.heightCorr[det.name] = Math.min(1 + sc.bound, Math.max(1 - sc.bound, median(s)));
    }

    return {
      id: tr?.id, cls: det.name, conf: det.score, box: det.box, truncated, inconsistent,
      zone: horizontalZone((x1 + x2) / 2, cam.width),
      bearing: pan - Math.atan(((x1 + x2) / 2 - cam.width / 2) / f), // + = left, robot frame
      dist, sigma, cues: { A, B, C }, ttc: tr?.ttc ?? null,
      status: obstacleStatus(dist, dc.flag_m, dc.warn_m),
    };
  }
}
