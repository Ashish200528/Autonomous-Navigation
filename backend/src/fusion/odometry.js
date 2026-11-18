// Differential-drive odometry from the signed encoder counts in telemetry. No IMU.
// Also keeps the distance travelled and the path (a point every trail_step_m) for the 3D view.

const TRAIL_STEP_M = 0.02, TRAIL_MAX = 20000; // 20000 x 2 cm = 400 m of path

export class Odometry {
  constructor(robot) { this.r = robot; this.resets = 0; this.reset(); }

  reset() {
    this.pose = { x: 0, y: 0, yaw: 0, v: 0, w: 0, sigma: 0 };
    this.dist = 0; this.wheel = [0, 0]; // metres travelled: centre, left, right (absolute, forward + back)
    this.trail = [[0, 0]];
    this.prev = null;
    this.resets++;
  }

  update(telem) {
    const p = this.pose;
    if (!this.prev || telem.tMs < this.prev.tMs) { this.prev = telem; return p; } // first sample or MCU reset
    const dl = ((telem.enc[0] - this.prev.enc[0]) * this.r.mm_per_pulse[0]) / 1000;
    const dr = ((telem.enc[1] - this.prev.enc[1]) * this.r.mm_per_pulse[1]) / 1000;
    const dt = (telem.tMs - this.prev.tMs) / 1000;
    this.prev = telem;
    const ds = (dl + dr) / 2;
    const dyaw = (dr - dl) / this.r.track_m;
    p.x += ds * Math.cos(p.yaw + dyaw / 2);
    p.y += ds * Math.sin(p.yaw + dyaw / 2);
    p.yaw = Math.atan2(Math.sin(p.yaw + dyaw), Math.cos(p.yaw + dyaw));
    if (dt > 0) { p.v = ds / dt; p.w = dyaw / dt; }
    // ponytail: crude drift model (2 % of distance + 5 cm per radian turned); fit it in Phase 1.
    p.sigma += 0.02 * Math.abs(ds) + 0.05 * Math.abs(dyaw);
    this.dist += Math.abs(ds); this.wheel[0] += Math.abs(dl); this.wheel[1] += Math.abs(dr);
    const [lx, ly] = this.trail.at(-1);
    if (Math.hypot(p.x - lx, p.y - ly) >= TRAIL_STEP_M) {
      this.trail.push([p.x, p.y]);
      if (this.trail.length > TRAIL_MAX) this.trail.splice(0, this.trail.length - TRAIL_MAX);
    }
    return p;
  }
}
