// Port of anc_autodrive/brain_node.py (same states, same rules), without ROS and the HUD.
// Inputs arrive through on*(); tick(now) returns {v, w, pan, state, note}. Time in seconds.
//
// "range" here is the distance to the nearest thing straight ahead: the minimum of the front
// Sharp/IR range and the camera distance of YOLO objects in the Center zone (index.js).
import { crossingFraction, findLane, flipX, laneError, lineMasks, obstacleFraction, overtakePlan, wrap } from '../perception/lanes.js';
import { nearestIn } from '../fusion/map3d.js';

export class Brain {
  constructor(p, frame, log = () => {}) {
    this.p = p;
    this.frameW = frame.width; this.frameH = frame.height; // camera pixels (YOLO boxes)
    this.log = log;
    this.img = null; this.imgT = -1e9; this.rangeT = -1e9;
    this.range = Infinity;
    this.pose = null;
    this.lane = [null, null, null, null];
    this.laneSeenT = -1e9;
    this.onCrossing = false; this.crossYaw = null;
    this.lastErr = 0;
    this.vision = { aruco: [], objects: [] }; this.visionT = -1e9;
    this.state = p.autostart ? 'DRIVE' : 'HOLD';
    this.goFlag = false;
    this.note = ''; this.t0 = 0; this.servo = 0;
    this.scan = []; this.scanResult = {};
    this.plan = []; this.stepStart = null; this.passHalf = null;
    this.laps = 0; this.nudges = 0;
    this.stationGoal = null; this.stationCooldown = 0; this.stopSignCooldown = 0;
    this.v = 0; this.w = 0; this.now = 0;
  }

  // ------------------------------------------------------------------ inputs
  onImage(img, t) {
    this.img = img; this.imgT = t;
    if (Math.abs(this.servo) < 0.05) { // lane is only meaningful with the camera looking ahead
      const masks = lineMasks(img);
      this.lane = findLane(img, this.lane[3], undefined, undefined, masks);
      this.onCrossing = crossingFraction(img, undefined, undefined, masks) > 0.08;
      if (this.lane[0] != null || this.onCrossing) this.laneSeenT = t;
    }
  }
  onRange(m, t) { this.range = Number.isFinite(m) ? m : Infinity; this.rangeT = t; }
  onPose(pose) { this.pose = [pose.x, pose.y, pose.yaw]; }
  onVision(v, t) { this.vision = v; this.visionT = t; }
  onScan(scan, t) { this.scanData = scan; this.scanT = t; } // virtual laser scan from map3d.js
  go() { this.goFlag = true; }

  // ----------------------------------------------------------------- helpers
  to(state, note = '') {
    if (state !== this.state) this.log(`${this.state} -> ${state} ${note}`);
    this.state = state; this.note = note; this.t0 = this.now;
  }

  nearest(cls, zones = ['Left', 'Center', 'Right']) {
    if (this.now - this.visionT > 1.5) return null;
    const d = this.vision.objects.filter((o) => o.cls === cls && zones.includes(o.zone) && o.dist).map((o) => o.dist);
    return d.length ? Math.min(...d) : null;
  }

  sideIsClear(label) {
    if (!this.img) return [false, 1, []];
    const frac = obstacleFraction(this.img, this.p.scan_roi);
    const fresh = this.visionT > this.t0 + 0.3; // YOLO result taken after the servo settled
    const W = this.frameW;
    // Looking left, the image's right quarter is the road ahead (the obstacle we want to pass).
    const ahead = label === 'left' ? (cx) => cx > 0.75 * W : (cx) => cx < 0.25 * W;
    const bodyY = this.p.scan_roi[1] * this.frameH;
    const seen = fresh ? this.vision.objects.filter((o) => o.dist != null && o.dist < this.p.yolo_block_distance_m
      && !ahead((o.box[0] + o.box[2]) / 2) && (o.box[1] + o.box[3]) / 2 < bodyY).map((o) => o.cls) : [];
    // Pseudo-LiDAR: anything in the 3D map inside that side's sector also blocks (plan §9).
    if (this.scanData && this.now - this.scanT < 2) {
      const [a, b] = this.p.side_sector_deg;
      const r = label === 'left' ? nearestIn(this.scanData, a, b) : nearestIn(this.scanData, -b, -a);
      if (r < this.p.side_block_m) seen.push(`map ${r.toFixed(2)} m`);
    }
    return [frac < this.p.blocked_fraction && !seen.length, frac, seen];
  }

  backedM() { return this.nudges * this.p.nudge_reverse_m; }

  // -------------------------------------------------------------------- loop
  tick(now) {
    this.now = now;
    this.v = this.w = 0;
    const stale = now - this.imgT > this.p.sensor_timeout_s || now - this.rangeT > this.p.sensor_timeout_s || !this.pose;
    if (stale) this.note = 'waiting for camera / range / odometry';
    else this['do' + this.state[0] + this.state.slice(1).toLowerCase().replace(/_(.)/g, (_, c) => c.toUpperCase())](now);
    if (this.range < this.p.flag_m && this.v > 0) { this.v = 0; this.note = `EMERGENCY STOP ${(this.range * 100).toFixed(0)} cm`; }
    return { v: this.v, w: this.w, pan: this.servo, state: this.state, note: this.note, laps: this.laps };
  }

  doDrive(now) {
    const p = this.p;
    this.servo = 0;
    const person = this.nearest('person', ['Center']), anyone = this.nearest('person');
    const stopSign = this.nearest('stop sign', ['Center', 'Right']);
    if (person != null && person < p.yield_distance_m) return this.to('YIELD', `pedestrian ${person.toFixed(1)} m ahead`);
    if (this.range < p.decide_distance_m && anyone != null && anyone < p.pedestrian_caution_m)
      return this.to('YIELD', `pedestrian stepped out (${this.range.toFixed(2)} m)`);
    if (stopSign != null && stopSign < p.stop_sign_stop_distance_m && now > this.stopSignCooldown)
      return this.to('STOP_SIGN', `stop sign at ${stopSign.toFixed(1)} m`);
    if (this.range < p.decide_distance_m) return this.startScan(`obstacle at ${this.range.toFixed(2)} m`);

    if (now - this.visionT < 1.0) for (const m of this.vision.aruco ?? []) {
      if (m.id === p.station_marker_id && m.dist < p.station_detect_distance_m && now > this.stationCooldown && !this.stationGoal) {
        const ahead = m.dist * Math.cos((m.angle_deg * Math.PI) / 180) - p.station_stop_before_m;
        this.stationGoal = [this.pose[0], this.pose[1], Math.max(0, ahead)];
        this.log(`station marker ${m.id} seen at ${m.dist.toFixed(2)} m, stopping in ${ahead.toFixed(2)} m`);
      }
    }
    if (this.stationGoal) {
      const [x0, y0, ahead] = this.stationGoal;
      if (Math.hypot(this.pose[0] - x0, this.pose[1] - y0) >= ahead) { this.stationGoal = null; return this.to('STATION', 'arrived at station'); }
    }

    const centre = this.lane[0];
    if (this.onCrossing) {
      this.crossYaw ??= this.pose[2];
      this.v = p.slow_speed;
      this.w = Math.max(-0.4, Math.min(0.4, 2 * wrap(this.crossYaw - this.pose[2])));
      this.note = 'zebra crossing - holding heading';
      return;
    }
    this.crossYaw = null;
    let err, speed, roam = false;
    if (centre == null) {
      if (now - this.laneSeenT > 1.0) {
        // No lane lines (e.g. a plain floor): free roam drives straight at slow speed; every obstacle
        // rule above (stop, slow, scan and pass, yield, stop sign) still applies.
        if (!p.free_roam) { this.note = 'LANE LOST - stopped'; return; }
        err = 0; speed = p.slow_speed; roam = true;
      } else { err = this.lastErr; speed = p.slow_speed; } // brief dropout: keep the last correction, go slow
    } else { err = laneError(centre, this.img.w); speed = p.cruise_speed; }
    if (this.stationGoal) speed = Math.min(speed, p.slow_speed);
    let caution = '';
    if (this.range < p.warn_m) caution = `obstacle ${this.range.toFixed(2)} m - slowing`;
    else if (anyone != null && anyone < p.pedestrian_caution_m) caution = 'pedestrians nearby - slowing';
    else if (stopSign != null && now > this.stopSignCooldown) caution = 'stop sign ahead - slowing';
    if (caution) speed = Math.min(speed, p.slow_speed);
    const derr = err - this.lastErr; this.lastErr = err;
    this.w = Math.max(-p.max_turn_rate, Math.min(p.max_turn_rate, -(p.steer_kp * err + (p.steer_kd * derr) / 0.05)));
    this.v = speed * (1 - 0.5 * Math.min(1, Math.abs(err))); // slow in sharp corrections
    this.note = this.stationGoal ? 'station ahead - slowing to stop' : caution
      || (roam ? 'no lane lines - driving straight (free roam)' : `lane error ${err >= 0 ? '+' : ''}${err.toFixed(2)}`);
  }

  doStopSign(now) {
    const left = this.p.stop_sign_wait_s - (now - this.t0);
    this.note = `STOP sign - full stop ${Math.max(0, left).toFixed(1)} s`;
    if (left <= 0) { this.stopSignCooldown = now + 10; this.to('DRIVE', 'stop complete, proceeding'); }
  }

  doYield(now) {
    const person = this.nearest('person');
    const close = person != null && person < this.p.pedestrian_caution_m;
    if (close) this.t0 = now; // the clear-for-1.5 s timer restarts while someone is close
    const clearFor = now - this.t0;
    this.note = close ? `yielding to pedestrian ${person.toFixed(1)} m` : `road clear ${clearFor.toFixed(1)}/1.5 s`;
    if (clearFor > 1.5) this.to('DRIVE', 'pedestrian has crossed');
  }

  doScan(now) {
    const p = this.p;
    if (this.range > p.warn_m + this.backedM()) { this.nudges = 0; return this.to('DRIVE', 'path cleared'); }
    if (now - this.t0 < p.scan_dwell_s) { this.note = `looking ${this.scan[0][1]}`; return; }
    const [, label] = this.scan.shift();
    if (label !== 'centre') {
      this.scanResult[label] = this.sideIsClear(label);
      const [c, f, seen] = this.scanResult[label];
      this.log(`scan ${label}: clear=${c} colour=${f.toFixed(3)} yolo=${JSON.stringify(seen)}`);
    }
    if (this.scan.length) { this.servo = this.scan[0][0]; this.t0 = now; return; }
    const left = this.scanResult.left[0], right = this.scanResult.right[0];
    const side = left ? 1 : right && p.allow_right_overtake ? -1 : 0;
    if (side === 0 && this.nudges < p.max_nudges) { // backing up widens the view past the obstacle
      this.nudges++;
      this.plan = [['reverse', p.nudge_reverse_m, this.pose[2]]];
      this.stepStart = null;
      return this.to('NUDGE', `both sides blocked - backing up (${this.nudges}/${p.max_nudges})`);
    }
    if (side === 0) return this.to('WAIT', 'both sides blocked');
    this.plan = overtakePlan(this.pose[2], side, p.overtake_shift_m, 0.7, p.overtake_pass_m);
    this.passHalf = null; this.stepStart = null;
    this.to('OVERTAKE', side > 0 ? 'left lane free' : 'right side free');
  }

  startScan(note) {
    this.scan = [[this.p.scan_pan_rad, 'left'], [-this.p.scan_pan_rad, 'right'], [0, 'centre']];
    this.scanResult = {};
    this.servo = this.p.scan_pan_rad;
    this.to('SCAN', note);
  }

  doWait(now) {
    if (this.range > this.p.decide_distance_m + 0.2 + this.backedM()) { this.nudges = 0; this.to('DRIVE', 'path cleared'); }
    else if (now - this.t0 > 3) this.startScan('rescan');
  }

  doNudge(now) { this.doOvertake(now); } // same step executor; the end of the plan triggers a rescan

  doOvertake(now) {
    const p = this.p;
    if (!this.plan.length) {
      if (this.state === 'NUDGE') return this.startScan('rescan after backing up');
      this.laneSeenT = now; this.nudges = 0;
      return this.to('DRIVE', 'back in lane');
    }
    const step = this.plan[0], [x, y, yaw] = this.pose;
    this.stepStart ??= [x, y];
    if (step[0] === 'turn') {
      const err = wrap(step[1] - yaw);
      if (Math.abs(err) < 0.04) { this.plan.shift(); this.stepStart = null; return; }
      this.w = Math.sign(err) * Math.max(0.25, Math.min(p.max_turn_rate, 2 * Math.abs(err)));
      this.note = `turning ${((err * 180) / Math.PI).toFixed(0)} deg`;
      return;
    }
    const done = Math.hypot(x - this.stepStart[0], y - this.stepStart[1]);
    if (done >= step[1]) { this.plan.shift(); this.stepStart = null; return; }
    this.v = step[0] === 'reverse' ? -p.slow_speed : p.cruise_speed;
    this.w = Math.max(-p.max_turn_rate, Math.min(p.max_turn_rate, 2 * wrap(step[2] - yaw)));
    this.note = step[0] === 'reverse' ? `backing up ${done.toFixed(2)}/${step[1].toFixed(2)} m` : `overtaking ${done.toFixed(2)}/${step[1].toFixed(2)} m`;
    if (step[0] === 'pass' && this.img && Math.abs(this.servo) < 0.05) {
      // In the left lane the markings are mirrored: flip the image and follow the overtaking lane.
      const [c, , , half] = findLane(flipX(this.img), this.passHalf);
      this.passHalf = half;
      if (c != null) {
        const e = laneError(this.img.w - c, this.img.w);
        this.w = Math.max(-p.max_turn_rate, Math.min(p.max_turn_rate, -p.steer_kp * e));
        this.note = `overtaking in left lane ${done.toFixed(2)}/${step[1].toFixed(2)} m`;
      }
    }
  }

  doStation(now) {
    const left = this.p.station_wait_s - (now - this.t0);
    this.note = `station stop ${Math.max(0, left).toFixed(0)} s`;
    if (left > 0) return;
    this.laps++;
    if (this.p.target_laps && this.laps >= this.p.target_laps) return this.to('ARRIVED', `${this.laps} laps done`);
    this.stationCooldown = now + 20;
    this.to('DRIVE', `lap ${this.laps}`);
  }

  doHold() {
    this.servo = 0;
    this.note = 'holding - press GO on the dashboard';
    if (this.goFlag) { this.goFlag = false; this.to('DRIVE', 'go'); }
  }

  doArrived() { this.note = `mission complete: ${this.laps} laps`; }
}
