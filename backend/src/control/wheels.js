// Body command (v m/s, w rad/s) -> per-wheel PWM with a speed PI loop on the Pi.
// Speed is measured from the encoder pulse *period* (only ~37 pulses/s at top speed).

export class Wheels {
  constructor(robot) {
    this.r = robot;
    this.target = [0, 0]; // ramped wheel targets, m/s
    this.integ = [0, 0];
    this.pwm = [0, 0];
  }

  measured(telem) {
    return telem.periodUs.map((p, i) =>
      p ? Math.sign(this.pwm[i] || 1) * (this.r.mm_per_pulse[i] / 1000) / (p / 1e6) : 0);
  }

  update(v, w, telem, dt) {
    const { track_m, max_speed_mps: vmax, accel_mps2, deadband_pwm, kp, ki } = this.r;
    let want = [v - (w * track_m) / 2, v + (w * track_m) / 2];
    const peak = Math.max(Math.abs(want[0]), Math.abs(want[1]));
    if (peak > vmax) want = want.map((x) => (x * vmax) / peak); // keep the turn ratio
    const meas = telem ? this.measured(telem) : [0, 0];
    for (let i = 0; i < 2; i++) {
      const step = accel_mps2 * dt;
      this.target[i] += Math.max(-step, Math.min(step, want[i] - this.target[i]));
      if (want[i] === 0 && Math.abs(this.target[i]) < step) this.target[i] = 0;
      const t = this.target[i];
      if (t === 0) { this.pwm[i] = 0; this.integ[i] = 0; continue; }
      const err = t - meas[i];
      this.integ[i] = Math.max(-100 / ki, Math.min(100 / ki, this.integ[i] + err * dt));
      const ff = Math.sign(t) * (deadband_pwm[i] + (Math.abs(t) / vmax) * (255 - deadband_pwm[i]));
      let pwm = Math.max(-255, Math.min(255, ff + kp * err + ki * this.integ[i]));
      if (Math.sign(pwm) !== Math.sign(t)) pwm = 0; // slow down by coasting, never by reversing
      this.pwm[i] = Math.round(pwm);
    }
    return this.pwm;
  }

  reset() { this.target = [0, 0]; this.integ = [0, 0]; this.pwm = [0, 0]; }
}
