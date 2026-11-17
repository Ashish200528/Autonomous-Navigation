// Raw ATmega ADC -> metres / volts. Pure functions, unit-tested.

// NEX Robotics formula (hardware manual §3.10, 8-bit ADC): mm = 10 * 2799.6 / adc8^1.1546.
// It matches the 10-80 cm GP2D12 curve. Phase 1 replaces it with a measured table if needed.
export const sharpM = (adc10) => (adc10 < 20 ? Infinity : 27.996 / Math.pow(adc10 / 4, 1.1546));
export const sharpAdcFor = (m) => Math.round(4 * Math.pow(27.996 / m, 1 / 1.1546));

export const battV = (mv) => mv / 1000;

// Front range from the Sharp + IR proximity sensors, with the Sharp blind-zone guard:
// below its minimum range a Sharp reads *far* again, so a near -> far jump while moving
// forward is held as "too close" instead of "clear".
export class FrontRange {
  constructor(cfg) { this.cfg = cfg; this.last = Infinity; this.holdUntil = 0; }

  update(telem, nowMs, movingForward) {
    const c = this.cfg;
    let m = sharpM(telem.sharp[c.sharp_front_index]);
    if (m < c.sharp_min_m) m = c.sharp_min_m;
    if (m > c.sharp_max_m) m = Infinity;
    if (movingForward && this.last < c.sharp_min_m + 0.1 && m > this.last + 0.3) this.holdUntil = nowMs + 1000;
    this.last = m;
    if (nowMs < this.holdUntil) m = c.sharp_min_m;
    let proxHit = false;
    for (let i = 0; i < 5; i++) if ((c.prox_front_mask >> i) & 1 && telem.prox[i] <= c.prox_hit_adc) proxHit = true;
    if (proxHit) m = Math.min(m, c.prox_hit_m);
    return { m, proxHit, blindHold: nowMs < this.holdUntil };
  }
}
