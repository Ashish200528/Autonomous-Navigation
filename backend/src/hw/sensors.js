// Raw ATmega ADC -> metres / volts. Pure functions, unit-tested.

// NEX Robotics formula (hardware manual §3.10, 8-bit ADC): mm = 10 * 2799.6 / adc8^1.1546.
// It matches the 10-80 cm GP2D12 curve. Phase 1 replaces it with a measured table if needed.
export const sharpM = (adc10) => (adc10 < 20 ? Infinity : 27.996 / Math.pow(adc10 / 4, 1.1546));
export const sharpAdcFor = (m) => Math.round(4 * Math.pow(27.996 / m, 1 / 1.1546));

export const battV = (mv) => mv / 1000;

// IR proximity signal -> metres by linear interpolation in a [signal, m] table (any order; the
// signal may rise or fall with distance). Past the far end = nothing seen = Infinity.
export function irM(adc, cal) {
  const pts = [...cal].sort((a, b) => a[1] - b[1]); // near -> far
  const s = pts.at(-1)[0] > pts[0][0] ? (a) => a : (a) => -a; // orient so s(adc) grows with distance
  if (s(adc) >= s(pts.at(-1)[0])) return Infinity;
  if (s(adc) <= s(pts[0][0])) return pts[0][1];
  for (let i = 1; i < pts.length; i++) {
    if (s(adc) > s(pts[i][0])) continue;
    const [a0, m0] = pts[i - 1], [a1, m1] = pts[i];
    return m0 + ((m1 - m0) * (adc - a0)) / (a1 - a0);
  }
}

const median = (a) => { const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]; };

// The 8 IR proximity sensors: 1-5 from the ATmega2560 ADC, 6-8 from the ATmega8 over SPI
// (8-bit, scaled to 10-bit). Hardware manual §3.11: each is an IR LED + photodiode whose output
// is ~3.3 V (ADC ~675) with nothing near and FALLS as an obstacle reflects more light.
// The signal is that drop, in ADC counts, measured one of two ways (ir.signal):
//   'raw'        drop = zero - lit. Works on any firmware with the LEDs on, as NEX's examples
//                read them. Each sensor's zero ("nothing near" level) differs a lot - measured
//                on one robot: 647, 535, 627, 463, 580 - so one fixed level read 5-12 cm on every
//                sensor with nothing near. The zero is therefore learned per sensor: an obstacle
//                only ever LOWERS the reading, so the zero is the upper envelope of the readings,
//                sinking ir.zero_decay counts per frame to follow room light. "Zero" on the
//                dashboard sets it at once; it starts at the saved one or the first reading.
//   'reflection' drop = dark - lit, the robot's own IR only: room light cancels. Needs firmware
//                v3 and the IRP jumper (J1-2 on the main board) removed, else lit == dark.
// Per frame: drop -> median of the last ir.median frames -> metres through that sensor's own
// calibration (recorded on the dashboard) or the shared default table ir.cal.
// Entry st: 'dist' | 'clear' | 'nodata' (no reading: rear sensors without J4 / firmware v2+).
export class IrRing {
  constructor(ir) {
    this.ir = ir; this.kind = null; this.ring = null;
    this.hist = Array.from({ length: 8 }, () => []); // drop
    this.litHist = Array.from({ length: 8 }, () => []);
    this.zeroLvl = Array(8).fill(null); // raw mode: learned "nothing near" level per sensor
  }

  table(n, kind) {
    const own = this.ir.cal_by_sensor?.[n];
    return own && own.kind === kind && own.pts.length >= 2 ? { pts: own.pts, own: true } : { pts: this.ir.cal, own: false };
  }

  update(telem) {
    const ir = this.ir, k = ir.rear_scale, push = (h, v) => { h.push(v); if (h.length > ir.history) h.shift(); };
    const lit = [...telem.prox, ...(telem.proxRear ? telem.proxRear.map((v) => v * k) : [null, null, null])];
    const dark = telem.proxDark?.map((v, i) => (i < 5 ? v : v * k)) ?? null;
    const kind = ir.signal === 'reflection' && dark ? 'reflection' : 'raw';
    if (kind !== this.kind) this.hist = this.hist.map(() => []); // never mix the two kinds in one median
    this.kind = kind;
    return this.ring = lit.map((l, i) => {
      const n = i + 1, e = { n, deg: ir.bearing_deg[i], mcu: i < 5 ? 'ATmega2560' : 'ATmega8', lit: l, dark: dark?.[i] ?? null, kind };
      // rear 0: the SPI link to the ATmega8 is open (J4 jumpers out) - a 3.3 V sensor never reads 0
      if (l == null || (i >= 5 && l === 0)) return { ...e, sig: null, m: null, own: false, st: 'nodata' };
      push(this.litHist[i], l);
      let sig;
      if (kind === 'reflection') {
        push(this.hist[i], dark[i] - l);
        sig = median(this.hist[i].slice(-ir.median));
      } else {
        const lm = median(this.litHist[i].slice(-ir.median)); // spike-free reading
        const z = this.zeroLvl[i] = Math.max(lm, (this.zeroLvl[i] ?? ir.baseline?.[n] ?? lm) - ir.zero_decay);
        sig = z - lm;
        push(this.hist[i], sig); // record() reads it
      }
      const t = this.table(n, kind), m = irM(sig, t.pts);
      return { ...e, sig, m, own: t.own, st: m === Infinity ? 'clear' : 'dist' };
    });
  }

  // "Zero": sensor n's reading now, with nothing near, becomes its raw-mode zero at once.
  zero(n) {
    const h = this.litHist[n - 1];
    if (h.length < this.ir.history) return { error: `sensor ${n}: not enough readings yet` };
    return { value: this.zeroLvl[n - 1] = median(h) };
  }

  // Calibration point: the median drop of the last ir.history frames (~0.5 s) of sensor n is its
  // reading at distance d. Returns that sensor's new table, or an error.
  record(n, d) {
    const h = this.hist[n - 1];
    if (h.length < this.ir.history) return { error: `sensor ${n}: not enough readings yet` };
    const own = this.ir.cal_by_sensor?.[n];
    const pts = (own?.kind === this.kind ? own.pts : []).filter(([, m]) => Math.abs(m - d) > 0.003);
    pts.push([Math.round(median(h)), d]);
    return { kind: this.kind, pts: pts.sort((a, b) => a[1] - b[1]) };
  }
}

// Nearest reading among sensors (1-based numbers); no data / uncalibrated count as Infinity.
export const nearestIr = (ring, nums) => Math.min(Infinity, ...nums.map((n) => ring?.[n - 1]?.m ?? Infinity));

// Front range from the Sharp + IR proximity sensors, with the Sharp blind-zone guard:
// below its minimum range a Sharp reads *far* again, so a near -> far jump while moving
// forward is held as "too close" instead of "clear".
export class FrontRange {
  constructor(cfg) { this.cfg = cfg; this.last = Infinity; this.holdUntil = 0; }

  update(telem, nowMs, movingForward) {
    const c = this.cfg;
    // sharp_front_index null = no Sharp fitted (an unconnected ADC input floats and would invent obstacles)
    let m = c.sharp_front_index == null ? Infinity : sharpM(telem.sharp[c.sharp_front_index]);
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
