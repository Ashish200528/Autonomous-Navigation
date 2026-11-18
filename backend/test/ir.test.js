// IR proximity ring (ATmega2560 sensors 1-5 + ATmega8 sensors 6-8), IR drive, odometry distance/path.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as P from '../src/protocol.js';
import { IrRing, irM } from '../src/hw/sensors.js';
import { IrDriver } from '../src/brain/ir-drive.js';
import { arbitrate } from '../src/safety/arbiter.js';
import { Odometry } from '../src/fusion/odometry.js';

const cfg = JSON.parse(readFileSync(new URL('../config/default.json', import.meta.url)));
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`);
const CAL = [[80, 0.02], [280, 0.07], [480, 0.15]];

test('irM interpolates, clamps the near end, and reads past the far end as clear', () => {
  near(irM(180, CAL), 0.045);
  assert.equal(irM(10, CAL), 0.02);
  assert.equal(irM(600, CAL), Infinity);
  const rising = CAL.map(([a, m]) => [1023 - a, m]); // same curve if the ADC ran the other way
  near(irM(1023 - 180, rising), 0.045);
  assert.equal(irM(100, rising), Infinity);
});

test('telemetry v2 (79 bytes) carries rear lit + all 8 ambient readings; v1 (57) decodes with nulls', () => {
  const b = Buffer.alloc(79);
  b.writeUInt16LE(70, 57); b.writeUInt16LE(255, 59); b.writeUInt16LE(20, 61);
  for (let i = 0; i < 8; i++) b.writeUInt16LE(100 + i, 63 + 2 * i);
  const t = P.decodeTelem(b);
  assert.deepEqual(t.proxRear, [70, 255, 20]); assert.deepEqual(t.proxDark, [100, 101, 102, 103, 104, 105, 106, 107]);
  const [f] = new P.Parser().push(P.encode(P.T.TELEM, b)); // fits MAX_PAYLOAD (96)
  assert.equal(f.payload.length, 79);
  const v1 = P.decodeTelem(Buffer.alloc(57));
  assert.equal(v1.proxRear, null); assert.equal(v1.proxDark, null);
  const on = Buffer.alloc(79); on.writeUInt16LE(0xffff, 63); // v3 with the LEDs kept on: no ambient
  assert.equal(P.decodeTelem(on).proxDark, null);
});

// frame: lit 1-5 (10-bit), rear lit 6-8 (8-bit), dark 1-8 (6-8 8-bit) or null
const frame = (lit, dark = null) => ({ prox: lit.slice(0, 5), proxRear: lit.slice(5), proxDark: dark });
const DROP = [[400, 0.02], [200, 0.05], [40, 0.12]]; // drop below the no-obstacle level: large = near
const ring0 = (over) => new IrRing({ ...cfg.ir, cal: DROP, cal_by_sensor: {}, baseline: {}, ...over });

test('IrRing reflection: signal is ambient - lit, so a room-light swing does not move the distance', () => {
  const r = ring0({ signal: 'reflection' });
  let ring;
  for (const amb of [300, 600, 450, 650, 350]) // ambient swings by hundreds, our own reflection stays 200
    ring = r.update(frame([amb - 200, 600, 600, 600, 600, 150, 150, 150], [amb, 600, 600, 600, 600, 150, 150, 150]));
  assert.equal(ring[0].kind, 'reflection');
  assert.equal(ring[0].sig, 200); near(ring[0].m, 0.05); assert.equal(ring[0].st, 'dist');
  assert.equal(ring[1].st, 'clear'); // lit == dark: no reflection
  assert.equal(ring[5].mcu, 'ATmega8'); assert.equal(ring[5].lit, 600); // 8-bit x4
});

test('IrRing raw (LEDs on, NEX style): drop below the zeroed baseline; falls back to raw without ambient', () => {
  const r = ring0({ signal: 'reflection', baseline_adc: 675, baseline: { 2: 535 } });
  const ring = r.update(frame([475, 335, 675, 675, 675, 168, 168, 168])); // no proxDark -> raw
  assert.equal(ring[0].kind, 'raw');
  assert.equal(ring[0].sig, 200); near(ring[0].m, 0.05);          // 675 - 475
  assert.equal(ring[1].sig, 200);                                   // its own zero: 535 - 335
  assert.equal(ring[2].st, 'clear'); assert.equal(ring[5].st, 'clear'); // 168 x4 = 672
});

test('IrRing: median rejects single-frame spikes; rear 0 (J4 open) and no rear are no data, not obstacles', () => {
  const r = ring0({ signal: 'raw' });
  let ring;
  for (const l of [475, 475, 0, 475, 475]) ring = r.update(frame([l, 675, 675, 675, 675, 0, 0, 0]));
  assert.equal(ring[0].sig, 200);
  assert.equal(ring[5].st, 'nodata'); assert.equal(ring[5].m, null);
  const v1 = ring0({ signal: 'raw' }).update({ prox: [675, 675, 675, 675, 675], proxRear: null, proxDark: null });
  assert.equal(v1[7].st, 'nodata');
});

test('IrRing: zero and per-sensor calibration from live readings override the defaults', () => {
  const ir = { ...cfg.ir, signal: 'raw', baseline_adc: 675, cal: DROP, cal_by_sensor: {}, baseline: {} }, r = new IrRing(ir);
  const feed = (lit) => { for (let k = 0; k < ir.history; k++) r.update(frame([lit, 675, 675, 675, 675, 168, 168, 168])); };
  assert.ok(new IrRing(ir).record(1, 0.03).error, 'needs a full history first');
  feed(640); ir.baseline[1] = r.zero(1).value;                   // nothing near: this sensor sits at 640
  assert.equal(ir.baseline[1], 640);
  feed(140); ir.cal_by_sensor[1] = r.record(1, 0.03);            // drop 500 at 3 cm
  feed(440); ir.cal_by_sensor[1] = r.record(1, 0.08);            // drop 200 at 8 cm
  assert.deepEqual(ir.cal_by_sensor[1], { kind: 'raw', pts: [[500, 0.03], [200, 0.08]] });
  feed(290);
  const ring = r.update(frame([290, 675, 675, 675, 675, 168, 168, 168]));
  near(ring[0].m, 0.055); assert.equal(ring[0].own, true);
  assert.equal(ring[1].own, false);
});

// ring from 8 distances (metres, Infinity = clear, null = no data)
const ringOf = (ms) => ms.map((m, i) => ({ n: i + 1, m }));
const CLEAR = Array(8).fill(Infinity);
const withM = (pairs) => { const a = [...CLEAR]; for (const [n, m] of pairs) a[n - 1] = m; return ringOf(a); };

test('IR drive: cruises when clear, turns toward the open side, reverses then turns when boxed ahead', () => {
  const d = new IrDriver(cfg.ir_drive), p = cfg.ir_drive;
  let c = d.tick(ringOf(CLEAR), Infinity, 0);
  assert.equal(c.state, 'CRUISE'); assert.equal(c.v, p.cruise_speed); assert.equal(c.w, 0);

  c = d.tick(withM([[3, 0.05], [4, 0.08], [5, 0.06]]), 0.2, 1); // blocked ahead, right side near
  assert.equal(c.state, 'TURN'); assert.equal(c.v, 0); assert.ok(c.w > 0, 'turns left, away from the right');
  c = d.tick(withM([[5, 0.06]]), Infinity, 1.5); // front opened
  assert.equal(c.state, 'CRUISE');

  d.reset();
  c = d.tick(withM([[1, 0.05], [2, 0.05], [3, 0.04], [4, 0.05], [5, 0.05]]), 0.15, 0);
  assert.equal(c.state, 'REVERSE'); assert.ok(c.v < 0);
  c = d.tick(withM([[1, 0.05], [2, 0.05], [3, 0.04], [4, 0.05], [5, 0.05], [7, 0.03]]), 0.15, 0.1); // rear hit too
  assert.equal(c.state, 'BOXED'); assert.equal(c.v, 0);
});

test('IR drive refuses to move when the front sensors have no distance', () => {
  const c = new IrDriver(cfg.ir_drive).tick(withM([[3, null]]), Infinity, 0);
  assert.equal(c.v, 0); assert.equal(c.w, 0); assert.match(c.note, /no distance/);
});

test('IR drive: side wall seen by sensor 1 steers right while cruising', () => {
  const c = new IrDriver(cfg.ir_drive).tick(withM([[1, 0.05]]), Infinity, 0);
  assert.equal(c.state, 'CRUISE'); assert.ok(c.v > 0 && c.w < 0);
});

test('arbiter: IR mode needs no camera; reversing stops on the rear IR', () => {
  const s = { mode: 'ir', ir: { v: 0.1, w: 0, note: 'cruising' }, telemAgeMs: 10, frameAgeMs: 1e9, yoloAgeMs: 1e9,
    mcuFlags: 0, rangeM: Infinity, rearM: Infinity, temp: 40, throttled: false };
  assert.equal(arbitrate(s, cfg).v, 0.1);
  assert.equal(arbitrate({ ...s, ir: { v: -0.06, w: 0, note: '' }, rearM: 0.03 }, cfg).v, 0);
  assert.equal(arbitrate({ ...s, ir: { v: -0.06, w: 0, note: '' }, rearM: Infinity }, cfg).v, -0.06);
});

test('odometry: distance counts forward and back; path gets a point every 2 cm', () => {
  const o = new Odometry(cfg.robot), n = Math.round(1000 / 5.44); // ~1 m of pulses
  o.update({ tMs: 0, enc: [0, 0] });
  o.update({ tMs: 1000, enc: [n, n] });
  o.update({ tMs: 2000, enc: [0, 0] }); // and back
  assert.ok(Math.abs(o.dist - 2) < 0.01, `dist ${o.dist}`);
  assert.ok(Math.abs(o.pose.x) < 1e-9);
  assert.ok(o.trail.length >= 2);
  const r = o.resets; o.reset();
  assert.equal(o.dist, 0); assert.equal(o.trail.length, 1); assert.equal(o.resets, r + 1);
});

test('ir_check diagnoses: J4 open, ATmega8 silent, working rear, IRP jumper in, old firmware, no data', async () => {
  const { diagnose } = await import('../tools/ir_check.mjs');
  const f = (rear, k, dark) => ({ fw: 3, prox: [600 - k, 600, 600 + k, 600, 600], proxRear: rear, proxDark: dark });
  const run = (mk) => diagnose(Array.from({ length: 20 }, (_, k) => mk(k))).join('\n');
  assert.match(run((k) => f([0, 0, 0], k)), /rear-right: always 0 .*J4/);
  assert.match(run((k) => f([255, 255, 255], k)), /rear: always 255 .*ATMEGA8\.hex/);
  const ok = run((k) => f([168 - 3 * k, 168, 168], k, [600 - k, 600, 600 + k, 600, 600, 168, 168, 168]));
  assert.match(ok, /rear-right: OK, reacts/); assert.match(ok, /rear: stuck at 168/); assert.match(ok, /IRP jumper/);
  assert.match(ok, /1 left: OK, reacts/);
  assert.match(run((k) => ({ ...f(null, k), fw: 1 })), /v1/);
  assert.match(diagnose([]).join(), /NO DATA/);
});
