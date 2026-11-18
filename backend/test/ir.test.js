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

test('IrRing raw: each sensor learns its own zero, so the robot measured clear readings read clear (was ~8 cm)', () => {
  const clear = [647, 535, 627, 463, 580, 168, 168, 168]; // measured on the robot with nothing near
  const r = ring0({ signal: 'raw' });
  let ring;
  for (let k = 0; k < 25; k++) ring = r.update(frame(clear));
  assert.ok(ring.every((x) => x.st === 'clear'), ring.map((x) => x.st).join());
  for (let k = 0; k < 5; k++) ring = r.update(frame([647, 535, 327, 463, 580, 168, 168, 168])); // hand in front of 3
  assert.equal(ring[2].st, 'dist'); near(ring[2].sig, 300 - 0.1 * 3); assert.ok(ring[2].m < 0.045);
  assert.equal(ring[0].st, 'clear'); assert.equal(ring[1].st, 'clear');
  for (let k = 0; k < 5; k++) ring = r.update(frame(clear)); // hand gone: clear again at once
  assert.equal(ring[2].st, 'clear');
});

test('IrRing raw: zero follows room light (sinks zero_decay per frame), rises at once; reflection needs ambient', () => {
  const r = ring0({ signal: 'reflection', zero_decay: 1 });
  let ring;
  for (let k = 0; k < 5; k++) ring = r.update(frame([600, 600, 600, 600, 600, 150, 150, 150])); // no proxDark -> raw
  assert.equal(ring[0].kind, 'raw');
  for (let k = 0; k < 50; k++) ring = r.update(frame([560, 600, 600, 600, 600, 150, 150, 150])); // room got brighter
  assert.equal(ring[0].st, 'clear'); assert.equal(ring[0].sig, 0); // the zero sank to the new level
  const ir = { ...cfg.ir, signal: 'raw', baseline: { 1: 650 }, cal: DROP, cal_by_sensor: {} };
  ring = new IrRing(ir).update(frame([450, 600, 600, 600, 600, 150, 150, 150]));
  near(ring[0].sig, 200 - cfg.ir.zero_decay); // a saved zero is the starting point
});

test('IrRing: median rejects single-frame spikes; rear 0 (J4 open) and no rear are no data, not obstacles', () => {
  const r = ring0({ signal: 'raw' });
  let ring;
  for (const l of [475, 475, 0, 475, 475]) ring = r.update(frame([l, 675, 675, 675, 675, 0, 0, 0]));
  assert.equal(ring[0].sig, 0); assert.equal(ring[0].st, 'clear');
  assert.equal(ring[5].st, 'nodata'); assert.equal(ring[5].m, null);
  const v1 = ring0({ signal: 'raw' }).update({ prox: [675, 675, 675, 675, 675], proxRear: null, proxDark: null });
  assert.equal(v1[7].st, 'nodata');
});

test('IrRing: zero and per-sensor calibration from live readings override the defaults', () => {
  const ir = { ...cfg.ir, signal: 'raw', zero_decay: 0, cal: DROP, cal_by_sensor: {}, baseline: {} }, r = new IrRing(ir);
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
const P_ = cfg.ir_drive; // block_m 0.06 = red

test('IR drive: clear -> full speed; amber (seen, not red) ahead -> keeps going slowly instead of spinning', () => {
  const d = new IrDriver(P_);
  let c = d.tick(ringOf(CLEAR), Infinity, 0);
  assert.equal(c.state, 'CRUISE'); assert.equal(c.v, P_.cruise_speed); assert.equal(c.w, 0);
  c = d.tick(withM([[3, 0.08]]), Infinity, 0.1); // 8 cm ahead: amber
  assert.equal(c.state, 'CRUISE'); assert.equal(c.v, P_.slow_speed);
});

test('IR drive: red ahead -> turns toward the side that is not red, drives on once the front stays clear', () => {
  const d = new IrDriver(P_);
  let c = d.tick(withM([[3, 0.04], [4, 0.05], [5, 0.05]]), Infinity, 1); // red ahead + right
  assert.equal(c.state, 'TURN'); assert.equal(c.v, 0); assert.ok(c.w > 0, 'turns left, away from the red right');
  c = d.tick(ringOf(CLEAR), Infinity, 1.1);
  assert.equal(c.state, 'TURN', 'waits free_s before driving'); assert.ok(c.w > 0);
  c = d.tick(ringOf(CLEAR), Infinity, 1.1 + P_.free_s);
  assert.equal(c.state, 'CRUISE');
  c = d.tick(withM([[2, 0.03]]), Infinity, 2); // red front-left -> turn right
  assert.equal(c.state, 'TURN'); assert.ok(c.w < 0);
});

test('IR drive: red ahead and both sides -> back up; rear red too -> boxed; a turn that never clears -> back up', () => {
  const allFront = [[1, 0.05], [2, 0.05], [3, 0.04], [4, 0.05], [5, 0.05]];
  const d = new IrDriver(P_);
  let c = d.tick(withM(allFront), Infinity, 0);
  assert.equal(c.state, 'REVERSE'); assert.ok(c.v < 0);
  c = d.tick(withM([...allFront, [7, 0.03]]), Infinity, 0.1);
  assert.equal(c.state, 'BOXED'); assert.equal(c.v, 0);
  c = d.tick(withM([[3, 0.04]]), Infinity, 0.2); // a side opened
  assert.equal(c.state, 'TURN');
  c = d.tick(withM([[3, 0.04]]), Infinity, 0.2 + P_.turn_timeout_s + 0.1);
  assert.equal(c.state, 'REVERSE');
});

test('IR drive: refuses to move without front readings; steers off a side wall; Sharp (if fitted) blocks too', () => {
  assert.match(new IrDriver(P_).tick(withM([[3, null]]), Infinity, 0).note, /no reading/);
  const c = new IrDriver(P_).tick(withM([[1, 0.05]]), Infinity, 0);
  assert.equal(c.state, 'CRUISE'); assert.ok(c.v > 0 && c.w < 0);
  assert.equal(new IrDriver(P_).tick(ringOf(CLEAR), 0.2, 0).state, 'TURN');
});

test('arbiter: IR mode needs no camera, stops forward only at the IR stop distance; rear IR stops reversing', () => {
  const s = { mode: 'ir', ir: { v: 0.1, w: 0, note: 'cruising' }, telemAgeMs: 10, frameAgeMs: 1e9, yoloAgeMs: 1e9,
    mcuFlags: 0, rangeM: Infinity, irFrontM: 0.08, rearM: Infinity, temp: 40, throttled: false };
  assert.equal(arbitrate(s, cfg).v, 0.1); // 8 cm (amber) does not stop it
  assert.equal(arbitrate({ ...s, irFrontM: 0.03 }, cfg).v, 0);
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

test('IR drive closed loop: in a 1 m box it drives around for 60 s, never touches a wall, does not just spin', () => {
  // Virtual robot: 8 cm radius, IR sees 12 cm past its edge at the Fire Bird bearings; walls at 0..1 m.
  const R = 0.08, RANGE = 0.12, bear = cfg.ir.bearing_deg.map((d) => (d * Math.PI) / 180);
  const wallDist = (x, y, a) => { // distance from (x, y) along heading a to the box wall
    const dx = Math.cos(a), dy = Math.sin(a), t = [];
    if (dx > 1e-9) t.push((1 - x) / dx); if (dx < -1e-9) t.push(-x / dx);
    if (dy > 1e-9) t.push((1 - y) / dy); if (dy < -1e-9) t.push(-y / dy);
    return Math.min(...t);
  };
  const d = new IrDriver(P_), pose = { x: 0.5, y: 0.5, yaw: 0.3 }, dt = 0.05;
  let travelled = 0, minGap = Infinity, spinS = 0;
  for (let k = 0; k < 60 / dt; k++) {
    const ring = bear.map((b, i) => { const g = wallDist(pose.x, pose.y, pose.yaw + b) - R; return { n: i + 1, m: g < RANGE ? Math.max(0.01, g) : Infinity }; });
    const c = d.tick(ring, Infinity, k * dt);
    const cmd = arbitrate({ mode: 'ir', ir: c, telemAgeMs: 10, frameAgeMs: 1e9, yoloAgeMs: 1e9, mcuFlags: 0, rangeM: Infinity,
      irFrontM: Math.min(ring[1].m, ring[2].m, ring[3].m), rearM: Math.min(ring[5].m, ring[6].m, ring[7].m), temp: 40, throttled: false }, cfg);
    pose.yaw += cmd.w * dt; pose.x += cmd.v * dt * Math.cos(pose.yaw); pose.y += cmd.v * dt * Math.sin(pose.yaw);
    travelled += Math.abs(cmd.v) * dt;
    if (cmd.v === 0 && cmd.w !== 0) spinS += dt;
    minGap = Math.min(minGap, pose.x - R, 1 - pose.x - R, pose.y - R, 1 - pose.y - R);
  }
  assert.ok(minGap > 0.01, `came within ${(minGap * 100).toFixed(1)} cm of a wall`);
  assert.ok(travelled > 2, `only drove ${travelled.toFixed(2)} m in 60 s`);
  assert.ok(spinS < 30, `spun in place ${spinS.toFixed(0)} of 60 s`);
});
