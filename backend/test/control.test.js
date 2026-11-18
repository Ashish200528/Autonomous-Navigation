import { test } from 'node:test';
import assert from 'node:assert/strict';
import { loadConfig } from '../src/config.js';
import { Wheels } from '../src/control/wheels.js';
import { Odometry } from '../src/fusion/odometry.js';
import { FrontRange, sharpAdcFor, sharpM } from '../src/hw/sensors.js';
import { arbitrate } from '../src/safety/arbiter.js';

const cfg = loadConfig({ local: false });
const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);
const telem = (o = {}) => ({ tMs: 0, enc: [0, 0], periodUs: [0, 0], sharp: [0, 0, 0, 0, 0], prox: [1023, 1023, 1023, 1023, 1023], flags: 0, ...o });

test('odometry: 2 m straight and a 360 deg spin', () => {
  const o = new Odometry(cfg.robot), n = Math.round(2000 / 5.44);
  o.update(telem());
  const p = o.update(telem({ tMs: 1000, enc: [n, n] }));
  near(p.x, 2, 0.01); near(p.y, 0, 1e-9); near(p.yaw, 0, 1e-9);
  const o2 = new Odometry(cfg.robot), k = Math.round((Math.PI * 0.15) / 2 / 0.00544); // half circle each wheel
  o2.update(telem());
  for (let i = 1; i <= 10; i++) o2.update(telem({ tMs: i * 100, enc: [-Math.round((k * i) / 10), Math.round((k * i) / 10)] }));
  near(Math.abs(o2.pose.yaw), Math.PI, 0.1);
  near(Math.hypot(o2.pose.x, o2.pose.y), 0, 0.01);
});

test('wheels: ramps, forward pwm above deadband, turn sign, zero stops at once', () => {
  const w = new Wheels(cfg.robot);
  let pwm;
  for (let i = 0; i < 20; i++) pwm = w.update(0.15, 0, telem(), 0.05);
  assert.ok(pwm[0] > 60 && pwm[1] > 60);
  w.reset();
  for (let i = 0; i < 20; i++) pwm = w.update(0, 1.0, telem(), 0.05);
  assert.ok(pwm[0] < 0 && pwm[1] > 0); // spin left
  w.reset();
  assert.deepEqual(w.update(0, 0, telem(), 0.05), [0, 0]);
});

test('sharp formula round trip and blind-zone hold', () => {
  near(sharpM(sharpAdcFor(0.3)), 0.3, 0.01);
  assert.equal(sharpM(5), Infinity);
  const f = new FrontRange(cfg.range);
  const at = (m) => telem({ sharp: [0, 0, sharpAdcFor(m), 0, 0] });
  near(f.update(at(0.15), 0, true).m, 0.15, 0.01);
  const r = f.update(at(0.7), 50, true); // near -> far jump while moving forward
  assert.ok(r.blindHold && r.m <= cfg.range.sharp_min_m + 1e-9);
  assert.ok(f.update(at(0.7), 1200, true).m > 0.6); // hold expires
});

test('IR proximity overrides the Sharp', () => {
  const f = new FrontRange({ ...cfg.range, prox_front_mask: 0b00100 });
  const r = f.update(telem({ prox: [1023, 1023, 100, 1023, 1023] }), 0, true);
  assert.ok(r.proxHit && r.m === cfg.range.prox_hit_m);
});

test('arbiter priorities', () => {
  const ok = { mode: 'auto', brain: { v: 0.15, w: 0.1 }, manual: { v: 0, w: 0 }, manualAgeMs: 0, telemAgeMs: 20,
    frameAgeMs: 50, yoloAgeMs: 100, mcuFlags: 0, rangeM: Infinity, temp: 50, throttled: false };
  assert.equal(arbitrate(ok, cfg).v, 0.15);
  assert.equal(arbitrate({ ...ok, mode: 'stop' }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, telemAgeMs: 500 }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, yoloAgeMs: 5000 }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, mcuFlags: 4 }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, temp: 85 }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, rangeM: 0.2 }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, rangeM: 0.2, brain: { v: -0.1, w: 0 } }, cfg).v, -0.1); // backing away is allowed
  assert.ok(arbitrate({ ...ok, rangeM: 0.32 }, cfg).v < 0.15);                            // braking-distance cap
  assert.equal(arbitrate({ ...ok, mode: 'manual', manual: { v: 0.1, w: 0 }, manualAgeMs: 900 }, cfg).v, 0);
  assert.equal(arbitrate({ ...ok, mode: 'manual', manual: { v: 0.1, w: 0 }, yoloAgeMs: 1e9 }, cfg).v, 0.1); // manual needs no YOLO
});
