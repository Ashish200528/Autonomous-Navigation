import { test } from 'node:test';
import assert from 'node:assert/strict';
import { loadConfig } from '../src/config.js';
import { Brain } from '../src/brain/state-machine.js';

const cfg = loadConfig();
const P = { ...cfg.brain, autostart: true, flag_m: cfg.distance.flag_m, warn_m: cfg.distance.warn_m };

function roadImg(yx = 50, wx = 130, w = 160, h = 120) {
  const img = { w, h, data: new Uint8Array(w * h * 3) };
  for (let i = 0; i < w * h; i++) img.data.set([66, 62, 62], 3 * i);
  for (let y = 0; y < h; y++) for (const [x0, c] of [[yx, [235, 200, 20]], [wx, [235, 235, 235]]])
    for (let x = x0 - 2; x < x0 + 2; x++) img.data.set(c, 3 * (y * w + x));
  return img;
}
function fresh(b, t, range = Infinity, objects = []) {
  b.onImage(roadImg(), t); b.onRange(range, t); b.onPose({ x: 0, y: 0, yaw: 0 }); b.onVision({ objects, aruco: [] }, t);
}

test('stale inputs -> no motion', () => {
  const b = new Brain(P, cfg.camera);
  assert.equal(b.tick(10).v, 0);
});

test('drives in the lane when clear', () => {
  const b = new Brain(P, cfg.camera);
  fresh(b, 1);
  const o = b.tick(1);
  assert.equal(o.state, 'DRIVE'); assert.ok(o.v > 0);
});

test('obstacle inside decide distance -> SCAN with the camera panned left', () => {
  const b = new Brain(P, cfg.camera);
  fresh(b, 1, 0.6);
  const o = b.tick(1);
  assert.equal(o.state, 'SCAN'); assert.equal(o.v, 0); assert.equal(o.pan, P.scan_pan_rad);
});

test('person ahead -> YIELD; stop sign close -> STOP_SIGN', () => {
  const b = new Brain(P, cfg.camera);
  fresh(b, 1, Infinity, [{ cls: 'person', zone: 'Center', dist: 1.0, box: [280, 100, 360, 400] }]);
  assert.equal(b.tick(1).state, 'YIELD');
  const c = new Brain(P, cfg.camera);
  fresh(c, 1, Infinity, [{ cls: 'stop sign', zone: 'Right', dist: 0.8, box: [500, 100, 560, 160] }]);
  assert.equal(c.tick(1).state, 'STOP_SIGN');
});

test('emergency rule: never forward under flag distance', () => {
  const b = new Brain({ ...P, decide_distance_m: 0.1 }, cfg.camera);
  fresh(b, 1, 0.2);
  assert.equal(b.tick(1).v, 0);
});

test('HOLD waits for GO', () => {
  const b = new Brain({ ...P, autostart: false }, cfg.camera);
  fresh(b, 1);
  assert.equal(b.tick(1).state, 'HOLD');
  b.go();
  fresh(b, 2);
  b.tick(2);
  assert.equal(b.tick(2).state, 'DRIVE');
});
