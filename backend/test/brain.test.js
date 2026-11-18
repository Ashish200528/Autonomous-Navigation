import { test } from 'node:test';
import assert from 'node:assert/strict';
import { loadConfig } from '../src/config.js';
import { Brain } from '../src/brain/state-machine.js';

const cfg = loadConfig({ local: false });
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
  fresh(b, 1, 0.15);
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

test('no lane lines: free roam drives straight slowly (obstacle rules still on); free_roam off stops', async () => {
  const floor = { w: 160, h: 120, data: new Uint8Array(160 * 120 * 3).fill(120) }; // plain grey floor
  const run = (p, range = Infinity) => {
    const b = new Brain(p, cfg.camera);
    b.onImage(floor, 0); b.onRange(range, 0); b.onPose({ x: 0, y: 0, yaw: 0 }); b.onVision({ objects: [], aruco: [] }, 0);
    b.tick(0); b.onImage(floor, 1.5); b.onRange(range, 1.5); b.onVision({ objects: [], aruco: [] }, 1.5);
    return b.tick(1.5);
  };
  const roam = run({ ...P, free_roam: true });
  assert.equal(roam.state, 'DRIVE'); assert.equal(roam.v, P.slow_speed); assert.ok(Math.abs(roam.w) < 1e-9); assert.match(roam.note, /free roam/);
  assert.equal(run({ ...P, free_roam: false }).v, 0);
  assert.equal(run({ ...P, free_roam: true }, 0.5).state, 'SCAN'); // something 0.5 m ahead: look for a way past

  const { VoxelMap, KIND } = await import('../src/fusion/map3d.js');
  const m = new VoxelMap(cfg), pose = { x: 0, y: 0, yaw: 0 };
  m.addRobotPoint(0.6, 0.05, 0.1, pose, 0, KIND.CAMERA);   // depth point straight ahead
  m.addRobotPoint(0.3, 0.4, 0.1, pose, 0, KIND.CAMERA);    // off to the side
  m.addRobotPoint(0.2, 0, 0.1, pose, 0, KIND.IR);          // other kinds are not "camera ahead"
  assert.ok(Math.abs(m.nearestAhead(pose, 12, [KIND.CAMERA]) - 0.6) < 0.06);
});
