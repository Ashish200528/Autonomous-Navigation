import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as M from '../src/perception/mono_distance.js';
import * as L from '../src/perception/lanes.js';
import { decode, nms, toFrame } from '../src/perception/yolo_post.js';
import { letterbox, yuvToRgb } from '../src/perception/preprocess.js';
import { Tracker } from '../src/perception/tracker.js';
import { DistanceEstimator } from '../src/fusion/distance.js';
import { loadConfig } from '../src/config.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);

// --- object_detection_new_babu.py port (same numbers as the Python) ---
test('focal from 90 deg HFOV at 640 px = 320 px', () => near(M.focalFromFov(640, 90), 320, 1e-9));
test('known-height distance: person 1.55 m, 155 px, f 320 -> 3.2 m', () =>
  near(M.knownHeightDistance(155, 'person', 320, { person: 1.55 }, 0.5), 3.2, 1e-9));
test('unknown class uses 0.50 m default', () => near(M.knownHeightDistance(100, 'cone', 320, {}, 0.5), 1.6, 1e-9));
test('no distance without focal or height', () => { assert.equal(M.knownHeightDistance(0, 'person', 320, {}, 0.5), null); });
test('calibrate inverts the formula', () => near(M.calibrateFocal(155, 3.2, 1.55), 320, 1e-9));
test('truncated, zones, status like the script', () => {
  assert.ok(M.isTruncated(2, 100, 480)); assert.ok(M.isTruncated(10, 478, 480)); assert.ok(!M.isTruncated(10, 470, 480));
  assert.equal(M.horizontalZone(100, 640), 'Left'); assert.equal(M.horizontalZone(320, 640), 'Center'); assert.equal(M.horizontalZone(500, 640), 'Right');
  assert.equal(M.obstacleStatus(0.3, 0.3, 1.2), 'FLAGGED'); assert.equal(M.obstacleStatus(1.2, 0.3, 1.2), 'WARNING');
  assert.equal(M.obstacleStatus(1.3, 0.3, 1.2), 'SAFE'); assert.equal(M.obstacleStatus(null, 0.3, 1.2), 'UNKNOWN');
});

// --- lanes: same cases as anc_autodrive/test/test_logic.py (colours converted BGR -> RGB) ---
const ASPHALT = [66, 62, 62], WHITE = [235, 235, 235], YELLOW = [235, 200, 20], ORANGE = [240, 90, 10];
function road(yx = null, wx = null, w = 320, h = 240) {
  const img = { w, h, data: new Uint8Array(w * h * 3) };
  paint(img, 0, h, 0, w, ASPHALT);
  if (yx != null) paint(img, 0, h, yx - 3, yx + 3, YELLOW);
  if (wx != null) paint(img, 0, h, wx - 3, wx + 3, WHITE);
  return img;
}
function paint(img, y0, y1, x0, x1, c) {
  for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) img.data.set(c, 3 * (y * img.w + x));
}
test('lanes: both lines give midpoint', () => {
  const [c, y, wx, half] = L.findLane(road(100, 260));
  near(y, 100, 2); near(wx, 258, 3); near(c, 179, 3); near(half, 79, 3);
});
test('lanes: single line uses remembered width', () => {
  near(L.findLane(road(null, 260), 80)[0], 178, 3);
  near(L.findLane(road(100, null), 80)[0], 180, 3);
});
test('lanes: zebra bars do not pull the lane', () => {
  const img = road(100, 260); paint(img, 150, 220, 120, 140, WHITE);
  const [c, , wx] = L.findLane(img, 80); near(wx, 258, 3); near(c, 179, 3);
});
test('lanes: full zebra band is a crossing', () => {
  const img = road();
  for (let x = 20; x < 320; x += 50) paint(img, 130, 230, x, Math.min(320, x + 25), WHITE);
  assert.equal(L.findLane(img, 80)[0], null);
  assert.ok(L.crossingFraction(img) > 0.5); assert.equal(L.crossingFraction(road(100, 260)), 0);
});
test('lanes: stop line ignored', () => {
  const img = road(100, 260); paint(img, 170, 185, 104, 258, WHITE);
  const [c, , wx] = L.findLane(img, 80); near(wx, 258, 3); near(c, 179, 3);
});
test('lanes: no lines, error sign, obstacle fraction', () => {
  assert.equal(L.findLane(road())[0], null);
  assert.ok(L.laneError(240, 320) > 0 && 0 > L.laneError(80, 320));
  const img = road(100, 260); assert.ok(L.obstacleFraction(img) < 0.01);
  paint(img, 150, 240, 120, 200, ORANGE); assert.ok(L.obstacleFraction(img) > 0.05);
});
test('overtake plan returns to base heading', () => {
  const plan = L.overtakePlan(0);
  assert.deepEqual(plan[0], ['turn', 0.7]); assert.deepEqual(plan.at(-1), ['turn', 0]);
  near(plan.filter((s) => s[0] === 'forward').reduce((a, s) => a + s[1] * Math.sin(s[2]), 0), 0, 1e-9);
});

// --- YOLO post-processing ---
test('decode channels-first (v8/11) + NMS + back to frame pixels', () => {
  const N = 3, C = 6; // 2 classes
  const d = new Float32Array(C * N);
  const set = (i, cx, cy, w, h, s0, s1) => { [cx, cy, w, h, s0, s1].forEach((v, c) => { d[c * N + i] = v; }); };
  set(0, 160, 128, 40, 80, 0.9, 0.1);
  set(1, 162, 128, 40, 80, 0.8, 0.1); // overlaps 0, same class -> suppressed
  set(2, 60, 60, 20, 20, 0.1, 0.3);   // below conf
  const dets = nms(decode(d, [1, C, N], 0.45), 0.5, 30);
  assert.equal(dets.length, 1); assert.equal(dets[0].cls, 0);
  toFrame(dets, { scale: 0.5, padX: 0, padY: 8 }, 640, 480);
  assert.deepEqual(dets[0].box, [280, 160, 360, 320]);
});
test('decode end-to-end (yolo26) rows', () => {
  const d = new Float32Array([10, 20, 30, 40, 0.7, 2, 0, 0, 1, 1, 0.2, 1]);
  const dets = decode(d, [1, 2, 6], 0.45);
  assert.equal(dets.length, 1); assert.equal(dets[0].cls, 2); assert.deepEqual(dets[0].box, [10, 20, 30, 40]);
});

// --- preprocessing ---
test('letterbox 640x480 -> 320x256: 8 px pad, grey stays grey', () => {
  const W = 640, H = 480, yuv = new Uint8Array((W * H * 3) / 2).fill(128);
  const out = new Float32Array(3 * 320 * 256);
  const lb = letterbox(yuv, W, H, 320, 256, out);
  assert.deepEqual(lb, { scale: 0.5, padX: 0, padY: 8 });
  near(out[0], 114 / 255, 1e-6);           // top pad
  near(out[100 * 320 + 100], 130 / 255, 2 / 255); // Y=128 limited range -> ~130
  assert.equal(yuvToRgb(yuv, W, H, 160, 120).data.length, 160 * 120 * 3);
});

// --- tracker + distance fusion + self-calibration ---
test('tracker keeps the id across frames', () => {
  const tr = new Tracker();
  const a = tr.update([{ name: 'bottle', score: 0.9, box: [100, 100, 140, 200] }], 0)[0].track.id;
  const b = tr.update([{ name: 'bottle', score: 0.9, box: [104, 102, 144, 202] }], 100)[0].track.id;
  assert.equal(a, b);
});
test('distance: floor contact + known height fuse; Sharp self-calibrates the class height', () => {
  const cfg = loadConfig({ local: false });
  cfg.camera.focal_px = 320; cfg.camera.tilt_deg = 0; cfg.camera.height_m = 0.18;
  const est = new DistanceEstimator(cfg), tr = new Tracker();
  // bottle really 0.20 m tall (table says 0.25) standing 0.6 m away (inside Sharp range):
  // h_px = 0.20*320/0.6 = 107, bottom row = 240 + 320*0.18/0.6 = 336
  const det = () => tr.update([{ name: 'bottle', score: 0.9, box: [300, 229, 340, 336] }], 0)[0];
  const first = est.estimate(det(), 0, Infinity);
  near(first.cues.B, 0.748, 0.01);     // table height over-estimates
  near(first.cues.A, 0.6, 0.01);       // floor contact is size-independent
  assert.ok(first.dist > 0.6 && first.dist < 0.748);
  for (let i = 0; i < 10; i++) est.estimate(det(), 0, 0.6); // Sharp sees it at 0.6 m
  near(est.heightCorr.bottle, 0.8, 0.02);
  near(est.estimate(det(), 0, Infinity).cues.B, 0.6, 0.02);
});
