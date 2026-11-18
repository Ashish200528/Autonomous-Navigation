// Geometry, dense-depth scale fit, 3D map + virtual scan, ArUco, brain side check, record/replay.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { loadConfig } from '../src/config.js';
import { camToRobot, floorDepthAt, intrinsics, project, robotToCam } from '../src/fusion/geometry.js';
import { DepthScale, buildAnchors } from '../src/fusion/depth_scale.js';
import { KIND, VoxelMap, nearestIn } from '../src/fusion/map3d.js';
import { createAruco, markerBits } from '../src/perception/aruco.js';
import { Brain } from '../src/brain/state-machine.js';
import { Recorder } from '../src/record/recorder.js';
import { replay } from '../src/record/replay.js';

const near = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);
const cfg = loadConfig({ local: false });
cfg.camera.focal_px = 320; cfg.camera.intrinsics = undefined;

test('geometry: camera <-> robot round trip, floor pixel lands on the floor', () => {
  for (const pan of [0, 0.7, -1.2]) {
    const [X, Y, Z] = camToRobot(0.1, -0.05, 1.3, pan, cfg);
    const back = robotToCam(X, Y, Z, pan, cfg);
    [0.1, -0.05, 1.3].forEach((v, i) => near(back[i], v, 1e-9));
  }
  const { f, cx, cy } = intrinsics(cfg), v = 400, z = floorDepthAt(v, cfg);
  const [, , Zr] = camToRobot(0, ((v - cy) * z) / f, z, 0, cfg);
  near(Zr, 0, 1e-9);
  const p = project(1.0, 0, 0, 0, cfg); // floor point 1 m ahead projects below the centre
  assert.ok(p.v > cy && Math.abs(p.u - cx) < 1e-9);
});

// Synthetic MiDaS output (true 1/Z = s0*d + t0) for a physically consistent scene: floor, a wall
// 1.5 m ahead, a box standing on the floor 0.6 m ahead (bottom at row ~289 with these camera
// numbers), and a nearer object at 0.3 m whose pixels fall among the floor anchors (outliers).
function scene(n = 128, s0 = 2, t0 = 0.1) {
  const W = cfg.camera.width, H = cfg.camera.height, disp = new Float32Array(n * n);
  const box = [280, 200, 360, 288], near2 = [80, 300, 180, 376];
  const inB = (b, u, v) => u >= b[0] && u <= b[2] && v >= b[1] && v <= b[3];
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const u = ((j + 0.5) * W) / n, v = ((i + 0.5) * H) / n;
    let z = Math.min(floorDepthAt(v, cfg) ?? Infinity, 1.5);
    if (inB(box, u, v)) z = 0.6;
    if (inB(near2, u, v)) z = 0.3;
    disp[i * n + j] = (1 / z - t0) / s0;
  }
  return { disp, n, box };
}

test('depth scale: RANSAC recovers metres from relative depth; box depth correct; outliers rejected', () => {
  const { disp, n, box } = scene();
  const ds = new DepthScale(disp, n, { t: 0, pan: 0, pose: { x: 0, y: 0, yaw: 0 } }, cfg);
  const anchors = buildAnchors(cfg, { pan: 0, sharpM: Infinity, objects: [] }); // box pixels are wrong floor anchors
  const fit = ds.solve(anchors);
  assert.ok(fit.ok, JSON.stringify(fit));
  near(fit.s, 2, 0.05);
  assert.ok(fit.inliers < fit.n, 'anchors on the near object should be outliers');
  near(ds.boxDepth(box), 0.6, 0.03);             // box 0.6 m
  near(ds.depthOf(ds.dispAt(320, 100)), 1.5, 0.08); // wall 1.5 m, far outside the anchors' 0.23-0.41 m
  assert.ok(ds.validFor(0, { x: 0, y: 0, yaw: 0 }, 100) && !ds.validFor(0.5, { x: 0, y: 0, yaw: 0 }, 100));
});

test('3D map: depth back-projection + virtual scan see the wall and the box', () => {
  const { disp, n } = scene();
  const ds = new DepthScale(disp, n, { t: 0, pan: 0, pose: { x: 0, y: 0, yaw: 0 } }, cfg);
  ds.solve(buildAnchors(cfg, { pan: 0, sharpM: Infinity, objects: [] }));
  const map = new VoxelMap(cfg);
  assert.ok(map.addDepth(ds, 2) > 50);
  const scan = map.scan({ x: 0, y: 0, yaw: 0 });
  near(nearestIn(scan, -3, 3), 0.6 * Math.cos((cfg.camera.tilt_deg * Math.PI) / 180) + cfg.mount.camera_x_m, 0.1); // box
  near(nearestIn(scan, 25, 35), 0.3, 0.08);  // the near object on the left
  assert.ok(nearestIn(scan, -35, -25) > 1.3); // right side: only the wall
  map.addRobotPoint(0, 0.5, 0.1, { x: 0, y: 0, yaw: 0 }, 0, KIND.SHARP);
  near(map.scan({ x: 0, y: 0, yaw: 0 })[90], 0.5, 1e-6); // left = +90°
  near(map.scan({ x: 0, y: 0, yaw: Math.PI / 2 })[0], 0.5, 1e-6); // turned left: now straight ahead
  map.prune(cfg.map.fade_s * 1000 + 1);
  assert.equal(map.vox.size, 0);
  assert.equal(map.cloud().length, 0);
});

test('ArUco: DICT_7X7_1000 marker found with the right id and distance', () => {
  const W = 640, H = 480, Y = new Uint8Array(W * H).fill(255), bits = markerBits(cfg, 7);
  const side = 63, cell = 7, x0 = 289, y0 = 209; // 0.10 m at ~0.5 m with f = 320, centred
  for (let r = 0; r < 9; r++) for (let c = 0; c < 9; c++) {
    const white = r > 0 && r < 8 && c > 0 && c < 8 && bits[(r - 1) * 7 + (c - 1)] === '1';
    for (let y = 0; y < cell; y++) for (let x = 0; x < cell; x++) Y[(y0 + r * cell + y) * W + x0 + c * cell + x] = white ? 255 : 0;
  }
  const m = createAruco(cfg)(Y, W, H, { f: 320, cx: 320, cy: 240 });
  assert.equal(m.length, 1);
  assert.equal(m[0].id, 7);
  near(m[0].dist, (0.1 * 320) / side, 0.03);
  near(m[0].angle_deg, 0, 2);
});

test('brain: the virtual scan blocks a side that colour + YOLO think is clear', () => {
  const P = { ...cfg.brain, autostart: true, flag_m: 0.3, warn_m: 1.2, side_sector_deg: [20, 100], side_block_m: 1.0 };
  const b = new Brain(P, cfg.camera);
  b.img = { w: 160, h: 120, data: new Uint8Array(160 * 120 * 3).fill(60) };
  const scan = new Float32Array(360).fill(Infinity);
  scan[60] = 0.5; // something 0.5 m away on the left
  b.onScan(scan, 0); b.now = 0.1;
  assert.equal(b.sideIsClear('left')[0], false);
  assert.equal(b.sideIsClear('right')[0], true);
});

test('record + replay: the current brain re-decides a recorded run', async () => {
  const { default: sharp } = await import('sharp');
  const dir = mkdtempSync(join(tmpdir(), 'run-'));
  const rcfg = { ...cfg, record: { ...cfg.record, dir, max_mb: 50 } };
  const rec = new Recorder(rcfg);
  const run = rec.start();
  const w = 320, h = 240, img = new Uint8Array(w * h * 3); // road: yellow line left, white line right
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const c = Math.abs(x - 100) < 4 ? [235, 200, 20] : Math.abs(x - 260) < 4 ? [235, 235, 235] : [66, 62, 62];
    img.set(c, 3 * (y * w + x));
  }
  const jpg = await sharp(img, { raw: { width: w, height: h, channels: 3 } }).jpeg({ quality: 90 }).toBuffer();
  for (let t = 0; t <= 3000; t += 50) {
    if (t % 500 === 0) rec.frame(t, jpg);
    rec.write('pose', t, { x: 0, y: 0, yaw: 0 });
    rec.write('range', t, { m: t < 2000 ? Infinity : 0.6 }); // obstacle appears at 2 s
    rec.write('vision', t, { objects: [], aruco: [] });
    rec.write('brain', t, { state: t < 2000 ? 'DRIVE' : 'SCAN' });
  }
  rec.stop();
  await new Promise((r) => setTimeout(r, 200)); // let the async frame writes land
  const out = await replay(run, cfg, { print: () => {} });
  const states = out.transitions.map((x) => x[1]);
  assert.ok(states.includes('DRIVE') && states.includes('SCAN'), JSON.stringify(out.transitions));
});
