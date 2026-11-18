// Boot, wiring and the 20 Hz control loop. Starts in mode 'stop': nothing moves until
// the dashboard switches to 'manual' or 'auto' (token required).
//
// Threads / processes (plan §10.3):
//   main          camera frames, lanes, fusion, map, brain, arbiter, server (20 Hz)
//   yolo worker   YOLO (latest frame wins)
//   aruco worker  ArUco on the luma plane, ≤ aruco.rate_hz
//   depth process MiDaS, `nice -n 10`, ≤ depth.max_hz
import { Worker } from 'node:worker_threads';
import { existsSync } from 'node:fs';
import { readFile } from 'node:fs/promises';
import { execFile, spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { loadConfig, loadSecret, merge, saveLocal } from './config.js';
import { FireBird } from './hw/firebird.js';
import { Camera } from './hw/camera.js';
import { FrontRange, IrRing, battV, nearestIr } from './hw/sensors.js';
import { IrDriver } from './brain/ir-drive.js';
import { Wheels } from './control/wheels.js';
import { Odometry } from './fusion/odometry.js';
import { DistanceEstimator } from './fusion/distance.js';
import { DepthScale, buildAnchors } from './fusion/depth_scale.js';
import { KIND, VoxelMap } from './fusion/map3d.js';
import { intrinsics } from './fusion/geometry.js';
import { Tracker } from './perception/tracker.js';
import { yuvToRgb } from './perception/preprocess.js';
import { calibrateFocal } from './perception/mono_distance.js';
import { Brain } from './brain/state-machine.js';
import { arbitrate } from './safety/arbiter.js';
import { Recorder } from './record/recorder.js';
import { startServer } from './server/http.js';

const cfg = loadConfig();
const syncBrain = () => Object.assign(cfg.brain, { flag_m: cfg.distance.flag_m, warn_m: cfg.distance.warn_m,
  side_sector_deg: cfg.map.side_sector_deg, side_block_m: cfg.map.side_block_m });
syncBrain();
const nowMs = () => performance.now();
const log = (...a) => console.log(new Date().toISOString().slice(11, 23), ...a);
const exists = (rel) => existsSync(new URL(`../${rel}`, import.meta.url));

const robot = new FireBird(cfg);
const camera = new Camera(cfg.camera);
const wheels = new Wheels(cfg.robot);
const odom = new Odometry(cfg.robot);
const front = new FrontRange(cfg.range);
const tracker = new Tracker();
const distance = new DistanceEstimator(cfg);
const map = new VoxelMap(cfg);
const rec = new Recorder(cfg, (m) => log('[record]', m));
const brain = new Brain(cfg.brain, cfg.camera, (m) => { log('[brain]', m); rec.write('event', nowMs(), { msg: m }); });
const irDriver = new IrDriver(cfg.ir_drive);
const irs = new IrRing(cfg.ir);
const vision = cfg.camera.enabled !== false; // false: IR-only robot, no camera / YOLO / ArUco / depth
if (!vision) log('camera disabled (camera.enabled = false): IR drive and manual only');
if (cfg.record.enabled) rec.start();

const S = {
  mode: 'stop', manual: { v: 0, w: 0, t: 0 }, link: false,
  telemT: -1e9, frameT: -1e9, yoloT: -1e9, yoloMs: null, yoloHz: 0,
  front: { m: Infinity, proxHit: false, blindHold: false },
  objects: [], aruco: [], arucoT: -1e9, arucoMs: null, depth: null, depthMs: null, scan: null,
  cmd: { v: 0, w: 0, reason: 'boot' }, brain: { v: 0, w: 0, pan: 0, state: 'HOLD', note: '' },
  pwm: [0, 0], pan: NaN /* forces the first servo command */, panChangedT: 0,
  temp: 0, throttled: false, calib: null, jpegAskT: -1e9, rgb320: null,
  ir: null, irCmd: { v: 0, w: 0, state: 'CRUISE', note: '' },
};
const servoSettled = () => nowMs() - S.panChangedT > 400;

// ------------------------------------------------------------------ robot link
let telemN = 0;
robot.on('port', (p) => log(`serial port ${p}${cfg.serial.path === 'auto' ? ' (auto-detected FTDI)' : ''}`));
robot.on('link', (up) => { S.link = up; log(up ? `serial port ${robot.lastPath} open` : 'serial port closed'); if (!up) wheels.reset(); });
// An open port is not a working robot: say when the ATmega is actually talking.
let telemFlowing = null;
setInterval(() => {
  if (!S.link) return;
  const now = nowMs() - S.telemT < 2000;
  if (now === telemFlowing) return;
  telemFlowing = now;
  log(now ? 'robot telemetry flowing' : 'no telemetry from the ATmega for 2 s: robot off, jumper J1 not on USB, or not running this firmware');
}, 1000);
robot.on('boot', (b) => { log('ATmega boot', b); odom.prev = null; S.pan = NaN; });
robot.on('telemetry', (t, at) => {
  S.telemT = at;
  const pose = odom.update(t);
  brain.onPose(pose);
  S.front = front.update(t, at, wheels.target[0] + wheels.target[1] > 0);
  S.ir = irs.update(t);
  const range = Math.min(S.front.m, cameraAhead());
  brain.onRange(range, at / 1000);
  if (++telemN % 5 === 0) { // 10 Hz: map + recording
    const m = S.front.m, r = cfg.range;
    if (Number.isFinite(m) && !S.front.proxHit && !S.front.blindHold && m <= r.sharp_max_m)
      map.addRobotPoint(cfg.mount.sharp_x_m + m, 0, cfg.mount.sharp_height_m, pose, at, KIND.SHARP);
    for (const s of S.ir) if (Number.isFinite(s.m)) { // IR hits: the walls around the driven path
      const a = (s.deg * Math.PI) / 180, d = cfg.ir.mount_r_m + s.m;
      map.addRobotPoint(d * Math.cos(a), d * Math.sin(a), cfg.ir.height_m, pose, at, KIND.IR);
    }
    rec.write('telem', at, { enc: t.enc, periodUs: t.periodUs, sharp: t.sharp, prox: t.prox, proxRear: t.proxRear, proxDark: t.proxDark, line: t.line, battMv: t.battMv, flags: t.flags });
    rec.write('pose', at, { x: pose.x, y: pose.y, yaw: pose.yaw });
    rec.write('range', at, { m: range });
  }
});

// Nearest YOLO object straight ahead (camera facing forward, fresh result).
function cameraAhead() {
  if (Math.abs(S.pan) > 0.05 || nowMs() - S.yoloT > 1000) return Infinity;
  return Math.min(Infinity, ...S.objects.filter((o) => o.zone === 'Center' && o.dist != null).map((o) => o.dist));
}

function pushVision(t) {
  const objects = nowMs() - S.yoloT < 1000 ? S.objects : [];
  const aruco = nowMs() - S.arucoT < 1000 ? S.aruco : [];
  brain.onVision({ objects, aruco }, t / 1000);
  rec.write('vision', t, { objects: objects.map(({ cls, conf, box, dist, zone }) => ({ cls, conf, box, dist, zone })), aruco });
}

// ------------------------------------------------------------------- YOLO worker
let yolo = null, yoloBusy = true, yoloCount = 0;
if (vision && exists(cfg.yolo.model)) {
  yolo = new Worker(new URL('./perception/yolo.worker.js', import.meta.url), { workerData: { yolo: cfg.yolo } });
  yolo.on('message', (m) => {
    yoloBusy = false;
    if (m.ready) return log('YOLO ready');
    onDetections(m);
  });
  yolo.on('error', (e) => { log('YOLO worker failed:', e.message); yolo = null; });
} else if (vision) log(`no model at ${cfg.yolo.model}: running without YOLO (auto mode stays stopped). See tools/export_yolo.py`);

function onDetections({ t, detections, ms }) {
  S.yoloT = t; S.yoloMs = ms; yoloCount++;
  tracker.update(detections, t);
  const pan = S.yoloPan ?? 0, pose = odom.pose;
  const depth = S.depth?.validFor(pan, pose, nowMs()) ? S.depth : null;
  S.objects = detections.map((d) => distance.estimate(d, pan, servoSettled() ? S.front.m : Infinity, depth));
  for (const o of S.objects) if (o.dist != null && o.dist < cfg.map.max_range_m) // objects into the 3D map
    map.addRobotPoint(cfg.mount.camera_x_m + o.dist * Math.cos(o.bearing), o.dist * Math.sin(o.bearing), 0.1, pose, t, KIND.OBJECT);
  pushVision(t);
  if (S.calib) {
    const o = S.objects.find((x) => x.cls === S.calib.cls && !x.truncated);
    const H = cfg.distance.known_heights_m[S.calib.cls];
    if (o && H) {
      const f = calibrateFocal(o.box[3] - o.box[1], S.calib.d, H);
      cfg.camera.focal_px = f; saveLocal({ camera: { focal_px: f } });
      log(`calibrated FOCAL_LENGTH_PX = ${f.toFixed(1)} from ${S.calib.cls} at ${S.calib.d} m`);
      S.calib = null;
    }
  }
}
setInterval(() => { S.yoloHz = yoloCount / 2; yoloCount = 0; }, 2000);

// ------------------------------------------------------------------ ArUco worker
let aruco = null, arucoBusy = true, arucoLast = -1e9;
if (vision && cfg.aruco.enabled) {
  aruco = new Worker(new URL('./perception/aruco.worker.js', import.meta.url), { workerData: { cfg } });
  aruco.on('message', (m) => {
    arucoBusy = false;
    if (m.ready) return;
    S.aruco = m.markers; S.arucoT = m.t; S.arucoMs = m.ms;
    if (m.markers.length) pushVision(m.t);
  });
  aruco.on('error', (e) => { log('ArUco worker failed:', e.message); aruco = null; });
}

// ------------------------------------------------------------------ depth process
let depthProc = null, depthBusy = true, depthLast = -1e9, depthMeta = null;
if (vision && exists(cfg.depth.model)) {
  const script = fileURLToPath(new URL('./perception/depth.js', import.meta.url));
  const [cmd, args] = process.platform === 'linux' ? ['nice', ['-n', '10', process.execPath, script]] : [process.execPath, [script]];
  depthProc = spawn(cmd, args, { stdio: ['ignore', 'inherit', 'inherit', 'ipc'], serialization: 'advanced',
    env: { ...process.env, DEPTH_CFG: JSON.stringify(cfg.depth) } });
  depthProc.on('message', (m) => {
    depthBusy = false;
    if (m.ready) return log('depth model ready');
    S.depthMs = m.ms;
    const ds = new DepthScale(m.disp, cfg.depth.size, depthMeta, cfg);
    const sameView = Math.abs(S.yoloPan - depthMeta.pan) < 0.05 && Math.abs(S.yoloT - depthMeta.t) < 500;
    ds.solve(buildAnchors(cfg, { pan: depthMeta.pan, sharpM: depthMeta.sharpM, objects: sameView ? S.objects : [] }));
    S.depth = ds;
    if (ds.fit.ok) map.addDepth(ds);
  });
  depthProc.on('exit', (c) => { log(`depth process exited (${c}); continuing without dense depth`); depthProc = null; });
} else if (vision) log(`no depth model at ${cfg.depth.model}: cue C and the dense 3D cloud are off. See tools/export_depth.py`);

// ---------------------------------------------------------------------- camera
camera.on('warn', (m) => log('[camera]', m));
camera.on('frame', (f) => {
  S.frameT = f.t;
  const now = f.t, { W, H } = f;
  brain.onImage(yuvToRgb(f.yuv, W, H, 160, 120), now / 1000);
  if (now - S.jpegAskT < 1000 || rec.wantsFrame(now)) S.rgb320 = yuvToRgb(f.yuv, W, H, 320, 240);
  if (rec.wantsFrame(now)) jpeg().then((j) => j && rec.frame(now, j));
  if (depthProc && !depthBusy && now - depthLast > 1000 / cfg.depth.max_hz && servoSettled()) {
    depthBusy = true; depthLast = now;
    depthMeta = { t: now, pan: S.pan, pose: { ...odom.pose }, sharpM: S.front.m };
    depthProc.send({ t: now, rgb: yuvToRgb(f.yuv, W, H, cfg.depth.size, cfg.depth.size).data });
  }
  if (aruco && !arucoBusy && now - arucoLast > 1000 / cfg.aruco.rate_hz) {
    arucoBusy = true; arucoLast = now;
    const Y = f.yuv.buffer.slice(f.yuv.byteOffset, f.yuv.byteOffset + W * H); // copy: the frame goes to YOLO
    aruco.postMessage({ t: now, pan: S.pan, Y, W, H, intr: intrinsics(cfg) }, [Y]);
  }
  if (yolo && !yoloBusy) { // latest-frame-wins: only sent when the worker is free (transfers the frame)
    yoloBusy = true;
    S.yoloPan = S.pan;
    const ab = f.yuv.buffer.byteLength === f.yuv.length ? f.yuv.buffer : f.yuv.buffer.slice(f.yuv.byteOffset, f.yuv.byteOffset + f.yuv.length);
    yolo.postMessage({ t: now, yuv: ab, W, H }, [ab]);
  }
});

// ------------------------------------------------------------- map + virtual scan
setInterval(() => {
  map.prune(nowMs());
  S.scan = map.scan(odom.pose);
  brain.onScan(S.scan, nowMs() / 1000);
}, 200);
setInterval(() => { if (rec.on && S.scan) rec.write('scan', nowMs(), { scan: Array.from(S.scan, (v) => (Number.isFinite(v) ? +v.toFixed(3) : null)) }); }, 1000);

// ------------------------------------------------------------------ Pi health
setInterval(async () => {
  S.temp = Number(await readFile('/sys/class/thermal/thermal_zone0/temp', 'utf8').catch(() => 0)) / 1000;
  execFile('vcgencmd', ['get_throttled'], (err, out) => { S.throttled = !err && (parseInt(out.split('=')[1], 16) & 0x4) !== 0; });
}, 2000);

// --------------------------------------------------------------- control loop
let lastTick = null;
setInterval(() => {
  const now = nowMs();
  lastTick ??= now - 50;
  const dt = Math.min(0.2, (now - lastTick) / 1000);
  if (now - lastTick > 200) log(`control tick late by ${(now - lastTick - 50).toFixed(0)} ms`);
  lastTick = now;

  S.brain = brain.tick(now / 1000);
  if (S.brain.pan !== S.pan) setPan(S.brain.pan);

  const t = robot.last;
  if (S.mode === 'ir') S.irCmd = irDriver.tick(S.ir, S.front.m, now / 1000);
  // ponytail: front IR prox only joins the stop range in IR mode until the IR table is calibrated
  const rangeM = S.mode === 'ir' ? Math.min(S.front.m, nearestIr(S.ir, [2, 3, 4])) : S.front.m;
  S.cmd = arbitrate({
    mode: S.mode, brain: S.brain, manual: S.manual, manualAgeMs: now - S.manual.t, ir: S.irCmd,
    telemAgeMs: now - S.telemT, frameAgeMs: now - S.frameT, yoloAgeMs: yolo ? now - S.yoloT : Infinity,
    mcuFlags: t?.flags ?? 0, rangeM, rearM: nearestIr(S.ir, [6, 7, 8]), temp: S.temp, throttled: S.throttled,
  }, cfg);
  if (S.cmd.v === 0 && S.cmd.w === 0) wheels.reset();
  S.pwm = [...wheels.update(S.cmd.v, S.cmd.w, t, dt)];
  robot.drive(S.pwm[0], S.pwm[1]); // also the ATmega's keep-alive
  rec.write('brain', now, { state: S.brain.state, note: S.brain.note, v: S.brain.v, w: S.brain.w, pan: S.brain.pan });
  rec.write('cmd', now, { mode: S.mode, v: S.cmd.v, w: S.cmd.w, reason: S.cmd.reason, pwm: S.pwm });
}, 50);

function setPan(rad) {
  const s = cfg.servo.pan;
  const us = Math.max(s.min_us, Math.min(s.max_us, s.center_us + rad * s.us_per_rad));
  robot.servo(s.ch, us);
  S.pan = rad; S.panChangedT = nowMs();
}

async function jpeg() {
  if (!S.rgb320) return null;
  const { default: sharp } = await import('sharp');
  return sharp(S.rgb320.data, { raw: { width: 320, height: 240, channels: 3 } }).jpeg({ quality: 60 }).toBuffer();
}

// -------------------------------------------------------------------- server
const secret = loadSecret();
const fin = (v, d = 3) => (Number.isFinite(v) ? +v.toFixed(d) : null);
startServer({
  getState: () => ({
    mode: S.mode, link: S.link, brain: S.brain, cmd: S.cmd, pwm: S.pwm, pose: odom.pose, pan: S.pan,
    front: { ...S.front, m: fin(S.front.m) },
    batt: robot.last ? battV(robot.last.battMv) : null, mcuFlags: robot.last?.flags ?? null,
    objects: S.objects.map(({ id, cls, conf, box, dist, sigma, zone, status, ttc, cues, inconsistent, truncated, bearing }) =>
      ({ id, cls, conf, box, dist, sigma, zone, status, ttc, cues, inconsistent, truncated, bearing })),
    aruco: nowMs() - S.arucoT < 1000 ? S.aruco : [],
    depth: S.depth ? { ms: S.depthMs, fit: S.depth.fit, ageMs: nowMs() - S.depth.meta.t } : null,
    scan: S.scan ? Array.from(S.scan, (v) => fin(v, 2)) : null,
    frame: { w: cfg.camera.width, h: cfg.camera.height },
    ages: { telem: nowMs() - S.telemT, frame: nowMs() - S.frameT, yolo: nowMs() - S.yoloT },
    yolo: { ms: S.yoloMs, hz: S.yoloHz, on: !!yolo }, arucoMs: S.arucoMs, temp: S.temp,
    heightCorr: distance.heightCorr, recording: rec.on ? rec.dir : null, mapVoxels: map.vox.size,
    // st: dist (m metres) | clear | uncal (no calibration for this signal) | nodata (rear: firmware v2 + J4)
    ir: S.ir?.map(({ n, deg, mcu, lit, dark, sig, m, own, st, kind }) => ({ n, deg, mcu, lit, dark, sig, m: fin(m), own, st, kind })) ?? null,
    irCal: cfg.ir.cal_by_sensor, irSignal: cfg.ir.signal, irBaseline: cfg.ir.baseline,
    irDrive: S.irCmd, fw: robot.last?.fw ?? robot.boot?.version ?? null, vision, port: robot.lastPath,
    odo: { dist: fin(odom.dist), wheel: odom.wheel.map((d) => fin(d)), enc: robot.last?.enc ?? null, resets: odom.resets },
  }),
  cloud: () => map.cloud(),
  path: () => odom.trail,
  resetOdom: () => { odom.reset(); map.vox.clear(); return { ok: true }; },
  // IR calibration point: an object distance_m in front of sensor n. 'front' records 1-5 at once
  // (each with its own object at that distance, e.g. the robot facing into a box corner).
  calibrateIr: (sensor, d) => {
    if (!(d > 0 && d < 1)) return { error: 'distance must be 0-1 m' };
    const ns = sensor === 'front' ? [1, 2, 3, 4, 5] : [Number(sensor)];
    if (!ns.every((n) => n >= 1 && n <= 8)) return { error: 'sensor must be 1-8 or front' };
    const patch = { ir: { cal_by_sensor: {} } };
    for (const n of ns) {
      const r = irs.record(n, d);
      if (r.error) return r;
      patch.ir.cal_by_sensor[n] = r;
    }
    merge(cfg, patch); saveLocal(patch);
    log(`IR calibration at ${d} m:`, JSON.stringify(patch.ir.cal_by_sensor));
    return { ok: true, cal: patch.ir.cal_by_sensor };
  },
  // Zero: nothing within ~20 cm of these sensors; their current readings become the raw baseline.
  zeroIr: (sensor) => {
    const ns = sensor === 'all' ? [1, 2, 3, 4, 5, 6, 7, 8] : sensor === 'front' ? [1, 2, 3, 4, 5] : [Number(sensor)];
    const patch = { ir: { baseline: {} } };
    for (const n of ns) {
      if (S.ir?.[n - 1]?.st === 'nodata') continue;
      const r = irs.zero(n);
      if (r.error) return r;
      patch.ir.baseline[n] = r.value;
    }
    merge(cfg, patch); saveLocal(patch);
    log('IR zero (raw baseline):', JSON.stringify(patch.ir.baseline));
    return { ok: true, baseline: patch.ir.baseline };
  },
  resetIrCal: (sensor) => {
    const ns = sensor === 'all' ? [1, 2, 3, 4, 5, 6, 7, 8] : [Number(sensor)];
    const patch = { ir: { cal_by_sensor: Object.fromEntries(ns.map((n) => [n, null])) } };
    merge(cfg, patch); saveLocal(patch);
    return { ok: true };
  },
  health: () => ({
    link: S.link, boot: robot.boot, crcErrors: robot.parser.crcErrors, temp: S.temp, throttled: S.throttled,
    ages: { telem: nowMs() - S.telemT, frame: nowMs() - S.frameT, yolo: nowMs() - S.yoloT },
    yolo: { ms: S.yoloMs, hz: S.yoloHz }, arucoMs: S.arucoMs, depthMs: S.depthMs, depthFit: S.depth?.fit ?? null,
    mapVoxels: map.vox.size, rssMb: process.memoryUsage().rss / 1e6, recording: rec.on,
  }),
  getConfig: () => cfg,
  patchConfig: (patch) => { merge(cfg, patch); syncBrain(); saveLocal(patch); robot.sendConfig(); return cfg; },
  setMode: (m) => {
    if (!['stop', 'manual', 'auto', 'ir'].includes(m)) return { error: 'mode must be stop|manual|auto|ir' };
    if (m === 'auto' && !vision) return { error: 'auto needs the camera (camera.enabled is false); use IR drive' };
    if (m === 'ir' && S.mode !== 'ir') irDriver.reset();
    S.mode = m; log('mode', m);
    if (m === 'stop') wheels.reset();
    return { mode: m };
  },
  manual: (v, w) => { S.manual = { v: Math.max(-0.24, Math.min(0.24, v)), w: Math.max(-2, Math.min(2, w)), t: nowMs() }; },
  go: () => { brain.go(); return { ok: true }; },
  clear: () => { robot.clear(); return { ok: true }; },
  record: (on) => (on ? { dir: rec.start() } : (rec.stop(), { ok: true })),
  clearMap: () => { map.vox.clear(); return { ok: true }; },
  calibrate: (cls, d) => {
    if (!(cls in cfg.distance.known_heights_m) || !(d > 0)) return { error: 'class must be in known_heights_m, distance_m > 0' };
    S.calib = { cls, d }; return { ok: true, waiting_for: cls };
  },
  resetSelfcal: () => { distance.heightCorr = {}; distance.samples = {}; return { ok: true }; },
  jpeg: async () => { S.jpegAskT = nowMs(); return jpeg(); },
}, cfg.server, secret);
log(`token for driving: ${secret}  (config/secret)`);

robot.start();
if (vision) camera.start();

// ------------------------------------------------------------------ shutdown
async function shutdown(code) {
  S.mode = 'stop';
  camera.stop();
  depthProc?.kill();
  rec.stop();
  await robot.close().catch(() => {});
  process.exit(code);
}
process.on('SIGINT', () => shutdown(0));
process.on('SIGTERM', () => shutdown(0));
process.on('uncaughtException', (e) => { log('FATAL', e); shutdown(1); }); // systemd restarts us
