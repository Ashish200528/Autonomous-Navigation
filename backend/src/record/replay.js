// Re-runs the brain on a recording, without hardware:  node src/record/replay.js runs/<dir>
// Feeds the recorded inputs (pose, range, vision, scan, camera frames) into the *current*
// brain code + config and compares its decisions with what the robot decided at the time.
// Use it after changing the brain: same run, see what would now happen differently.
import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';
import { loadConfig } from '../config.js';
import { Brain } from '../brain/state-machine.js';

export async function replay(dir, cfg = loadConfig(), { print = console.log } = {}) {
  const P = { ...cfg.brain, autostart: false, flag_m: cfg.distance.flag_m, warn_m: cfg.distance.warn_m,
    side_sector_deg: cfg.map.side_sector_deg, side_block_m: cfg.map.side_block_m };
  const brain = new Brain(P, cfg.camera, (m) => print(`  [brain] ${m}`));
  const events = readFileSync(join(dir, 'log.ndjson'), 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l));
  let frames = [];
  try { frames = readdirSync(join(dir, 'frames')).map((f) => Number(f.split('.')[0])).sort((a, b) => a - b); } catch {}
  const sharp = frames.length ? (await import('sharp')).default : null;
  let fi = 0, ticks = 0, diffs = 0;
  const transitions = [];
  let last = null;
  for (const e of events) {
    while (fi < frames.length && frames[fi] <= e.t) { // camera frames in time order
      const { data, info } = await sharp(join(dir, 'frames', `${frames[fi]}.jpg`)).resize(160, 120, { kernel: 'nearest' }).raw().toBuffer({ resolveWithObject: true });
      if (info.channels === 3) brain.onImage({ w: 160, h: 120, data: new Uint8Array(data) }, frames[fi] / 1000);
      fi++;
    }
    const s = e.t / 1000;
    if (e.k === 'pose') brain.onPose(e);
    else if (e.k === 'range') brain.onRange(e.m ?? Infinity, s);
    else if (e.k === 'vision') brain.onVision({ objects: e.objects, aruco: e.aruco ?? [] }, s);
    else if (e.k === 'scan') brain.onScan(Float32Array.from(e.scan, (v) => v ?? Infinity), s);
    else if (e.k === 'brain') {
      if (e.state !== 'HOLD' && brain.state === 'HOLD') brain.go(); // GO was pressed at this point
      const out = brain.tick(s);
      ticks++;
      if (out.state !== e.state) diffs++;
      if (out.state !== last) { transitions.push([s.toFixed(1), out.state, out.note]); last = out.state; }
    }
  }
  print(`replayed ${ticks} ticks, ${frames.length} frames; state differs from the recording in ${diffs} ticks`);
  for (const [t, st, note] of transitions) print(`  t=${t}s  ${st}  ${note}`);
  return { ticks, diffs, transitions };
}

if (import.meta.url === `file://${process.argv[1]}` || process.argv[1]?.endsWith('replay.js')) {
  if (!process.argv[2]) { console.error('usage: node src/record/replay.js runs/<dir>'); process.exit(2); }
  await replay(process.argv[2]);
}
