// Phase 3 check: the Pi's YOLO pipeline vs object_detection_new_babu.py's (Ultralytics) on the
// same frames. Needs ffmpeg, python with ultralytics, and the .pt the model was exported from.
//   node tools/compare_python.mjs <frames_dir> [model.pt]
// Each image is scaled to 640x480 and converted to yuv420p exactly as the camera delivers it; Python
// sees the same pixels (decoded back from that yuv). Pass: matched boxes within 2 px (FP32) / 4 px (INT8).
import { Worker } from 'node:worker_threads';
import { execFileSync } from 'node:child_process';
import { mkdtempSync, readFileSync, readdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { iou } from '../src/perception/yolo_post.js';

const [dir, pt = 'yolo26n.pt'] = process.argv.slice(2);
if (!dir) { console.error('usage: node tools/compare_python.mjs <frames_dir> [model.pt]'); process.exit(2); }
const cfg = JSON.parse(readFileSync(new URL('../config/default.json', import.meta.url))).yolo;
const tmp = mkdtempSync(join(tmpdir(), 'cmp-'));
const files = readdirSync(dir).filter((f) => /\.(jpe?g|png)$/i.test(f)).sort();

const frames = files.map((f, i) => {
  const yuv = join(tmp, `${i}.yuv`), png = join(tmp, `${i}.png`);
  execFileSync('ffmpeg', ['-loglevel', 'error', '-y', '-i', join(dir, f), '-vf', 'scale=640:480', '-pix_fmt', 'yuv420p', '-f', 'rawvideo', yuv]);
  execFileSync('ffmpeg', ['-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-s', '640x480', '-i', yuv, png]);
  return { name: f, yuv, png };
});

const py = JSON.parse(execFileSync('python', ['-c', `
import json, sys
from ultralytics import YOLO
m = YOLO(sys.argv[1]); out = []
for p in sys.argv[2:]:
    r = m(p, imgsz=[${cfg.input_h},${cfg.input_w}], conf=${cfg.conf}, iou=${cfg.iou}, max_det=${cfg.max_det}, verbose=False)[0]
    out.append([[r.names[int(b.cls)], float(b.conf), [min(int(v), lim - 1) for v, lim in zip(b.xyxy[0].tolist(), (640, 480, 640, 480))]] for b in r.boxes])
print(json.dumps(out))`, pt, ...frames.map((f) => f.png)], { maxBuffer: 1e8 }).toString());

const w = new Worker(new URL('../src/perception/yolo.worker.js', import.meta.url), { workerData: { yolo: cfg } });
const js = await new Promise((done) => {
  const res = []; let i = 0;
  const send = () => { const b = readFileSync(frames[i].yuv); const ab = b.buffer.slice(b.byteOffset, b.byteOffset + b.length); w.postMessage({ t: i, yuv: ab, W: 640, H: 480 }, [ab]); };
  w.on('message', (m) => { if (m.ready) return send(); res.push(m.detections); if (++i < frames.length) send(); else done(res); });
});
await w.terminate();

let worst = 0, missed = 0, extra = 0, matched = 0;
frames.forEach((f, k) => {
  const used = new Set();
  for (const [cls, conf, box] of py[k]) {
    let best = null, bi = 0;
    js[k].forEach((d, j) => { const o = d.name === cls && !used.has(j) ? iou(d.box, box) : 0; if (o > bi) { bi = o; best = j; } });
    if (best == null || bi < 0.5) { missed++; console.log(`${f.name}: Python ${cls} ${conf.toFixed(2)} not found by the Pi${conf < cfg.conf + 0.03 ? ' (borderline confidence)' : ''}`); continue; }
    used.add(best); matched++;
    worst = Math.max(worst, ...box.map((v, i) => Math.abs(v - js[k][best].box[i])));
  }
  extra += js[k].length - used.size;
});
console.log(`\n${frames.length} frames: ${matched} boxes matched, worst corner difference ${worst} px, ${missed} missed, ${extra} extra`);
console.log(worst <= 2 ? 'PASS (≤ 2 px)' : worst <= 4 ? 'PASS for INT8 (≤ 4 px)' : 'FAIL: boxes differ by more than 4 px');
