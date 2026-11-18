// YOLO in a worker thread. Receives {t, yuv, W, H} (yuv transferred, zero-copy) and
// replies {t, detections, ms}. The main thread sends a new frame only when this one replied,
// so frames never queue up (latest-frame-wins lives in index.js).
import { parentPort, workerData } from 'node:worker_threads';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import ort from 'onnxruntime-node';
import { letterbox } from './preprocess.js';
import { decode, nms, toFrame } from './yolo_post.js';

const cfg = workerData.yolo;
const names = JSON.parse(readFileSync(new URL(`../../${cfg.names}`, import.meta.url), 'utf8'));
const allowed = cfg.classes ? new Set(cfg.classes.map((n) => names.indexOf(n)).filter((i) => i >= 0)) : null;

const session = await ort.InferenceSession.create(fileURLToPath(new URL(`../../${cfg.model}`, import.meta.url)), {
  executionProviders: ['cpu'],
  graphOptimizationLevel: 'all',
  executionMode: 'sequential',
  intraOpNumThreads: cfg.threads,
  enableCpuMemArena: true,
});
const inputName = session.inputNames[0];
const buf = new Float32Array(3 * cfg.input_w * cfg.input_h); // allocated once
const tensor = new ort.Tensor('float32', buf, [1, 3, cfg.input_h, cfg.input_w]);

for (let i = 0; i < 3; i++) await session.run({ [inputName]: tensor }); // warm-up
parentPort.postMessage({ ready: true });

parentPort.on('message', async ({ t, yuv, W, H }) => {
  const t0 = performance.now();
  const lb = letterbox(new Uint8Array(yuv), W, H, cfg.input_w, cfg.input_h, buf);
  const out = Object.values(await session.run({ [inputName]: tensor }))[0];
  let dets = decode(out.data, out.dims, cfg.conf, allowed);
  if (out.dims[2] !== 6) dets = nms(dets, cfg.iou, cfg.max_det);
  else dets = dets.sort((a, b) => b.score - a.score).slice(0, cfg.max_det);
  toFrame(dets, lb, W, H);
  for (const d of dets) d.name = names[d.cls] ?? String(d.cls);
  parentPort.postMessage({ t, detections: dets, ms: performance.now() - t0, yuv }, [yuv]);
});
