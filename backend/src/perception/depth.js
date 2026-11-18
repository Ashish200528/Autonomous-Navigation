// Dense monocular depth (MiDaS v2.1 small) in its own low-priority process (plan §7.3, §10.3):
// started by index.js with `nice -n 10`, so it only gets CPU the YOLO worker and control loop
// leave over, and its memory is separate. IPC: {t, rgb: Uint8Array n*n*3} -> {t, disp: Float32Array n*n, ms}.
import ort from 'onnxruntime-node';
import { fileURLToPath } from 'node:url';

const cfg = JSON.parse(process.env.DEPTH_CFG);
const n = cfg.size;
const session = await ort.InferenceSession.create(fileURLToPath(new URL(`../../${cfg.model}`, import.meta.url)), {
  executionProviders: ['cpu'], graphOptimizationLevel: 'all', intraOpNumThreads: cfg.threads, interOpNumThreads: 1,
});
const input = new Float32Array(3 * n * n);
const tensor = new ort.Tensor('float32', input, [1, 3, n, n]);
const name = session.inputNames[0];
const MEAN = [0.485, 0.456, 0.406], STD = [0.229, 0.224, 0.225]; // MiDaS small normalisation

await session.run({ [name]: tensor }); // warm-up
process.send({ ready: true });

process.on('message', async ({ t, rgb }) => {
  const t0 = performance.now();
  for (let i = 0; i < n * n; i++) for (let c = 0; c < 3; c++) input[c * n * n + i] = (rgb[3 * i + c] / 255 - MEAN[c]) / STD[c];
  const out = Object.values(await session.run({ [name]: tensor }))[0];
  process.send({ t, disp: Float32Array.from(out.data), ms: performance.now() - t0 });
});
process.on('disconnect', () => process.exit(0)); // parent died: don't linger
