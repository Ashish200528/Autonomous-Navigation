// Phase 0 benchmark on the Pi (plan §10, §13): every .onnx in models/ × thread counts.
//   node tools/benchmark.js [runs=30]
// Prints median / p90 ms, Hz, process RSS and CPU temperature: the table that replaces the plan's (est.) numbers.
// Stop the backend first (sudo systemctl stop anc-backend) so it does not compete for the CPU.
import { readdirSync, readFileSync } from 'node:fs';
import ort from 'onnxruntime-node';
import { fileURLToPath } from 'node:url';

const runs = Number(process.argv[2] ?? 30);
const dir = new URL('../models/', import.meta.url);
const temp = () => { try { return (Number(readFileSync('/sys/class/thermal/thermal_zone0/temp', 'utf8')) / 1000).toFixed(0) + ' °C'; } catch { return '—'; } };
const pct = (xs, q) => [...xs].sort((a, b) => a - b)[Math.min(xs.length - 1, Math.floor(q * xs.length))];

console.log('model | threads | median ms | p90 ms | Hz | RSS MB | temp');
console.log('---|---|---|---|---|---|---');
for (const file of readdirSync(dir).filter((f) => f.endsWith('.onnx')).sort()) {
  for (const threads of [1, 2, 3, 4]) {
    const s = await ort.InferenceSession.create(fileURLToPath(new URL(file, dir)),
      { graphOptimizationLevel: 'all', intraOpNumThreads: threads, executionMode: 'sequential' });
    const meta = s.inputMetadata?.[0];
    const dims = meta?.shape?.map((d) => (typeof d === 'number' ? d : 1)) ?? guessShape(file);
    const x = new ort.Tensor('float32', new Float32Array(dims.reduce((a, b) => a * b, 1)).fill(0.5), dims);
    for (let i = 0; i < 3; i++) await s.run({ [s.inputNames[0]]: x });
    const ms = [];
    for (let i = 0; i < runs; i++) { const t = performance.now(); await s.run({ [s.inputNames[0]]: x }); ms.push(performance.now() - t); }
    const med = pct(ms, 0.5);
    console.log(`${file} | ${threads} | ${med.toFixed(1)} | ${pct(ms, 0.9).toFixed(1)} | ${(1000 / med).toFixed(1)} | ${(process.memoryUsage().rss / 1e6).toFixed(0)} | ${temp()}`);
    await s.release?.();
  }
}

function guessShape(file) { // older onnxruntime-node without inputMetadata: shape from the file name
  const m = file.match(/_(\d+)x(\d+)/) ?? file.match(/_(\d+)/);
  return m ? [1, 3, Number(m[1]), Number(m[2] ?? m[1])] : [1, 3, 256, 256];
}
