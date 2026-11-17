// Port of anc_autodrive/logic.py (lane finding, zebra crossing, colour obstacle, overtake plan).
// Images are {w, h, data: RGB Uint8Array}. HSV follows OpenCV's 8-bit convention (H 0-180)
// so the thresholds are the same numbers as the Python.
import { median } from './mono_distance.js';

export const FLAG_DISTANCE_M = 0.30;
export const WARN_DISTANCE_M = 1.20;

function hsv(r, g, b) {
  const v = Math.max(r, g, b), mn = Math.min(r, g, b), d = v - mn;
  const s = v === 0 ? 0 : (255 * d) / v;
  let h = 0;
  if (d) h = v === r ? (60 * (g - b)) / d : v === g ? 120 + (60 * (b - r)) / d : 240 + (60 * (r - g)) / d;
  if (h < 0) h += 360;
  return [Math.round(h / 2), Math.round(s), v];
}

export function lineMasks(img) {
  const n = img.w * img.h, yellow = new Uint8Array(n), white = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const [h, s, v] = hsv(img.data[3 * i], img.data[3 * i + 1], img.data[3 * i + 2]);
    yellow[i] = h >= 19 && h <= 40 && s >= 80 && v >= 100;
    white[i] = s <= 45 && v >= 175;
  }
  return { yellow, white };
}

// [start, end] spans of consecutive non-zero pixels in mask[off+x0 .. off+x1), relative to x0.
export function whiteRuns(mask, off, x0, x1) {
  const runs = [];
  let a = -1;
  for (let x = x0; x < x1; x++) {
    if (mask[off + x]) { if (a < 0) a = x; } else if (a >= 0) { runs.push([a - x0, x - 1 - x0]); a = -1; }
  }
  if (a >= 0) runs.push([a - x0, x1 - 1 - x0]);
  return runs;
}

const isZebraRow = (white, off, w) => whiteRuns(white, off, Math.trunc(0.2 * w), Math.trunc(0.8 * w)).length >= 3;
const rowRange = (h, roi, step) => { const r = []; for (let y = Math.trunc(h * roi[0]); y < Math.trunc(h * roi[1]); y += step) r.push(y); return r; };

export function crossingFraction(img, roi = [0.55, 0.92], step = 4, masks = lineMasks(img)) {
  const rows = rowRange(img.h, roi, step);
  return rows.filter((r) => isZebraRow(masks.white, r * img.w, img.w)).length / Math.max(1, rows.length);
}

// Returns [centreX, yellowX, whiteX, halfLanePx]; x values may be null.
export function findLane(img, halfLanePx = null, roi = [0.55, 0.92], step = 4, masks = lineMasks(img)) {
  const { w, h } = img, { yellow, white } = masks, ys = [], ws = [];
  for (const r of rowRange(h, roi, step)) {
    const off = r * w;
    if (isZebraRow(white, off, w)) continue;
    let sum = 0, cnt = 0;
    for (let x = 0; x < w; x++) if (yellow[off + x]) { sum += x; cnt++; }
    const yX = cnt >= 2 ? sum / cnt : null;
    const start = yX != null ? Math.trunc(yX + 4) : Math.trunc(w * 0.45);
    if (yX != null) ys.push(yX);
    let runs = start < w ? whiteRuns(white, off, start, w).map(([a, b]) => [a + start, b + start]) : [];
    if (halfLanePx != null) {
      runs = runs.filter(([a, b]) => b - a < 1.2 * halfLanePx);
      if (yX != null) runs = runs.filter(([a]) => Math.abs(a - yX - 2 * halfLanePx) < 0.7 * halfLanePx);
    }
    if (runs.length) ws.push(runs[0][0]);
  }
  const yX = ys.length >= 2 ? median(ys) : null, wX = ws.length >= 2 ? median(ws) : null;
  if (halfLanePx == null) halfLanePx = w * 0.35;
  if (yX != null && wX != null && wX > yX) { halfLanePx = (wX - yX) / 2; return [(yX + wX) / 2, yX, wX, halfLanePx]; }
  if (yX != null) return [yX + halfLanePx, yX, null, halfLanePx];
  if (wX != null) return [wX - halfLanePx, null, wX, halfLanePx];
  return [null, null, null, halfLanePx];
}

export const laneError = (centreX, width) => Math.max(-1, Math.min(1, (centreX - width / 2) / (width / 2)));

// ponytail: colour heuristic; misses grey/green obstacles. YOLO detections are OR-ed in by the brain.
export function obstacleFraction(img, roi = [0.35, 1.0]) {
  const { w, h, data } = img, y0 = Math.trunc(h * roi[0]), y1 = Math.trunc(h * roi[1]);
  let hits = 0;
  for (let i = y0 * w; i < y1 * w; i++) {
    const [hu, s, v] = hsv(data[3 * i], data[3 * i + 1], data[3 * i + 2]);
    if (s > 90 && v > 60 && !(hu >= 35 && hu <= 90) && !(hu >= 19 && hu < 35)) hits++;
  }
  return hits / Math.max(1, (y1 - y0) * w);
}

export const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export function overtakePlan(baseYaw, side = 1, shiftM = 0.5, angle = 0.7, passM = 1.3) {
  const diag = shiftM / Math.sin(angle), outYaw = wrap(baseYaw + side * angle), backYaw = wrap(baseYaw - side * angle);
  return [['turn', outYaw], ['forward', diag, outYaw], ['turn', baseYaw], ['pass', passM, baseYaw],
    ['turn', backYaw], ['forward', diag, backYaw], ['turn', baseYaw]];
}

export function flipX(img) {
  const { w, h, data } = img, out = new Uint8Array(data.length);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const s = 3 * (y * w + x), d = 3 * (y * w + (w - 1 - x));
    out[d] = data[s]; out[d + 1] = data[s + 1]; out[d + 2] = data[s + 2];
  }
  return { w, h, data: out };
}
