// YOLO ONNX output -> boxes in camera-frame pixels. Handles both export layouts:
//   [1, 4+nc, N]  YOLOv8 / YOLO11 (cx, cy, w, h, class scores...)  -> needs NMS
//   [1, N, 6]     YOLO26 end-to-end (x1, y1, x2, y2, score, class)  -> NMS-free

export function decode(data, dims, conf, allowed = null) {
  const out = [];
  if (dims[2] === 6) {
    for (let i = 0; i < dims[1]; i++) {
      const o = i * 6, score = data[o + 4], cls = Math.round(data[o + 5]);
      if (score >= conf && (!allowed || allowed.has(cls))) out.push({ box: [data[o], data[o + 1], data[o + 2], data[o + 3]], score, cls });
    }
    return out;
  }
  const [, C, N] = dims; // channels-first
  for (let i = 0; i < N; i++) {
    let best = -1, cls = -1;
    for (let c = 4; c < C; c++) { const s = data[c * N + i]; if (s > best) { best = s; cls = c - 4; } }
    if (best < conf || (allowed && !allowed.has(cls))) continue;
    const cx = data[i], cy = data[N + i], w = data[2 * N + i], h = data[3 * N + i];
    out.push({ box: [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], score: best, cls });
  }
  return out;
}

export function iou(a, b) {
  const ix = Math.max(0, Math.min(a[2], b[2]) - Math.max(a[0], b[0]));
  const iy = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[1], b[1]));
  const inter = ix * iy;
  return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter || 1);
}

// Per-class greedy NMS (same as Ultralytics with agnostic_nms=False).
export function nms(dets, thr, maxDet) {
  dets.sort((a, b) => b.score - a.score);
  const keep = [];
  for (const d of dets) {
    if (keep.every((k) => k.cls !== d.cls || iou(k.box, d.box) <= thr)) keep.push(d);
    if (keep.length >= maxDet) break;
  }
  return keep;
}

// Input-tensor pixels -> camera frame pixels, rounded and clamped like the Python script.
export function toFrame(dets, lb, W, H) {
  for (const d of dets) {
    d.box = d.box.map((v, i) => {
      const f = (v - (i % 2 ? lb.padY : lb.padX)) / lb.scale;
      return Math.max(0, Math.min(Math.trunc(f), (i % 2 ? H : W) - 1));
    });
  }
  return dets;
}
