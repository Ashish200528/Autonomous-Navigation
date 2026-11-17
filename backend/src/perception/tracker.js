// IoU tracker: keeps an ID per object across YOLO frames so distances can be smoothed
// (the Python script's 5-sample median) and closing speed / time-to-collision measured.
import { iou } from './yolo_post.js';

export class Tracker {
  constructor(maxAgeMs = 1000, iouMin = 0.3) { this.maxAge = maxAgeMs; this.iouMin = iouMin; this.tracks = []; this.nextId = 1; }

  update(dets, t) {
    const free = new Set(this.tracks);
    for (const d of [...dets].sort((a, b) => b.score - a.score)) {
      let best = null, bestIou = this.iouMin;
      for (const tr of free) {
        const o = tr.name === d.name ? iou(tr.box, d.box) : 0;
        if (o > bestIou) { best = tr; bestIou = o; }
      }
      if (best) free.delete(best);
      else this.tracks.push((best = { id: this.nextId++, name: d.name, hist: [], dist: null, distT: 0, ttc: null }));
      best.box = d.box; best.t = t;
      d.track = best;
    }
    this.tracks = this.tracks.filter((tr) => t - tr.t <= this.maxAge);
    return dets;
  }
}
