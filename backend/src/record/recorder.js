// Records what the brain saw and did: runs/<date>/log.ndjson + frames/<t>.jpg (320x240).
// Off by default (config record.enabled or the dashboard); stops itself at record.max_mb.
// Everything is buffered/async: recording never blocks the control loop.
import { createWriteStream, mkdirSync } from 'node:fs';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

export class Recorder {
  constructor(cfg, log = () => {}) { this.cfg = cfg; this.log = log; this.stream = null; }

  get on() { return !!this.stream; }

  start() {
    if (this.stream) return this.dir;
    this.dir = join(this.cfg.record.dir, new Date().toISOString().replace(/[:.]/g, '-'));
    mkdirSync(join(this.dir, 'frames'), { recursive: true });
    this.stream = createWriteStream(join(this.dir, 'log.ndjson'));
    this.bytes = 0; this.lastFrame = -1e9;
    this.log(`recording to ${this.dir}`);
    return this.dir;
  }

  stop() {
    if (!this.stream) return;
    this.stream.end(); this.stream = null;
    this.log(`recording stopped (${(this.bytes / 1e6).toFixed(1)} MB)`);
  }

  write(k, t, data) {
    if (!this.stream) return;
    const line = JSON.stringify({ k, t: Math.round(t * 10) / 10, ...data }, (_, v) => (v === Infinity ? null : v)) + '\n';
    this.bytes += line.length;
    this.stream.write(line);
    if (this.bytes > this.cfg.record.max_mb * 1e6) this.stop();
  }

  wantsFrame(t) { return this.on && t - this.lastFrame >= this.cfg.record.frame_every_ms; }

  frame(t, jpg) {
    if (!this.stream) return;
    this.lastFrame = t;
    this.bytes += jpg.length;
    writeFile(join(this.dir, 'frames', `${Math.round(t)}.jpg`), jpg).catch(() => {});
  }
}
