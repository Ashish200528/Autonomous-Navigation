// USB webcam through ffmpeg (V4L2 -> raw yuv420p on stdout). Emits 'frame' {t, yuv, W, H}.
// Only the newest frame is kept; consumers that are busy simply miss frames (no queue, no lag).
import { EventEmitter } from 'node:events';
import { spawn } from 'node:child_process';

export class Camera extends EventEmitter {
  constructor(cam) {
    super();
    this.cam = cam;
    this.size = (cam.width * cam.height * 3) / 2;
    this.latest = null;
    this.proc = null;
  }

  start() {
    const { width, height, ffmpeg_input } = this.cam;
    this.proc = spawn('ffmpeg', ['-hide_banner', '-loglevel', 'error', '-fflags', 'nobuffer', ...ffmpeg_input,
      '-vf', `scale=${width}:${height}`, '-pix_fmt', 'yuv420p', '-f', 'rawvideo', '-'], { stdio: ['ignore', 'pipe', 'inherit'] });
    // ponytail: one new Buffer per frame (~460 KB at 15 fps); a buffer pool if GC ever shows up in the timings.
    let cur = Buffer.allocUnsafeSlow(this.size), fill = 0;
    this.proc.stdout.on('data', (chunk) => {
      let off = 0;
      while (off < chunk.length) {
        const n = Math.min(this.size - fill, chunk.length - off);
        chunk.copy(cur, fill, off, off + n);
        fill += n; off += n;
        if (fill === this.size) {
          this.latest = { t: performance.now(), yuv: cur, W: width, H: height };
          this.emit('frame', this.latest);
          cur = Buffer.allocUnsafeSlow(this.size); fill = 0;
        }
      }
    });
    this.proc.on('exit', (code) => {
      this.emit('warn', `ffmpeg exited (${code}), restarting`);
      setTimeout(() => this.start(), 1000);
    });
  }

  stop() { this.proc?.removeAllListeners('exit'); this.proc?.kill(); }
}
