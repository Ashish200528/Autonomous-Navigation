// USB webcam through ffmpeg (V4L2 -> raw yuv420p on stdout). Emits 'frame' {t, yuv, W, H}.
// Only the newest frame is kept; consumers that are busy simply miss frames (no queue, no lag).
import { EventEmitter } from 'node:events';
import { spawn } from 'node:child_process';
import { readdirSync } from 'node:fs';

// "-i auto": first USB webcam (UVC capture node), never the SoC codec nodes /dev/video0-1 on the Uno Q.
function usbCam() {
  try { const n = readdirSync('/dev/v4l/by-id').filter((f) => f.endsWith('-video-index0')).sort()[0]; return n && `/dev/v4l/by-id/${n}`; } catch { return null; }
}

export class Camera extends EventEmitter {
  constructor(cam) {
    super();
    this.cam = cam;
    this.size = (cam.width * cam.height * 3) / 2;
    this.latest = null;
    this.proc = null;
  }

  start() {
    const { width, height } = this.cam;
    const ffmpeg_input = this.cam.ffmpeg_input.slice(), i = ffmpeg_input.indexOf('auto');
    let input = ffmpeg_input;
    if (i >= 0) {
      const dev = usbCam();
      if (dev) ffmpeg_input[i] = dev;
      // No USB webcam: wait for a network stream instead (tools/laptop-camera.ps1 sends a laptop's webcam).
      else if (this.cam.fallback_input) {
        input = this.cam.fallback_input;
        if (this.source !== 'net') this.emit('warn', 'no USB webcam: waiting for a camera stream on port 5000 (run tools/laptop-camera.ps1 on the laptop)');
      } else { this.emit('warn', 'no USB webcam found, retrying'); return void setTimeout(() => this.start(), 2000); }
    }
    this.source = input === ffmpeg_input ? 'usb' : 'net';
    clearInterval(this.usbWatch);
    if (this.source === 'net') { // a USB webcam plugged in later wins: drop the network wait, restart on it
      this.usbWatch = setInterval(() => { if (usbCam()) { clearInterval(this.usbWatch); this.emit('warn', 'USB webcam plugged in: switching to it'); this.proc?.kill(); } }, 2000);
    }
    this.proc = spawn('ffmpeg', ['-hide_banner', '-loglevel', 'error', '-fflags', 'nobuffer', ...input,
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
      if (this.source !== 'net') this.emit('warn', `ffmpeg exited (${code}), restarting`); // net: sender left, listen again
      setTimeout(() => this.start(), 1000);
    });
  }

  stop() { clearInterval(this.usbWatch); this.proc?.removeAllListeners('exit'); this.proc?.kill(); }
}
