// Serial driver for the Fire Bird V ATmega2560 bridge (USB, FT232 on UART2).
// Emits 'telemetry' (decoded + Pi receive time), 'boot', 'link' (true/false).
import { EventEmitter } from 'node:events';
import { writeFile } from 'node:fs/promises';
import { SerialPort } from 'serialport';
import * as P from '../protocol.js';
import { sharpAdcFor } from './sensors.js';

export class FireBird extends EventEmitter {
  constructor(cfg) {
    super();
    this.cfg = cfg;
    this.parser = new P.Parser();
    this.seq = 0;
    this.port = null;
    this.last = null;      // latest telemetry
    this.lastT = 0;        // Pi ms when it arrived
    this.boot = null;
    this.lastPath = null;  // port actually in use (serial.path may be "auto")
  }

  // serial.path "auto": the first FTDI FT232 (the Fire Bird's USB chip, VID 0403), so swapping
  // robots or USB sockets (COM13 -> COM16 ...) needs no config change. Re-resolved on every retry.
  async resolvePath() {
    const { path } = this.cfg.serial;
    if (path !== 'auto') return path;
    const ports = await SerialPort.list().catch(() => []);
    return ports.find((p) => p.vendorId?.toLowerCase() === '0403')?.path ?? null;
  }

  async start() {
    const path = await this.resolvePath(), { baud } = this.cfg.serial;
    if (!path) { setTimeout(() => this.start(), 1000); return; }
    if (path !== this.lastPath) { this.lastPath = path; this.emit('port', path); }
    const port = new SerialPort({ path, baudRate: baud, autoOpen: false });
    port.on('data', (d) => this.onData(d));
    port.on('close', () => this.lost(port));
    port.on('error', () => this.lost(port));
    port.open((err) => {
      if (err) return this.lost(port);
      this.port = port;
      // FTDI default latency is 16 ms per read; 1 ms keeps telemetry fresh (udev rule also sets it).
      writeFile(`/sys/bus/usb-serial/devices/${port.path.split('/').pop()}/latency_timer`, '1').catch(() => {});
      this.sendConfig();
      this.emit('link', true);
    });
  }

  lost(port) {
    if (port.dead) return; // 'error' and 'close' both fire for one loss
    port.dead = true;
    if (this.port === port) { this.port = null; this.emit('link', false); }
    port.removeAllListeners();
    port.on('error', () => {}); // a late error with no listener would crash the process
    setTimeout(() => this.start(), 1000); // the ATmega already stopped the motors on its own
  }

  onData(chunk) {
    for (const f of this.parser.push(chunk)) {
      if (f.type === P.T.TELEM && f.payload.length >= 57) {
        this.last = P.decodeTelem(f.payload);
        this.lastT = performance.now();
        this.emit('telemetry', this.last, this.lastT);
      } else if (f.type === P.T.BOOT) {
        this.boot = P.decodeBoot(f.payload);
        this.sendConfig(); // the MCU reset: its config is back to defaults
        this.emit('boot', this.boot);
      }
    }
  }

  write(buf) { if (this.port) this.port.write(buf); }

  sendConfig() {
    const f = this.cfg.firmware;
    const C = P.CFG, sharp = f.reflex_sharp_m && this.cfg.range.sharp_front_index != null; // no Sharp: reflex off
    for (const [k, v] of [[C.TIMEOUT_MS, f.timeout_ms], [C.DWELL_MS, f.dwell_ms], [C.BATT_CUTOFF_MV, f.batt_cutoff_mv],
      [C.REFLEX_SHARP_CH, f.reflex_sharp_ch], [C.REFLEX_SHARP_ADC, sharp ? sharpAdcFor(f.reflex_sharp_m) : 0],
      [C.REFLEX_PROX_MASK, f.reflex_prox_mask], [C.REFLEX_PROX_ADC, f.reflex_prox_adc],
      [C.IR_OFF_MS, f.ir_off_ms], [C.IR_ON_MS, f.ir_on_ms], [C.IR_WIN_MS, f.ir_win_ms], [C.IR_LED_PINS, f.ir_led_pins]]) this.write(P.config(k, v));
  }

  drive(l, r) { this.seq = (this.seq + 1) & 0xff; this.write(P.drive(this.seq, l, r)); }
  servo(ch, us) { this.write(P.servo(ch, us)); }
  stop() { this.write(P.encode(P.T.STOP)); }
  clear() { this.write(P.encode(P.T.CLEAR)); }

  // Best effort on shutdown: stop now and wait for the bytes to leave.
  close() {
    return new Promise((res) => {
      if (!this.port) return res();
      this.port.write(P.drive(0, 0, 0));
      this.port.drain(() => this.port.close(() => res()));
    });
  }
}
