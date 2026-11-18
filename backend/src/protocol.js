// Pi <-> ATmega2560 frame protocol. Mirror of atmega/protocol.c (same test vectors).
// Frame: 0xAA 0x55 | type | len | payload | crc8(type, len, payload), little-endian.

export const T = { DRIVE: 0x01, SERVO: 0x02, CONFIG: 0x03, STOP: 0x04, CLEAR: 0x05, BOOT: 0x80, TELEM: 0x81 };
export const CFG = { TIMEOUT_MS: 1, REFLEX_SHARP_ADC: 2, REFLEX_SHARP_CH: 3, REFLEX_PROX_MASK: 4,
  REFLEX_PROX_ADC: 5, BATT_CUTOFF_MV: 6, DWELL_MS: 7, IR_OFF_MS: 8, IR_ON_MS: 9, IR_WIN_MS: 10, IR_LED_PINS: 11 };
export const FLAGS = { TIMEOUT: 1, REFLEX: 2, ESTOP: 4, BATT_LOW: 8, DWELL: 16, CRC_ERR: 32, UNARMED: 64 };
const MAX_PAYLOAD = 96;

export function crc8(bytes, crc = 0) {
  for (const b of bytes) {
    crc ^= b;
    for (let i = 0; i < 8; i++) crc = crc & 0x80 ? ((crc << 1) ^ 0x07) & 0xff : (crc << 1) & 0xff;
  }
  return crc;
}

export function encode(type, payload = Buffer.alloc(0)) {
  const f = Buffer.alloc(payload.length + 5);
  f[0] = 0xaa; f[1] = 0x55; f[2] = type; f[3] = payload.length;
  payload.copy(f, 4);
  f[f.length - 1] = crc8(f.subarray(2, f.length - 1));
  return f;
}

export const drive = (seq, l, r) => {
  const p = Buffer.alloc(5);
  p[0] = seq & 0xff; p.writeInt16LE(clampPwm(l), 1); p.writeInt16LE(clampPwm(r), 3);
  return encode(T.DRIVE, p);
};
export const servo = (ch, us) => { const p = Buffer.alloc(3); p[0] = ch; p.writeUInt16LE(Math.round(us), 1); return encode(T.SERVO, p); };
export const config = (key, value) => { const p = Buffer.alloc(3); p[0] = key; p.writeUInt16LE(value & 0xffff, 1); return encode(T.CONFIG, p); };
const clampPwm = (v) => Math.max(-255, Math.min(255, Math.round(v)));

// Byte-stream parser; returns complete valid frames, counts CRC errors.
export class Parser {
  constructor() { this.buf = Buffer.alloc(0); this.crcErrors = 0; }
  push(chunk) {
    let b = this.buf.length ? Buffer.concat([this.buf, chunk]) : chunk;
    const out = [];
    let i = 0;
    while (b.length - i >= 5) {
      if (b[i] !== 0xaa || b[i + 1] !== 0x55) { i++; continue; }
      const len = b[i + 3];
      if (len > MAX_PAYLOAD) { this.crcErrors++; i++; continue; }
      if (b.length - i < len + 5) break;
      const body = b.subarray(i + 2, i + 4 + len);
      if (crc8(body) === b[i + 4 + len]) {
        out.push({ type: b[i + 2], payload: Buffer.from(b.subarray(i + 4, i + 4 + len)) });
        i += len + 5;
      } else { this.crcErrors++; i++; }
    }
    this.buf = Buffer.from(b.subarray(i));
    return out;
  }
}

export function decodeTelem(p) {
  const u16s = (off, n) => Array.from({ length: n }, (_, k) => p.readUInt16LE(off + 2 * k));
  return {
    tMs: p.readUInt32LE(0), seqAck: p[4], flags: p.readUInt16LE(5),
    enc: [p.readInt32LE(7), p.readInt32LE(11)], periodUs: [p.readUInt32LE(15), p.readUInt32LE(19)],
    sharp: u16s(23, 5), prox: u16s(33, 5), line: u16s(43, 3), battMv: p.readUInt16LE(49), servoUs: u16s(51, 3),
    // IR proximity, firmware v2+: prox/proxRear are read with the IR LEDs on, proxDark (1-8) with
    // them off (ambient light). 6-8 come from the ATmega8 (8-bit). v1: prox only, ambient included.
    // v3 sends proxDark 0xFFFF when it keeps the LEDs on (ir_off_ms 0): no ambient reading.
    proxRear: p.length >= 63 ? u16s(57, 3) : null,
    proxDark: p.length >= 79 && p.readUInt16LE(63) !== 0xffff ? u16s(63, 8) : null,
    fw: p.length >= 80 ? p[79] : p.length >= 63 ? 2 : 1, // v3+ sends it; older ones by length
  };
}

export const decodeBoot = (p) => ({ version: p[0], mcusr: p[1], fCpuKhz: p.readUInt16LE(2) });
