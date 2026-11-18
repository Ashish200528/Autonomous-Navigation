// Same vectors as atmega/test_protocol.c.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../src/protocol.js';

const DRIVE_FRAME = Buffer.from([0xaa, 0x55, 0x01, 0x05, 0x01, 0x64, 0x00, 0x9c, 0xff, 0xd3]);

test('crc8 check value', () => assert.equal(P.crc8(Buffer.from('123456789')), 0xf4));

test('drive frame matches the C encoder byte for byte', () => assert.deepEqual(P.drive(1, 100, -100), DRIVE_FRAME));

test('drive clamps pwm', () => assert.equal(P.drive(0, 999, -999).readInt16LE(5), 255));

test('parser resyncs after junk, rejects bad crc, handles split chunks', () => {
  const p = new P.Parser();
  const bad = Buffer.from(DRIVE_FRAME); bad[6] ^= 1;
  const all = Buffer.concat([Buffer.from([0x00, 0xaa, 0x13, 0x55]), DRIVE_FRAME, bad, DRIVE_FRAME]);
  const frames = [...p.push(all.subarray(0, 7)), ...p.push(all.subarray(7))];
  assert.equal(frames.length, 2);
  assert.equal(frames[0].type, P.T.DRIVE);
  assert.deepEqual(frames[0].payload, DRIVE_FRAME.subarray(4, 9));
  assert.ok(p.crcErrors >= 1);
});

test('telemetry decode (offsets match protocol.h)', () => {
  const b = Buffer.alloc(57);
  b.writeUInt32LE(123456, 0); b[4] = 7; b.writeUInt16LE(P.FLAGS.REFLEX | P.FLAGS.DWELL, 5);
  b.writeInt32LE(-42, 7); b.writeInt32LE(99, 11); b.writeUInt32LE(30000, 15); b.writeUInt32LE(0, 19);
  for (let i = 0; i < 5; i++) { b.writeUInt16LE(100 + i, 23 + 2 * i); b.writeUInt16LE(200 + i, 33 + 2 * i); }
  for (let i = 0; i < 3; i++) b.writeUInt16LE(300 + i, 43 + 2 * i);
  b.writeUInt16LE(10450, 49); b.writeUInt16LE(1500, 51);
  const [f] = new P.Parser().push(P.encode(P.T.TELEM, b));
  const t = P.decodeTelem(f.payload);
  assert.equal(t.tMs, 123456); assert.equal(t.seqAck, 7); assert.equal(t.flags, 18);
  assert.deepEqual(t.enc, [-42, 99]); assert.deepEqual(t.periodUs, [30000, 0]);
  assert.deepEqual(t.sharp, [100, 101, 102, 103, 104]); assert.equal(t.prox[4], 204);
  assert.deepEqual(t.line, [300, 301, 302]); assert.equal(t.battMv, 10450); assert.equal(t.servoUs[0], 1500);
});
