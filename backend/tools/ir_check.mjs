// IR proximity self-check: reads the robot's telemetry straight from the serial port (stop the
// backend first) and says, per sensor, what works and what to fix.
//   node tools/ir_check.mjs            auto-detects the Fire Bird's FTDI port
//   node tools/ir_check.mjs COM16 8    port and seconds
// While it runs, move a hand in and out ~3 cm in front of each sensor, the rear ones too.
import { pathToFileURL } from 'node:url';
import * as P from '../src/protocol.js';

const NAMES = ['1 left', '2 front-left', '3 front', '4 front-right', '5 right', '6 rear-right', '7 rear', '8 rear-left'];
const span = (a) => Math.max(...a) - Math.min(...a);
const sd = (a) => { const m = a.reduce((s, x) => s + x, 0) / a.length; return Math.sqrt(a.reduce((s, x) => s + (x - m) ** 2, 0) / a.length); };

// frames: decoded telemetry. Returns report lines; pure, so it is unit-tested.
export function diagnose(frames, bytes = 0) {
  if (!frames.length) {
    return [bytes ? `${bytes} bytes but no valid frames: wrong baud / crystal, or not this firmware`
      : 'NO DATA: robot switched off, jumper J1 not on USB, or the ATmega2560 is not running this firmware'];
  }
  const t = frames.at(-1), out = [`${frames.length} frames, firmware v${t.fw}${t.fw < 3 ? ' (old: flash atmega/firebird.hex)' : ''}`];
  const col = (f) => frames.map(f);

  // Rear 6-8: ATmega8 over SPI
  if (!t.proxRear) out.push('rear 6-8: this firmware does not read them (v1): flash atmega/firebird.hex');
  else {
    for (let i = 0; i < 3; i++) {
      const a = col((f) => f.proxRear[i]), name = NAMES[5 + i];
      if (a.every((v) => v === 0)) out.push(`${name}: always 0 -> SPI to the ATmega8 not connected: fit the three J4 jumpers on the adaptor board (NEX ships them open)`);
      else if (a.every((v) => v === 255)) out.push(`${name}: always 255 -> the ATmega8 does not answer: check all three J4 jumpers; if they are on, reload ATMEGA8.hex (kit CD, "GUI and Related Firmware")`);
      else if (span(a) <= 3) out.push(`${name}: stuck at ${a[0]} (x4 = ${a[0] * 4}) -> not reacting: was a hand moved in front of it? else a wiring / sensor fault`);
      else out.push(`${name}: OK, reacts (${Math.min(...a)}..${Math.max(...a)}, x4 = ${Math.min(...a) * 4}..${Math.max(...a) * 4})`);
    }
  }

  // Front 1-5: ATmega2560 ADC
  for (let i = 0; i < 5; i++) {
    const a = col((f) => f.prox[i]), name = NAMES[i];
    out.push(span(a) <= 3 ? `${name}: stuck at ${a[0]} -> not reacting (was a hand moved in front of it?)`
      : `${name}: OK, reacts (${Math.min(...a)}..${Math.max(...a)})`);
  }
  if (t.fw < 3) { // JTAG on PF4-7 makes IR 1-4 noisy compared with IR 5 (PK0)
    const n14 = Math.max(...[0, 1, 2, 3].map((i) => sd(col((f) => f.prox[i])))), n5 = sd(col((f) => f.prox[4]));
    if (n14 > 2 * n5 + 5) out.push(`IR 1-4 much noisier than IR 5 (sd ${n14.toFixed(0)} vs ${n5.toFixed(0)}): JTAG is on those pins - firmware v3 turns it off`);
  }

  // LED switching (reflection mode): needs the IRP jumper out
  const withDark = frames.filter((f) => f.proxDark);
  if (!withDark.length) out.push('LED switching: no ambient reading (LEDs kept on) - use raw mode');
  else {
    const eff = Math.max(...withDark.flatMap((f) => [0, 1, 2, 3, 4].map((i) => Math.abs(f.proxDark[i] - f.prox[i]))));
    out.push(eff < 6 ? `LED switching has no effect (max ${eff}): the IRP jumper (J1-2, main board) is in - use raw mode, or take it out for reflection mode`
      : `LED switching works (up to ${eff} counts): reflection mode available`);
  }
  return out;
}

async function main() {
  const { SerialPort } = await import('serialport');
  let [path, secs = '6'] = process.argv.slice(2);
  path ??= (await SerialPort.list()).find((p) => p.vendorId?.toLowerCase() === '0403')?.path;
  if (!path) { console.log('No Fire Bird FTDI port found: is the robot\'s USB cable plugged in?'); process.exit(1); }
  console.log(`Reading ${path} for ${secs} s - move a hand in and out ~3 cm in front of every sensor, rear ones too...`);
  const port = new SerialPort({ path, baudRate: 115200 }), parser = new P.Parser(), frames = [];
  let bytes = 0;
  port.on('error', (e) => { console.log(`Cannot open ${path}: ${e.message} (is the backend still running?)`); process.exit(1); });
  port.on('data', (d) => { bytes += d.length; for (const f of parser.push(d)) if (f.type === P.T.TELEM) frames.push(P.decodeTelem(f.payload)); });
  await new Promise((r) => setTimeout(r, Number(secs) * 1000));
  port.close();
  for (const line of diagnose(frames, bytes)) console.log(' -', line);
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) main();
