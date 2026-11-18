// Last word on every motion command (plan §9). Highest priority first; returns the command
// that is actually sent plus the reason. The ATmega's own fail-safes sit below all of this.

export function arbitrate(s, cfg) {
  const c = cfg.safety;
  const stop = (reason) => ({ v: 0, w: 0, reason });
  if (s.mode === 'stop') return stop('mode: stop');
  if (s.telemAgeMs > c.telemetry_stale_ms) return stop(`robot telemetry stale ${s.telemAgeMs.toFixed(0)} ms`);
  if (s.mcuFlags & 0x4c) return stop('ATmega stop (E-stop / battery / unarmed)'); // ESTOP | BATT_LOW | UNARMED handled there too
  if (s.temp >= c.temp_stop_c || s.throttled) return stop(`Pi too hot (${s.temp} °C) or throttled`);

  let { v, w } = s.mode === 'manual' ? s.manual : s.mode === 'ir' ? s.ir : s.brain;
  let reason = s.mode === 'manual' ? 'manual' : s.mode === 'ir' ? `IR drive: ${s.ir.note}` : 'brain';
  if (s.mode === 'manual' && s.manualAgeMs > 500) return stop('manual: dashboard silent');
  if (s.mode === 'auto') {
    if (s.frameAgeMs > c.camera_stale_ms) return stop(`camera stale ${s.frameAgeMs.toFixed(0)} ms`);
    if (s.yoloAgeMs > c.yolo_stale_ms) return stop(`YOLO stale ${s.yoloAgeMs.toFixed(0)} ms`);
  }
  if (v > 0) {
    if (s.rangeM < c.range_stop_m) return { v: 0, w, reason: `front ${(s.rangeM * 100).toFixed(0)} cm` };
    // always able to stop inside what the sensors see
    const vmax = Math.sqrt(2 * c.brake_mps2 * Math.max(0, s.rangeM - c.range_stop_m));
    if (v > vmax) { v = vmax; reason += ' (range-limited)'; }
  }
  if (v < 0 && s.rearM < c.rear_stop_m) return { v: 0, w, reason: `rear IR ${(s.rearM * 100).toFixed(0)} cm` };
  if (s.temp >= c.temp_slow_c) { v /= 2; w /= 2; reason += ' (hot: half speed)'; }
  return { v, w, reason };
}
