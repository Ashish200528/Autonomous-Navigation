// Camera-free driving on the 8 IR proximity sensors + the front Sharp (mode 'ir').
// Reactive: CRUISE ahead, TURN in place toward the open side when the front is blocked,
// REVERSE a little when turning does not free it, BOXED when every side is blocked.
// tick() returns {v, w, state, note}; w > 0 turns left. Time in seconds.
//
// Sensor numbers (Fire Bird V): 1 left, 2 front-left, 3 front, 4 front-right, 5 right,
// 6 rear-right, 7 rear, 8 rear-left. Distances in metres, Infinity = clear.
import { nearestIr } from '../hw/sensors.js';

export class IrDriver {
  constructor(p) { this.p = p; this.reset(); }

  reset() { this.state = 'CRUISE'; this.t0 = 0; this.dir = 1; this.note = ''; }

  to(state, now, note) { this.state = state; this.t0 = now; this.note = note; }

  // ring: IrRing.update() output; sharpM: front Sharp range (Infinity = clear or no Sharp)
  tick(ring, sharpM, now) {
    const p = this.p, ir = (...n) => nearestIr(ring, n);
    const front = Math.min(sharpM, ir(2, 3, 4));
    const left = ir(1, 2), right = ir(4, 5), rear = ir(6, 7, 8);
    const rearKnown = ring?.[6]?.m != null;
    const sideOpen = (d) => d > p.side_block_m;
    const out = (v, w, note) => ({ v, w, state: this.state, note: note ?? this.note });

    if (!ring) return out(0, 0, 'waiting for IR telemetry');
    // nearestIr reads "no distance" as clear, so never drive with blind front sensors
    if ([2, 3, 4].some((n) => ring[n - 1].m == null)) return out(0, 0, 'front IR 2-4 give no distance: flash firmware v2 / calibrate');

    if (this.state === 'CRUISE') {
      if (front < p.decide_m) {
        if (!sideOpen(left) && !sideOpen(right)) this.to('REVERSE', now, `front ${cm(front)} and both sides blocked`);
        else {
          this.dir = left === right ? this.dir : left > right ? 1 : -1;
          this.to('TURN', now, `front ${cm(front)} - turning ${this.dir > 0 ? 'left' : 'right'}`);
        }
      } else {
        const v = front < p.slow_m ? p.slow_speed : p.cruise_speed;
        const l = ir(1), r = ir(5); // keep off side walls the 90° sensors see
        const w = l < r ? -p.side_turn : r < l ? p.side_turn : 0;
        return out(v, w, w ? `side wall ${cm(Math.min(l, r))} - steering away` : front < p.slow_m ? `front ${cm(front)} - slowing` : 'cruising');
      }
    }

    if (this.state === 'TURN') {
      if (front > p.clear_m) { this.to('CRUISE', now, 'path clear'); return out(0, 0); }
      if (now - this.t0 > p.turn_timeout_s) this.to('REVERSE', now, 'turn did not clear the front');
      else return out(0, this.dir * p.turn_rate);
    }

    if (this.state === 'REVERSE') {
      if (rear < p.rear_stop_m) {
        if (front < p.decide_m && !sideOpen(left) && !sideOpen(right)) { this.to('BOXED', now, `boxed in (rear ${cm(rear)})`); return out(0, 0); }
        this.dir = -this.dir; this.to('TURN', now, `rear ${cm(rear)} - turning instead`);
        return out(0, this.dir * p.turn_rate);
      }
      if (now - this.t0 > p.reverse_s) { this.dir = -this.dir; this.to('TURN', now, 'backed up - turning'); return out(0, this.dir * p.turn_rate); }
      return out(-p.slow_speed, 0, rearKnown ? 'backing up' : 'backing up (rear IR has no data)');
    }

    // BOXED: wait until any way out opens
    if (front > p.clear_m) this.to('CRUISE', now, 'front opened');
    else if (sideOpen(left) || sideOpen(right)) this.to('TURN', now, 'a side opened');
    else if (rear > p.rear_stop_m * 2) this.to('REVERSE', now, 'rear opened');
    return out(0, 0);
  }
}

const cm = (m) => (Number.isFinite(m) ? `${(m * 100).toFixed(0)} cm` : 'clear');
