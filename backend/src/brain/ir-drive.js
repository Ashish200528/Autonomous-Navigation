// Camera-free driving on the 8 IR proximity sensors (+ the front Sharp if one is fitted), mode 'ir'.
// The IR sensors see ~2-12 cm, so every rule is in IR distances:
//   red   = closer than block_m: blocked (the dashboard shows exactly these in red)
//   amber = something seen, not red: slow down and steer away from it
//   clear = nothing seen
// CRUISE ahead while the front (2, 3, 4) is not red. Front red -> TURN in place toward the side
// (1-2 vs 4-5) that is more open, until the front has stayed non-red for free_s -> CRUISE.
// Both sides red too, or the turn takes over turn_timeout_s -> REVERSE (stops early if the rear
// 6-8 goes red), then turn the other way. Everything red -> BOXED: wait for an opening.
// tick() returns {v, w, state, note}; w > 0 turns left. Time in seconds.
import { nearestIr } from '../hw/sensors.js';

export class IrDriver {
  constructor(p) { this.p = p; this.reset(); }

  reset() { this.state = 'CRUISE'; this.t0 = 0; this.dir = 1; this.note = ''; this.freeSince = null; }

  to(state, now, note) { this.state = state; this.t0 = now; this.note = note; this.freeSince = null; }

  // ring: IrRing.update() output; sharpM: front Sharp range (Infinity = clear or no Sharp fitted)
  tick(ring, sharpM, now) {
    const p = this.p, ir = (...n) => nearestIr(ring, n), red = (d) => d < p.block_m;
    const out = (v, w, note) => ({ v, w, state: this.state, note: note ?? this.note });

    if (!ring) return out(0, 0, 'waiting for IR telemetry');
    // nearestIr reads "no data" as clear, so never drive with blind front sensors
    if ([2, 3, 4].some((n) => ring[n - 1].m == null)) return out(0, 0, 'front IR 2-4 give no reading');

    const front = ir(2, 3, 4), left = ir(1, 2), right = ir(4, 5), rear = ir(6, 7, 8);
    const frontRed = red(front) || sharpM < p.sharp_decide_m;
    const frontFree = !frontRed && !(sharpM < p.sharp_clear_m);
    const sidesRed = red(left) && red(right);
    const turnToOpen = (now, why) => { // toward the side that sees less (Infinity = nothing)
      this.dir = left === right ? this.dir : left > right ? 1 : -1;
      this.to('TURN', now, `${why} - turning ${this.dir > 0 ? 'left' : 'right'}`);
    };

    if (this.state === 'CRUISE') {
      if (frontRed) {
        if (sidesRed) this.to('REVERSE', now, `blocked ahead (${cm(front)}) and on both sides`);
        else turnToOpen(now, `blocked ahead (${cm(Math.min(front, sharpM))})`);
      } else {
        const seen = Number.isFinite(front) || sharpM < p.sharp_slow_m; // amber ahead: slow down
        const w = left < right ? -p.side_turn : right < left ? p.side_turn : 0; // away from the nearer side
        const note = Number.isFinite(front) ? `object ahead ${cm(front)} - slowing, steering away`
          : w ? `${w < 0 ? 'left' : 'right'} side ${cm(Math.min(left, right))} - steering away` : 'path clear - driving forward';
        return out(seen ? p.slow_speed : p.cruise_speed, w, note);
      }
    }

    if (this.state === 'TURN') {
      if (frontFree) {
        this.freeSince ??= now;
        if (now - this.freeSince >= p.free_s) { this.to('CRUISE', now, 'front clear - driving forward'); return out(0, 0); }
      } else this.freeSince = null;
      if (now - this.t0 > p.turn_timeout_s) this.to('REVERSE', now, 'turning did not clear the front');
      else return out(0, this.dir * p.turn_rate);
    }

    if (this.state === 'REVERSE') {
      if (red(rear)) {
        if (frontRed && sidesRed) { this.to('BOXED', now, `blocked on all sides (rear ${cm(rear)})`); return out(0, 0); }
        this.dir = -this.dir; this.to('TURN', now, `rear ${cm(rear)} - turning instead`);
        return out(0, this.dir * p.turn_rate);
      }
      if (now - this.t0 > p.reverse_s) { this.dir = -this.dir; this.to('TURN', now, 'backed up - turning'); return out(0, this.dir * p.turn_rate); }
      return out(-p.slow_speed, 0, ring[6].m == null ? 'backing up (rear IR has no reading)' : 'backing up');
    }

    // BOXED: wait until any way out opens
    if (frontFree) this.to('CRUISE', now, 'front opened');
    else if (!sidesRed) turnToOpen(now, 'a side opened');
    else if (!red(rear)) this.to('REVERSE', now, 'rear opened');
    return out(0, 0);
  }
}

const cm = (m) => (Number.isFinite(m) ? `${(m * 100).toFixed(0)} cm` : 'clear');
