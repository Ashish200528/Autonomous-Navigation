# Fire Bird V ATmega2560 bridge firmware

The ATmega2560 is an I/O bridge for the Raspberry Pi: it applies motor and servo
commands and streams raw sensor readings. All decisions are made on the Pi
(`backend/`). Design: `backend/IMPLEMENTATION_PLAN.md` §5.

| File | What |
|---|---|
| `main.c` | registers, ISRs, fail-safes, 50 Hz telemetry |
| `protocol.c/.h` | frame format + CRC-8; mirror of `backend/src/protocol.js` |
| `test_protocol.c` | host check, same vectors as `backend/test/protocol.test.js` |

## Build and flash

```sh
sudo apt install gcc-avr avr-libc avrdude   # on the Pi (or any Linux / WSL)
make            # -> firebird.hex
make test       # host protocol check
```

Flashing uses the same USB port the Pi drives the robot through:

1. Stop the backend (`sudo systemctl stop anc-backend`) so the port is free.
2. Put the robot in bootloader mode the way you do now (boot switch + reset).
3. `make flash` — `PROGRAMMER=avr109` is a first guess. Phase 0 checks which
   avrdude mode the NEX bootloader speaks (`avr109`, then `stk500v2`). If neither
   works, load `firebird.hex` with the NEX AVR Bootloader GUI on Windows instead.
4. Reset the robot and start the backend again.

## Board settings

- Crystal **14.7456 MHz** (robots after Dec 2010). If yours is 11.0592 MHz, change
  `F_CPU`, `UBRR2` (5 for 115200), `TICKS_PER_MS` and the tick conversions.
- Jumper **J1** = USB (default), **J2** on (IR proximity on the ADC), **J4** off.
- Pad **P1** open (default), so PE7 is the on-board button, used as E-stop.

## Fail-safes (all stop-only)

| Rule | Default | Changed by |
|---|---|---|
| No valid `DRIVE` frame | stop after 250 ms | `CONFIG 1` |
| E-stop button (PE7) or `STOP` frame | latched until `CLEAR` | — |
| Battery below cutoff for 2 s | 8000 mV, latched | `CONFIG 6` (0 = off) |
| Front reflex: Sharp ≥ threshold or front IR proximity hit | forward part removed, turning/reversing allowed | `CONFIG 2–5` |
| Direction reversal | wheel held at 0 for 300 ms first | `CONFIG 7` |
| Firmware hang | 60 ms hardware watchdog reset, motors off until next `DRIVE` | — |
