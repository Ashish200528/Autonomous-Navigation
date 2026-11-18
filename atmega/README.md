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
- Jumper **J1** = USB (default), **J2** shorted (IR proximity 1–4 on ADC4–7).
- The three **J4** jumpers on the ATmega2560 adaptor board **on**: they join the SPI bus to the
  ATmega8, which reads IR proximity 6–8 (rear). NEX ships them **open**; remove them only while
  flashing over ISP, then put them back. Without them the rear sensors read 0 (shown as "no data").
- **IRP jumper (J1-2 on the main board) out** if you want room light cancelled ("reflection"
  mode). Robots ship with it in, which keeps the IR LEDs on for good, so the firmware's LED-off
  reading equals the LED-on one. With it in, use "raw" mode (LEDs on, like NEX's examples).
- JTAG is switched off at start-up (`JTD`), because IR proximity 1–4 share PF4–PF7 with the JTAG
  pins: with the JTAGEN fuse set (factory default) those four readings were random and IR3
  (TDO) read 5 V.

## IR proximity sensors (verified against NEX examples 10, 14_A, 14_B)

| Sensor | Read by | Channel | Bearing |
|---|---|---|---|
| 1 | ATmega2560 | ADC4 (PF4) | 90° left |
| 2 | ATmega2560 | ADC5 (PF5) | 45° front-left |
| 3 | ATmega2560 | ADC6 (PF6) | 0° front |
| 4 | ATmega2560 | ADC7 (PF7) | 45° front-right |
| 5 | ATmega2560 | ADC8 (PK0) | 90° right |
| 6 | ATmega8 via SPI | its ADC5 | 135° rear-right |
| 7 | ATmega8 via SPI | its ADC6 | 180° rear |
| 8 | ATmega8 via SPI | its ADC7 | 135° rear-left |

Sensors 1–5 are 10-bit; 6–8 come back 8-bit from the ATmega8 (the Pi scales them ×4).
The ATmega8 keeps NEX's default firmware; nothing on it needs flashing. The bearings are the
Fire Bird V layout and live in `backend/config/default.json` (`ir.sensors`) if yours differ.
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
