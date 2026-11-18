# Fire Bird V + Raspberry Pi 4B Backend — Implementation Plan

Status: **all components written** (`atmega/` firmware, `backend/` Pi side: drivers, YOLO, distance + depth optimization, ArUco, dense depth, pseudo-LiDAR, brain, safety, dashboard, recording/replay, tools; 40 unit tests passing on a PC). Not yet run on the robot: Phases 0–9 are the bench and track work.
What exists and what is still to build: `backend/README.md`. Every number marked *(est.)* is an estimate
that Phase 0 replaces with a measurement. Every number marked *(manual)* comes
from the *Fire Bird V ATMEGA2560 Hardware Manual V1.08 (2012-10-12)* and NEX
Robotics example code, and still gets checked on this robot.

## 1. Goal and split of responsibilities

The car is now a **Fire Bird V ATMEGA2560** robot (NEX Robotics / IIT Bombay)
with a **Raspberry Pi 4B** mounted on top.

| Board | Job | Does **not** do |
|---|---|---|
| **Raspberry Pi 4B** (Node.js, `backend/`) | All intelligence: camera, YOLO, depth, distance fusion, pseudo-LiDAR, odometry, wheel-speed control, state machine, dashboard | Touch any motor/sensor pin directly (no GPIO, no pigpio) |
| **ATmega2560** (C firmware, `atmega/`) | Executes commands from the Pi (motor PWM + direction, servo angle, sensor power) and streams raw sensor readings (encoders, Sharp IR, IR proximity, white line, battery) | ML, path decisions, speed control, state machine |

The ATmega2560 is an **I/O bridge**. The only logic it keeps is fail-safe that
must work even if the Pi hangs, crashes or the USB cable falls out (§5.3):
command timeout ⇒ stop, battery cutoff, hardware watchdog, and an optional
front-range reflex. These never *choose* where to go; they only stop.

The Pi:

1. Detects objects with YOLO and estimates **metric distance from the camera**,
   using the same known-height method as `object_detection_new_babu.py`, improved
   ("depth optimization") by floor geometry and dense monocular depth, and
   self-calibrated against the robot's Sharp IR range sensor (§7.2).
2. Builds a **LiDAR-like 3D point cloud** ("pseudo-LiDAR") from the camera swept
   by the pan servo, plus a 2D obstacle map and a virtual laser scan. **Wheel
   encoders now exist**, so the map is kept while driving.
3. Drives with the decision logic proven in simulation
   (`simulation/ros2_ws/src/anc_autodrive`), ported to JavaScript, and fails safe.
4. Serves a live dashboard over Wi-Fi.

The ROS 2 / Gazebo simulation stays the place to try logic changes first.

### What "perfect" means here (honest scope)

- **Every decision is explainable**: each command carries its reason and inputs.
- **Uncertainty is tracked**: low confidence ⇒ slower or stop, never "guess and go".
- **Fail-closed at two levels**: the Pi stops on stale or conflicting data; the
  ATmega stops on its own if the Pi goes quiet.
- **Measured, not assumed**: every phase ends with a measured acceptance test.

## 2. Hardware

### 2.1 Fire Bird V ATMEGA2560 *(manual)*

| Item | Spec | Notes for us |
|---|---|---|
| Master MCU | ATmega2560 @ **14.7456 MHz** (robots shipped after 2010-12-01; older: 11.0592 MHz) | Phase 0 confirms the crystal; the UART baud divisor depends on it |
| Slave MCU | ATmega8 on SPI (extra ADCs: IR prox 6–8, white line 4–7) | Not needed at first (§14) |
| Drive | 2 × 75 RPM DC geared motors, differential + front caster | Top speed **≈ 24 cm/s** |
| Motor driver | On-board **L293D**, 600 mA/channel | Already wired; nothing to add |
| Wheels | Ø 51 mm, track width ≈ 15 cm | Calibrated in Phase 1 |
| Encoders | MOC7811 slotted disc, **30 pulses/rev**, ≈ 5.44 mm/pulse, ≈ 4.09°/pulse in spin turn | **Single channel: count only, no direction.** Direction comes from the commanded sign |
| Range | Sharp IR range sensors (5 sockets; **1 front sensor fitted**). NEX's formula labelled "GP2Y0A02YK" actually fits the 10–80 cm GP2D12 curve; the real model is read off the sensor in Phase 0 | Non-linear, has a blind zone close in (§7.2) |
| Proximity | 8 analog IR proximity sensors, ≈ 10 cm range, around the body | Covers the Sharp blind zone |
| Floor | 3 white-line sensors (left / centre / right) | Optional line/stop-line cue |
| Servo | 3 servo outputs (S1–S3) on Timer1, own "5V servo" regulator, **800 mA max** | Pan/tilt camera pod |
| Power | 9.6 V NiMH 2.1 Ah (12 V full → 8 V empty), **2 A max draw**, on-board smart charger | Never discharge below 8 V |
| Size / weight | Ø 16 cm, 8.5 cm high, 1.1 kg | Pi + webcam + power bank go on the top plate |
| Comms | USB (FT232 on **UART2**), RS232 DB9 (UART1), XBee socket (UART0), UART3 on the expansion socket | **Pi uses the USB port** |
| Extras | 2×16 LCD, buzzer, LED bargraph (PORTJ), interrupt switch (PE7) | LCD/buzzer show status; PE7 = on-board E-stop |

### 2.2 Parts on the Pi side

| Part | Status |
|---|---|
| Raspberry Pi 4B, **4 GB RAM**, 32 GB SD, Pi OS **Lite** Bookworm 64-bit | confirmed: full plan including dense depth; backend targets < 600 MB (§10) |
| FINGERS 1080 USB webcam → Pi USB 3 port | confirmed |
| Heatsink + fan on the Pi | required |
| HC-SR04, MPU6500 | **not used** (your decision). Range comes from the Fire Bird's IR sensors; heading from the encoders |
| L293D module, separate DC motors, PP3 batteries | **retired**. The Fire Bird has its own driver, motors and NiMH pack |

**Fire Bird sensors confirmed:** the 8 IR proximity sensors around the body and a
**Sharp IR range sensor**. Phase 0 confirms its socket/ADC channel (front =
ADC11) and model, since range depends on it.

### 2.3 Connections

```
 Pi 4B USB3 ── webcam
 Pi 4B USB2 ── USB cable ── Fire Bird USB port ── FT232 ── ATmega2560 UART2 (115200 8N1)
 Pi 4B USB-C ── 5 V / 3 A USB power bank (Pi's own supply)
 Camera on servo pod: pan = S1, tilt = S2 (check, see below)
```

- **One USB port does both jobs: programming and running.** The Pi connects to
  the same USB port you already use to load code onto the ATmega (on-board FT232
  → UART2, used by the NEX bootloader). The ATmega runs at 5 V and the Pi at
  3.3 V; the FT232 avoids any level shifting and shares ground through the cable.
  It shows up as `/dev/ttyUSB0` (udev rule → `/dev/firebird`).
- **Flashing from the Pi:** the bootloader and our firmware share UART2, so they
  never run at the same time. To flash (`atmega/README.md`): stop the backend so it releases the port → robot
  enters the bootloader (reset/power-cycle with the boot switch, as you do now) →
  upload → reset → backend reconnects and waits for `BOOT`. Phase 0 checks whether
  `avrdude` speaks the NEX bootloader's protocol (try `-c avr109`, then
  `-c stk500v2`). If neither works, keep flashing from the Windows PC with the NEX
  bootloader GUI and plug the cable into the Pi afterwards; nothing else changes.
- Jumper **J1** on the microcontroller board must be in the USB position (factory
  default). **J2 on** (IR proximity 1–4 on the ADC, default). **J3** (bargraph)
  any. **J4 off** (ATmega8 not used).
- **Power the Pi separately** (USB power bank, 5 V 3 A). The robot's 5 V rail only
  has 400 mA spare and the battery is limited to 2 A total; a Pi 4 + webcam
  (≈ 1–1.5 A at 5 V *(est.)*) on the robot battery would roughly halve run time and
  brown out the ATmega when both motors start. The USB cable gives a common ground.
- **Servo channel check:** the servo-pod note and example code use S1 (OC1A, PB5)
  for **pan**, while the camera-pod note says OC1A is **tilt**. Phase 1 sweeps
  each channel and records which is which in `config/default.json`.
- The webcam is heavier than the Sharp sensor the pod was designed for. Phase 1
  checks the pod servos hold it without jitter at the 800 mA servo-regulator limit;
  move one servo at a time (NEX's own advice).

## 3. Architecture

```
 ┌──────────────────────────── Raspberry Pi 4B — Node.js ───────────────────────────────┐
 │ camera.js (ffmpeg) ──frame──▶ [worker] yolo.worker.js   [nice'd process] depth.js      │
 │                                    │ detections               │ depth map              │
 │ firebird.js ◀──telemetry 50 Hz──┐  ▼                          ▼                        │
 │   (serial, framed, CRC)         │ fusion/distance.js · depth_scale.js · map3d.js      │
 │        │                        │ fusion/odometry.js (encoders)                         │
 │        │                        ▼                                                     │
 │        │      brain/state-machine.js ──▶ safety/arbiter.js ──▶ control/wheels.js (PI)  │
 │        ◀──────────── DRIVE(pwmL, pwmR) · SERVO(pan, tilt) · CONFIG 20–50 Hz ──────────┘ │
 │   server/ (HTTP + WS + MJPEG) ◀── state, cloud, video                                    │
 └────────────────────────────────────────┬──────────────────────────────────────────────┘
                                          │ USB (FT232) 115200
 ┌────────────────────────────── ATmega2560 firmware ─────────────────────────────────────┐
 │ UART2 frame parser ─▶ apply PWM (Timer5 OC5A/B) + direction (PA0–PA3), servos (Timer1)  │
 │ INT4/INT5 encoder ISRs (count + edge timestamp) · ADC scan · battery · E-stop (PE7)      │
 │ fail-safes: command timeout · AVR watchdog · battery cutoff · optional front reflex      │
 └────────────────────────────────────────────────────────────────────────────────────────┘
```

Rules:

- **Heavy compute never runs on the main thread.** YOLO runs in a
  `worker_thread`; depth runs in a separate low-priority process. Latest frame
  wins, old frames are dropped (§10).
- **Pi control loop runs at a fixed 20 Hz** on the latest fused state and its age.
- **Every message is timestamped.** Telemetry carries the ATmega's millisecond
  clock; the Pi maps it to its own clock, so encoder counts, servo angle and camera
  frames line up in time.

### Folder layout

```
atmega/                          # ATmega2560 bridge firmware (avr-gcc, plain C) — own folder at repo root
  main.c  protocol.c  protocol.h  Makefile (build, flash, host test)  README.md
  test_protocol.c                # host-compiled check: same test vectors as the JS side
backend/
  IMPLEMENTATION_PLAN.md  README.md  package.json
  config/default.json            # channels, calibration, thresholds: every tuning knob
  config/local.json              # dashboard changes (git-ignored); config/secret = drive token
  models/                        # exported .onnx + class names .json (onnx git-ignored)
  src/
    index.js                     # boot, wiring, 20 Hz control loop, shutdown (motors stop on exit)
    config.js  protocol.js       # protocol.js mirrors atmega/protocol.c
    hw/firebird.js  camera.js  sensors.js     # sensors.js: Sharp formula, blind-zone guard, IR prox
    perception/preprocess.js  yolo.worker.js  yolo_post.js  tracker.js  lanes.js
               mono_distance.js  # 1:1 port of object_detection_new_babu.py distance logic
               aruco.js  aruco.worker.js     # js-aruco2 DICT_7X7_1000 + POSIT pose, own worker
               depth.js                      # MiDaS small in its own nice'd process
    fusion/geometry.js                       # camera/robot/world transforms shared by everything below
           distance.js  odometry.js          # distance = cues A+B+C fused, median, Sharp self-calibration
           depth_scale.js                    # RANSAC 1/Z = s·d + t against floor / Sharp / object anchors
           map3d.js                          # voxel map, fading, cloud, 1° virtual laser scan
    control/wheels.js            # per-wheel speed PI using encoder telemetry
    brain/state-machine.js       # port of anc_autodrive brain_node.py + logic.py
    safety/arbiter.js
    server/http.js               # HTTP + MJPEG + WebSocket in one file
    record/recorder.js  replay.js  # replay: re-run the current brain on a recorded drive
  public/index.html              # dashboard: video + boxes, objects, 3D map (plain canvas, no library), controls
  tools/export_yolo.py           # PC only: Ultralytics → ONNX 256×320, simplify, INT8 static quant (§10.1)
  tools/export_depth.py          # MiDaS v2.1 small ONNX → INT8 (17 MB)
  tools/benchmark.js             # Phase 0: every model × 1-4 threads → ms, Hz, RSS, °C table
  tools/calibrate_camera.py      # Phase 2: checkerboard → camera.intrinsics in config/local.json
  tools/compare_python.mjs       # Phase 3: Pi pipeline vs Ultralytics on the same frames
  scripts/install-pi.sh  99-firebird.rules  anc-backend.service
  test/*.test.js                 # node:test (npm test)
```

## 4. Software stack

| Need | Choice | Why |
|---|---|---|
| Pi runtime | Node.js 22 LTS (arm64) | worker threads, built-in test runner |
| Serial to ATmega | `serialport` | maintained, arm64 prebuilds, proper termios |
| Camera | `ffmpeg` (apt) child process, V4L2 | any USB webcam, no native addon |
| Inference | `onnxruntime-node` | official arm64, INT8 models |
| Dashboard JPEG | `sharp` | prebuilt arm64 |
| WebSocket | `ws` | binary point clouds |
| Firmware | `avr-gcc` + `avr-libc` (apt `gcc-avr avr-libc avrdude`) | builds on the Pi or PC; the 2009 WinAVR on the Drive is not needed |
| Flashing | NEX USB bootloader over the same USB port (from the Pi with `avrdude` if compatible, else the NEX Windows GUI) | no ISP programmer, no jumper changes |

Dropped compared with the old plan: `pigpio`, GPIO wiring, root service, HC-SR04
divider, servo/motor drivers on the Pi.

ArUco: markers are `DICT_7X7_1000`, 0.10 m. Phase 0 checks a JS detector supports
it (`js-aruco2` custom dictionary or `@techstark/opencv-js`); otherwise reprint.

## 5. ATmega2560 bridge firmware (`atmega/` at the repo root)

Written fresh in C. NEX's example code (cc by-nc-sa) is used only as a register
reference. Pin facts below are *(manual)*.

### 5.1 What it drives and reads

| Function | Pins / peripheral | Setup |
|---|---|---|
| Left / right motor PWM | PL3 (OC5A) / PL4 (OC5B) | Timer5 8-bit fast PWM, prescaler 64 ⇒ ≈ 900 Hz |
| Motor direction | PA0 L-back, PA1 L-fwd, PA2 R-fwd, PA3 R-back | forward = `0x06`, reverse = `0x09`, stop = `0x00` |
| Encoders | PE4 / INT4 (left), PE5 / INT5 (right) | falling edge; ISR increments a **signed** count (sign = current commanded direction) and stores the edge time from a free-running timer, so the Pi gets speed from pulse **period**, not just count (only ≈ 37 pulses/s at top speed) |
| Servos S1/S2/S3 | PB5 OC1A / PB6 OC1B / PB7 OC1C | Timer1 **mode 14, prescaler 8, ICR1 = 36864 ⇒ 20 ms, 0.54 µs steps** (NEX example uses 1.86° steps; too coarse for pointing a camera) |
| Sharp IR 1–5 | ADC9–13 (PK1–PK5); front = sensor 3 (ADC11) | 10-bit; power switch PH3 (if jumper J1-3 removed) |
| IR proximity 1–5 | ADC4–7 (PF4–PF7, needs J2 on), ADC8 (PK0) | power switch PH2 |
| White line L/C/R | ADC3 / ADC2 / ADC1 | LED power PG2 |
| Battery | ADC0 (PF0) | `V = 0.7 + adc10 × 5/1024 × 13.3/3.3` |
| Servo-pod ADC | ADC14 / ADC15, GPIO PB4 | for a Sharp/EZ4 mounted on the pod (optional) |
| E-stop | PE7 / INT7 interrupt switch (needs pad P1 open, default) | latching stop until the Pi sends CLEAR. This is also the bootloader switch, but the bootloader only looks at it during reset, so at run time it is free |
| Buzzer / LCD | PC3 / LCD port | status only (link up, E-stop, battery low) |

The ADC uses the **external AREF pin** (REFS = 00; the board ties AREF to "5V uC" via pad SJ1, so the internal references must not be selected). It scans all used channels continuously in a free-running interrupt loop and
keeps the latest value per channel. No servo or motor change happens inside the
scan (NEX notes servo movement disturbs ADC readings; the Pi ignores ranges taken
while a servo is moving).

### 5.2 Protocol (USB serial, 115 200 8N1; UBRR = 7 at 14.7456 MHz ⇒ 0 % error)

Binary frames: `0xAA 0x55 | type | len | payload | CRC-8`. Little-endian.

| Dir | Type | Payload | Rate |
|---|---|---|---|
| Pi → MCU | `DRIVE` 0x01 | `seq u8, pwm_l i16, pwm_r i16` (−255…255, sign = direction) | 20–50 Hz |
| Pi → MCU | `SERVO` 0x02 | `ch u8, pulse_us u16` | on change |
| Pi → MCU | `CONFIG` 0x03 | `key u8, value i16` (timeout, reflex threshold, sensor power, …) | on change |
| Pi → MCU | `STOP` 0x04 / `CLEAR` 0x05 | — | on demand |
| MCU → Pi | `BOOT` 0x80 | firmware version, reset cause (`MCUSR`), crystal | once at boot |
| MCU → Pi | `TELEM` 0x81 | `t_ms u32, seq_ack u8, enc_l i32, enc_r i32, period_l_us u32, period_r_us u32, sharp[5] u16, prox[5] u16, line[3] u16, batt_mv u16, servo_us[2] u16, flags u16` | 50 Hz |

`flags`: timeout-stop, reflex-stop, E-stop, battery-low, reversal-dwell, ADC-stale.
Bad CRC ⇒ frame dropped and counted; the Pi shows the error rate.

On the Pi, set the FTDI latency timer to 1 ms
(`/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`, via udev rule) or every
read waits up to 16 ms.

### 5.3 Fail-safes on the ATmega (independent of the Pi)

1. **Command timeout:** no valid `DRIVE` for **250 ms** ⇒ PWM 0, direction 0,
   `timeout-stop` flag. Covers Pi crash, Node crash, unplugged cable.
2. **AVR hardware watchdog** (60 ms). A firmware hang resets the MCU; at boot
   the motors are off until the first valid `DRIVE`.
3. **Battery cutoff:** < 8.0 V for 2 s ⇒ stop + buzzer (NiMH must not deep-discharge).
4. **Reversal dwell:** a sign change on a wheel goes through ≥ 0.3 s at zero (NEX
   recommends 0.5 s: current surge resets the MCU, especially on aux power). This
   also keeps the single-channel encoder sign correct.
5. **E-stop** on the PE7 button: latched until `CLEAR`.
6. **Optional front reflex** (on by default, `CONFIG` can turn it off): front Sharp
   closer than a threshold, or any front IR proximity sensor triggered ⇒ the
   *forward* part of the command is zeroed (turning in place and reversing still
   allowed). It is a last-resort bumper, not a decision. The threshold is sent by
   the Pi from its Sharp calibration.

Phase 1 also checks that the motors do not twitch while the ATmega resets or sits
in the bootloader (the L293D inputs float during that time).

## 6. Pi-side drivers and control

| Module | Job | Output |
|---|---|---|
| `hw/camera.js` | `ffmpeg -f v4l2 … -video_size 640x480 -framerate 30 -f rawvideo -pix_fmt yuv420p -`, cut into fixed frames, restart on exit, frame-age watchdog, lock exposure/focus if possible | `{t, w, h, yuv}` |
| `hw/firebird.js` | open `/dev/firebird`, parse frames, CRC, map MCU clock → Pi clock, convert raw ADC → metres/volts with calibration tables, send `DRIVE` every control tick (that is also the keep-alive), reconnect on USB loss | `{t, enc, speed, sharp_m[], prox[], line[], batt_v, servo, flags}` |
| `control/wheels.js` | per-wheel PI: target speed (m/s) → PWM, with feed-forward from the measured PWM→speed table, deadband, per-side gain, ramp limit | `pwm_l, pwm_r` |
| `fusion/odometry.js` | differential-drive odometry from encoders (no IMU); heading uncertainty grows with distance and turning, and is corrected by lane lines and ArUco when visible | `{t, x, y, yaw, v, w, sigma}` |

Why the speed loop is on the Pi: it keeps the firmware an I/O bridge, and at
≈ 37 pulses/s the loop gains nothing from running faster than 20–50 Hz. The edge
period in telemetry gives a usable speed even at < 1 pulse per tick.

Calibration knobs (`config/default.json`, editable live from the dashboard):
mm per encoder pulse (left/right), track width, PWM→speed table, deadband, servo µs
range and trim per channel, Sharp ADC→distance table, IR proximity thresholds,
camera height/tilt/mount offsets, camera intrinsics.

## 7. Perception

**This is a computer-vision project: the camera + YOLO decide.** Every object the
brain reacts to is a YOLO detection with a camera-estimated distance. The Sharp
and IR proximity sensors never replace that. They do two things: **calibrate**
the camera distance (the "depth optimization" loop, §7.2 step 4) and act as a
**safety veto** close in (§9). The dashboard shows each object's camera
distance, and the Sharp reading separately, so the vision result is always visible.

### 7.1 YOLO object detection (`yolo.worker.js`)

- **Same model family as `object_detection_new_babu.py`** (`yolov8n.pt` /
  `yolo26n.pt` / YOLO11n), exported on the PC to a small ONNX file (§10.1).
  **Ultralytics and PyTorch are never installed on the Pi.** The Pi runs only the
  exported file with `onnxruntime-node`: same weights, same classes. The Python
  script stays the reference: `tools/compare_python.mjs` runs both on the same
  recorded frames; boxes must match within 2 px (FP32) / 4 px (INT8).
- Same settings as the Python script: confidence ≥ **0.45**, NMS IoU **0.50**,
  `max_det` **30**, per-class NMS (no NMS needed if YOLO26n is chosen; it is
  NMS-free).
- YUV → RGB + letterbox in one pass into a **preallocated** input buffer; boxes
  mapped back to 640×480 and **clamped to the frame** (as the script does, so a
  bad box cannot produce an impossible near distance).
- Speed and memory settings: §10.
- Later: fine-tune with your own classes (cone, barrier, toy car, sign) using the
  `docs/academic/phase9*` data; each new class gets a row in the height table.

### 7.2 Distance estimation with depth optimization (`fusion/distance.js`)

**Step 1 — your method, ported 1:1 from `object_detection_new_babu.py`**
(`perception/mono_distance.js`, with JS tests using the same numbers as the Python):

| Python | JS port |
|---|---|
| `estimate_obstacle_distance`: `Z = H_class · f / h_px` | same formula, `KNOWN_HEIGHTS_M` table moved to `config/default.json` (editable live) |
| `DEFAULT_OBSTACLE_HEIGHT_M = 0.50` | same default |
| `focal_length_from_fov(w, hfov)` with `CAMERA_HFOV_DEG = 90` | used only until Phase 2 calibration; then `f_y` from `camera_intrinsics.json` |
| `--calibrate CLASS DIST`: `f = h_px · D / H` | `POST /api/calibrate {class, distance_m}` + a dashboard button, same formula |
| `is_truncated(y1, y2, h)` | same: truncated boxes get no height-based distance |
| median of last 5 per `class:x_center//80` | same 5-sample median, keyed on the tracker ID (more stable than the x-bucket) |
| `get_horizontal_zone`: left / centre / right thirds | same, plus the servo angle so zones are relative to the **robot**, not the image |
| `get_obstacle_status`: FLAGGED ≤ 0.30 m, WARNING ≤ 1.20 m, SAFE | same thresholds and colours on the dashboard |
| ArUco: `solvePnP(IPPE_SQUARE)` with 0.10 m marker | same in `aruco.js` |

**Step 2 — more camera cues for the same box.** Your known-height formula is
wrong when the real object is a different size from the table (a toy car vs a
real car). The camera gives two more independent estimates for the same box:

| # | Cue | Formula | Good at | Weak at |
|---|---|---|---|---|
| A | **Floor contact** | `Z = h_cam / tan(tilt + atan((v_b − c_y) / f_y))` | anything standing on the floor, any class, size-independent, < 2 m | box bottom hidden / cut off |
| B | **Known height** (your method, step 1) | `Z = f_y · H_class / h_px` | far objects, bottom hidden | object size differs from table |
| C | **Dense depth** (§7.3) | median metric depth in the lower centre of the box | odd shapes, unknown classes | slow (0.5–2 Hz) |

**Step 3 — fuse per object.** Inverse-variance weighting per detection combines
A, B, C, each with its own uncertainty (grows with distance, shrinks with YOLO
confidence; A is dropped when the box touches the bottom edge, B gets 3× the
uncertainty when the box is truncated). Cues disagreeing by > 3σ ⇒
`inconsistent`, the brain uses the **nearest** one. The result is smoothed by
your script's 5-sample median per tracked object. Output per object:
`{id, class, conf, zone, distance_m, sigma_m, status, cues:{A,B,C}, ttc_s}`.

**Step 4 — optimize the camera with the Sharp (online self-calibration).**
When a detected object's box contains the Sharp beam's spot in the image (front
sensor, robot frame known), and the Sharp reading is valid (inside its range, not
in the blind zone), that pair is a free ground-truth sample. It is used exactly
like your `--calibrate`, but automatically:

- **Per-class height correction**: `H_class_effective = Z_sharp · h_px / f_y`,
  kept as a running median per class (bounded to ±40 % of the table value). So
  "chair" learns the size of *your* chairs, toy cars learn their real size.
- **Camera tilt/height correction** for cue A from the floor-contact error.
- **Depth-model scale** for cue C (one more anchor point for the RANSAC fit).
- All corrections are logged, shown on the dashboard, saved to config on request,
  and can be reset. They never change the formulas, only the numbers.

This is the "depth optimization": the camera distance stays the answer; the range
sensor makes it more accurate over time. Phase 4 measures camera-only error
**before and after** step 4.

**Sharp role in safety** (separate from the above): it is also the authoritative
close-range check for the 0.30 m stop (§9), with these rules.

**Sharp blind zone (safety):** below its minimum range a Sharp sensor's output
falls again, so a very close object can read as *far*. Rules: (1) any front IR
proximity hit overrides the Sharp; (2) a reading that jumps from near to far
within one sample while the robot moves forward is treated as "too close", not
"clear"; (3) the ATmega reflex uses the IR proximity sensors too.

- Tracking frame to frame (box overlap + class) gives closing speed and
  **time-to-collision**. Ego motion now comes from the encoders, so closing speed
  of a static object is no longer confused with the robot's own speed.
- Bearing = box centre + servo angle at capture time.

Accuracy targets (Phase 4, **camera only**, Sharp covered up so it cannot help):
fused distance error ≤ 10 % at 0.3–2 m, ≤ 20 % at 2–4 m, over 30+ tape-measured
placements; known-height cue alone reported separately, before and after
self-calibration.

### 7.3 Dense monocular depth (`depth.js`)

- Default **MiDaS v2.1 small, 256×256, INT8 ONNX (≈ 17 MB file)**.
  Depth-Anything-V2-Small (≈ 25 M params) is more accurate but several times slower
  on a Pi 4 CPU; Phase 0 times both and keeps Depth-Anything only if it reaches
  ≥ 0.5 Hz. Never in the stop path.
- Relative depth → metres with a RANSAC fit `1/Z = s·d + t` against: floor pixels
  (known from camera height/tilt), the Sharp reading where its beam hits the image,
  and cue A/B object distances. Bad fit ⇒ frame ignored.
- Runs in its **own low-priority process** (`nice -n 10`, 1 thread), so it can
  never slow YOLO or the control loop (§10.3). It runs every SCAN stop, and while
  driving only when its core is free (≤ 1 Hz).

### 7.4 Lanes, stop lines, ArUco

- `lanes.js`: port of `logic.find_lane` / `crossing_fraction` /
  `obstacle_fraction` (colour thresholds on 160×120, < 5 ms). The cases in
  `anc_autodrive/test/test_logic.py` become JS tests.
- **White-line sensors** (bonus): they see a white tape line directly under the
  robot, so a white stop line or station line is detected exactly, independent
  of the camera. Used as a confirmation cue for STATION / STOP_SIGN.
- `aruco.js`: marker pose from 4 corners (0.10 m) for station stop and parking,
  same behaviour as `vision_node.py`.

## 8. Pseudo-LiDAR 3D (`fusion/map3d.js`)

1. **Back-project** metric depth pixels (every 4th): `X = (u − c_x)·Z / f_x`,
   `Y = (v − c_y)·Z / f_y`.
2. **Transform** camera → pod (pan/tilt at capture time) → robot → **odometry
   frame** (pose at capture time from `odometry.js`).
3. **Filter**: drop floor (< 3 cm), beyond 4 m, poor-fit frames, frames taken while
   the servo moves fast.
4. **Voxel map**: 5 cm cubes, hit count + last-seen; old voxels fade.
5. **Servo sweep** ±90° in ≈ 5 steps in SCAN ⇒ ≈ 230–250° coverage (exact value
   after Phase 2 FOV measurement).
6. **Encoders ⇒ the map persists while driving.** Odometry drifts (wheel slip,
   caster), so voxels older than a configurable time (default 10 s) fade, and
   points are re-anchored on each SCAN stop.
7. **Real range rays**: every installed Sharp sensor adds its ray to the map and
   the virtual scan, marked as "measured" (camera points are "estimated").
8. **Outputs**: 3D cloud to the dashboard (≤ 20 k points, 2 Hz, binary WS), 2D
   obstacle grid (3 cm … robot height **≈ 8.5 cm + pod/Pi stack, measured**),
   **virtual laser scan** (1° bins), YOLO objects as labelled 3D boxes.

The dashboard says plainly the cloud is camera-derived, with measured accuracy.

## 9. Decision making (`brain/state-machine.js`)

Port of the tested `anc_autodrive` brain: DRIVE, SCAN, OVERTAKE, WAIT, STOP_SIGN,
YIELD, STATION, ARRIVED. Thresholds from simulation: `0.30 m` emergency stop,
`1.20 m` slow down, stop-and-scan at `0.8 m`.

Changes for the Fire Bird:

- **Speeds rescaled**: top speed is ≈ 0.24 m/s, so cruise ≈ 0.15–0.18 m/s,
  slow ≈ 0.08 m/s *(tuned in Phase 7)*.
- The 1.20 m slow-down relies on the camera (beyond Sharp range); the 0.30 m stop
  has the Sharp + IR proximity as the authoritative check.
- Side-clear check uses the **virtual laser scan + tracked objects**.
- `v_max = min(cruise, k · distance_to_nearest_in_path)`.
- **Overtake and station approach are distance/angle manoeuvres** from encoders
  (≈ 5.4 mm and ≈ 4° per pulse), corrected by lane lines, ArUco and the
  white-line sensors. No more timed manoeuvres.
- Spin turns are possible (zero turning radius) and are used in SCAN/WAIT recovery.
- Every tick emits `{state, cmd, reason, input_ages_ms, nearest_obstacle}`.

### Safety arbiter on the Pi (`safety/arbiter.js`)

Highest priority first:

1. ATmega fail-safes (§5.3): always active, below everything here.
2. Front range: Sharp/IR proximity `< 0.30 m` or blind-zone rule hit ⇒ no forward
   motion. No valid range telemetry for > 200 ms ⇒ stop.
3. Stale inputs (camera > 300 ms, YOLO > 1 s, telemetry > 200 ms, control tick
   missed > 200 ms) ⇒ stop.
4. Pi CPU > 80 °C or throttling ⇒ stop; > 75 °C ⇒ half speed. Robot battery
   < 8.4 V ⇒ return-to-base speed limit and dashboard warning.
5. Dashboard manual drive (token) ⇒ manual command, still subject to 1–4.
   Dashboard disconnect during manual ⇒ stop.
6. Brain command.

Speed cap so the robot can always stop inside sensed range:
`v ≤ sqrt(2 · a_brake · (range − 0.30))`, `a_brake` measured in Phase 1.

## 10. Performance: no lag, small memory

Goal: YOLO **≥ 10 Hz**, camera → motor command **< 200 ms**, whole backend
**< 600 MB RAM** (so it would even fit a 1–2 GB Pi; the 4 GB Pi keeps plenty spare).
All numbers *(est.)* until Phase 0 measures them on your Pi.

### 10.1 Make YOLO small (done once on the PC, `tools/export_yolo.py`)

| Step | What | Effect |
|---|---|---|
| 1 | **No Ultralytics / PyTorch / OpenCV-Python on the Pi.** Export on the PC, copy one `.onnx` file | Pi-side "YOLO library" goes from ≈ 2 GB of Python packages to **one ≈ 3–6 MB model file + `onnxruntime-node`** |
| 2 | **Nano model only**: YOLO26n (NMS-free, built for CPU/edge), YOLO11n, or YOLOv8n. Phase 0 benchmarks all three and keeps the fastest that passes the accuracy check | 2.4–3.2 M params |
| 3 | **Smaller input matching the 4:3 webcam**: export at `imgsz=[256,320]` (H×W) instead of 640×640 | ≈ **5× less compute** than 640×640, and no wasted letterbox padding compared with 320×320 |
| 4 | **Static shapes, batch 1, `simplify=True`, opset 17** | ORT can fuse layers; no dynamic-shape overhead |
| 5 | **INT8 static quantization** (QDQ, `onnxruntime.quantization.quantize_static`) calibrated on ≈ 200 frames recorded from **your** webcam on **your** track | file ≈ 4× smaller (≈ 12 MB → ≈ 3 MB), less memory bandwidth. Note: the Pi 4's Cortex-A72 has no int8 dot-product instructions, so INT8 is only ≈ 1.2–1.8× faster than FP32 there. Phase 0 keeps INT8 only if it is faster **and** boxes stay within 4 px of FP32 |
| 6 | **Only the classes you use** decoded in post-processing (classes in the height table + your track objects); later, fine-tuning on only your classes shrinks the output head | less JS work per frame, fewer false alarms |
| 7 | Same export for the depth model (MiDaS small, 256², INT8) | ≈ 17 MB file |

Fallback if ONNX Runtime on the Pi 4 stays under 8 Hz: the same `.pt` exported to
**NCNN** (Ultralytics' fastest Pi format) run by a tiny C++ helper process that
returns boxes over a pipe; or a **Coral USB** accelerator (Edge TPU INT8 TFLite).
Neither is built unless Phase 0 shows it is needed.

### 10.2 Keep it fast at run time

- **Pi OS Lite 64-bit** (no desktop): ≈ 300–400 MB RAM and some CPU back. Dashboard
  is viewed from your phone/laptop browser.
- **Camera at 15 fps, 640×480**, raw YUYV if the webcam offers it at that size (no
  MJPEG decode). YOLO cannot use more than ~10–15 fps anyway; 30 fps only burns CPU.
- **Latest-frame-wins everywhere**: one slot per worker; a new frame overwrites the
  old one. Delay can never build up; slow inference just means fewer frames, not
  older ones.
- **Zero per-frame allocation** in the hot path: input tensor, RGB buffer and output
  arrays are allocated once and reused (no garbage-collection pauses).
- ORT session: `graphOptimizationLevel: 'all'`, `executionMode: 'sequential'`,
  `intraOpNumThreads: 3`, memory arena on, model loaded once and warmed up with 3
  dummy runs at boot.
- **Tracker fills the gaps**: between YOLO frames the IoU tracker predicts each box
  (constant velocity + robot motion from the encoders), so distance and TTC update
  at the 20 Hz control rate even when YOLO runs at 10 Hz.
- **Dashboard costs nothing when nobody watches**: MJPEG only encoded while a client
  is connected, at 320×240, quality 60, ≤ 8 fps; point cloud 2 Hz.
- **No synchronous disk writes**: logs and recordings are buffered; recording is
  off by default (SD cards stall).
- Fan + heatsink; temperature-based derating (§9) so it never thermal-throttles
  mid-run. No overclock unless Phase 0 shows headroom and stable temps.

### 10.3 CPU and memory budget (Pi 4, 4 cores)

| Task | Where | Rate | CPU *(est.)* | RAM *(est.)* |
|---|---|---|---|---|
| OS (Lite) | Pi | — | — | ≈ 150 MB |
| ffmpeg camera capture | Pi | 15 Hz | 0.1–0.2 core | ≈ 20 MB |
| Node main: fusion, tracker, odometry, wheels PI, brain, arbiter, server | Pi | 20 Hz | ≈ 0.3–0.5 core | ≈ 80 MB |
| YOLO worker (3 threads) | Pi | **10–15 Hz** | ≈ 2.5 cores | ≈ 60–100 MB |
| Depth process (1 thread, `nice 10`) | Pi | ≤ 1 Hz | the remaining ≈ 0.5–1 core | ≈ 100–150 MB |
| Telemetry parse | Pi | 50 Hz | tiny | — |
| **Total** | | | **≈ 4 cores, depth gets only what's left** | **≈ 450–550 MB** |
| ADC scan, encoders, PWM, servos, fail-safes | ATmega | continuous | ≈ 20 % of the MCU | — |

Pi 4 YOLO speed reasoning: Ultralytics' published Pi 5 numbers for nano models at
640 px are ≈ 100–200 ms; the Pi 4 is ≈ 2–3× slower, and 256×320 is ≈ 5× less work
than 640², so ≈ 50–120 ms ⇒ **8–20 Hz** *(est.)*. Phase 0 replaces this with the
measured value.

### 10.4 Latency targets (measured per stage, shown on the dashboard)

| Path | Target |
|---|---|
| Frame age when YOLO starts | < 80 ms |
| YOLO inference | < 100 ms |
| Camera → motor command (vision decision) | **< 200 ms** |
| Pi range reading → stop command | < 80 ms |
| Front range → ATmega reflex stop (no Pi involved) | < 30 ms |

Every message carries its capture timestamp; the dashboard shows the per-stage
times live, so lag is a number you can see, not a feeling.

## 11. Dashboard and API (`server/`)

- `GET /`: live video, boxes + distances, state + reason, sensor ages, Pi
  temperature, **robot battery voltage**, encoder speed, ATmega flags, and a
  three.js view (point cloud + grid + robot + odometry trail).
- `GET /stream.mjpg`: annotated video, 5–10 fps.
- `GET /api/health`: sensor ages, rates, temperature, serial CRC error rate,
  last ATmega reset cause.
- `GET/PUT /api/config`: live tuning (Pi-side knobs and ATmega `CONFIG` keys).
- `WS /ws`: `state` (10 Hz), `detections`, `cloud`, `grid`.
- `POST /api/mode`: `auto | manual | stop`.
- **Security:** moving the robot needs a token (`config/secret`, generated at
  install). Viewing needs none. Local network only.

## 12. Recording, replay, testing

- `recorder.js`: every Nth frame (JPEG) + all telemetry and commands (NDJSON), size cap.
- `replay.js`: runs perception + brain on a recording without hardware.
- `node:test`: protocol encode/decode + CRC, Sharp ADC→m table, odometry,
  wheel PI, cue fusion, floor-contact formula, depth fit, back-projection, NMS, state
  machine, arbiter priorities. Lane tests reuse the Python cases.
- Firmware: `test_protocol.c` compiled on the host with `gcc` uses the **same
  test vectors** as the JS protocol test, so both sides agree byte for byte.
- Every hardware phase is tried **with the wheels off the ground first**.

## 13. Phases and acceptance tests

| Phase | Work | Done when (measured and written down) |
|---|---|---|
| **0 Bench check** | Pi OS Lite 64-bit, Node 22, ffmpeg, avr-gcc, avrdude. Webcam formats + fps. `tools/benchmark.js`: YOLO26n / YOLO11n / v8n × FP32 / INT8 × 2–4 threads at 256×320, MiDaS small vs Depth-Anything-V2-Small, peak RAM. ArUco dictionary in JS. Thermal under load. **Fire Bird inventory**: crystal (14.7456 vs 11.0592 MHz), Sharp sensor model (printed on it) and socket (ADC11 = front), servo pod present. **Flashing from the Pi**: try `avrdude` against the NEX bootloader on the USB port; if it fails, flash from the Windows GUI. Flash a blink + UART echo firmware. Charge and check the NiMH pack. | Benchmark table (Hz, ms, MB, °C); fastest model that passes the 4 px box check chosen; inventory list; flashing method chosen; echo firmware round-trips at 115200 from the Pi |
| **1 Firmware + link + safety** | Bridge firmware (§5), `firebird.js`, `wheels.js`, `odometry.js`, arbiter, manual drive from dashboard. Calibrate mm/pulse, track width, PWM→speed, braking, Sharp ADC→m, IR proximity thresholds, servo channels and µs range | Motors stop ≤ 300 ms after USB unplug, `kill -9` of Node, and Pi power-off; no twitch during MCU reset; 10 min telemetry at 50 Hz with < 0.1 % CRC errors; 2 m straight run: encoder distance within 2 %, lateral drift < 5 cm after trim; 360° spin within 5°; reflex stop works with the Pi unplugged |
| **2 Camera + calibration** | camera.js, checkerboard calibration, camera height/tilt/pod offsets, servo pointing check | Reprojection error < 0.5 px; frame age < 80 ms |
| **3 YOLO worker** | export the same `.pt` as the Python script, preprocessing, inference, NMS, latest-wins | **≥ 10 Hz** for 30 min with depth running, frame age at inference < 80 ms, no delay build-up, no thermal throttling, backend RSS < 600 MB; on 50 recorded frames the boxes match the Python script within 2 px (FP32) / 4 px (INT8) |
| **4 Distance + depth optimization** | `mono_distance.js` (port of your script), cues A/B/C, tracker, fusion, TTC, Sharp self-calibration, blind-zone rules | Your-method distance matches the Python script on the same frames (≤ 1 %); camera-only fused error ≤ 10 % at 0.3–2 m over 30+ placements; error before vs after self-calibration reported; object at 5 cm never reads "clear" |
| **5 Dense depth** | depth process, RANSAC fit, quality score | Metric error ≤ 15 % at 0.5–2 m |
| **6 Pseudo-LiDAR** | cloud, voxel map in odometry frame, sweep, grid, virtual scan, Sharp rays | Box at known spots within 10 cm (< 2 m) when stopped, within 15 cm after a 2 m drive |
| **7 Brain** | state machine port, lanes, white-line cue, ArUco, TTC limit, encoder manoeuvres | Simulation README scenarios, 10 runs each, 0 collisions |
| **8 Dashboard** | video, 3D, health, battery, live config | Works on a phone over Wi-Fi |
| **9 Hardening** | systemd, auto-restart, recorder, thermal derating, battery derating, 30-min run | 30 min autonomous: no crash, no throttling, no MCU brown-out reset, every safety stop works |

Results go in `docs/`, like the existing phase reports.

## 14. Still open (changes config, not the design)

Answered: Pi has 4 GB RAM (full plan); Fire Bird has its 8 IR proximity
sensors and a Sharp range sensor; the Pi uses the same USB port as the
bootloader; no HC-SR04, no IMU; camera distance (your YOLO method + depth
optimization) is the core, range sensors calibrate and veto.

1. **Sharp model and socket**: checked in Phase 0 (sets its usable range).
2. **Crystal frequency and servo pod / servo model**: checked in Phase 0–1.
3. **avrdude vs NEX bootloader**: checked in Phase 0; fallback is the Windows GUI.
4. **Rear IR proximity 6–8** sit on the ATmega8 (SPI, needs J4 on and its
   firmware). Only needed if the robot reverses on its own; skipped until then.
   Until then, the Pi does not reverse more than one wheel's length without a
   camera look behind (spin turn first).

## 15. Risks

| Risk | What we do about it |
|---|---|
| Pi 4 too slow for YOLO + depth | Small input, nano model, INT8 if faster, depth niced and optional; NCNN helper or Coral USB as measured fallbacks (§10.1) |
| Lag builds up | Latest-frame-wins, per-stage timestamps on the dashboard, tracker fills gaps between YOLO frames |
| INT8 loses accuracy | Calibrated on your own frames; kept only if boxes stay within 4 px of FP32 |
| Monocular depth wrong on glass, mirrors, plain walls | Sharp/IR veto, depth quality score, nearest-distance rule |
| Sharp blind zone reads "far" | IR proximity override, jump rule, ATmega reflex uses both (§7.2) |
| Single-channel encoders: no direction, slip; no IMU | Direction from command + reversal dwell; odometry uncertainty grows; corrected by lanes/ArUco; re-anchor on SCAN |
| Known-height table wrong for small/toy objects | Floor-contact + depth cues; Sharp self-calibration of per-class heights; fine-tuned classes |
| Self-calibration learns from a bad sample | Only valid Sharp readings inside the box, running median, ±40 % bound, reset button |
| Bootloader and firmware share the USB port | stop the backend before `make flash` (`atmega/README.md`); ATmega fail-safes keep motors off while flashing |
| Low encoder resolution (≈ 37 pulses/s max) | Edge-period speed measurement; PI at 20–50 Hz only |
| Brown-out resets the ATmega on motor surges | Pi on its own power bank; PWM ramp; reversal dwell; reset cause reported in `BOOT` |
| NiMH deep discharge / short run time (≈ 2 h) | Battery telemetry, derating at 8.4 V, cutoff at 8.0 V; charge only with the on-board charger, never while powered on |
| Webcam too heavy for pod servos / 800 mA servo regulator | Phase 1 load check; move one servo at a time; stronger servo if needed |
| USB serial latency (FTDI 16 ms default) | latency_timer = 1 via udev |
| USB link lost mid-run | ATmega stops in 250 ms on its own; Pi reconnects and waits for `BOOT`/telemetry before moving |
| Servo channel mapping differs between NEX documents | Checked in Phase 1, stored in config |
| Webcam auto-exposure/focus hunting | Locked via V4L2 controls in Phase 2 if supported |
