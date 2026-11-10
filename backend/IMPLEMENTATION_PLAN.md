# Raspberry Pi 4B Node.js Backend — Implementation Plan

Status: **plan only, no code yet.** Every performance number marked *(est.)* is
an estimate that Phase 0 replaces with a measurement on the real Pi.

## 1. Goal

**Everything runs on the Raspberry Pi 4 Model B** — no laptop, no ESP32, no ROS
on the car. One Node.js process (`backend/`) on the Pi:

1. Talks directly to every part on the car: USB webcam, HC-SR04, pan servo,
   L293D motor driver, and (if installed) an MPU6500 IMU.
2. Detects objects with YOLO and estimates **metric distance** to each one by
   fusing several depth cues (floor geometry, object size, dense monocular depth,
   ultrasonic).
3. Builds a **LiDAR-like 3D point cloud** ("pseudo-LiDAR") from the camera, swept
   by the pan servo, plus a 2D obstacle map and a virtual laser scan.
4. Drives the car with the decision logic already proven in simulation
   (`simulation/ros2_ws/src/anc_autodrive`), ported to JavaScript, and **fails safe**.
5. Serves a live dashboard (video, detections, 3D view, state) over the Pi's Wi-Fi
   to a phone or any browser.

The ROS 2 / Gazebo simulation stays in the repo as the place to try logic
changes first. It is not used on the car.

### What "perfect" means here (honest scope)

No perception system is perfect, and one camera cannot match a real LiDAR. The
design goal is:

- **Every decision is explainable**: each motor command carries its reason and
  the sensor data behind it (logged and shown on the dashboard).
- **Uncertainty is tracked**: every distance has a confidence; low confidence ⇒
  slower or stop, never "guess and go".
- **Fail-closed**: stale sensor, overheating, crashed worker, or conflicting data
  ⇒ motors stop. The ultrasonic sensor always has veto power.
- **Measured, not assumed**: every phase ends with a measured acceptance test.

## 2. Hardware (from your answers)

| Part | What you have | Status |
|---|---|---|
| Computer | Raspberry Pi 4B, 32 GB SD card | **RAM unknown**: run `free -h` |
| OS | Raspberry Pi OS Bookworm **64-bit** (Lite or Desktop) | to install |
| Camera | FINGERS 1080 Hi-Res USB webcam → Pi USB 3 port | confirmed |
| Motor driver | L293D dual H-bridge module | confirmed |
| Motors | 2 × DC geared motors, **no encoders** | **voltage unknown**: read motor label |
| Front range | HC-SR04 | from earlier plan |
| Pan servo | SG90 / MG90S class | from earlier plan |
| IMU | MPU6500 | **not confirmed on this car**: optional in the design |
| Cooling | heatsink **and fan** on the Pi | required (YOLO runs all 4 cores) |

### Power (needs fixing before Phase 1)

- **9 V rectangular (PP3) batteries are not suitable for the motors.** They sag
  hard and run flat in minutes under motor load. Use a 2S Li-ion pack (7.4 V) or
  6 × AA, chosen once the motor's rated voltage is known.
- The L293D loses about 1.5–2.5 V inside the chip, so the motors see less than the
  battery voltage. The `max_duty` setting in config caps the motor voltage to
  their rating.
- Pi: its own 5 V 3 A supply (a buck converter from the main pack or a USB power
  bank). Servo: its own 5 V buck (servo current spikes reset the Pi).
  **All grounds connected together.**

### Wiring (BCM numbering), single source of truth in `config/default.json`

| Signal | Pi GPIO | Notes |
|---|---|---|
| L293D EN1 (left PWM) | 12 | **remove the module's EN jumper**; add 10 kΩ pull-down to GND |
| L293D IN1 / IN2 | 5 / 6 | |
| L293D EN2 (right PWM) | 13 | remove jumper; 10 kΩ pull-down |
| L293D IN3 / IN4 | 16 / 20 | |
| L293D VCC1 (logic) | 5 V | Pi's 3.3 V outputs are valid L293D inputs (high threshold ≈ 2.3 V) |
| L293D VCC2 (motor) | battery + | |
| HC-SR04 TRIG | 23 | |
| HC-SR04 ECHO | 24 | **through a 1 kΩ / 2 kΩ divider (5 V → 3.3 V), never direct** |
| HC-SR04 VCC | 5 V | |
| Servo signal | 18 | servo powered from its own 5 V |
| MPU6500 SDA / SCL (optional) | 2 / 3 | 3.3 V, I²C address 0x68 |
| E-stop button (optional) | 26 | to GND, internal pull-up |

The pull-downs on EN1/EN2 are a **hardware safety**: while the Pi boots, crashes,
or the program exits, the enable pins float low and the motors cannot run.

## 3. Architecture (all on the Pi)

```
                   ┌──────────────────── Node.js main thread (event loop) ───────────────────┐
 USB webcam ─────▶ │ camera.js (ffmpeg child, raw frames) ──frame──┐                         │
 HC-SR04 ────────▶ │ ultrasonic.js ──────────────┐                 │                         │
 MPU6500 (opt.) ─▶ │ imu.js ── motion.js ◀── commanded speed       │                         │
                   │                              ▼                ▼                         │
                   │        fusion/distance.js ◀── detections ── [worker] yolo.worker.js     │
                   │        fusion/pointcloud.js ◀── depth map ── [worker] depth.worker.js   │
                   │        fusion/occupancy.js                                              │
                   │                 │                                                       │
                   │                 ▼                                                       │
                   │   brain/state-machine.js ──cmd──▶ safety/arbiter.js ──▶ motors.js (L293D)│
                   │                                         │                servo.js       │
                   │   server/ (HTTP + WebSocket + MJPEG) ◀──┘ state, cloud, video            │
                   └─────────────────────────────────────────────────────────────────────────┘
                                         ▲ Wi-Fi
                                   phone / browser dashboard
```

Rules:

- **Heavy compute never runs on the main thread.** YOLO and depth run in
  `worker_threads`; frames are passed as transferable `ArrayBuffer`s (no copy).
- **Workers always take the newest frame and drop old ones** (latest-wins), so
  delay never builds up when inference is slower than the camera.
- **The control loop runs at a fixed 20 Hz**, separate from perception. It uses
  the latest fused state and its age; too old ⇒ stop.
- **Every message is timestamped at capture**, so fusion uses the servo angle and
  car motion *from the moment the frame was taken*.

### Folder layout

```
backend/
  IMPLEMENTATION_PLAN.md
  package.json
  config/default.json            # pins, calibration, thresholds: every tuning knob
  config/camera_intrinsics.json  # produced by camera calibration (Phase 2)
  models/                        # yolo11n.onnx, depth .onnx (git-ignored, downloaded)
  src/
    index.js                     # boot, wiring, shutdown (motors off on any exit)
    hw/camera.js  ultrasonic.js  servo.js  motors.js  imu.js
    perception/yolo.worker.js  depth.worker.js  lanes.js  aruco.js  preprocess.js
    fusion/distance.js  kalman.js  pointcloud.js  occupancy.js  motion.js
    brain/state-machine.js       # port of anc_autodrive brain_node.py + logic.py
    safety/arbiter.js  health.js
    server/http.js  ws.js
    record/recorder.js  replay.js
  public/                        # dashboard: index.html + local copy of three.js (works offline)
  tools/calibrate_camera.py      # one-off OpenCV checkerboard calibration
  tools/export_models.md         # how to export/quantize the ONNX models
  scripts/install-pi.sh          # apt packages, Node, pigpio, ffmpeg, model download
  scripts/anc-backend.service    # systemd unit: starts on boot, restarts on crash
  test/*.test.js                 # node:test (built-in), no test framework
```

## 4. Software stack (kept small)

| Need | Choice | Why |
|---|---|---|
| Runtime | Node.js 22 LTS (arm64) | worker threads, built-in test runner |
| Camera capture | `ffmpeg` (apt) as a child process, V4L2 input | works with any USB webcam, no native Node addon |
| Inference | `onnxruntime-node` | official arm64 build, INT8 models |
| Resize / JPEG for dashboard | `sharp` | prebuilt for arm64, fast |
| GPIO, PWM, servo, echo timing | `pigpio` | microsecond edge timestamps (needed for HC-SR04), jitter-free servo pulses, DMA PWM for motors |
| I²C (IMU, optional) | `i2c-bus` | small, maintained |
| WebSocket | `ws` | binary frames for point clouds |
| HTTP, events, tests, threads | Node built-ins | no Express, no Jest, no message broker |

Deliberately not used: OpenCV for Node (painful to build on the Pi), ROS on the
Pi (Node only, as requested), TensorFlow.js (slower than ONNX Runtime on ARM).

ArUco: your markers are `DICT_7X7_1000`, 0.10 m. **Phase 0 checks** whether a JS
detector supports that dictionary (`js-aruco2` with a custom dictionary, or
`@techstark/opencv-js` `ArucoDetector`). Fallback: reprint markers in a supported
dictionary.

## 5. Hardware drivers (Phases 1–2)

| Driver | How | Output |
|---|---|---|
| `camera.js` | spawn `ffmpeg -f v4l2 -input_format <mjpeg\|yuyv422> -video_size 640x480 -framerate 30 -i /dev/video0 -f rawvideo -pix_fmt yuv420p -`; cut stdout into fixed 460 800-byte frames. The Y plane is a free grayscale image. Restart ffmpeg if it exits; frame-age watchdog. Phase 0 lists the webcam's formats with `v4l2-ctl --list-formats-ext` and picks the cheapest one that gives 30 fps. Auto-exposure and focus get locked if the webcam allows it (stable images, stable YOLO). | `{t, w, h, yuv}` |
| `ultrasonic.js` | pigpio: 10 µs TRIG pulse at 15 Hz; time between ECHO rising and falling edges → `d = Δt · 343 / 2` (speed of sound from config). Median of the last 5 readings; reject anything outside 0.02–4.0 m; a timeout means `null` (unknown), never "far". | `{t, range_m, valid}` |
| `servo.js` | pigpio `servoWrite(µs)`; angle ↔ µs mapping with `min_us`, `max_us`, `trim_deg` calibration; slew-rate limit. The servo gives no position feedback, so the driver reports an *estimated* angle from its measured speed and a `settled` flag. | `{t, cmd_rad, est_rad, settled}` |
| `motors.js` | L293D: PWM duty on EN1/EN2, direction on IN1–IN4. Per-side `gain` trim (two motors never match), `deadband` (minimum duty that moves the wheel), `max_duty` (motor voltage cap), ramp limit. Motors off on stop, exit, or crash. | — |
| `imu.js` (optional) | MPU6500 at 100 Hz (WHO_AM_I = 0x70); gyro bias averaged at startup while still. Disabled when `imu.enabled = false` or the chip doesn't answer. | `{t, gz_radps}` |
| `motion.js` | **No encoders, so motion is estimated, not measured.** Speed = calibrated map from PWM duty → m/s (measured on the floor in Phase 1). Turn rate = gyro if the IMU exists, otherwise the commanded rate from calibration. Every estimate carries a large uncertainty that grows with time. | `{t, x, y, yaw, v, w, sigma}` |

Calibration knobs (all in `config/default.json`, editable live from the
dashboard): speed of sound, servo µs range and trim, motor gains, deadband,
`max_duty`, duty→speed table, turn-rate table, gyro bias, camera height and tilt
above the floor (**measure!**), camera intrinsics.

The service runs as root under systemd (pigpio needs it). It is the only program
with GPIO access.

## 6. Perception

### 6.1 YOLO object detection (`yolo.worker.js`)

- Model: **YOLO11n** (or YOLOv8n) exported to ONNX at **320×320**, INT8
  quantized on a PC (`tools/export_models.md`). Optional later: fine-tune with your
  own classes (cone, barrier, ArUco sign) using the `docs/academic/phase9*` data.
- Preprocessing: YUV → RGB + letterbox to 320 in one pass over typed arrays.
- Postprocessing: decode boxes, per-class NMS in JS, confidence ≥ 0.45 (same as
  `object_detection_new_babu.py`).
- Threads: `intraOpNumThreads = 2`, leaving cores for the depth worker, ffmpeg,
  and the main loop.
- Expected **5–10 Hz** on the Pi 4 *(est.)*. If Phase 0 measures under 5 Hz: drop
  the input to 256 px, or add a Coral USB accelerator later.

### 6.2 Distance estimation: "depth optimization" (`fusion/distance.js`)

Each detected object gets up to four independent distance readings, each with its
own uncertainty, combined per object by a small Kalman filter (`kalman.js`):

| # | Cue | Formula | Good at | Weak at |
|---|---|---|---|---|
| A | **Floor contact** (main cue) | box bottom row `v_b`: `Z = h_cam / tan(tilt + atan((v_b − c_y) / f_y))` | anything standing on the floor, any class; very accurate under 2 m | object not on the floor; box bottom hidden or cut off by the image edge |
| B | **Known height** | `Z = f_y · H_class / h_px` (height table from `object_detection_new_babu.py`) | far objects; bottom hidden | wrong when the real object's size differs |
| C | **Dense depth** (§6.3) | median metric depth in the lower centre of the box | odd shapes, unknown classes | slow; needs scale anchoring |
| D | **Ultrasonic** | direct range when the object's bearing is inside the ±7.5° beam | 0.02–2 m, authoritative | narrow beam; soft or angled surfaces |

- Uncertainty grows with distance and shrinks with detection confidence. Cue A is
  switched off when the box touches the bottom edge of the image.
- If the cues disagree by more than 3σ, the object is marked `inconsistent` and the
  brain uses the **nearest** plausible distance (the safe choice).
- Objects are tracked frame to frame (box overlap + class), so the filter has
  history and a closing speed, giving **time-to-collision** = distance / closing speed.
- Bearing from the box centre + servo angle at capture time ⇒ object position
  relative to the car.

Accuracy target (Phase 4): floor-contact error ≤ 10 % at 0.3–2 m and ≤ 20 % at
2–4 m, checked with a tape measure over 30+ placements.

### 6.3 Dense monocular depth (`depth.worker.js`)

- Model: **Depth-Anything-V2-Small** or **MiDaS v2.1 small**, ONNX INT8, about 256 px
  input. Phase 0 times both on the Pi and keeps the better one.
- Expected **0.5–2 Hz** on the Pi 4 *(est.)*. It refines the picture; it is
  never in the emergency-stop path.
- These models give **relative** depth. Each frame is converted to metres with a
  robust fit `1/Z = s·d + t` (RANSAC) against points whose real distance is known:
  1. floor pixels in the lower image (distance known from camera height and tilt),
  2. the ultrasonic reading at the image centre (when the servo faces forward),
  3. object distances from cues A and B.
  The fit error becomes the depth map's quality score; a bad fit ⇒ that frame is ignored.
- Runs only when the YOLO worker is idle, so it never slows detection.
- **If the Pi has 1–2 GB RAM**, this model may not fit next to YOLO. The car still
  drives safely on cues A, B and D; the 3D view then uses those points only.

### 6.4 Lanes and ArUco

- `lanes.js`: port of `logic.find_lane` / `crossing_fraction` /
  `obstacle_fraction` (colour thresholds on a 160×120 downscale, < 5 ms in plain
  JS). The cases in `anc_autodrive/test/test_logic.py` become JS tests.
- `aruco.js`: marker detection + pose from its 4 corners (0.10 m marker) for the
  station stop and parking, same behaviour as `vision_node.py`.

## 7. Pseudo-LiDAR 3D (`fusion/pointcloud.js`, `fusion/occupancy.js`)

1. **Back-project** metric depth pixels (every 4th pixel):
   `X = (u − c_x)·Z / f_x`, `Y = (v − c_y)·Z / f_y`, in the camera frame.
2. **Transform** camera → servo (pan angle at capture time) → car frame, using the
   measured mount offsets from config.
3. **Filter**: drop floor points (< 3 cm above the floor), points beyond 4 m, and
   frames with a poor depth fit.
4. **Voxel map**: 5 cm cubes in a hash map, each with a hit count and last-seen
   time; old voxels fade out so moving things clear.
5. **Servo sweep**: in the SCAN state the servo turns ±90° in about 5 steps. With
   the webcam's field of view this covers roughly 230–250° in front of the car (the
   exact figure depends on the webcam's FOV, measured in Phase 2). Not behind the car.
6. **No encoders ⇒ the map is built while the car is stopped.** The SCAN state
   already stops before sweeping, so the sweep is sharp. While driving, the map
   only keeps the last ~1 s of points (motion is estimated, so older points drift).
   Adding wheel encoders later lets the map persist while moving.
7. **Outputs**
   - 3D point cloud → dashboard (binary `Float32Array` over WebSocket, ≤ 20 k points, 2 Hz).
   - 2D obstacle grid (points between 3 cm and car height) for path checks.
   - **Virtual laser scan**: 1° bins, nearest obstacle per direction, the same
     shape as a LiDAR scan, so "is the left lane clear?" becomes a simple lookup.
   - YOLO objects drawn as labelled 3D boxes in the same view.

The dashboard states plainly that this is camera-derived, with the accuracy
measured in Phase 6. It is worse than a real LiDAR, especially beyond 2 m and on
plain walls.

## 8. Decision making (`brain/state-machine.js`)

Port of the tested `anc_autodrive` brain: DRIVE, SCAN, OVERTAKE, WAIT, STOP_SIGN,
YIELD, STATION, ARRIVED. Thresholds come from the simulation: `0.30 m` emergency
stop, `1.20 m` slow down, stop-and-scan at `0.8 m`. Speeds are rescaled for the
real car.

Changes for the real car:

- The side-clear check uses the **virtual laser scan + tracked objects** instead
  of the simulation's colour heuristic.
- Speed limit from **time-to-collision**:
  `v_max = min(cruise, k · distance_to_nearest_in_path)`.
- **No encoders**: overtake and station approach become timed manoeuvres
  (calibrated duty × time), corrected by the gyro when the IMU exists and by
  the lane lines and ArUco marker when visible. They are less precise than in
  simulation, and Phase 7 measures how much.
- Every control tick emits `{state, cmd, reason, input_ages_ms, nearest_obstacle}`
  → log + dashboard.

### Safety arbiter (`safety/arbiter.js`): has the last word on every motor command

Highest priority first:

1. Hardware: pull-downs on L293D EN pins; motors off on exit, crash, or E-stop button.
2. Ultrasonic `< 0.30 m`, or ultrasonic has no valid reading for > 300 ms while
   moving forward ⇒ stop.
3. Stale inputs (camera > 300 ms, YOLO > 1 s, control loop missed > 200 ms) ⇒ stop.
4. CPU temperature > 80 °C or throttling flags set ⇒ stop; > 75 °C ⇒ half speed.
5. Dashboard manual drive (needs the token) ⇒ manual command, still subject to 1–4.
   Dashboard disconnect during manual drive ⇒ stop.
6. Brain command.

Forward speed is also capped so the car can always stop inside the ultrasonic
range: `v ≤ sqrt(2 · a_brake · (range − 0.30))`, with `a_brake` measured in Phase 1.

## 9. Real-time budget (Pi 4B, 4 cores)

| Task | Rate | CPU *(est.)* |
|---|---|---|
| ffmpeg webcam capture | 30 Hz | ~0.2–0.4 core (more if the webcam only gives MJPEG at 640×480) |
| YOLO worker (2 threads) | 5–10 Hz | ~2 cores |
| Depth worker (only when YOLO idle) | 0.5–2 Hz | shares the YOLO cores |
| Main loop: fusion, lanes, brain, arbiter, server | 20 Hz | ~0.5 core |
| pigpio (ultrasonic, servo, PWM) | 15 Hz / continuous | small |

Dropping frames is fine; growing delay is not. Targets, logged continuously:
camera → motor command **< 250 ms** for YOLO decisions; ultrasonic → stop
**< 50 ms**.

## 10. Dashboard and API (`server/`)

The Pi serves everything over its own Wi-Fi (or your router):

- `GET /`: dashboard with live video, boxes + distances, state and reason, sensor
  ages, CPU temperature, and a **three.js 3D point cloud + obstacle grid + car model**.
- `GET /stream.mjpg`: annotated video, 5–10 fps.
- `GET /api/health`: sensor ages, rates, temperature, throttling, worker status.
- `GET /api/config`, `PUT /api/config`: live tuning of calibration knobs, saved to disk.
- `WS /ws`: `state` (10 Hz JSON), `detections`, `cloud` (binary), `grid` (binary).
- `POST /api/mode`: `auto | manual | stop`; manual driving over the WebSocket.
- **Security:** anything that moves the car needs a token (`config/secret`,
  generated at install). Viewing needs no token. Bound to the local network only.

## 11. Recording, replay, testing

- `recorder.js` (optional): saves every Nth frame (JPEG) + all sensor and command
  messages (NDJSON, timestamped) to `/home/pi/runs/<date>/`, with a size cap.
- `replay.js`: feeds a recording back through perception and brain without any
  hardware, so the same run can be re-tested on any computer.
- Tests with `node:test`: Kalman filter, floor-contact formula, depth scale fit,
  back-projection, NMS, state-machine transitions, arbiter priorities, ultrasonic
  median/timeout, duty→speed mapping. Lane tests reuse the Python test cases.
- Every hardware phase is tried **with the wheels off the ground first**.

## 12. Phases and acceptance tests

| Phase | Work | Done when (measured and written down) |
|---|---|---|
| **0 Bench check** | Pi OS 64-bit, Node 22, pigpio, ffmpeg. `free -h` for RAM. Webcam formats + real fps. Time YOLO11n/v8n 320 INT8 and both depth models on the Pi. Check ArUco dictionary in JS. Temperature under full load with fan. Read the motor label. | Table of real fps, RAM, temperatures; models chosen; every *(est.)* here replaced |
| **1 Drivers + safety** | motors (L293D), servo, ultrasonic, optional IMU, arbiter, manual drive from dashboard. Calibrate duty→speed, turn rate, braking distance on the floor. | Motors stop ≤ 200 ms after dashboard disconnect and after `kill -9`; ultrasonic ±2 cm at 0.1–2 m; car drives straight within 10 cm over 2 m after gain trim |
| **2 Camera + calibration** | camera.js, checkerboard calibration, measured camera height/tilt/mount offsets | Reprojection error < 0.5 px; frame age < 80 ms |
| **3 YOLO worker** | preprocessing, inference, NMS, latest-wins scheduling | ≥ 5 Hz for 10 minutes with no delay build-up |
| **4 Distance fusion** | cues A, B, D, tracker, Kalman, time-to-collision | Error ≤ 10 % at 0.3–2 m over 30+ tape-measured placements |
| **5 Dense depth** | depth worker, RANSAC scale fit, quality score (skipped if RAM too small) | Metric depth error ≤ 15 % at 0.5–2 m on a test scene |
| **6 Pseudo-LiDAR** | point cloud, voxel map, servo sweep, obstacle grid, virtual scan | A box at known spots shows up within 10 cm (< 2 m) in the map |
| **7 Brain** | state machine port, lanes, ArUco, time-to-collision speed limit, timed manoeuvres | Same scenarios as the simulation README, 10 runs each, 0 collisions |
| **8 Dashboard** | video, 3D view, health, live config | Works on a phone browser over Wi-Fi |
| **9 Hardening** | systemd autostart, auto-restart, recorder, thermal derating, 30-minute run | 30 minutes autonomous: no crash, no throttling, every safety stop works |

Each phase is finished only when its test passes. Results go in `docs/`, like
the existing phase reports.

## 13. Still open (affects config, not the design)

1. **RAM**: run `free -h` on the Pi. 4 GB+ ⇒ full plan; 2 GB ⇒ probably fine,
   checked in Phase 0; 1 GB ⇒ no dense depth model.
2. **Motor voltage**: read the label or seller listing (yellow "TT" gear motors are
   usually 3–6 V). This picks the battery and `max_duty`.
3. **MPU6500**: is it on this car? With it, turns and timed manoeuvres are much
   more accurate. Without it, the plan still works but is less precise.
4. **Servo**: confirm the pan servo model and that the webcam is mounted on it
   (the webcam is heavier than a Pi camera; an MG90S with metal gears is safer than an SG90).
5. **Recommended cheap upgrade**: wheel encoders (slotted discs + LM393 speed
   sensors, or replacement motors with built-in encoders). They make motion
   measured instead of guessed, so the 3D map holds up while driving.

## 14. Risks

| Risk | What we do about it |
|---|---|
| Pi 4 too slow for YOLO + depth together | Depth is optional; floor-contact cue + ultrasonic are enough to drive safely; Coral USB accelerator as a later option |
| Monocular depth wrong on glass, mirrors, plain walls | Ultrasonic veto; depth quality score; fusion always takes the nearest plausible distance |
| No encoders ⇒ motion guessed | Map built while stopped; gyro if IMU present; large uncertainty on motion; encoders recommended |
| L293D heat and voltage loss | `max_duty` cap, short duty cycles, heatsink on the chip; move to a TB6612 if it overheats |
| Weak batteries (PP3) | Replace with 2S Li-ion or 6 × AA before Phase 1 |
| Thermal throttling | Fan required; temperature monitored; slow down before 80 °C |
| Servo has no position feedback | Settling delay + speed model; frames ignored while the servo moves fast |
| Brown-outs when motors start | Separate supplies, common ground, ramped PWM |
| Webcam auto-exposure/auto-focus hunting | Lock both via V4L2 controls in Phase 2 if the webcam supports it |
| pigpio availability on current Pi OS | Checked in Phase 0; fallback is `node-libgpiod` + kernel PWM (less precise ultrasonic timing) |
