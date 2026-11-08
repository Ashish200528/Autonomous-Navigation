# anc_autodrive — camera-only autonomous robot car

A small robot car drives itself through a 1:5 scale town using **only a camera**
for navigation, a **pan servo** to look around before it decides, YOLO + ArUco
vision, and a **front ultrasonic sensor** as the last-resort stop. No LiDAR, no
map, no simulator truth.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/anc/simulation/ros2_ws/install/setup.bash
export DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1      # Docker/VNC only
ros2 launch anc_autodrive autodrive.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo window: dark theme, chase camera, HUD top-left (close/reopen with `gz sim -g` any time) |
| `quality` | `standard` | `standard` sky (~80 % real time on CPU-only rendering) · `lite` fastest (~88 %) · `ultra` sky + shadows (needs a real GPU; ~12 % on CPU) |
| `use_yolo` | `true` | YOLO object detection (needs `pip install ultralytics`) |
| `laps` | `2` | Station stops before parking; `0` = drive forever |
| `autostart` | `true` | `false`: car holds until `ros2 param set /autodrive_brain go true` |
| `start_x/start_y/start_yaw` | `-3.6 / -3.3 / 0` | Start pose on the track |

Watch what the car thinks: the **HUD** panel (camera view, lane lines, YOLO boxes,
decision, ultrasonic, servo) or `ros2 topic echo /autodrive/state`.

## The course

The car drives counter-clockwise in the right-hand lane (0.6 m = a 3 m lane at 1:5):

| Where | What | Car's decision |
|---|---|---|
| South straight | STOP sign + fire hydrant | full stop 3 s, then go |
| South straight | zebra crossing with a pedestrian walking over it | holds heading over the zebra; yields while someone is in the path |
| South straight | orange barrier in the lane | stop at 0.8 m, servo-scan left/right, overtake |
| East curve | people on the sidewalk | slow down |
| North straight | broken-down red car + warning cone | scan, overtake |
| North straight | ArUco station sign | stop next to it, count a lap / park |
| West curve | parked SUV, town buildings, street lamps | — |

## What it does

```
          ultrasonic < 0.8 m                both sides blocked
  DRIVE ─────────────────────▶ SCAN ── back up 15 cm, rescan (max 3x) ──▶ WAIT ──(3 s)──▶ SCAN
    ▲   (servo left, right, centre)│ left lane clear              │ path cleared
    │                              ▼                              ▼
    └────────── back in lane ── OVERTAKE                         DRIVE
  DRIVE ── YOLO stop sign ≤ 1 m ──▶ STOP_SIGN (3 s) ──▶ DRIVE
  DRIVE ── YOLO person in path ───▶ YIELD (until clear 1.5 s) ──▶ DRIVE
  DRIVE ── ArUco station seen ≤ 3 m ──▶ approach by odometry ──▶ STATION (5 s) ──▶ DRIVE / ARRIVED
  any state: ultrasonic < 0.30 m  → emergency stop;  stale camera/ultrasonic/odometry → stop
```

* **Lane following** – yellow centre line on the left, white edge on the right,
  found in the lower image band; PD steering on the lane-centre error
  (`logic.find_lane`). Stop lines and zebra bars are rejected; on a crossing the car
  holds its heading by odometry until the lines reappear.
* **Speed** – cruise 0.216 m/s; slow to 0.096 m/s near obstacles (≤ 1.2 m, thresholds
  from `object_detection_new_babu.py`), people, stop signs and the station.
* **Decide** – at 0.8 m the car stops, pans the camera ±69°, and judges each side
  (nearby colour obstacles + YOLO objects closer than 2 m). Left lane free → overtake;
  otherwise back up 15 cm for a wider view and rescan (up to 3 times), then wait.
  People are never overtaken.
* **Vision** – YOLO26s on the 640×480 camera at 2 Hz (HUD boxes drawn on the exact frame they came from) (road classes only: person, car,
  truck, stop sign, …) with height-prior distances; `object_scale:=0.2` in the 1:5
  town, `1.0` on the real car.

## Nodes and topics

| Node | Subscribes | Publishes |
|---|---|---|
| `brain` | `/camera/image_raw`, `/ultrasonic/range`, `/odometry/filtered`, `/vision/detections` | `/cmd_vel`, `/servo/pan_cmd`, `/autodrive/hud`, `/autodrive/state` |
| `vision` | `/camera/image_raw`, `/camera/camera_info` | `/vision/detections` (JSON) |
| `ultrasonic_sim` | `/ultrasonic/scan` (sim only) | `/ultrasonic/range` |
| `pedestrian_sim` | — (sim only) | `/model/pedestrian/cmd_vel` |

## Moving to the Raspberry Pi car

`brain` and `vision` run unchanged on a Raspberry Pi 5 (Ubuntu 24.04 + ROS 2
Jazzy). Only the simulation pieces are replaced by hardware drivers that keep the
same topics:

| Simulation | Raspberry Pi | Topic contract |
|---|---|---|
| Gazebo camera | Pi Camera / USB webcam (`camera_ros` or `v4l2_camera`) | `/camera/image_raw` + `/camera/camera_info` (calibrate!) |
| `ultrasonic_sim` | HC-SR04 driver (gpiozero `DistanceSensor`, ECHO via 5 V→3.3 V divider) | `/ultrasonic/range` (`sensor_msgs/Range`, metres) |
| Gazebo pan joint | SG90/MG90S servo on a hardware-PWM pin | `/servo/pan_cmd` (`std_msgs/Float64`, rad, + = left) |
| Gazebo DiffDrive | Motor driver (L298N / TB6612) + wheel encoders | `/cmd_vel` in, `/odom` out |
| Gazebo IMU | MPU6050 / BNO055 | `/imu/data` |

Re-tune on the real car: `scan_roi_top/bottom` (side-view band; the car's own body
appears below it), `steer_kp/kd`, `object_scale` (1.0), `aruco_dictionary` +
`aruco_marker_size_m` (your printed markers are `DICT_7X7_1000`, 0.10 m), and the
HSV ranges in `logic.py` for your floor and tape colours.

## Limits (be honest when demoing)

* Lane detection is colour-threshold based: it needs clear yellow/white lines and
  even lighting.
* The side-clear check is a colour heuristic OR-ed with YOLO; grey or green
  obstacles can be missed by the heuristic.
* Monocular YOLO distance assumes typical object heights; the ultrasonic sensor is
  the authority for stopping. YOLO does not know barriers or cones — the ultrasonic
  and colour check handle those.
* Overtaking is a fixed manoeuvre (0.5 m shift at 40°, 2 m pass) for obstacles on straights.
* The pedestrian glides instead of animating: Gazebo's skinned actors hang the
  CPU-only renderer.

## Files and credits

`tools/make_world.py` generates the three `worlds/` files · `tools/make_textures.py`
generates the track, facades, STOP sign and ArUco textures/mesh · `test/test_logic.py`
logic tests. People, cars, hydrant and cone are [Gazebo Fuel](https://app.gazebosim.org)
models by OpenRobotics (CC-BY 4.0), downloaded on first launch.
