# Phase 8: RGB-constrained local traversability and safe stopping

Date: 2026-11-02 (Asia/Calcutta)  
Project: `/home/ashish/AutonomousNavigationCar`  
Classification: **PASS WITH LIMITS for the stop-only baseline; autonomous motion is BLOCKED by the feasibility gate.**

Phase 8 connects Phase 7 RGB perception to a conservative local grid and a single safe-stop command authority. The system remains stationary. It does not claim RGB-only goal navigation.

## Feasibility decision

The S00 Phase 7 mask projects FREE image evidence only to a short floor patch:

| Measure | Finding |
|---|---:|
| Robot simulated front envelope | x = 0.215 m in base_link |
| Camera origin | x = 0.10 m, z = 0.10 m above base_link; 0.18 m above floor |
| Camera orientation | Forward and level; verified from live TF |
| Camera model | 320 x 240, fx/fy about 277.13 px, principal point (160,120), zero distortion, 60-degree horizontal FOV |
| Projected FREE patch | x = 0.604-0.694 m; about 0.090 m longitudinal extent |
| UNKNOWN gap after robot front | about 0.389 m |
| Camera age p95 | 0.0538 s |
| Perception latency p95 | 29.884 ms |
| Supervisor period | 0.050 s |
| Reaction delay p95 | about 0.1337 s |
| Reaction-only distance at 0.18 m/s | about 0.0241 m |
| Margin before braking at 0.18 m/s | about 0.2241 m |
| Emergency braking distance | **Not measured** |

The observed FREE patch is disconnected from the footprint by UNKNOWN and is shorter than reaction distance plus the explicit 0.20 m margin, even before braking. RGB OCCUPIED pixels have no defensible metric distance. Phase 7's accepted challenge aggregate also has 80.98% UNKNOWN coverage and 26.67% occupied recall. Phase 4 measured about 0.468 m filtered endpoint error under wheel slip while the chassis remained stationary; a known-start single-run localization result does not bound that error.

Accordingly, the motion gate is STOP_ONLY. The Phase 8 command path is hard-locked to zero, and no realistic motion or goal was attempted. Phase 6 ORACLE_REFERENCE navigation remains a separately runnable benchmark.

Machine-readable calculations: `docs/evidence/phase8/phase8-feasibility.json`.

## Runtime architecture

Build and launch from WSL:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch anc_realistic_navigation phase8_realistic.launch.py
~~~

The separate launch is headless. It starts Gazebo, RGB perception, the wheel/IMU EKF, the local mapper, and the safe-stop supervisor. It does not start Nav2 or a map server. Phase 8 passes `publish_known_start_tf:=false`; no known-start map-to-odom transform is published. The shared launch parameter still defaults to true so the existing Phase 6 reference profile is preserved.

Runtime topic and frame contract:

| Node | Inputs | Outputs |
|---|---|---|
| `anc_rgb_perception` | `/camera/image_raw`, `/camera/camera_info`, `/clock` for timestamp age | `/perception/image_mask`, `/perception/confidence`, `/perception/health`, overlay |
| `anc_local_traversability` | mask, confidence, health, CameraInfo, clock, `/odometry/filtered`, TF `base_link <- camera_optical_frame` | `/realistic/local_traversability`, `/realistic/navigation_state` |
| `anc_realistic_safe_stop_supervisor` | `/cmd_vel/requested`, navigation state | sole `/cmd_vel` publisher, `/realistic/supervisor_state` |

The grid is a fresh snapshot in `base_link`, 75 x 75 cells at 0.02 m resolution. FREE is 0, OCCUPIED is 100, UNKNOWN is -1. Only high-confidence FREE pixels with valid zero-distortion CameraInfo and the checked TF transform are projected to the ground plane. Unknown cells are never filled from history. RGB OCCUPIED pixels remain UNKNOWN metric cells and add `unlocalized_occupied_evidence` to the block reasons.

The supervisor accepts requested commands only on `/cmd_vel/requested`; its Phase 8 code publishes all-zero Twist messages on `/cmd_vel` at 20 Hz, including during command timeout. There is no competing command publisher in the Phase 8 graph. Runtime nodes do not subscribe to Gazebo truth, scenario manifests, obstacle geometry, depth, or the Phase 6 reference map.

## Verification and evidence

- Full seven-package `colcon build --symlink-install`: passed.
- Static projection and fail-closed checks: 7/7 passed.
- Nine bounded live S00/S01/S02/S03/S09 trials: 9/9 passed, including S02 seeds 101 and 202.
- Actual `/cmd_vel` samples across those runs: 1,052; all 1,052 were zero. One run separately verified command-timeout behavior.
- Maximum evaluator-only truth displacement among those runs: 0.00001055 m, below the declared 0.02 m stop tolerance.
- S00 with fresh RGB: 164 FREE grid cells, 5,461 UNKNOWN, 0 fabricated OCCUPIED cells. The map state reports the unknown forward corridor and unlocalized obstacle evidence.
- S00 without camera: all 5,625 grid cells UNKNOWN.
- S00 with filtered localization unavailable: all 5,625 grid cells UNKNOWN.
- Live command graph: one `/cmd_vel` publisher, the Phase 8 supervisor; no Nav2 node and no `map -> odom` TF.
- Contact telemetry: repeated caster-to-ground surface reports were filtered; no non-ground contact event occurred.
- Phase 7 live fault harness: missing RGB, stale timestamp, fresh RGB, and stale-to-fresh recovery passed.
- Phase 6 S00 was rerun separately under `ORACLE_REFERENCE`: NavigateToPose SUCCEEDED, truth path 1.8546 m, final goal error 0.1454 m, conservative clearance 1.7314 m, localization drift 0.0019 m, and no collision event. This is a privileged map/known-start benchmark, not Phase 8 navigation.

Per-run JSON, CSV, and launch logs are under `docs/evidence/phase8/runs/`. Aggregates are in `docs/evidence/phase8/phase8-summary.json`. The separate Phase 6 rerun is under `docs/evidence/phase6/runs/phase8-final-reference-s00-20261102/`.

## Limits and remaining work

- No locally validated movement or goal-reaching was demonstrated; these remain blocked by the disconnected FREE patch and unmeasured stopping distance.
- No sudden camera dropout while the robot was moving was tested, because the controller never enabled motion. Camera-absent startup and perception stale-to-fresh recovery were tested.
- No emergency-braking distance is claimed. Ground-contact reports are not interpreted as obstacle collisions.
- The grid's camera pose, robot footprint, floor plane, lighting, and pinhole calibration remain simulated assumptions, not measured hardware values.
- Phase 8 does not modify the Arduino UNO Q application, transfer commands to hardware, implement parking, or start a later phase.

The correct current behavior is STOP/BLOCKED when evidence does not establish a continuous safe corridor.
