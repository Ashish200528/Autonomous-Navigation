# Phase 8C Branch 1 architecture

Branch: feature/phase8c-lidar
Base: Phase 8B commit 61c59790ad36194c86e5db86f5a56be170dd6e42
Status: implemented with limits; Stage D avoidance and goal navigation not run.

## Reusable infrastructure

anc_navigation_common defines FREE=0, OCCUPIED=100 and UNKNOWN=-1; validates grids/sensor values; composes local grids without allowing visual evidence to erase LiDAR OCCUPIED cells; calculates speed caps from stopping samples; and supplies one authoritative command guard. /cmd_vel/requested is accepted only when request and explicit safety state are fresh and valid. Missing, stale, malformed or unsupported input yields zero.

These contracts are sensor-neutral: validated scans/camera observations feed robot-relative grids, health JSON carries validity and failure details, gates consume sensor/map and localization status, and the guard owns final velocity output. They can be reused by future ToF/RGB-only branches without anc_lidar dependencies.

## LiDAR-specific profile

anc_lidar and realistic_rgb_lidar.launch.py implement REALISTIC_RGB_LIDAR: RGB/CameraInfo, sanitized IMU, wheel odometry/EKF, parameterized planar Gazebo LiDAR, ROS bridge, scan conditioner, one-scan mapper, optional RGB fusion, measured speed gate and command guard. Topics include /realistic/scan/raw, /realistic/scan, /realistic/scan/health, /realistic/local_map/lidar, /realistic/local_map/rgb, /realistic/local_map/fused, /realistic/safety_state, /cmd_vel/requested and the sole /cmd_vel.

Generic settings in config/robot_parameters.yaml: 360 samples over ±π, 0.12–6.0 m, 5 Hz, 0.01 m noise sigma, 0.5% missed, 0.2% invalid, scan plane 0.16 m above ground. These are not YDLIDAR X2/RPLIDAR A1 or hardware specifications. Shared lidar_enabled defaults false. The LiDAR profile sets camera to 5 Hz and defaults enable_slam:=false, enable_nav2:=false for 3.5 GiB WSL.

## Mapping and runtime boundary

Finite return endpoints are OCCUPIED; only measured ray space before them is FREE. Behind hits, out of range/view, invalid and unsupported space remains UNKNOWN. Captured Gazebo +Inf no-return is normalized to max range; NaN and negative infinity remain invalid. The local map is a fresh single-scan snapshot.

The optional fuser only sees sensor-derived grids; it cannot erase LiDAR occupied cells and visual pixels do not invent metric obstacles. Graph audits show no oracle map or Gazebo truth inputs to runtime. Truth is queried only by evaluators. ORACLE_REFERENCE and REALISTIC_RGB_ONLY remain independent, and Phase 8 still publishes unconditional zero.

## Safety and results

The gate checks scan validity/freshness/rate and map health, localization freshness/covariance, processing/supervisor delay, Phase 8B stop data, Phase 4 slip allowance, footprint corridor and margin. S00 corridor was 3.20 m; 0.18 m/s required 0.656429 m. A 0.50 m clearance probe stayed unauthorized (0.582701 m required at 0.02 m/s). One command guard owned /cmd_vel.

Bounded straight and LiDAR/RGB/localization-loss trials passed at 0.05 m/s requests. Angular authority is zero because yaw stopping and turning clearance were not measured. These tests show guarded movement and fault stops, not obstacle avoidance or goal navigation.

SLAM generated /map and stationary map→odom but scan-queue-full and EKF update warnings prevented a trustworthy drift assessment. It did not show wheel-slip correction. SLAM/Nav2 remain off; Stage D is blocked pending localization, turning and resource evidence.

## Commit organization

1. Reusable sensor/map/safety contracts and unit tests.
2. LiDAR model, bridge, conditioner, local map, optional RGB fusion, launch and evidence.

Future names remain reserved: feature/phase8c-tof and feature/phase8c-rgb-only. This branch does not create them, merge into main, start Phase 9, buy hardware or modify the Arduino UNO Q application.


## Supplemental remaining-stage results (2026-11-02)

Six short S00 pure-yaw calibration pulses at +/-0.02, +/-0.05 and +/-0.10 rad/s passed. Command-to-zero output was 0.00386-0.04985 s; pose-rate settling was 0.579-0.722 s; max center drift 0.004337 m. A conservative 0.523593 m swept disk denied the near-wall probe. Production yaw remains 0.0 rad/s because angular stop distance and mixed-motion envelope are not established. Near-box/constriction denial also coincided with invalid RGB floor support.

Tuned stationary S00 SLAM generated a 155 x 115 map at 0.05 m resolution. It received 128 scans and 8 map updates in 25 seconds, but no SLAM pose messages arrived and no robot motion was evaluated. The tuned log omitted earlier queue-full/EKF missed-rate warnings but included a startup TF-cache drop and diagnostic QoS warnings. Drift reduction remains unproven.

The parking controller is a sensor-neutral, non-authoritative proposal in anc_navigation_common/parking.py; 7 unit tests pass. It is not wired to runtime velocity. Goal navigation, avoidance, replanning and live parking remain blocked. See docs/phase8c-stage-results.md, docs/phase8c-lidar-run-guide.md and docs/phase8c-hardware-transfer.md.
