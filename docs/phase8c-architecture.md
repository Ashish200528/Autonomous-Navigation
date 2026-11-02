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
