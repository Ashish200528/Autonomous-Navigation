# Phase 8C Branch 1 architecture

Branch: feature/phase8c-lidar
Baseline: Phase 8B commit 61c59790ad36194c86e5db86f5a56be170dd6e42

## Reusable infrastructure

The anc_navigation_common package defines standard occupancy values (FREE 0, OCCUPIED 100, UNKNOWN -1), validates grid geometry, classifies invalid sensor values as UNKNOWN, composes same-frame occupancy grids without allowing visual evidence to erase LiDAR OCCUPIED cells, calculates a conservative speed cap from measured stop-distance samples, and provides one command-authority node with independent command and safety-state watchdogs.

The command guard receives controller output on /cmd_vel/requested and publishes authoritative /cmd_vel only when both the request and an explicit valid safety state are fresh. Missing, stale, malformed, non-finite, or unsupported differential-drive commands produce zero. Hard velocity limits remain separate from a sensor-specific gate's lower limits.

Future RGB-only and ToF branches can reuse this package, OccupancyGrid API, safety-state JSON contract, command watchdog, and tests without depending on the LiDAR package.

## LiDAR-specific target design (not implemented by the reusable-infrastructure commit)

The planned REALISTIC_RGB_LIDAR profile will compose the existing simulated RGB camera, CameraInfo, sanitized IMU, wheel odometry/EKF, and camera perception with a planar LiDAR, scan conditioner, local scan occupancy grid, conservative RGB fusion, scan-matching localization, and Nav2. It must not load a reference occupancy map or feed evaluator geometry to runtime nodes.

The intended ROS contracts are /realistic/scan/raw (Gazebo bridge input), /realistic/scan (validated/noisy LaserScan), /realistic/local_map/lidar, /realistic/local_map/rgb, /realistic/local_map/fused, /realistic/safety_state, /cmd_vel/requested, and the sole authoritative /cmd_vel.

The provisional scanner is a physical 2D plane at the configured lidar_link height. It cannot detect objects that do not intersect that plane or surfaces outside the scan range. Gaussian distance noise is simulated in Gazebo; the ROS scan conditioner can inject bounded missed/invalid samples. Range, rate, resolution, FOV, pose, and error parameters are explicit assumptions, not purchased-device specifications.

## Safety boundary

A map cell becomes FREE only when a valid finite ray crosses it before a valid return, within the sensor's justified range. A valid return endpoint is OCCUPIED. NaN, infinity, below-minimum, above-maximum, occluded, unobserved, and out-of-map cells remain UNKNOWN. Single scans are not accumulated into unbounded free space. The footprint is inflated when checking trajectories.

The speed gate must use recent scan/map and localization-health inputs, observed scan-processing delay, supervisor period, a measured stopping-distance table, localization allowance, footprint clearance, and explicit safety margin. A LiDAR-only corridor may pass a limited simulation gate even while RGB-only remains blocked; this is not a hardware safety claim. Phase 8 continues to publish unconditional zero and is tested independently.

## Branch commit split

1. Reusable sensor/map/safety contracts and tests.
2. Gazebo LiDAR model, ROS bridge and scan conditioner.
3. LiDAR local map and optional RGB fusion.
4. REALISTIC_RGB_LIDAR SLAM/Nav2 and measured safety gate.
5. Evidence, regression checks and updated project docs.

Future branch names reserved only: feature/phase8c-tof, feature/phase8c-rgb-only. Neither is created here. No physical sensor purchase, UNO Q change, Phase 9, or merge into main is part of this work.
