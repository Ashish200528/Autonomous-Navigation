# Configuration inventory

Measured hardware values remain null in config/robot_parameters.yaml until they are measured. The simulation_model block is a separate, explicitly provisional design used by the Phase 2/3 simulator. The Phase 3 camera and IMU parameters are also provisional. Do not use these values for sim-to-real claims.

## Hardware values still unresolved

| Parameter | Key | Current value | Source/status |
|---|---|---:|---|
| Chassis mass (kg) | robot.mass_kg | null | Measure actual chassis |
| Chassis length (m) | robot.footprint.length_m | null | Measure actual chassis |
| Chassis width (m) | robot.footprint.width_m | null | Measure actual chassis |
| Chassis height (m) | robot.height_m | null | Measure actual chassis |
| Wheel radius (m) | robot.wheel_radius_m | null | Measure actual wheels |
| Wheel separation (m) | robot.wheel_separation_m | null | Measure center-to-center |
| Camera mount XYZ (m) | sensors.camera.position_xyz_m | null | Measure relative to base_link |
| Camera mount RPY (rad) | sensors.camera.orientation_rpy_rad | null | Measure relative to base_link |
| Camera resolution | sensors.camera.resolution_px | null | Select after performance check |
| Camera horizontal FOV | sensors.camera.horizontal_fov_deg | null | Camera specification/calibration |
| Camera intrinsics/distortion | sensors.camera.calibration_file | null | Calibrate; no metric-depth assumption |
| IMU noise/covariance | sensors.imu | null | Measure or use labeled model |
| Baseline mission goal | mission.baseline_goal_map_xyyaw | (2.0, 0.0, 0.0) | Provisional project coordinate |
| Parking position error | mission.parking_position_tolerance_m | 0.20 | Provisional target |
| Parking yaw error | mission.parking_yaw_tolerance_deg | 10 | Provisional target |
| Stationary dwell | mission.parking_dwell_s | 5 | Provisional target |

## Phase 2 simulation-only placeholders

| Parameter | Value | Status |
|---|---:|---|
| Chassis length x width x height | 0.32 x 0.24 x 0.08 m | Placeholder |
| Chassis mass | 2.0 kg | Placeholder |
| Wheel radius / width / mass | 0.04 m / 0.025 m / 0.15 kg | Placeholder |
| Wheel center separation | 0.28 m | Placeholder |
| Wheel axle x | -0.04 m | Placeholder |
| Caster x / radius / mass | +0.19 m / 0.025 m / 0.05 kg | Placeholder |
| Maximum linear / angular command | 0.25 m/s / 1.0 rad/s | Placeholder |
| Camera transform | (0.10, 0, 0.10) m; zero RPY | Provisional simulated RGB camera; not measured |
| Footprint envelope | x from -0.16 to +0.215 m; y +/-0.1525 m | Conservative placeholder including wheels/caster |

## Phase 3 simulated sensor assumptions

| Sensor | Configuration | Status and limitations |
|---|---|---|
| RGB camera | Mount (0.10, 0, 0.10) m; 320x240; 60 deg horizontal FOV; 15 Hz; 0.05-10 m clipping | Provisional; simulated pinhole CameraInfo; zero distortion and no camera noise |
| IMU | Mount at base center; 100 Hz; gyro sigma 0.002 rad/s; acceleration sigma 0.02 m/s^2; zero bias | Provisional, not hardware-calibrated; Gazebo raw orientation is hidden from the public ROS IMU interface |
| ROS IMU orientation | /imu/data carries a zero quaternion with orientation_covariance[0] = -1 | Consumers must treat orientation as unavailable; do not use /anc_internal/imu_raw for navigation |

The published covariance diagonals are generated from the configured provisional noise variances. Re-measure and recalibrate before using them in a state estimator.

## Phase 4 localization parameters

All values below are simulation assumptions. EKF tuning was checked against Gazebo scenarios only and is not hardware-calibrated.

| Setting | Value | Purpose / limitation |
|---|---:|---|
| Raw odometry input | /odom | Gazebo DiffDrive output, retained as raw comparison |
| Wheel odometry output | /odometry/wheel | Same stamp, pose, and twist relayed with explicit covariance |
| Wheel vx variance | 0.0004 m^2/s^2 | Provisional; standard deviation 0.02 m/s |
| Wheel vy variance | 0.01 m^2/s^2 | Provisional; standard deviation 0.10 m/s |
| Unused wheel pose covariance | 1,000,000 | Prevents EKF from using simulator wheel pose as a localization measurement |
| EKF inputs | /odometry/wheel vx/vy; /imu/data angular_velocity.z | No wheel pose, raw Gazebo pose, or IMU orientation |
| EKF frequency / timeout | 30 Hz / 0.25 s | Filter remains active during tested IMU-only dropout while wheel data continues |
| Frames | map -> odom -> base_footprint -> base_link | Known-start map -> odom is identity at [0,0,0]; unknown-pose acquisition is not implemented |
| Wheel surface friction | mu=1.2 default; mu=0.001 slip test | Simulation values; slip test intentionally demonstrates uncorrected wheel motion |
| Slip test command | 0.25 m/s for 2 s | Test setting, not a physical motor recommendation |

Process-noise and initial-estimate covariance arrays are provisional in anc_localization/config/ekf.yaml. Measure wheel/IMU performance before hardware use. At mu=0.001 the chassis stayed stationary while wheel odometry advanced 0.4705 m and filtered odometry advanced 0.4681 m.

Update the YAML and this table together when hardware is measured or a simulation assumption changes.
