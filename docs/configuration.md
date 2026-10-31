# Configuration inventory

Measured hardware values remain null in config/robot_parameters.yaml until they are measured. The simulation_model block is a separate, explicitly provisional design used only by the Phase 2 simulator. Do not use its values for sim-to-real claims.

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
| Camera transform | (0.10, 0, 0.10) m; zero RPY | Frame only; no sensor |
| Footprint envelope | x from -0.16 to +0.215 m; y +/-0.1525 m | Conservative placeholder including wheels/caster |

Update the YAML and this table together when hardware is measured or a simulation assumption changes.
