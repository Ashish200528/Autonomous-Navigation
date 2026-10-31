# Configuration inventory

All physical values below are explicit configuration inputs. `null` means unknown and must be resolved before a dependent model/test is claimed complete. Do not silently substitute guesses.

| Parameter | Key | Current value | Source/status |
|---|---|---:|---|
| Chassis mass (kg) | `robot.mass_kg` | null | Measure/select before inertial model |
| Chassis length (m) | `robot.footprint.length_m` | null | Measure actual chassis |
| Chassis width (m) | `robot.footprint.width_m` | null | Measure actual chassis |
| Chassis height (m) | `robot.height_m` | null | Measure actual chassis |
| Wheel radius (m) | `robot.wheel_radius_m` | null | Measure actual wheels |
| Wheel separation (m) | `robot.wheel_separation_m` | null | Measure center-to-center |
| Camera mount XYZ (m) | `sensors.camera.position_xyz_m` | null | Measure relative to `base_link` |
| Camera mount RPY (rad) | `sensors.camera.orientation_rpy_rad` | null | Measure relative to `base_link` |
| Camera image resolution | `sensors.camera.resolution_px` | null | Select after performance check |
| Camera horizontal FOV (deg) | `sensors.camera.horizontal_fov_deg` | null | Camera spec/calibration |
| Camera intrinsics/distortion | `sensors.camera.calibration` | null | Calibrate; no metric depth assumption |
| Baseline goal | `mission.baseline_goal_map` | `(2.0, 0.0, 0.0)` | Provisional open-arena test coordinate |
| Parking position error | `mission.parking_position_tolerance_m` | `0.20` | Provisional target |
| Parking yaw error | `mission.parking_yaw_tolerance_deg` | `10` | Provisional target |
| Stationary dwell | `mission.parking_dwell_s` | `5` | Provisional target |

The YAML file `config/robot_parameters.yaml` is the machine-readable placeholder inventory. Update this table and YAML together when values are measured or decided.
