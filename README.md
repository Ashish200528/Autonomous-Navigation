# Autonomous Navigation Car Simulation

Independent simulation project for the Autonomous Navigation Car. The existing Arduino UNO Q application is out of scope and must remain untouched.

## Mission

Navigate an open indoor arena with arbitrary obstacles from a start pose to a map-frame goal, replan when needed, stop safely when required inputs are stale or uncertain, and park within a measured tolerance. The realistic profile must not use simulator truth, perfect depth, hidden obstacle locations, or an ideal range feed as controller inputs.

## Baseline decisions

- Host: Windows 11 + WSL2 Ubuntu 24.04 LTS.
- Simulation stack: ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2.
- Robot: differential drive; chassis dimensions and dynamics remain explicit configuration placeholders until measured.
- Goal: map-frame coordinate, with known initial map transform for the first baseline; unknown-pose acquisition is a separate experiment.
- Sensor profiles: `REFERENCE` for clearly labeled ideal/reference sensing; `REALISTIC` for camera/odometry/IMU inputs. Never merge their results.
- Perception state: `FREE`, `OCCUPIED`, `UNKNOWN`; unknown is never treated as clear.

See `docs/decisions.md`, `docs/requirements.md`, `docs/configuration.md`, `docs/metrics.md`, and `docs/issues.md` for the frozen planning baseline and open questions.

## Repository layout

- `simulation/ros2_ws/src/`: future ROS packages for description, Gazebo, bringup, perception, localization, navigation, and evaluation.
- `simulation/scenarios/`: seeded scenario registry and future worlds.
- `simulation/data/`: future logs and calibration artifacts; do not commit large raw recordings without review.
- `hardware/uno_q/`: future adapters only; preserve the existing UNO Q application.
- `scripts/`: future reproducible setup and scenario scripts.
- `tests/`: test registry and future automated checks.
- `config/robot_parameters.yaml`: configuration inventory; null values are intentional unresolved placeholders.

## Current status

Phase 0 planning baseline and initial Phase 1 environment checks are recorded in `docs/PHASE_REPORT.md` and `docs/environment.md`. No ROS/Gazebo packages have been installed and no simulation tests have been run.
