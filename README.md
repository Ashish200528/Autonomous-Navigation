# Autonomous Navigation Car Simulation

Independent simulation project for the Autonomous Navigation Car. The existing Arduino UNO Q application is out of scope and must remain untouched.

## Mission

Navigate an open indoor arena with arbitrary obstacles from a start pose to a map-frame goal, replan when needed, stop safely when required inputs are stale or uncertain, and park within a measured tolerance. The realistic profile must not use simulator truth, perfect depth, hidden obstacle locations, or an ideal range feed as controller inputs.

## Baseline decisions

- Host: Windows 11 + WSL2 Ubuntu 24.04 LTS.
- Simulation stack: ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2.
- Robot: differential drive; chassis dimensions and dynamics remain explicit configuration placeholders until measured.
- Goal: map-frame coordinate, with known initial map transform for the first baseline; unknown-pose acquisition is a separate experiment.
- Sensor profiles: REFERENCE for clearly labeled ideal/reference sensing; REALISTIC for camera/odometry/IMU inputs. Never merge their results.
- Perception state: FREE, OCCUPIED, UNKNOWN; unknown is never treated as clear.

See docs/decisions.md, docs/requirements.md, docs/configuration.md, docs/metrics.md, and docs/issues.md for the frozen planning baseline and open questions.

## Repository layout

- simulation/ros2_ws/src/: future ROS packages for description, Gazebo, bringup, perception, localization, navigation, and evaluation.
- simulation/scenarios/: seeded scenario registry and future worlds.
- simulation/data/: future logs and calibration artifacts; do not commit large raw recordings without review.
- hardware/uno_q/: future adapters only; preserve the existing UNO Q application.
- scripts/: project-scoped environment and future reproducible scenario scripts.
- tests/: test registry and future automated checks.
- config/robot_parameters.yaml: configuration inventory; null values are intentional unresolved placeholders.

## Start a project shell

From a WSL terminal:

    cd /home/ashish/AutonomousNavigationCar
    source scripts/source_ros_jazzy.sh
    echo $ROS_DISTRO

The project-scoped script sources ROS 2 Jazzy only in the current terminal. It does not alter the user's global shell startup file.

## Headless packaged sample

To launch the installed TurtleBot3/Nav2 sample without RViz:

    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False

The sample starts Gazebo Harmonic and the Nav2 stack. The initial-pose estimate is still an input to localization; the smoke run did not submit an initial pose or navigation goal. It is a stack check, not a project navigation result.

## Current status

Phase 0 planning baseline is complete and tagged v0.1-phase0. The Phase 1 environment and integration smoke checks are recorded in docs/PHASE_REPORT.md and docs/environment.md. ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2, robot-localization, SLAM Toolbox, and the packaged TurtleBot3 sample are installed in the E:-hosted WSL distro. T00-T03 are recorded as passing within their stated smoke-test limits; no custom vehicle or end-to-end goal run has been completed.
