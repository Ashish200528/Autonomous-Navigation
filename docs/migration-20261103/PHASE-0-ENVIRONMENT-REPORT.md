# Autonomous Navigation Car — Phase 0/1 Setup Report

**Date:** 2026-11-01 (Asia/Calcutta)  
**Scope:** Roadmap Phase 0, initial Phase 1 environment checks, requested ROS/Gazebo/Nav2 installation, and bounded integration smoke checks.

## Outcome

The independent project is set up at /home/ashish/AutonomousNavigationCar inside Ubuntu 24.04 on the E: WSL disk. Phase 0 is complete and tagged **v0.1-phase0**. ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2, robot-localization, SLAM Toolbox, and the TurtleBot3 sample are installed and verified.

T00-T03 passed for their smoke-test scope. This verifies the software baseline and launchability. It does not verify the project's own car, autonomous drive-to-goal behavior, parking, or measured vehicle dynamics.

## Environment

- Windows 11; WSL 2.5.9.0; Ubuntu 24.04.5 LTS.
- WSL has 3.5 GiB RAM and 1 GiB swap. About 3.06 GiB RAM and almost all swap remained available after setup checks.
- E: had 52.77 GiB free after installation. The E:-hosted Ubuntu VHDX file length is 7.57 GiB.
- NVIDIA GTX 1650 is visible to WSL. RViz2 opened under WSLg and reported OpenGL 4.5; GPU acceleration was not independently measured.
- No WSL memory settings were changed. No restart occurred, and no installation was directed to C:.

## Installed and verified

- ROS 2 Jazzy: ros-jazzy-ros-base 0.11.0.
- Gazebo Harmonic / Gazebo Sim 8.15.0 and ros-jazzy-ros-gz 1.0.24.
- RViz2 14.1.24.
- Nav2 / nav2-bringup 1.3.13; TurtleBot3 minimal sim 1.0.1.
- Robot localization 3.8.3; SLAM Toolbox 2.8.5.
- dpkg audit returned no unfinished package configuration.

Installation used the official ROS repository and the no-recommends option. Required dependencies added 506 packages in the first transaction and 836 in the second; E: still has ample free space. A full Ubuntu upgrade was not run. The ROS-supported Jazzy/Harmonic vendor packages were used, without adding a separate Gazebo repository.

A project-scoped helper, scripts/source_ros_jazzy.sh, was added. Source it from the project directory in a WSL shell; it does not modify the global shell startup file or the separate Arduino UNO Q application.

## Smoke checks

| Check | Result |
|---|---|
| ROS talker/listener | PASS — six messages published and four received. |
| Gazebo empty world | PASS — Gazebo Sim 8.15.0 loaded and initialized the empty world headlessly. |
| RViz2 / WSLg | PASS — window opened and rendered; screenshot inspected in this setup conversation, not saved as a repository image. |
| Nav2 TurtleBot3 sample | PASS WITH LIMITS — headless launch created the robot; live probes received /clock, /odom, /tf, and /scan. No initial pose or goal was submitted. |

RViz2 logged a non-fatal runtime-directory permission warning, but remained usable. The first Nav2 launch logged transform lookup warnings; the later topic probe confirmed live odometry, transforms, and laser data. Map-frame localization and goal performance remain untested because this smoke run did not supply an AMCL initial pose.

## Preserved project evidence

The full phase report, environment inventory, test registry, command logs, and test outputs are in the WSL project at /home/ashish/AutonomousNavigationCar/docs/PHASE_REPORT.md, /home/ashish/AutonomousNavigationCar/docs/environment.md, and /home/ashish/AutonomousNavigationCar/docs/evidence/.

The attached roadmap was used as reference material. The user's constraints were followed: the simulation project remains separate from the Arduino UNO Q app; no source files were edited; no remote push or restart was performed.

## Next

The next meaningful test is to supply an AMCL initial pose in the packaged sample and send a map-frame navigation goal. After that, proceed with the measured vehicle model and custom world. Keep the normal test run headless on this 3.5 GiB WSL memory limit.

