# Phase 0 and Phase 1 setup report

**Date:** 2026-11-01 (Asia/Calcutta)
**Project:** /home/ashish/AutonomousNavigationCar on Ubuntu-24.04 in E:/WSL/Ubuntu-24.04
**Scope:** Roadmap Phase 0 plus initial Phase 1 environment, package setup, and integration smoke checks. The attached roadmap was treated as project reference material; the user's constraints govern machine changes.

## Outcome

- Phase 0 repository and planning baseline: PASS. Local Git repository exists on main; baseline tag v0.1-phase0 remains at the Phase 0 handoff. Physical robot measurements remain unresolved placeholders.
- Phase 1 platform/environment gate: PASS. WSL2, Ubuntu 24.04.5, memory, E: storage, GPU visibility, and WSLg were checked.
- Requested stack install: PASS. ROS 2 Jazzy, Gazebo Harmonic/Sim 8.15.0, ros_gz, RViz2, Nav2, robot-localization, SLAM Toolbox, and the packaged TurtleBot3 simulation installed.
- Phase 1 integration smoke gate: PASS within its stated limits. ROS talker/listener exchanged messages; Gazebo headless empty world initialized; RViz2 opened under WSLg; the packaged Nav2 TurtleBot3 sample launched and published expected simulation data.
- Full autonomous drive-to-goal, custom robot, measured vehicle dynamics, and parking acceptance: NOT TESTED. No custom robot was implemented.

## Changes

- Created the independent repository tree and planning/configuration/scenario/test documentation at /home/ashish/AutonomousNavigationCar. The Phase 0 tag is v0.1-phase0.
- Installed packages from the official ROS 2 Jazzy/Noble repository using --no-install-recommends. The ROS-supported Gazebo Harmonic vendor packages were used; a separate OSRF apt source was not added.
- Added scripts/source_ros_jazzy.sh so ROS is sourced only in a project terminal. No global ~/.bashrc setting was changed.
- Updated README, environment inventory, test registry, evidence index, and this report.
- No edits to the existing Arduino UNO Q application; no push to a remote; no Windows/WSL restart.

## Machine and storage findings

- Windows 11 build 10.0.26200.9711; WSL 2.5.9.0 / kernel 6.6.87.2-1; Ubuntu 24.04.5 LTS (Noble); Linux account ashish.
- WSL exposes 3,669,200 kB RAM (about 3.5 GiB) and 1,048,576 kB swap. After the install and smoke checks, 3,211,992 kB RAM and 1,047,016 kB swap remained available.
- E: had 52.77 GiB free after installation. The distro's ext4.vhdx file length was 8,132,755,456 bytes (7.57 GiB). The guest's 1 TiB virtual ext4 report is not the physical E: capacity.
- GTX 1650 (4 GiB) and WSLg were visible. RViz2 reported OpenGL 4.5 and displayed successfully. The graphics vendor/acceleration was not independently measured. RViz2 emitted a non-fatal /run/user/1000 permission warning; no system permission change was made.

## Installed versions

- ROS_DISTRO=jazzy; ros-jazzy-ros-base 0.11.0-1noble.20261006.035407.
- ros-jazzy-rviz2 14.1.24-1noble.20260915.204213.
- Gazebo Sim 8.15.0 (Harmonic); ros-jazzy-ros-gz 1.0.24-1noble.20261006.041805.
- navigation2 and nav2-bringup 1.3.13; minimal TurtleBot3 sim 1.0.1.
- robot-localization 3.8.3; SLAM Toolbox 2.8.5; ros-dev-tools 1.0.3.
- dpkg --audit returned no unfinished package configuration.

The ROS base/RViz transaction added 506 packages and upgraded 29 packages (235 MB downloaded; about 808 MB additional disk). The simulator/Nav2 transaction added 836 packages and upgraded 24 packages (559 MB downloaded; about 2.7 GB additional disk). No full Ubuntu upgrade was run.

## Smoke-test results

| Test | Result | Evidence and limit |
|---|---|---|
| T00 ROS talker/listener | PASS | Six talker messages were published and four were received by the listener. Logs and counts are in docs/evidence/ros-talker.log, ros-listener.log, and ros-talker-listener-result.txt. Both processes were stopped after the receive window. |
| T01 Gazebo empty world | PASS | Gazebo Sim Server v8.15.0 loaded its installed empty.sdf, initialized world empty, and advertised world control/create services before the deliberate 12-second bound. Startup log is docs/evidence/gazebo-headless.log. |
| T02 RViz2 / WSLg | PASS | RViz2 opened and rendered under WSLg with OpenGL 4.5. The screenshot was inspected in the setup conversation but is not stored as a repository image. Startup output is docs/evidence/rviz2.log. |
| T03 Nav2 TurtleBot3 sample | PASS for launch/data smoke only | The headless launch initialized robot_state_publisher, loaded Nav2 localization, and reported Entity creation successful. Direct topic probes received /clock, /odom, /tf, and /scan messages; Gazebo listed the corresponding topics. Logs: docs/evidence/nav2-headless.log, nav2-diagnostic-launch.log, nav2-diagnostic-graph.log. No AMCL initial pose or navigation goal was sent, so map-frame localization and goal performance remain untested. |
| T04 Phase 2 differential-drive behavior | NOT RUN | Future phase. |

The first Nav2 launch log includes early transform lookup warnings. The later live probe confirmed odometry, transforms, and laser data flowing. The headless smoke did not provide an AMCL initial pose, so it did not test the map-to-odom localization transform or a goal. RViz2 was tested separately rather than concurrently with Gazebo/Nav2 to keep within the 3.5 GiB WSL memory cap.

## Errors and limitations

- Non-elevated WSL commands initially returned Wsl/E_ACCESSDENIED. The checks/installations succeeded through the authorized WSL access route.
- An initial repository helper invocation returned E: Invalid operation update; a direct apt-get update succeeded. Package configuration completed and dpkg --audit was clean.
- APT's non-interactive session fell back from Dialog to Readline; installation completed.
- RViz2 reported /run/user/1000 permissions 0755 rather than 0700, but the GUI remained usable. Do not infer discrete GPU acceleration from the OpenGL version alone.
- The Nav2 sample smoke is not an autonomous-navigation result. Initial pose, goal selection, path completion, safety stop, and parking remain to be exercised.

## Phase 1 gate

The roadmap's environment and launchability checks are satisfied: ROS nodes communicated, Gazebo initialized headlessly, RViz2 rendered under WSLg, and the packaged Nav2 simulation launched and exposed expected simulator topics. The phase result is a verified software baseline, not a claim that the project's own car is implemented or has passed navigation tests.

## Next

Begin the next roadmap phase with a measured robot model and a project-specific world. Keep Nav2 test runs headless by default; enable GUI only for focused inspection. Before claiming navigation success, supply an AMCL initial pose, send a map-frame goal, and preserve the result plus pose/trajectory metrics.

## References used

- ROS 2 Jazzy deb installation: https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst
- ROS 2 Jazzy and Gazebo Harmonic pairing: https://gazebosim.org/docs/harmonic/ros_installation/
- Gazebo ROS vendor packages: https://gazebosim.org/docs/harmonic/ros2_gz_vendor_pkgs/
- Nav2 Jazzy quickstart: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
