# Environment and Phase 1 verification

Checked: 2026-11-01 (Asia/Calcutta). This report records the actual Windows/WSL queries and ROS/Gazebo runs. The end-to-end Phase 1 reference goal was verified after the initial environment smoke checks.

## Host and WSL

| Item | Observed |
|---|---|
| Windows | Windows 11, build 10.0.26200.9711 |
| WSL | Default version 2; Ubuntu-24.04 listed as running under WSL 2 |
| Distribution | Ubuntu-24.04, Ubuntu 24.04.5 LTS (Noble) |
| Linux user/home | ashish, /home/ashish |
| Distro location | E:/WSL/Ubuntu-24.04 |
| Dynamic VHDX length after installs | 8,132,755,456 bytes (7.57 GiB); the file is on E: |
| WSL memory after final tests | 3.5 GiB total; about 3.0 GiB available |
| WSL swap after final tests | 1.0 GiB total and free |
| E: free after final tests | About 53 GiB |
| WSL configuration | No %USERPROFILE%/.wslconfig was present in the original environment check; no memory settings were changed |

The guest reports about 1 TiB of virtual ext4 capacity. E: free space is the useful physical storage limit. No package was installed to C:, and no Windows or WSL restart occurred.

## Graphics

- NVIDIA GeForce GTX 1650 with 4096 MiB and driver 592.82 was visible to WSL through nvidia-smi.
- WSLg variables DISPLAY=:0 and WAYLAND_DISPLAY=wayland-0 were present in the original environment check.
- RViz2 opened separately under WSLg and reported OpenGL 4.5 (GLSL 4.5). A screenshot was inspected in the setup conversation but is not saved in the repository.
- GPU vendor/acceleration was not independently measured. RViz2 logged a /run/user/1000 permission warning (0755 instead of 0700), but the window remained usable. No permission change was made.
- Gazebo/Nav2 runs and the custom robot motion trial used headless mode. Running Gazebo GUI and RViz2 together was not tested.

## Installed stack

ROS packages were installed from the official ROS 2 repository for Ubuntu Noble. Gazebo Harmonic vendor packages came through the supported Jazzy ROS package pairing; a second OSRF repository was not added.

| Component | Verified version |
|---|---|
| ROS 2 distro | jazzy |
| ros-jazzy-ros-base | 0.11.0-1noble.20261006.035407 |
| ros-jazzy-rviz2 | 14.1.24-1noble.20260915.204213 |
| ros-jazzy-ros-gz | 1.0.24-1noble.20261006.041805 |
| Gazebo Sim (Harmonic) | 8.15.0 |
| ros-jazzy-navigation2 | 1.3.13-1noble.20260921.033624 |
| ros-jazzy-nav2-bringup | 1.3.13-1noble.20261006.042503 |
| ros-jazzy-nav2-minimal-tb3-sim | 1.0.1-1noble.20261006.041307 |
| ros-jazzy-robot-localization | 3.8.3-1noble.20260915.182602 |
| ros-jazzy-slam-toolbox | 2.8.5-1noble.20260915.204151 |
| ros-dev-tools | 1.0.3 |

Both package transactions used --no-install-recommends. The first installed ROS base, RViz2, demo nodes, and developer tools (29 upgrades, 506 new packages; 235 MB archives; about 808 MB additional disk). The second installed ros_gz, Nav2, robot localization, SLAM Toolbox, and the minimal TurtleBot3 sample (24 upgrades, 836 new packages; 559 MB archives; about 2.7 GB additional disk). These were dependency upgrades required for the original toolchain setup, not a full Ubuntu upgrade. dpkg --audit produced no output. No package transaction was run during the Phase 1 goal or Phase 2 validation turn.

## Phase 1 reference navigation follow-up

The packaged TurtleBot3 Waffle/Nav2 sample was launched headlessly with a known initial map pose. The trial is classified REFERENCE because navigation used the sample's static occupancy map and simulated laser scan.

- Start pose: map (-2.0, -0.5, 0.0); initial AMCL estimate (-1.9750, -0.5351).
- Goal: map (2.0, -0.5, 0.0).
- Corridor validation: 161 samples at 0.025 m spacing; all occupancy values were known free (0), with map resolution 0.05 m.
- /navigate_to_pose returned SUCCEEDED with error code 0 and 919 feedback poses.
- Feedback displacement was 3.7296 m over 9.18 simulated seconds.
- Final AMCL pose was (1.7728, -0.6526) m, 0.2737 m from target. This met the reference trial's explicit 0.30 m XY check. The separate provisional parking target remains 0.20 m and was not evaluated.
- /cmd_vel and /cmd_vel_nav contained 78 and 80 nonzero samples. /clock, /odom and /amcl_pose were recorded.
- The MCAP ended before the last feedback message; action result and postflight snapshots provide the completion evidence.

One diagnostic launch timed out during planner lifecycle startup when no initial pose had been applied. The successful trial applies the initial pose early. The detailed status and limits are in docs/PHASE_REPORT.md and docs/evidence/README.md.

## Phase 2 custom robot follow-up

The custom headless robot built and passed forward, reverse, rotation, odometry, TF-chain, stability, and geometry-based wall-stop checks. Gazebo truth remained evaluator-only. The contact bridge publisher was present but the trial received zero contact messages; wheel odometry diverged while the robot was held against the wall. These are limitations, not hidden passes. Exact metrics and failed earlier attempts are indexed under docs/evidence/.

## Project

- Independent Git repository: /home/ashish/AutonomousNavigationCar on local branch main.
- Phase 0 baseline tag: v0.1-phase0.
- Physical robot dimensions and dynamics remain unresolved configuration values; the Phase 2 model uses documented placeholders only.
- The UNO Q application was not changed and no remote push was performed.
- scripts/source_ros_jazzy.sh sources ROS only in the current project shell; no global .bashrc edit was made.

## Official references

- ROS 2 Jazzy deb installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- ROS 2 Jazzy + Gazebo Harmonic pairing: https://gazebosim.org/docs/harmonic/ros_installation/
- Gazebo vendor packages: https://gazebosim.org/docs/harmonic/ros2_gz_vendor_pkgs/
- Nav2 Jazzy quickstart: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
