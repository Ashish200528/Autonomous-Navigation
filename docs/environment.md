# Environment and Phase 1 verification

Checked: 2026-11-01 (Asia/Calcutta). Results below come from actual Windows/WSL queries and captured ROS/Gazebo smoke runs.

## Host and WSL

| Item | Observed |
|---|---|
| Windows | Windows 11, build 10.0.26200.9711 |
| WSL | 2.5.9.0; kernel 6.6.87.2-1; WSLg 1.0.66; Direct3D 1.611.1-81528511 |
| Distribution | Ubuntu-24.04, Ubuntu 24.04.5 LTS (Noble), WSL 2 |
| Linux user/home | ashish, /home/ashish |
| Distro location | E:/WSL/Ubuntu-24.04 |
| Dynamic VHDX length after installs | 8,132,755,456 bytes (7.57 GiB); the file is on E: |
| E: free after installs | 56,664,424,448 bytes (52.77 GiB) |
| WSL memory | 3,669,200 kB total; 3,211,992 kB available after install/smoke checks |
| WSL swap | 1,048,576 kB total; 1,047,016 kB free after smoke checks |
| WSL configuration | No %USERPROFILE%/.wslconfig was present; no memory settings were changed |

The guest reports a roughly 1 TiB virtual ext4 capacity. E: free space above is the useful physical storage limit. No install was directed to C:, and no WSL or Windows restart occurred.

## Graphics

- NVIDIA GeForce GTX 1650, 4096 MiB, driver 592.82 was visible to WSL.
- WSLg variables DISPLAY=:0 and WAYLAND_DISPLAY=wayland-0 were present.
- RViz2 opened under WSLg and reported OpenGL 4.5 (GLSL 4.5). A screenshot was inspected in the setup conversation; it was not saved as a repository image.
- The screenshot showed the RViz2 interface at 31 fps. GPU vendor/acceleration was not independently verified because glxinfo and vulkaninfo are absent.
- RViz2 logged: QStandardPaths: wrong permissions on runtime directory /run/user/1000/, 0755 instead of 0700. The window remained usable. No system permission change was made.
- RViz2 was closed after the visual check to release memory.

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

Both package transactions used --no-install-recommends. The first installed ROS base, RViz2, demo nodes, and developer tools (29 upgrades, 506 new packages; 235 MB archives; about 808 MB additional disk). The second installed ros_gz, Nav2, robot localization, SLAM Toolbox, and the minimal TurtleBot3 sample (24 upgrades, 836 new packages; 559 MB archives; about 2.7 GB additional disk). These were dependency upgrades required for the requested stack, not a full Ubuntu upgrade. dpkg --audit produced no output.

## Project

- Independent Git repository: /home/ashish/AutonomousNavigationCar on local branch main.
- Phase 0 baseline tag: v0.1-phase0.
- The physical robot dimensions/dynamics remain unresolved configuration values.
- No changes were made to the UNO Q application and no remote push was performed.
- scripts/source_ros_jazzy.sh provides a project-scoped ROS setup; no global .bashrc edits were made.

## Official references

- ROS 2 Jazzy deb installation: https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst
- ROS 2 Jazzy + Gazebo Harmonic pairing: https://gazebosim.org/docs/harmonic/ros_installation/
- Gazebo vendor packages: https://gazebosim.org/docs/harmonic/ros2_gz_vendor_pkgs/
- Nav2 Jazzy quickstart: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
