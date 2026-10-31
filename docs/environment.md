# Environment inventory — initial Phase 1 check

Date checked: 2026-11-01 (Asia/Calcutta). Read-only commands only; no package installation or configuration edits.

| Item | Result |
|---|---|
| Windows | 11, build 10.0.26200.9711 |
| WSL | 2.5.9.0; kernel 6.6.87.2-1; WSLg 1.0.66; Direct3D 1.611.1-81528511 |
| Distribution | Ubuntu-24.04 running under WSL 2; default distribution; default WSL version 2 |
| Ubuntu | 24.04.5 LTS, Noble (`noble`) |
| Linux user/home | ashish, `/home/ashish` |
| WSL location | `E:\WSL\Ubuntu-24.04`; VHDX currently 1,757,413,376 bytes |
| E: storage | NTFS, 104,857,595,904 bytes total; 62,905,548,800 bytes free (~58.58 GiB) |
| WSL memory | 3,669,200 kB total; 3,221,908 kB available; swap 1,048,576 kB total/free |
| WSL settings | No `%USERPROFILE%\.wslconfig` found |
| GPU | NVIDIA GeForce GTX 1650, 4096 MiB, driver 592.82 visible in Windows and WSL |
| Graphics | `/dev/dxg`, `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`; WSLg/Direct3D reported. `glxinfo`/`vulkaninfo` absent; `/dev/dri` absent; render test pending. |
| ROS/Gazebo/RViz | `ros2`, `gz`, `ign`, `rviz2` absent; no matching packages installed |
| APT | Noble, updates, backports and security suites configured in `ubuntu.sources`; apt metadata not refreshed |
| Project path before Phase 0 | `/home/ashish/AutonomousNavigationCar` absent |

WSL `df -h /` reports about 1 TiB virtual ext4 capacity and 955 GiB free; this is not physical host free space. E: free space above is the practical storage limit.

Initial non-elevated WSL status/list and E: volume queries returned access errors. Repeating these read-only checks with elevated terminal access succeeded.

Before installation: inspect current official ROS 2 Jazzy and Gazebo Harmonic instructions and package candidates, then install incrementally. Keep GUI smoke tests separate from headless tests due the 3.5 GiB WSL memory limit.
