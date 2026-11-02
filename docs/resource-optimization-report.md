# Windows + WSL Resource Optimization Report

Date: 2026-11-02
Project: Autonomous Navigation Car
Repository: /home/ashish/AutonomousNavigationCar
Branch: feature/phase8c-lidar

## Outcome

The resource audit and launcher safety changes are complete. Runtime benchmarking is **blocked by the current Windows memory preflight**. The latest read-only sample at 22:54 IST showed 1,665 MiB available on Windows, below the headless Gazebo runner's 2,048 MiB minimum. No Gazebo, SLAM, Nav2, GUI, or RViz benchmark was launched in this pass.

## Resource findings

| Area | Observed | Implication |
|---|---|---|
| Windows memory | 7.34 GiB visible physical RAM. Available RAM varied between 990 MiB and 3,335 MiB; latest 1,665 MiB (77% load). | Headless Gazebo host gate is not met; 2,048 MiB is only a minimum. |
| Windows CPU | 12 logical processors; a 22:49 sample recorded 4.6% system CPU. Windows CPU/paging performance-counter queries returned c0000bb8; later per-process samples were mostly 0-2%. | No sustained CPU bottleneck was observed, but CPU/paging telemetry is incomplete. |
| Windows GPU | NVIDIA GeForce GTX 1650, 4,096 MiB reported; query showed 0 MiB used / 0%. | WSL can query the GPU; this does not prove Gazebo rendering performance. |
| WSL | Ubuntu 24.04 / WSL 2, kernel 6.6.87.2. Baseline 3.5 GiB total, 3.1 GiB available, 1 GiB swap unused; latest launcher sample 3,140 MiB available. | WSL had headroom; Windows host RAM is the blocker. |
| Disk | Linux project filesystem reported about 970,480 MiB free. Windows E: had about 51 GiB free. Distro path: E:\WSL\Ubuntu-24.04. | Existing disk checks pass; no cleanup was needed. |
| Simulation | ROS 2 Jazzy, ros_gz_sim, Gazebo Sim and project packages found. No active project session. WSLg display variables are present. | Environment preflight succeeds. |
| Graphics | /usr/lib/wsl/lib/nvidia-smi sees the GTX 1650. glxinfo is absent. | OpenGL renderer path remains unverified; no package was installed to query it. |

Host available RAM varied without any cleanup: 3,335 MiB at 22:49, later 1,937 and 1,840 MiB, then 1,665 MiB at 22:54. These swings are not attributable to this task.

## Safe resource actions and WSL allocation

- No Windows apps, WSL services, or background processes were closed. The active ChatGPT/Codex process tree and Chrome may contain unsaved work. Memory Compression, Windows Defender, Explorer, display/input services and vmmemWSL were treated as protected.
- No startup apps were disabled, software uninstalled, caches/files deleted, or system settings changed. No WSL or Windows restart was performed.
- No performance recovery is claimed: no app was closed and host free RAM decreased during the last few diagnostic samples.
- The Windows profile had no .wslconfig. Microsoft documents WSL 2's default memory limit as 50% of Windows memory, consistent with the observed approximately 3.5 GiB WSL total. .wslconfig is global across WSL 2 distributions and may require a WSL VM shutdown/restart. Microsoft documents experimental autoMemoryReclaim default as dropCache. With ample WSL headroom, a Windows-side memory bottleneck and the no-restart constraint, changing global WSL allocation was not justified. Source: [Microsoft Learn: Advanced settings configuration in WSL](https://learn.microsoft.com/en-us/windows/wsl/wsl-config).

## Launcher changes

The three Windows demo BAT entry points delegate resource checks and session handling to scripts/windows_demo_launcher.sh. Desktop and Downloads copies were observed pairwise identical, so the shared helper remains the single implementation point.

The helper now preserves the active ChatGPT/Codex execution and all user apps. It no longer offers to close ChatGPT/Codex after a timeout and no longer calls CloseMainWindow. Diagnostics still report protected process identity; tracked project-session cleanup, resource recheck and GUI/RViz/disk gates remain. The session stop sends graceful SIGINT only and does not escalate to SIGTERM/SIGKILL. BAT copies were not changed.

## Verification and limits

- bash -n scripts/windows_demo_launcher.sh: PASS
- python3 tests/windows_demo_resource_policy_test.py: 5/5 PASS
- git diff --check: PASS
- bash scripts/windows_demo_launcher.sh diagnostics: PASS; detected ROS, Gazebo, packages, branch, WSL path and no active project session.
- Headless Gazebo, LiDAR-SLAM, Nav2, GUI and RViz runtime benchmarks: NOT RUN; Windows memory was below the 2,048 MiB headless gate.
- No screenshot/video was produced because no simulation was started.

See docs/evidence/resource-optimization/README.md for evidence logs.
