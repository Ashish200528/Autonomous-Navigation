# Windows Gazebo Demonstration Delivery and Verification

Date: 2026-11-02
Project: `/home/ashish/AutonomousNavigationCar`
Branch: `feature/phase8c-lidar`

## Outcome

The three Windows launchers and their README were copied to the actual Windows Desktop and Downloads Known Folders. A CMD-specific quoting problem was then reproduced and fixed in all six launcher copies. The Desktop and Downloads copies now match by SHA-256. The real Desktop menu opened, displayed its choices, and exited on option 7 without starting a simulation. Launcher code and ROS launch descriptions passed static/help checks; the Phase 6 and LiDAR RViz profiles are installed in the ROS workspace.

During the initial delivery verification, no Gazebo, RViz, navigation, sensor, or motion demonstration was started. A later user-triggered Phase 6 GUI attempt is documented below; it opened the world but failed before goal dispatch.

## Environment observed

- WSL distro: `Ubuntu-24.04`; registered base path: `E:\WSL\Ubuntu-24.04`.
- WSL kernel: `6.6.87.2-microsoft-standard-WSL2`; WSLg variables: `WAYLAND_DISPLAY=wayland-0`, `DISPLAY=:0`.
- Windows physical memory reported by the diagnostic: `7.34 GiB`; latest available reading: `709 MiB` (recent readings ranged from `628 MiB` to `856 MiB`), load `90%`, 12 logical CPUs.
- WSL available memory: `3134 MiB`; swap: `1024 MiB`; project filesystem available: `970562 MiB`.
- GPU query: `NVIDIA GeForce GTX 1650`, `4096 MiB`, driver `592.82`.
- ROS 2 Jazzy imports successfully (`rclpy` from `/opt/ros/jazzy/lib/python3.12/site-packages/rclpy`). `ros_gz_sim` resolves under `/opt/ros/jazzy`; `gz` resolves to `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`.
- Project packages resolve from the local workspace install: `anc_gazebo`, `anc_navigation`, and `anc_lidar`.

## Project changes

- Phase 6 runner supports opt-in Gazebo GUI, RViz, and holding the paused result window; existing headless defaults remain.
- Reference navigation launch forwards its GUI option and can start the configured Phase 6 RViz view.
- LiDAR launch separates Gazebo GUI from optional RViz; its RViz profile includes the RGB image panel.
- LiDAR demo wrapper and GUI monitor add status presentation and a bounded S00-only straight request through the existing guarded navigation topic. The command guard remains the `/cmd_vel` owner. Other scenarios are sensing-only; LiDAR Nav2 remains readiness-gated.
- Windows WSL launcher helpers provide diagnostics, menu/session tracking, and bounded cleanup without force-killing a process. They do not install packages or change Windows/WSL settings.

## Verification performed

- Initial ROS Python `--help` invocation before sourcing the ROS environment returned `ModuleNotFoundError: No module named 'rclpy'`. Repeating after sourcing `/opt/ros/jazzy/setup.bash` and the workspace succeeded; `rclpy` import succeeded.
- `python3 scripts/phase6_navigation_runner.py --help`: passed; includes `--gui`, `--rviz`, `--hold-window`.
- `python3 scripts/phase8c_gui_demo.py --help`: passed.
- `bash scripts/run_lidar_demo.sh --help`: passed; documents GUI, RViz, demo status/forward, and hold options.
- `ros2 launch anc_navigation reference_navigation.launch.py --show-args`: passed; showed `gui` and `rviz` defaults as false.
- `ros2 launch anc_lidar realistic_rgb_lidar.launch.py --show-args`: passed; showed separate `gui` and `rviz` defaults as false; Nav2 and SLAM remain opt-in.
- First installed-file check found the Phase 6 RViz profile missing from the stale install tree. Ran `colcon build --executor sequential --packages-select anc_navigation --event-handlers console_direct+`; result: `Finished <<< anc_navigation [9.91s]`, `Summary: 1 package finished [10.2s]`. The Phase 6 RViz profile then appeared in the install tree. Both installed RViz profiles were subsequently confirmed.
- The earlier workspace build completed three packages (`anc_bringup`, `anc_navigation`, `anc_lidar`); the focused navigation package rebuild above was done after discovering the stale installed config.
- `bash -n` on all three helper scripts, Python AST parsing of both changed Python scripts, and `git diff --check`: passed.
- Launcher read-only diagnostics: passed; reported the correct distro base path, ROS/Gazebo availability, current branch and `NO_ACTIVE_PROJECT_SESSION`. A separate scan reported `NO_PROJECT_SIM_PROCESS`.
- Parking status command explicitly reported: `AUTONOMOUS PARKING — NOT YET LIVE-VALIDATED`.
- No screenshots or recording were produced because no graphical simulation was started.

## Windows files delivered

Each folder contains these exact four files:

- `C:\Users\ajayk\Desktop\01_WATCH_ROBOT_NAVIGATE.bat`
- `C:\Users\ajayk\Desktop\02_WATCH_RGB_LIDAR.bat`
- `C:\Users\ajayk\Desktop\03_GAZEBO_DEMO_MENU.bat`
- `C:\Users\ajayk\Desktop\README_GAZEBO_DEMOS.txt`
- The same four filenames under `C:\Users\ajayk\Downloads\`.

The source staging folder had exactly four files. Existing destination files were checked first; no conflicting names existed. After the quoting correction, SHA-256 hashes for each Desktop launcher match the corresponding Downloads launcher. The table lists the current launcher hashes; the README hash remains unchanged:

| File | SHA-256 |
|---|---|
| `01_WATCH_ROBOT_NAVIGATE.bat` | `7036A99C7C7FBEE4990D781ADC045C5B742EA3D55427FB5B7F31425CF0D14258` |
| `02_WATCH_RGB_LIDAR.bat` | `A538A9D8888E787B2DA34A4574A5CC59D5D90D9B21922C146B0D2281948EF7A0` |
| `03_GAZEBO_DEMO_MENU.bat` | `DD5B6789B54E7C013DD715BF2A4A1FE039C2CC1C9112A239D49528AEDFAC8A39` |
| `README_GAZEBO_DEMOS.txt` | `2E44CEAE4A1035CBBB243D3C1FC053E96B2AAF19C161EDF05AD4758FD6489479` |

## Not run / remaining

The launcher requires at least `2560 MiB` free Windows RAM for Gazebo and `3328 MiB` to add RViz (also at least `1536 MiB` free in WSL). At initial delivery Windows free memory was `709 MiB`; a later diagnostic during this diagnosis read `982 MiB` and WSL available memory `3118 MiB`. The helper will block the launch at either Windows reading. No app was closed, no process was force-killed, and no restart was triggered.

After Windows memory is freed, the remaining work is to run and visually verify each requested demonstration, observe the safety/sensor displays and shutdown behavior, and capture the requested video or screenshots if the project task still calls for them. Live Phase 6 GUI navigation, RGB/LiDAR GUI behavior, movement, LiDAR Nav2 goal navigation, repeated-launch cleanup, and parking remain unverified. Historical headless Phase 6 evidence is not evidence of these GUI runs.

## CMD launcher correction

The first user launch showed `WSL_E_DISTRO_NOT_FOUND` after the distribution list, even though `Ubuntu-24.04` was registered. Reproducing the BAT argument form in CMD isolated the issue to quoting `%DISTRO%` in `--distribution "%DISTRO%"`. WSL accepted the literal/unquoted selector, while the quoted variable form failed. Updated all three launchers in both Known Folders to use `--distribution %DISTRO%`. Re-ran the actual Desktop menu interactively: the menu appeared, option 7 exited cleanly, and no demo or Gazebo process was started. The WSL1 unsupported message remains a status warning; WSL2 is active and the launcher works.


## Follow-up: stationary Phase 6 GUI run (2026-11-02)

The user's S03 screenshot corresponds to `phase6-S03-seed101-20261102T151437Z-496.log`. The run report says `Goal status: unavailable`, `action: null`, and `runner_error: RuntimeError: Evaluator-only Gazebo truth service returned no robot pose`. The same failure appears in the immediately preceding GUI S00 and S01 run logs. In `scripts/phase6_navigation_runner.py`, the evaluator waits for `/gzserver/get_entity_state` and raises before it sends `/navigate_to_pose`; therefore Nav2 received no goal and published no motion command. This is why the robot stayed at its spawn location. The scenario was not the cause.

The GUI branch in `anc_bringup/launch/sim.launch.py` had started standalone `gz sim` with both server and GUI. That path did not provide the ROS `GzServer` simulation-interface service used by the evaluator. The launch now uses `ros_gz_sim`'s `gz_server.launch.py` in both headless and GUI modes, and starts Gazebo with `gz sim -g -v 3` as a separate GUI-only client when requested. This restores the service layer while keeping the visible scene.

Rebuilt only `anc_bringup`: `Finished <<< anc_bringup [3.18s]`, `Summary: 1 package finished [3.33s]`. The launch file passed Python AST parsing, `ros2 launch anc_bringup sim.launch.py --show-args`, `ros2 launch anc_navigation reference_navigation.launch.py --show-args`, source/install comparison, and `git diff --check`. No post-fix Gazebo run was started: the latest Windows free-memory reading was `982 MiB`, below the launcher's `2560 MiB` minimum. There was no active project simulation process at the time of diagnosis. A live movement check remains pending until Windows memory is freed.

The split server/GUI arrangement follows the Gazebo Harmonic `-s` server / `-g` GUI-only model and `ros_gz_sim`'s `GzServer` simulation-interface API: [Gazebo Harmonic getting started](https://gazebosim.org/docs/harmonic/getstarted/), [ros_gz_sim user API](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/doc/user_api.rst).
