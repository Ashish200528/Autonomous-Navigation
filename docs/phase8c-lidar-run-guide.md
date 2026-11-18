# Phase 8C LiDAR demonstration and operation guide

Branch: feature/phase8c-lidar

## Safe headless sensor run

From Ubuntu 24.04 WSL:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
ros2 launch anc_lidar realistic_rgb_lidar.launch.py \
  world_file:=/home/ashish/AutonomousNavigationCar/simulation/scenarios/worlds/s00_empty_open_arena.sdf \
  enable_slam:=false enable_nav2:=false rotation_calibration_max_radps:=0.0
~~~

This starts headless Gazebo with RGB, 2D LiDAR, local scan map, odometry, safety gate and command guard. It does not request motion. Production angular authority is zero. Stop with Ctrl+C.

## Stationary SLAM map capture (diagnostic only)

SLAM passed only a stationary S00 map check. It has not demonstrated moving localization or reduced drift. For a fresh map capture, launch with enable_slam:=true, enable_nav2:=false and rotation_calibration_max_radps:=0.0. In another WSL terminal:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -t /map -f docs/evidence/phase8c/lidar/my-stationary-slam-map --fmt pgm --mode trinary
~~~

The saved evaluation snapshot is stage2-s00-stationary-slam-map.pgm and .yaml. It is not navigation-ready.

## RViz view

With SLAM running, use another terminal:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
rviz2 -d config/phase8c_lidar.rviz
~~~

This config shows /map, /realistic/scan, TF and robot model. It requires WSLg/desktop support. YAML syntax was checked, but RViz was not opened during the low-load headless run. Nav2 is off, so no global plan is displayed.

## Saved demonstration and limits

The headless sensor composite is at docs/evidence/live-demo-2026-11-02/phase8c-live-demo.mp4; its report is beside it. The video shows RGB, LiDAR, gate state and /cmd_vel. It is not a desktop capture or Nav2 demonstration. The MP4 remains local because the existing *.mp4 ignore rule excludes video.

Do not submit Nav2 goals, attempt obstacle traversal or start live parking. Production yaw and moving localization are not validated. The launch default remains zero.
## Repeatable scenario launcher

The launcher checks both Windows host RAM and WSL RAM before generating a scenario or starting ROS/Gazebo. Minimum available memory is 2 GiB on Windows for headless runs, 2.5 GiB for GUI runs, and 1.5 GiB within WSL. Headless is the default.

~~~bash
cd /home/ashish/AutonomousNavigationCar
./scripts/run_lidar_demo.sh --scenario S00 --seed 101 --duration 90
~~~

To request the intended Gazebo + RViz goal demonstration:

~~~bash
cd /home/ashish/AutonomousNavigationCar
./scripts/run_lidar_demo.sh --scenario S01 --seed 101 --gui --goal
~~~

The second command only enables Nav2 after the checked-in readiness gate confirms production turning, moving SLAM and the live single-guard command chain with existing evidence files. Those prerequisites are currently false. After the RAM preflight passes, the command therefore runs RGB + LiDAR + online SLAM only and clearly records that the goal is blocked; it does not count as navigation. The GUI path has not yet been opened or recorded. See docs/phase8c-stage9-report.md.
