# ROS 2 simulation workspace

## Implemented packages

- anc_description: Xacro robot model using the root parameter inventory. Defines chassis, wheels, caster, camera, camera optical frame, and IMU frame.
- anc_gazebo: headless test world with ground, a static wall, and Gazebo Harmonic physics/sensor systems.
- anc_bringup: headless Gazebo launch, robot spawn, robot_state_publisher, ros_gz bridges, and an IMU adapter that removes Gazebo's ideal orientation from the public ROS interface.

No Phase 4 perception or navigation package is implemented here.

## Model and topic contract

Values under robot.simulation_model and sensors in config/robot_parameters.yaml are provisional placeholders, not measured hardware. Camera calibration, distortion/noise, IMU noise/bias, odometry, and contact behavior must not be presented as hardware-faithful.

Gazebo entity state is test-evaluator data only. The smoke-test script queries /gzserver/get_entity_state to check the wall stop; no navigation/perception node receives that data. The bridge exposes only the sanitized IMU on /imu/data. The raw Gazebo IMU bridge is /anc_internal/imu_raw; it contains a pose-derived orientation and is not a navigation input or rosbag topic.

| ROS interface | Source / purpose |
|---|---|
| /clock | Gazebo simulation clock |
| /cmd_vel | ROS command to Gazebo DiffDrive |
| /camera/image_raw | Gazebo RGB image; camera_optical_frame |
| /camera/camera_info | Simulated pinhole calibration; camera_optical_frame |
| /imu/data | Sanitized gyro and acceleration; orientation explicitly unavailable; imu_link |
| /odom | Gazebo DiffDrive wheel odometry; odom to base_footprint |
| /joint_states | Gazebo model joint state for left/right wheel transforms |
| /tf | Dynamic odom to base_footprint from DiffDrive plus robot_state_publisher transforms |
| /tf_static | Fixed base, camera, optical, IMU, and other link transforms |
| /contacts | Gazebo contact sensor bridged from /world/anc_phase2_test/model/anc_robot/link/base_footprint/sensor/chassis_contact/contact |

UserCommands is loaded before Gazebo's Contact system so the contact sensor registers on the runtime-spawned robot. The test reports repeated contact points/messages while contact persists; interpret them as telemetry, not unique collision episodes.

## Build

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install --packages-up-to anc_bringup
source install/setup.bash
~~~

## Launch and verify

In terminal 1:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
ros2 launch anc_bringup sim.launch.py
~~~

In terminal 2:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase3_sensor_smoke_test.py
~~~

The test saves timestamped JSON, CSV, and a rendered PNG under docs/evidence/. config/phase3_sensor_view.rviz displays the robot model, TF, and RGB camera with RViz2 while Gazebo runs headlessly.
