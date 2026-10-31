# ROS 2 simulation workspace

## Implemented Phase 2 packages

- anc_description: Xacro differential-drive robot. It loads the single parameter inventory from the project root and defines chassis, wheels, caster, and placeholder camera frames.
- anc_gazebo: headless Phase 2 world with a ground plane and static wall.
- anc_bringup: Gazebo server launch, robot spawn, robot_state_publisher, and ros_gz topic bridges.

Future packages for perception, localization, navigation, and evaluation remain planned and are not yet implemented as ROS packages.

## Model and topic contract

Values under robot.simulation_model in config/robot_parameters.yaml are design placeholders, not measured hardware. The footprint rectangle conservatively includes the current wheel and caster envelope. Replace the values only after measuring the physical robot, then rerun the checks.

Gazebo truth stays inside the simulator. The Phase 2 evaluator reads GetEntityState for test measurements; the robot controller and navigation nodes do not subscribe to that service.

- ROS /cmd_vel to Gazebo /model/anc_robot/cmd_vel
- Gazebo /model/anc_robot/odom to ROS /odom
- Gazebo /model/anc_robot/tf to ROS /tf
- ROS /clock from Gazebo /clock
- Gazebo /model/anc_robot/contacts to ROS /contacts

The dynamic odom-to-base_footprint transform comes from Gazebo DiffDrive. robot_state_publisher owns the static base and fixed-link transforms. tf2_echo odom base_link resolved in the check. /tf had publishers from the bridge and robot_state_publisher; /tf_static had one robot_state_publisher publisher.

The contact publisher and bridge were present, but the test received zero contact messages; do not use /contacts as a verified collision counter yet. The camera_link exists as a frame only. RGB/CameraInfo, IMU, wheel joint-state visualization, and realistic perception are later work.

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
python3 scripts/phase2_motion_test.py
~~~

Gazebo entity state is used only by that test script as evaluator data. The script records JSON results and CSV telemetry in docs/evidence/.
