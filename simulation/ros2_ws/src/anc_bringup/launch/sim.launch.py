from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("anc_gazebo"), "worlds", "phase2_test_world.sdf"
    ])
    robot_xacro = PathJoinSubstitution([
        FindPackageShare("anc_description"), "urdf", "anc_robot.urdf.xacro"
    ])
    bridge_config = PathJoinSubstitution([
        FindPackageShare("anc_bringup"), "config", "bridge.yaml"
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare("anc_localization"), "launch", "localization.launch.py"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_server.launch.py"
            ])
        ),
        launch_arguments={
            "world_sdf_file": world,
            "verbosity_level": "3",
        }.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                FindExecutable(name="xacro"), " ", robot_xacro,
                " wheel_surface_mu:=", LaunchConfiguration("wheel_surface_mu"),
            ]),
            "use_sim_time": True,
        }],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="anc_parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    imu_sanitizer = Node(
        package="anc_bringup",
        executable="imu_sanitizer",
        name="anc_imu_sanitizer",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "use_sim_time": "true",
            "known_start_x": "0.0",
            "known_start_y": "0.0",
            "known_start_yaw": "0.0",
        }.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_anc_robot",
        output="screen",
        arguments=[
            "-world", "anc_phase2_test",
            "-topic", "robot_description",
            "-name", "anc_robot",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "wheel_surface_mu", default_value="1.2",
            description="Test-only Gazebo wheel surface friction coefficient.",
        ),
        gazebo,
        state_publisher,
        bridge,
        imu_sanitizer,
        localization,
        TimerAction(period=4.0, actions=[spawn]),
    ])
