from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
            "robot_description": Command([FindExecutable(name="xacro"), " ", robot_xacro]),
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
        gazebo,
        state_publisher,
        bridge,
        TimerAction(period=4.0, actions=[spawn]),
    ])
