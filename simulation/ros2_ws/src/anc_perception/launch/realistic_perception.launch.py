from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare("anc_gazebo"), "worlds", "phase2_test_world.sdf"
    ])
    sim_launch = PathJoinSubstitution([
        FindPackageShare("anc_bringup"), "launch", "sim.launch.py"
    ])
    config = PathJoinSubstitution([
        FindPackageShare("anc_perception"), "config", "realistic.yaml"
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "world_file": LaunchConfiguration("world_file"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "camera_enabled": "true",
        }.items(),
    )
    perception = Node(
        package="anc_perception",
        executable="rgb_perception_node",
        name="anc_rgb_perception",
        output="screen",
        parameters=[config, {"use_sim_time": False}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("world_file", default_value=default_world,
                              description="Gazebo world; no reference occupancy map is loaded."),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        simulation,
        perception,
    ])
