from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            "camera_mount_z_m": LaunchConfiguration("camera_mount_z_m"),
            "camera_pitch_down_rad": LaunchConfiguration("camera_pitch_down_rad"),
        }.items(),
    )
    perception = Node(
        package="anc_perception",
        executable="rgb_perception_node",
        name="anc_rgb_perception",
        output="screen",
        parameters=[config, {
            "use_sim_time": False,
            "camera_height_m": ParameterValue(
                LaunchConfiguration("camera_height_m"), value_type=float),
            "camera_pitch_down_rad": ParameterValue(
                LaunchConfiguration("camera_pitch_down_rad"), value_type=float),
            "min_ground_forward_m": ParameterValue(
                LaunchConfiguration("min_ground_forward_m"), value_type=float),
            "max_ground_forward_m": ParameterValue(
                LaunchConfiguration("max_ground_forward_m"), value_type=float),
            "floor_seed_row_fraction": ParameterValue(
                LaunchConfiguration("floor_seed_row_fraction"), value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("world_file", default_value=default_world,
                              description="Gazebo world; no reference occupancy map is loaded."),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera_mount_z_m", default_value="0.10"),
        DeclareLaunchArgument("camera_pitch_down_rad", default_value="0.0"),
        DeclareLaunchArgument("camera_height_m", default_value="0.18"),
        DeclareLaunchArgument("min_ground_forward_m", default_value="0.50"),
        DeclareLaunchArgument("max_ground_forward_m", default_value="0.60"),
        DeclareLaunchArgument("floor_seed_row_fraction", default_value="0.90"),
        simulation,
        perception,
    ])
