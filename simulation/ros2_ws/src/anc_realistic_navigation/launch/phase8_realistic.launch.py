"""Separate headless REALISTIC_RGB_ONLY launch; no Nav2 or reference map."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution([
        FindPackageShare("anc_bringup"), "launch", "sim.launch.py"
    ])
    perception_config = PathJoinSubstitution([
        FindPackageShare("anc_perception"), "config", "realistic.yaml"
    ])
    mapper_config = PathJoinSubstitution([
        FindPackageShare("anc_realistic_navigation"), "config", "realistic.yaml"
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare("anc_gazebo"), "worlds", "phase2_test_world.sdf"
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "world_file": LaunchConfiguration("world_file"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "camera_enabled": LaunchConfiguration("camera_enabled"),
            "camera_mount_z_m": LaunchConfiguration("camera_mount_z_m"),
            "camera_pitch_down_rad": LaunchConfiguration("camera_pitch_down_rad"),
            "publish_known_start_tf": "false",
        }.items(),
    )
    perception = Node(
        package="anc_perception", executable="rgb_perception_node",
        name="anc_rgb_perception", output="screen",
        parameters=[perception_config, {
            "use_sim_time": True,
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
    mapper = Node(
        package="anc_realistic_navigation", executable="local_traversability_node",
        name="anc_local_traversability", output="screen",
        parameters=[mapper_config, {
            "use_sim_time": True,
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "expected_camera_z_in_base_m": ParameterValue(
                LaunchConfiguration("camera_mount_z_m"), value_type=float),
            "expected_camera_height_above_floor_m": ParameterValue(
                LaunchConfiguration("camera_height_m"), value_type=float),
            "expected_camera_pitch_down_rad": ParameterValue(
                LaunchConfiguration("camera_pitch_down_rad"), value_type=float),
        }],
    )
    supervisor = Node(
        package="anc_realistic_navigation", executable="safe_stop_supervisor",
        name="anc_realistic_safe_stop_supervisor", output="screen",
        parameters=[{"use_sim_time": True, "publish_rate_hz": 20.0}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("world_file", default_value=default_world,
            description="Test world only; runtime navigation receives no world geometry."),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("camera_mount_z_m", default_value="0.10"),
        DeclareLaunchArgument("camera_pitch_down_rad", default_value="0.0"),
        DeclareLaunchArgument("camera_height_m", default_value="0.18"),
        DeclareLaunchArgument("min_ground_forward_m", default_value="0.50"),
        DeclareLaunchArgument("max_ground_forward_m", default_value="0.60"),
        DeclareLaunchArgument("floor_seed_row_fraction", default_value="0.90"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered",
            description="Localization input; alternate topic is only for fail-closed tests."),
        simulation, perception, mapper, supervisor,
    ])
