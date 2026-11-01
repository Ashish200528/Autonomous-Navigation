from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    start_x = LaunchConfiguration("known_start_x")
    start_y = LaunchConfiguration("known_start_y")
    start_yaw = LaunchConfiguration("known_start_yaw")
    share = FindPackageShare("anc_localization")
    ekf_config = PathJoinSubstitution([share, "config", "ekf.yaml"])
    wheel_config = PathJoinSubstitution([share, "config", "wheel_odometry.yaml"])

    wheel_covariance = Node(
        package="anc_localization",
        executable="wheel_odometry_covariance",
        name="anc_wheel_odometry_covariance",
        output="screen",
        parameters=[wheel_config, {"use_sim_time": use_sim_time}],
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="anc_ekf_filter",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )
    known_start = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="anc_known_start_map_to_odom",
        output="screen",
        arguments=[
            "--x", start_x, "--y", start_y, "--z", "0.0",
            "--yaw", start_yaw,
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("known_start_x", default_value="0.0"),
        DeclareLaunchArgument("known_start_y", default_value="0.0"),
        DeclareLaunchArgument("known_start_yaw", default_value="0.0"),
        known_start,
        wheel_covariance,
        ekf,
    ])
