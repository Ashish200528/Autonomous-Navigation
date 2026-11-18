"""Low-memory Nav2 bringup with all motion routed through the safety guard."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)
    autostart = ParameterValue(
        LaunchConfiguration("autostart"), value_type=bool)
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    request_remaps = tf_remaps + [("cmd_vel", "/cmd_vel_nav_raw")]
    lifecycle_nodes = [
        "controller_server", "planner_server", "behavior_server",
        "bt_navigator", "velocity_smoother", "collision_monitor",
    ]

    def nav_node(package, executable, name, remappings=None):
        return Node(
            package=package, executable=executable, name=name,
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=tf_remaps if remappings is None else remappings,
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("anc_lidar"), "config", "nav2_lidar.yaml"
            ]),
            description="LiDAR Nav2 config; global costmap reads live SLAM map.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        nav_node("nav2_controller", "controller_server",
                 "controller_server", request_remaps),
        nav_node("nav2_planner", "planner_server", "planner_server"),
        nav_node("nav2_behaviors", "behavior_server",
                 "behavior_server", request_remaps),
        nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        nav_node("nav2_velocity_smoother", "velocity_smoother",
                 "velocity_smoother", request_remaps),
        nav_node("nav2_collision_monitor", "collision_monitor",
                 "collision_monitor"),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }],
            remappings=tf_remaps,
        ),
    ])
