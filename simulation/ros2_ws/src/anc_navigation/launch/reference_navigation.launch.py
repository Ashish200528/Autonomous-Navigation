"""Headless REFERENCE-only Nav2 bringup for Phase 6."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("anc_bringup")
    nav_share = FindPackageShare("anc_navigation")
    sim_launch = PathJoinSubstitution([bringup_share, "launch", "sim.launch.py"])
    nav_params = PathJoinSubstitution([nav_share, "config", "nav2_reference.yaml"])
    rviz_config = PathJoinSubstitution([nav_share, "config", "phase6_reference.rviz"])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "world_file": LaunchConfiguration("world_file"),
            "camera_enabled": "false",
            "gui": LaunchConfiguration("gui"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="phase6_reference_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": LaunchConfiguration("map_yaml"),
            "topic_name": "map",
            "frame_id": "map",
            "use_sim_time": True,
        }],
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["map_server"],
            "bond_timeout": 4.0,
        }],
    )

    common_remaps = [("/tf", "/tf"), ("/tf_static", "/tf_static")]
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav_params],
        remappings=common_remaps,
    )
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_params],
        remappings=common_remaps + [("cmd_vel", "/cmd_vel_nav")],
    )
    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav_params],
        remappings=common_remaps + [("cmd_vel", "/cmd_vel_nav")],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav_params],
        remappings=common_remaps,
    )
    nav_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "planner_server", "controller_server",
                "behavior_server", "bt_navigator",
            ],
            "bond_timeout": 4.0,
        }],
    )
    command_guard = Node(
        package="anc_navigation",
        executable="cmd_vel_guard",
        name="anc_navigation_cmd_vel_guard",
        output="screen",
        parameters=[{
            "max_linear_x_mps": 0.25,
            "max_angular_z_radps": 1.0,
            "command_timeout_s": 0.40,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_file", description="Generated Phase 5 SDF world."),
        DeclareLaunchArgument("gui", default_value="false", description="Use the Gazebo graphical interface."),
        DeclareLaunchArgument("rviz", default_value="false", description="Open optional RViz displays."),
        DeclareLaunchArgument("map_yaml", description="Run-specific ORACLE_REFERENCE occupancy map."),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        map_server,
        map_lifecycle,
        command_guard,
        simulation,
        rviz,
        TimerAction(period=7.0, actions=[
            planner, controller, behavior, bt_navigator, nav_lifecycle,
        ]),
    ])
