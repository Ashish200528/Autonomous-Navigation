from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_share = FindPackageShare("anc_lidar")
    sim_launch = PathJoinSubstitution([
        FindPackageShare("anc_bringup"), "launch", "sim.launch.py"
    ])
    perception_config = PathJoinSubstitution([
        FindPackageShare("anc_perception"), "config", "realistic.yaml"
    ])
    rgb_map_config = PathJoinSubstitution([
        FindPackageShare("anc_realistic_navigation"), "config", "realistic.yaml"
    ])
    lidar_config = PathJoinSubstitution([lidar_share, "config", "realistic_lidar.yaml"])
    slam_config = PathJoinSubstitution([lidar_share, "config", "slam_toolbox.yaml"])
    nav2_config = PathJoinSubstitution([lidar_share, "config", "nav2_lidar.yaml"])
    rviz_config = PathJoinSubstitution([lidar_share, "config", "phase8c_lidar.rviz"])
    rotation_calibration_cap = ParameterValue(
        LaunchConfiguration("rotation_calibration_max_radps"), value_type=float)
    slam_launch = PathJoinSubstitution([
        FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"
    ])
    nav2_launch = PathJoinSubstitution([
        lidar_share, "launch", "nav2_lidar.launch.py"
    ])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "world_file": LaunchConfiguration("world_file"),
            "gui": LaunchConfiguration("gui"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "camera_enabled": "true",
            "camera_mount_z_m": "0.16",
            "camera_pitch_down_rad": "0.8",
            "camera_update_rate_hz": "5.0",
            "lidar_enabled": "true",
            "publish_known_start_tf": "false",
        }.items(),
    )
    perception = Node(
        package="anc_perception",
        executable="rgb_perception_node",
        name="anc_rgb_perception",
        output="screen",
        parameters=[perception_config, {
            "use_sim_time": True,
            "camera_height_m": 0.24,
            "camera_pitch_down_rad": 0.8,
            "min_ground_forward_m": 0.10,
            "max_ground_forward_m": 0.90,
            "floor_seed_row_fraction": 0.75,
        }],
    )
    rgb_map = Node(
        package="anc_realistic_navigation",
        executable="local_traversability_node",
        name="anc_local_traversability",
        output="screen",
        parameters=[rgb_map_config, {
            "use_sim_time": True,
            "publish_rate_hz": 5.0,
            "expected_camera_z_in_base_m": 0.16,
            "expected_camera_height_above_floor_m": 0.24,
            "expected_camera_pitch_down_rad": 0.8,
        }],
    )
    lidar_params = [lidar_config]
    scan_conditioner = Node(
        package="anc_lidar", executable="scan_conditioner",
        name="anc_lidar_scan_conditioner", output="screen",
        parameters=lidar_params,
    )
    local_mapper = Node(
        package="anc_lidar", executable="local_scan_mapper",
        name="anc_local_scan_mapper", output="screen",
        parameters=lidar_params,
    )
    fuser = Node(
        package="anc_lidar", executable="local_grid_fuser",
        name="anc_rgb_lidar_grid_fuser", output="screen",
        parameters=lidar_params,
    )
    safety_gate = Node(
        package="anc_lidar", executable="lidar_safety_gate",
        name="anc_lidar_safety_gate", output="screen",
        parameters=[lidar_config, {
            "rotation_calibration_max_radps": rotation_calibration_cap,
            "production_turning": ParameterValue(LaunchConfiguration('production_turning'), value_type=bool),
            "turn_validation_only": ParameterValue(LaunchConfiguration('turn_validation_only'), value_type=bool),
            "production_turn_evidence_file": ParameterValue(LaunchConfiguration('production_turn_evidence_file'), value_type=str),
        }],
    )
    command_guard = Node(
        package="anc_navigation_common", executable="safety_command_guard",
        name="anc_safety_command_guard", output="screen",
        parameters=[{
            "requested_topic": "/cmd_vel_nav",
            "safety_state_topic": "/realistic/safety_state",
            "output_topic": "/cmd_vel",
            "guard_state_topic": "/realistic/command_guard_state",
            "publish_rate_hz": 20.0,
            "command_timeout_s": 0.30,
            "safety_timeout_s": 0.25,
            "hard_max_linear_speed_mps": 0.18,
            "hard_max_angular_speed_radps": ParameterValue(PythonExpression([
                "0.1 if '", LaunchConfiguration('production_turning'),
                "'.lower() == 'true' else ", LaunchConfiguration('rotation_calibration_max_radps')]), value_type=float),
        }],
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "slam_params_file": slam_config,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_slam")),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "params_file": nav2_config,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="phase8c_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([
        DeclareLaunchArgument('production_turning', default_value='false'),
        DeclareLaunchArgument('turn_validation_only', default_value='true'),
        DeclareLaunchArgument('production_turn_evidence_file', default_value=''),
        DeclareLaunchArgument("gui", default_value="false", description="Open Gazebo GUI."),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("anc_gazebo"), "worlds", "phase2_test_world.sdf"
            ]),
            description="World geometry is provided to Gazebo only, never to runtime navigation nodes.",
        ),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false", description="Open RViz sensor and map displays; independent of Gazebo GUI."),
        DeclareLaunchArgument(
            "rotation_calibration_max_radps", default_value="0.0",
            description="Test-only pure rotation cap (0.02..0.10 rad/s). Translation remains locked; 0 disables rotation."),
        DeclareLaunchArgument("enable_slam", default_value="false",
                              description="Opt in to scan-matching after resource checks; no Gazebo truth is used."),
        DeclareLaunchArgument("enable_nav2", default_value="false",
                              description="Enable Nav2 after scan, SLAM and command-guard validation."),
        simulation,
        perception,
        rgb_map,
        scan_conditioner,
        local_mapper,
        fuser,
        safety_gate,
        command_guard,
        slam,
        nav2,
        rviz,
    ])
