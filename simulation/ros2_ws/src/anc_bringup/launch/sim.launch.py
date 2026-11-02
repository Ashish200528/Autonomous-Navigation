from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare("anc_gazebo"), "worlds", "phase2_test_world.sdf"
    ])
    world = LaunchConfiguration("world_file")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_yaw = LaunchConfiguration("start_yaw")
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
                " camera_enabled:=", LaunchConfiguration("camera_enabled"),
                " lidar_enabled:=", LaunchConfiguration("lidar_enabled"),
                " camera_mount_z_m:=", LaunchConfiguration("camera_mount_z_m"),
                " camera_pitch_down_rad:=", LaunchConfiguration("camera_pitch_down_rad"),
                " camera_update_rate_hz:=", LaunchConfiguration("camera_update_rate_hz"),
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
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="anc_lidar_parameter_bridge",
        output="screen",
        arguments=["/realistic/scan/raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        parameters=[{"override_frame_id": "lidar_link"}],
        condition=IfCondition(LaunchConfiguration("lidar_enabled")),
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
            "known_start_x": start_x,
            "known_start_y": start_y,
            "known_start_yaw": start_yaw,
            "publish_known_start_tf": LaunchConfiguration("publish_known_start_tf"),
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
            "-x", start_x, "-y", start_y, "-z", "0.0", "-Y", start_yaw,
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "wheel_surface_mu", default_value="1.2",
            description="Test-only Gazebo wheel surface friction coefficient.",
        ),
        DeclareLaunchArgument(
            "camera_enabled", default_value="true",
            description="Enable RGB camera sensor; Phase 6 reference navigation disables it to reduce headless rendering load.",
        ),
        DeclareLaunchArgument(
            "lidar_enabled", default_value="false",
            description="Enable the parameterized planar LiDAR and its ROS bridge only in the LiDAR profile.",
        ),
        DeclareLaunchArgument(
            "camera_mount_z_m", default_value="0.10",
            description="Provisional camera mount Z relative to base_link, meters.",
        ),
        DeclareLaunchArgument(
            "camera_pitch_down_rad", default_value="0.0",
            description="Positive provisional downward camera pitch in radians.",
        ),
        DeclareLaunchArgument(
            "camera_update_rate_hz", default_value="15.0",
            description="Camera update rate in Hz; default preserves existing profiles.",
        ),
        DeclareLaunchArgument(
            "world_file", default_value=default_world,
            description="Absolute SDF world file; defaults to the Phase 2 baseline world.",
        ),
        DeclareLaunchArgument("start_x", default_value="0.0",
                              description="Known-start robot world X in meters."),
        DeclareLaunchArgument("start_y", default_value="0.0",
                              description="Known-start robot world Y in meters."),
        DeclareLaunchArgument("start_yaw", default_value="0.0",
                              description="Known-start robot world yaw in radians."),
        DeclareLaunchArgument("publish_known_start_tf", default_value="true",
                              description="Publish the test-only known-start map-to-odom transform."),
        gazebo,
        state_publisher,
        bridge,
        lidar_bridge,
        imu_sanitizer,
        localization,
        TimerAction(period=4.0, actions=[spawn]),
    ])
