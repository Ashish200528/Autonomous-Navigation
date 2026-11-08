"""Camera-only autonomous driving demo on the race track.

    ros2 launch anc_autodrive autodrive.launch.py              # GUI + YOLO
    ros2 launch anc_autodrive autodrive.launch.py gui:=false use_yolo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("anc_autodrive")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("anc_bringup"), "launch", "sim.launch.py")),
        launch_arguments={
            "world_file": PythonExpression(["'", os.path.join(share, "worlds", "autodrive_track"),
                                            "' + {'lite': '_lite', 'ultra': '_ultra'}.get('", LaunchConfiguration("quality"),
                                            "', '') + '.sdf'"]),
            "world_name": "anc_autodrive",
            "gui": LaunchConfiguration("gui"),
            "gui_required": "false",  # window can be closed/reopened (gz sim -g) while the car drives
            "start_x": LaunchConfiguration("start_x"), "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "robot_style": "true",
            "gui_args": "--gui-config " + os.path.join(share, "config", "autodrive_gui.config"),
            "pan_servo_enabled": "true",
            "ultrasonic_enabled": "true",
            "camera_hfov_rad": "1.5708",
            "camera_width_px": "640", "camera_height_px": "480",  # same as the detector script
            "camera_mount_z_m": "0.12",
            "camera_pitch_down_rad": "0.30",
        }.items(),
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="autodrive_bridge",
        arguments=[
            "/ultrasonic/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/anc_robot/servo/pan@std_msgs/msg/Float64]gz.msgs.Double",
            "/autodrive/hud@sensor_msgs/msg/Image]gz.msgs.Image",
            "/model/pedestrian/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/pedestrian/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        remappings=[("/model/anc_robot/servo/pan", "/servo/pan_cmd")],
        parameters=[{"use_sim_time": True}],
    )
    # Keep YOLO's math libraries to 2 threads so the simulator keeps the remaining cores.
    few_threads = {k: "2" for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")}
    nodes = [
        Node(package="anc_autodrive", executable=exe, name=name, output="screen",
             parameters=[{"use_sim_time": True, **params}], additional_env=few_threads)
        for exe, name, params in (
            ("ultrasonic_sim", "ultrasonic_sim", {}),
            ("pedestrian_sim", "pedestrian_sim", {}),
            ("vision", "autodrive_vision", {"use_yolo": LaunchConfiguration("use_yolo"), "rate_hz": 2.0,
                                                 "yolo_imgsz": 640, "object_scale": 0.2}),
            ("brain", "autodrive_brain", {"target_laps": LaunchConfiguration("laps"),
                                                "autostart": LaunchConfiguration("autostart")}),
        )
    ]
    follow = TimerAction(period=10.0, condition=IfCondition(LaunchConfiguration("gui")), actions=[
        ExecuteProcess(cmd=["gz", "service", "-s", "/gui/follow/offset", "--reqtype", "gz.msgs.Vector3d",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                            "--req", "x: -0.85, y: 0.0, z: 0.42"]),
        ExecuteProcess(cmd=["gz", "service", "-s", "/gui/follow", "--reqtype", "gz.msgs.StringMsg",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                            "--req", 'data: "anc_robot"']),
    ])
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_yolo", default_value="true"),
        DeclareLaunchArgument("laps", default_value="2", description="0 = drive forever"),
        DeclareLaunchArgument("quality", default_value="standard",
                              description="standard: sky; lite: fastest; ultra: sky + shadows (needs a GPU)"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="false: car holds until `ros2 param set /autodrive_brain go true`"),
        DeclareLaunchArgument("start_x", default_value="-3.6"),
        DeclareLaunchArgument("start_y", default_value="-3.3"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
        sim, bridge, *nodes, follow,
    ])
