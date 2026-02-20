#!/usr/bin/env python3
# hw_amcl.launch.py
#
# Hardware AMCL Localization + Nav2 Launch File for UGV
#
# Runs:
#   - micro-ROS agent (ArduPilot bridge)
#   - robot_state_publisher + joint_state_publisher
#   - IMU transformer + UKF (robot_localization)
#   - RPLidar driver + laser_filters
#   - Nav2 bringup (bringup_launch.py) which includes:
#       * map_server (loads your saved map)
#       * AMCL (publishes map->odom)
#       * planner/controller/bt navigator, costmaps, etc.
#   - twist_stamper: cmd_vel -> ap/cmd_vel for ArduPilot
#   - ap_odom_bridge: publishes /ap/odom using TF map->base_link + twist
#
# Usage:
#   ros2 launch ugv_description hw_amcl.launch.py map:=/absolute/path/to/my_room.yaml
#
# Notes:
#   - Do NOT run Cartographer SLAM when running AMCL (both publish map->odom).
#   - use_sim_time defaults to false (hardware).
#

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== Package directories ==========
    pkg_ugv_description = get_package_share_directory("ugv_description")
    pkg_share = FindPackageShare("ugv_description")

    # ========== Launch arguments ==========
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Saved map yaml
    map_yaml = LaunchConfiguration("map")

    # Nav2 params file
    nav2_params_file = LaunchConfiguration("params_file")

    # ========== Config paths ==========
    scan_filter_params = PathJoinSubstitution([pkg_share, "config", "irl", "laser_filter_hw.yaml"])
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "ugv.xacro"])
    ekf_config = PathJoinSubstitution([pkg_share, "config", "irl", "ekf_amcl.yaml"])

    # Default params file location (adjust to your folder layout)
    default_params = PathJoinSubstitution([pkg_share, "config", "irl", "nav2_amcl.yaml"])

    # ========== 1. Robot State Publisher ==========
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_path]),
            "use_sim_time": use_sim_time,
        }],
    )

    # ========== 2. Joint State Publisher ==========
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "rate": 60.0}],
    )

    # ========== 3. Micro-ROS Agent ==========
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=["udp4", "--port", "2019"],
    )

    # ========== 4. base_link -> base_link_ned static TF ==========
    base_link_to_base_link_ned = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_link_ned",
        arguments=[
            "--frame-id", "base_link",
            "--child-frame-id", "base_link_ned",
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.141592653589793",
            "--pitch", "0",
            "--yaw", "1.570796367948966",
        ],
    )

    # ========== 5. IMU Transformer ==========
    imu_transformer = Node(
        package="ugv_description",
        executable="imu_data_transformer.py",
        name="imu_transformer",
        output="screen",
    )

    # ========== 6. RPLidar C1 Driver ==========
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_c1",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": lidar_port,
            "serial_baudrate": lidar_baud,
            "frame_id": "360lidar_link",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "DenseBoost",
            "use_sim_time": use_sim_time,
        }],
        remappings=[("scan", "scan_raw")],
    )

    # ========== 7. Laser Scan Filter ==========
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[scan_filter_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("scan", "scan_raw"),
            ("scan_filtered", "ugv/lidar"),
        ],
    )

    # ========== 8. Wheel Odometry ==========
    wheel_odom = Node(
        package="ugv_description",
        executable="wheel_odom",
        name="wheel_odometery_publisher",
        output="screen",
        parameters=[{
            "count_to_meter_conversion_factor": 0.0020,
            "standard_deviation": 1.0e-04,
            "y_slip_factor": 0.5,
        }],
    )

    # ========== 9. EKF/UKF fusion ==========
    ekf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    # ========== 10. Nav2 Bringup (AMCL + map_server + navigation) ==========
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
    )

    # ========== 11. Twist Stamper (cmd_vel -> ap/cmd_vel) ==========
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        parameters=[{
            "frame_id": "base_link",
            "use_sim_time": use_sim_time,
            "qos_override": {
                "/cmd_vel_in": {"depth": 10, "reliability": "reliable"},
                "/ap/cmd_vel": {"depth": 10, "reliability": "best_effort"},
            },
        }],
        remappings=[
            ("cmd_vel_in", "cmd_vel"),
            ("cmd_vel_out", "ap/cmd_vel"),
        ],
    )

    # ========== 12. AP Odom Bridge (map->base_link pose + filtered twist) ==========
    odom_to_ap_relay = Node(
        package="ugv_description",
        executable="ap_odom_bridge.py",
        name="odom_to_ap_relay",
        output="screen",
        parameters=[{
            "vel_topic": "/odometry/filtered",
            "tf_parent": "map",
            "tf_child": "base_link",
            "publish_rate": 50.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for RPLidar C1",
        ),
        DeclareLaunchArgument(
            "lidar_baud",
            default_value="460800",
            description="Baud rate for RPLidar C1",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (false for hardware)",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Absolute path to saved map YAML (e.g. /home/user/maps/my_room.yaml)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Nav2 params YAML for AMCL localization mode",
        ),

        micro_ros_agent,
        robot_state_publisher,
        joint_state_publisher,
        base_link_to_base_link_ned,
        imu_transformer,
        sllidar_node,
        scan_filter,
        wheel_odom,
        ekf_node,

        nav2_launch,
        twist_stamper,
        odom_to_ap_relay,
    ])