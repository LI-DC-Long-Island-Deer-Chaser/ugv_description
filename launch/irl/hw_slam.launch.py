#!/usr/bin/env python3
# hw_slam.launch.py
#
# Hardware SLAM Launch File for UGV (Cartographer + sensors + EKF)
#
# Runs:
#   - micro-ROS agent (ArduPilot bridge)
#   - robot_state_publisher + joint_state_publisher
#   - IMU transformer + UKF (robot_localization)
#   - RPLidar driver + laser_filters
#   - Cartographer SLAM + occupancy grid (/map)
#   - ap_odom_bridge (publishes /ap/odom using TF map->base_link + twist)
#
# Usage:
#   ros2 launch ugv_description hw_slam.launch.py
#
# Saving a map (Nav2 map_saver_cli):
#   1) Make sure cartographer_occupancy_grid_node is running and publishing /map
#   2) Save:
#        ros2 run nav2_map_server map_saver_cli -f /absolute/path/to/my_room
#      This creates:
#        /absolute/path/to/my_room.yaml
#        /absolute/path/to/my_room.pgm
#
# Notes:
#   - Do NOT run AMCL at the same time as Cartographer SLAM (both publish map->odom).
#   - use_sim_time defaults to false (hardware).
#

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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

    # ========== Config file paths ==========
    cartographer_config_dir = os.path.join(pkg_ugv_description, "config", "cartographer")
    cartographer_config_basename = "cartographer_rover.lua"

    scan_filter_params = PathJoinSubstitution([pkg_share, "config", "irl", "laser_filter_hw.yaml"])
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "ugv.xacro"])
    ekf_config = PathJoinSubstitution([pkg_share, "config", "ekf.yaml"])

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

    # ========== 10. Cartographer SLAM Node ==========
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"qos_overrides./imu.reliability": "best_effort"},
            {"qos_overrides./odom.reliability": "best_effort"},
            {"qos_overrides./scan.reliability": "reliable"},
        ],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", cartographer_config_basename,
        ],
        remappings=[
            ("/imu", "/ap/imu/corrected/data"),
            ("/odom", "/odometry/filtered"),
            ("/scan", "/ugv/lidar"),
        ],
    )

    # ========== 11. Cartographer Occupancy Grid Node ==========
    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"resolution": 0.05},
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

        micro_ros_agent,
        robot_state_publisher,
        joint_state_publisher,
        base_link_to_base_link_ned,
        imu_transformer,
        sllidar_node,
        scan_filter,
        wheel_odom,
        ekf_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        odom_to_ap_relay,
    ])