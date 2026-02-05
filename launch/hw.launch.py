# Copyright 2024 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Hardware SLAM Launch File for UGV
Launches Cartographer SLAM with RPLidar C1 and ArduPilot IMU.

ros2 launch ugv_description hw_slam.launch.py
"""

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

    # ========== Config file paths ==========
    cartographer_config = os.path.join(
        pkg_ugv_description, "config", "cartographer", "cartographer_rover.lua"
    )
    scan_filter_params = PathJoinSubstitution(
        [pkg_share, "config", "irl", "laser_filter_hw.yaml"]
    )
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "ugv.xacro"])
    rviz_config = os.path.join(
        pkg_ugv_description, "config", "cartographer", "cartographer_rover.rviz"
    )
    ekf_config = PathJoinSubstitution([
    pkg_share, "config", "ekf.yaml"
    ])

    # ========== 1. Robot State Publisher ==========
    # Publishes TF tree from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_path]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ========== 2. RPLidar C1 Driver ==========
    # Publishes raw 360° scans to /scan_raw
    # Frame: ugv/lidar_2d_link/lidar_2d (or configure to match your URDF)
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_c1",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": lidar_baud,
                "frame_id": "360lidar_link",  # Match your URDF sensor frame
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",  # C1 DenseBoost: 10 Hz, 40m range, 5K points
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("scan", "scan_raw")],  # Raw 360° scan
    )

    # ========== 3. Laser Scan Filter ==========
    # Filters /scan_raw (360°) -> /ugv/lidar (front arc)
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[scan_filter_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("scan", "scan_raw"),  # Input: raw 360° scan
            ("scan_filtered", "ugv/lidar"),  # Output: filtered scan for Cartographer
        ],
    )

    # ========== 4. Cartographer SLAM Node ==========
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"qos_overrides./imu.reliability": "best_effort"},  # ArduPilot uses best_effort
            {"qos_overrides./odom.reliability": "best_effort"},
            {"qos_overrides./scan.reliability": "reliable"},
        ],
        arguments=[
            "-configuration_directory",
            os.path.join(pkg_ugv_description, "config", "cartographer"),
            "-configuration_basename",
            "cartographer_rover.lua",
        ],
        remappings=[
            ("/imu", "/imu/data"),  # Transformed IMU in base_link frame
            ("/odom", "/ugv/odom"),  # ArduPilot odometry
            ("/scan", "/ugv/lidar"),  # Filtered lidar scan
        ],
    )

    # ========== 5. Cartographer Occupancy Grid Node ==========
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

    # ========== 6. Micro-ROS Agent ==========
    # DDS bridge for ArduPilot communication over Ethernet
    # Connects to Cube Orange Plus at 192.168.114.15
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=[
            "udp4",
            "--port",
            "2019",
        ],
    )

    # ========== 7. Joint State Publisher ==========
    # Publishes joint states for wheels and steering
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "rate": 60.0}],
    )

    # ========== 8. Wheel Odometry ==========
    wheel_odom = Node(
        package="ugv_description",
        executable="wheel_yapper",
        name="wheel_odometery_publisher",
        output="screen",
    )

    # ========== 9. Base Link to BaseLink NED Transform ==========
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
            "--yaw", "0",
        ],  
    )
    # ========== 10 EKF Fusion ==========
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )
    

    # ========== 10. IMU Transformer ==========
    # Transforms IMU data from base_link_ned to base_link frame
    imu_transformer = Node(
        package="ugv_description",
        executable="imu_transformer.py",
        name="imu_transformer",
        output="screen",
    )
    
    ## ***** TF Relay - Publish Cartographer's odom->base_link to ArduPilot *****
    # This relays Cartographer's TF (map->odom->base_link) to /ap/tf topic
    # which ArduPilot subscribes to for external odometry
    tf_to_ap_relay = Node(
        package="topic_tools",
        executable="relay",
        name="tf_to_ap_relay",
        output="screen",
        arguments=[
            "/tf",
            "/ap/tf",
        ],
    )


    # ========== Launch Description ==========
    return LaunchDescription(
        [
            # Arguments
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
            # Nodes (order matters for TF dependencies)
            micro_ros_agent,  # Start DDS bridge first
            robot_state_publisher,  # Must be early - publishes URDF TF tree
            joint_state_publisher,  # Publishes joint states
            base_link_to_base_link_ned,
            imu_transformer,  # Transform IMU from NED to base_link
            sllidar_node,  # RPLidar C1 driver
            scan_filter,  # Filter to front arc
            cartographer_node,  # Cartographer SLAM
            cartographer_occupancy_grid_node,  # Occupancy grid publisher
            tf_to_ap_relay
        ]
    )
