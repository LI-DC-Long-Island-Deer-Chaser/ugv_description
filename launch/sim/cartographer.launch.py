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
Launch Cartographer SLAM for UGV rover.

ros2 launch ugv_description cartographer.launch.py
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ## ***** Package directories *****
    pkg_ugv_description = get_package_share_directory("ugv_description")

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true",
        description="Use simulation time"
    )

    ## ***** Cartographer nodes *****
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"qos_overrides./imu.reliability": "reliable"},
            {"qos_overrides./odom.reliability": "reliable"},
            {"qos_overrides./scan.reliability": "reliable"},
        ],
        arguments=[
            "-configuration_directory",
            os.path.join(pkg_ugv_description, "config", "sim"),
            "-configuration_basename",
            "cartographer.lua",
        ],
        remappings=[
            ("/imu", "/ugv/imu/data"),
            ("/odom", "/ugv/odom"),
            ("/scan", "/ugv/lidar"),
        ],
    )

    ## ***** Static TF for IMU sensor *****
    # Publish static TF from base_link to IMU sensor frame so that it fits inside the tf tree
    imu_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "ugv/base_link/imu_sensor"],
    )

    ## ***** Static TF for Lidar sensor *****
    # Publish static TF from 360lidar_link to lidar sensor frame
    # 180° yaw to cancel the URDF rotation on 360lidar_link (needed for RPLidar C1 data orientation)
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        arguments=["0", "0", "0", "3.141592653589793", "0", "0", "360lidar_link", "ugv/base_link/gpu_lidar"],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"resolution": 0.05},
        ],
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


    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            # Nodes
            cartographer_node,
            cartographer_occupancy_grid_node,
            imu_static_tf,
            lidar_static_tf,
            tf_to_ap_relay,
        ]
    )
