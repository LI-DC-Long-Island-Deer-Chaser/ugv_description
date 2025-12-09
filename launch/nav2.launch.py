#!/usr/bin/env python3
"""
Nav2 Stack Launch File for UGV Hardware
========================================
Launches only Nav2-related nodes for autonomous navigation.

Prerequisites:
    - slam.launch.py must be running (provides SLAM mapping + localization)
    
Hardware Configuration:
    - TRX-4 Chassis with Ackermann steering
    - RPLidar C1 sensor (/scan topic)
    - EKF localization (/odometry/filtered)
    - Conservative settings for real-world operation
    
Usage:
    ros2 launch ugv_description nav2.launch.py
    
Topics:
    Subscribed:
        - /scan: Laser scan from RPLidar C1
        - /odometry/filtered: EKF-fused odometry
        - /tf, /tf_static: Transforms from SLAM + robot
    Published:
        - /cmd_vel: Velocity commands (consumed by unified_encoder_control)
        - /plan: Global path
        - /local_plan: Local trajectory
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('ugv_description')
    
    # Paths
    nav2_params_file = os.path.join(pkg_dir, 'config', 'hardware', 'nav2_params.yaml')
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file for Nav2'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )
    
    # Create our own configured nav2 parameters with use_sim_time override
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    # Nav2 Navigation Stack
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items()
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add Nav2 stack
    ld.add_action(nav2_bringup_cmd)
    
    return ld
