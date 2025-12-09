#!/usr/bin/env python3
"""
Joystick Teleoperation Launch File for UGV Hardware
====================================================
Enables manual control of the rover using an Xbox USB controller.

Hardware Requirements:
    - Xbox One or Xbox 360 USB controller connected to laptop
    - Controller appears as /dev/input/js0

Control Layout:
    - Left Stick Y-axis: Forward/Backward (linear.x)
    - Right Stick X-axis: Left/Right steering (angular.z)
    - R1 (RB): Deadman button (MUST be held to enable movement)

Usage:
    # On laptop (with controller connected):
    ros2 launch ugv_description joystick.launch.py
    
    # Hold R1 and move sticks to control the rover
    # Release R1 to immediately stop

Topics:
    Published:
        - /cmd_vel: Twist commands (consumed by unified_encoder_control on Jetson)
        
    Subscribed:
        - /joy: Raw joystick data from joy_node

Notes:
    - Rover must be running slam.launch.py for motion control
    - DDS discovery must be configured for laptop-Jetson communication
    - Velocity limits are conservative (0.5 m/s linear, 0.8 rad/s angular)
    - Deadman button provides safety - rover stops when released
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # ========== Arguments ==========
    joy_dev = LaunchConfiguration('joy_dev')
    config_file = LaunchConfiguration('config_file')
    
    # Package path
    pkg_share = FindPackageShare('ugv_description')
    
    # Joystick config path
    default_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'hardware',
        'joystick.yaml'
    ])
    
    # ========== Nodes ==========
    
    # 1. Joy Node - Reads raw joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,  # /dev/input/js0
            'deadzone': 0.1,  # 10% deadzone on all axes
            'autorepeat_rate': 20.0,  # 20 Hz publishing
            'coalesce_interval_ms': 1,
        }],
        remappings=[
            ('joy', 'joy')  # Publishes to /joy topic
        ]
    )
    
    # 2. Teleop Twist Joy - Converts joy messages to cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file],
        remappings=[
            ('joy', 'joy'),  # Subscribes to /joy from joy_node
            ('cmd_vel', 'cmd_vel')  # Publishes to /cmd_vel
        ]
    )
    
    # ========== Launch Description ==========
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path (usually /dev/input/js0 for first controller)'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to joystick configuration YAML file'
        ),
        
        # Nodes
        joy_node,
        teleop_node,
    ])
