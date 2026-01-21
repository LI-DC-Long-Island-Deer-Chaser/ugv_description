#!/usr/bin/env python3
"""
Xbox Controller Joystick Launch File
Enables manual control of the rover using an Xbox USB controller.

Hardware Requirements:
    - Xbox One or Xbox 360 USB controller connected
    - Controller appears as /dev/input/js0

Control Layout:
    - Left Stick Y-axis: Forward/Backward (linear.x)
    - Right Stick X-axis: Left/Right steering (angular.z)

Usage:
    ros2 launch ugv_description joystick.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Joy Node - Reads raw Xbox controller input
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
        output='screen',
    )
    
    # Xbox Teleop Node - Converts joy messages to cmd_vel
    xbox_teleop = Node(
        package='ugv_description',
        executable='xbox_teleop',
        name='xbox_teleop_node',
        output='screen',
    )

    # Twist stamper to convert cmd_vel to /ap/cmd_vel so ardupilot works with it 
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[
            {"frame_id": "base_link"},
        ],
        remappings=[
            ("cmd_vel_in", "cmd_vel"),
            ("cmd_vel_out", "ap/cmd_vel"),
        ],
    )
    
    return LaunchDescription([
        joy_node,
        xbox_teleop,
        twist_stamper,
    ])