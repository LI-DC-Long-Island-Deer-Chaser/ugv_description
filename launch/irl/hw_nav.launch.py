"""
Navigation launch file for UGV with Nav2 and Ackermann steering.
Launches Nav2 stack with twist_stamper for ArduPilot compatibility.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for UGV navigation."""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(
            Path(
                FindPackageShare("ugv_description").find("ugv_description"),
                "config",
                "irl",
                "nav2_slam.yaml"
            )
        ),
        description="Full path to Nav2 parameters file"
    )
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(
                    FindPackageShare("nav2_bringup").find("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                )
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    
    # Twist stamper node
    # Converts Twist (from Nav2) to TwistStamped (for ArduPilot)
    # Uses BEST_EFFORT QoS to match ArduPilot's subscription
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        parameters=[
            {
                "frame_id": "base_link",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "qos_override": {
                    "/cmd_vel_in": {
                        "depth": 10,
                        "reliability": "reliable",
                    },
                    "/ap/cmd_vel": {
                        "depth": 10,
                        "reliability": "best_effort",
                    }
                }
            }
        ],
        remappings=[
            ("cmd_vel_in", "cmd_vel"),
            ("cmd_vel_out", "ap/cmd_vel"),
        ],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        nav2_bringup,
        twist_stamper,
    ])
