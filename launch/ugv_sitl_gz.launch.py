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
Launch UGV rover in Gazebo with ArduPilot SITL and RViz.

ros2 launch ugv_description ugv_gz_sitl.launch.py
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for UGV rover with ArduPilot SITL."""
    
    # Package directories
    pkg_ugv_description = get_package_share_directory("ugv_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    
    # Paths
    xacro_file = os.path.join(pkg_ugv_description, "urdf", "ugv.xacro")
    rviz_config = os.path.join(pkg_ugv_description, "config", "sim", "display.rviz")
    world_file = os.path.join(pkg_ugv_description, "worlds", "iris_maze.sdf")
    bridge_config = os.path.join(pkg_ugv_description, "config", "sim", "ugv_bridge.yaml")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    
    # Ensure SDF_PATH is populated for sdformat_urdf
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path
    
    # Robot description from xacro
    robot_description = Command(
        [
            "xacro ",
            xacro_file,
        ]
    )
    
    # ArduPilot SITL + DDS 
    # Micro-ROS Agent 
    micro_ros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "micro_ros_agent.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "transport": "udp4",
            "middleware": "dds",
            "port": "2019",
            "verbose": "4",
        }.items(),
    )

    # ArduPilot SITL (without MAVProxy)
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "command": "ardurover",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "defaults": os.path.join(
                pkg_ugv_description,
                "config",
                "sim",
                "mav.parm",
            ),
            "sim_address": "127.0.0.1",
            "synthetic_clock": "True",
        }.items(),
    )

    # Gazebo Sim Server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": f"-v4 -s -r {world_file}"
        }.items(),
    )
    
    # Gazebo Sim GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "frame_prefix": "",
            }
        ],
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "",
            "-name",
            "ugv",
            "-topic",
            "/robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
        ],
        output="screen",
    )
    
    # ros_gz_bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_config,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )
    
    # Relay Gazebo TF to ROS TF
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )
    
    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="screen",
    )
    
    # Static transform to map Gazebo's lidar frame to TF frame
    lidar_frame_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_frame_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "360lidar_link", "ugv/base_footprint/gpu_lidar"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    # Event handler to start TF relay after bridge starts
    bridge_event = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[topic_tools_tf],
        )
    )
    
    # Delay SITL start until after Gazebo and robot are fully initialized
    delayed_sitl = TimerAction(
        period=3.0,
        actions=[micro_ros_agent, sitl],
    )
    
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "use_gz_tf",
                default_value="true",
                description="Use Gazebo TF",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz",
            ),
            DeclareLaunchArgument(
                "x",
                default_value="0",
                description="Initial x position (m)",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0",
                description="Initial y position (m)",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.4",
                description="Initial z position (m)",
            ),
            # Start Gazebo and support nodes first
            gz_sim_server,
            gz_sim_gui,
            robot_state_publisher,
            spawn_robot,
            bridge,
            bridge_event,
            rviz_node,
            lidar_frame_publisher,
            # Delay ArduPilot SITL to ensure Gazebo is ready
            delayed_sitl,
        ]
    )
