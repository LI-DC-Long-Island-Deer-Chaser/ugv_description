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
Hardware Navigation Launch File for UGV — AMCL on Saved Cartographer Map

==========================================================================
 SAVE A MAP FROM CARTOGRAPHER  (run while hw_slam.launch.py is active)
==========================================================================

 1. Finish the Cartographer serialisation (writes .pbstream):
      ros2 service call /finish_trajectory \
        cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

      ros2 service call /write_state \
        cartographer_ros_msgs/srv/WriteState \
        "{filename: '/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/my_map.pbstream', include_unfinished_submaps: true}"

 2. Convert .pbstream → ROS map (PGM + YAML):
      ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
        -pbstream_filename=/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/my_map.pbstream \
        -map_filestem=/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/my_map \
        -resolution=0.05

    This creates:
      config/irl/maps/my_map.pgm
      config/irl/maps/my_map.yaml

 3. Launch AMCL navigation on the saved map:
      ros2 launch ugv_description hw_nav.launch.py \
        map:=/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/my_map.yaml

    Optional arguments:
      lidar_port:=/dev/ttyUSB0  lidar_baud:=460800
==========================================================================
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== Package directories ==========
    pkg_ugv_description = get_package_share_directory("ugv_description")
    pkg_share = FindPackageShare("ugv_description")

    # ========== Launch arguments ==========
    map_arg = LaunchConfiguration("map")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # ========== Config file paths ==========
    amcl_params = os.path.join(
        pkg_ugv_description, "config", "irl", "amcl_params.yaml"
    )
    scan_filter_params = PathJoinSubstitution(
        [pkg_share, "config", "irl", "laser_filter_hw.yaml"]
    )
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "ugv.xacro"])
    rviz_config = os.path.join(
        pkg_ugv_description, "config", "cartographer", "cartographer_rover.rviz"
    )
    ekf_config = PathJoinSubstitution(
        [pkg_share, "config", "irl", "ekf.yaml"]
    )

    # ====================================================================
    #  HARDWARE NODES  (same as hw_slam.launch.py)
    # ====================================================================

    # 1. Micro-ROS Agent — DDS bridge to ArduPilot (Cube Orange+)
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=["udp4", "--port", "2019"],
    )

    # 2. Robot State Publisher — URDF → TF tree
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

    # 3. Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "rate": 60.0}],
    )

    # 4. Static TF: base_link → base_link_ned
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

    # 5. IMU Transformer — NED → base_link frame
    imu_transformer = Node(
        package="ugv_description",
        executable="imu_data_transformer.py",
        name="imu_transformer",
        output="screen",
    )

    # 6. RPLidar C1 Driver → /scan_raw
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
                "frame_id": "360lidar_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("scan", "scan_raw")],
    )

    # 7. Laser Scan Filter: /scan_raw → /ugv/lidar (front arc)
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

    # 8. Wheel Odometry (encoder-based)
    wheel_odom = Node(
        package="ugv_description",
        executable="wheel_odom",
        name="wheel_odometery_publisher",
        output="screen",
        parameters=[
            {
                "count_to_meter_conversion_factor": 0.00205,
                "standard_deviation": 2.0e-04,
                "y_slip_factor": 0.5,
            }
        ],
    )

    # 9. UKF Fusion (robot_localization) → /odometry/filtered
    # NOTE: publish_tf MUST be true here (unlike hw_slam where Cartographer
    # provides odom→base_link).  AMCL only publishes map→odom, so the UKF
    # must publish odom→base_link for the full TF chain to exist.
    ekf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[ekf_config, {"publish_tf": True, "world_frame": "odom"}],
    )

    # ====================================================================
    #  NAV2 LOCALIZATION & NAVIGATION
    # ====================================================================

    # 10. Map Server — loads the saved Cartographer PGM map
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            amcl_params,
            {"yaml_filename": map_arg},
        ],
    )

    # 11. AMCL — localises on the static map using lidar scans
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params],
        remappings=[
            ("/scan", "/ugv/lidar"),
        ],
    )

    # 12. Planner Server
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[amcl_params],
    )

    # 13. Controller Server (MPPI Ackermann)
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[amcl_params],
        remappings=[
            ("cmd_vel", "cmd_vel"),
        ],
    )

    # 14. Behavior Server (spin, backup, wait recoveries)
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[amcl_params],
    )

    # 15. BT Navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[amcl_params],
    )

    # 16. Waypoint Follower
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[amcl_params],
    )

    # 17. Lifecycle Manager — brings up Nav2 nodes in correct order
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                ],
            }
        ],
    )

    # ====================================================================
    #  AP ODOM BRIDGE — AMCL-corrected pose + wheel encoder velocity
    # ====================================================================
    # Pose:     AMCL TF (map → base_link) — map-localised
    # Velocity: robot_localization /odometry/filtered — wheel encoder twist
    # The existing ap_odom_bridge.py looks up TF map→base_link, so it
    # works identically whether the TF comes from Cartographer or AMCL.
    odom_to_ap_relay = Node(
        package="ugv_description",
        executable="ap_odom_bridge.py",
        name="odom_to_ap_relay",
        output="screen",
        parameters=[
            {
                "vel_topic": "/odometry/filtered",
                "tf_parent": "map",
                "tf_child": "base_link",
                "publish_rate": 50.0,
            }
        ],
    )

    # ====================================================================
    #  RVIZ (optional)
    # ====================================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    # ====================================================================
    #  LAUNCH DESCRIPTION
    # ====================================================================
    return LaunchDescription(
        [
            # ── Arguments ──
            DeclareLaunchArgument(
                "map",
                description="Full path to map YAML file (from Cartographer pbstream conversion)",
            ),
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
                "use_rviz",
                default_value="true",
                description="Launch RViz2 for visualisation",
            ),
            # ── Hardware nodes (order matters for TF dependencies) ──
            micro_ros_agent,
            robot_state_publisher,
            joint_state_publisher,
            base_link_to_base_link_ned,
            imu_transformer,
            sllidar_node,
            scan_filter,
            wheel_odom,
            #ekf_node, ### DISABLED FOR NAVSAT
            # ── Nav2 stack ──
            #map_server, ### DISABLED FOR NAVSAT
            #amcl_node, ### DISABLED FOR NAVSAT
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            lifecycle_manager,
            # ── AP bridge ──
            odom_to_ap_relay,
            # ── Visualisation ──
            #rviz_node,
        ]
    )
