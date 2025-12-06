#!/usr/bin/env python3
"""
SLAM Launch File for UGV Hardware
Launches SLAM Toolbox with RPLidar C1, RF2O laser odometry, and EKF sensor fusion.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== Arguments ==========
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ========== Package Paths ==========
    pkg_share = FindPackageShare('ugv_description')
    
    # Config file paths (hardware)
    ekf_params = PathJoinSubstitution([pkg_share, 'config', 'hardware', 'ekf_hw.yaml'])
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'hardware', 'slam_toolbox.yaml'])
    scan_filter_params = PathJoinSubstitution([pkg_share, 'config', 'hardware', 'laser_filter_hw.yaml'])
    
    # URDF path for robot_state_publisher
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'ugv.urdf'])

    # ========== 1. Robot State Publisher ==========
    # Publishes TF tree from URDF (including lidar frame: ugv/lidar_2d_link/lidar_2d)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path]),
            'use_sim_time': use_sim_time
        }]
    )

    # ========== 2. RPLidar C1 Driver ==========
    # Publishes raw 360° scans to /scan_raw
    # Frame: ugv/lidar_2d_link/lidar_2d (matches URDF)
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_c1',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id': 'ugv/lidar_2d_link/lidar_2d',  # Matches URDF sensor frame
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('scan', 'scan_raw')  # Raw 360° scan
        ]
    )

    # ========== 3. Laser Scan Filter ==========
    # Filters /scan_raw (360°) -> /scan (front 180°)
    # Front 180° arc for SLAM and navigation
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[
            scan_filter_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', 'scan_raw'),        # Input: raw 360° scan
            ('scan_filtered', 'scan')     # Output: filtered 180° scan
        ]
    )

    # ========== 4. RF2O Laser Odometry ==========
    # Uses /scan to generate /odom_rf2o
    # Does NOT publish TF (EKF will handle odom->base_link)
    rf2o_laser_odom = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,              # EKF owns the TF tree
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,                     # 20 Hz odometry
            'use_sim_time': use_sim_time
        }]
    )

    # ========== 5. Robot Localization EKF ==========
    # Fuses RF2O odometry + IMU data from MAVROS
    # ArduPilot AHRS_ORIENTATION=8 handles 180° roll correction
    # MAVROS publishes with frame_id: base_link (already corrected)
    # Publishes odom->base_footprint TF and /odometry/filtered
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ========== 6. SLAM Toolbox ==========
    # Creates map from /scan
    # Publishes map->odom TF and /map
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ========== 7. Joint State Publisher ==========
    # Publishes joint states for wheels and steering
    # Currently publishes zeros - will be replaced with actual encoder/feedback data later
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50.0  # 50 Hz publication rate
        }]
    )

    # ========== 8. Unified Encoder Control ==========
    # Single node combining: wheel encoder odometry + PID velocity control
    # - Direct serial reading at 200Hz for minimal latency
    # - Dual-stage velocity filtering (Arduino + low-pass filter)
    # - ESC deadband compensation (Traxxas XL-5 HV: 1455-1565 PWM no movement)
    # - Publishes /wheel_odom for EKF fusion
    # - Publishes /joint_states_encoder for steering visualization
    # - Subscribes to /cmd_vel and publishes /mavros/rc/override
    unified_encoder_control = Node(
        package='ugv_description',
        executable='unified_encoder_control',
        name='unified_encoder_control',
        output='screen',
        parameters=[{
            # Serial Configuration
            'serial_port': '/dev/ttyACM2',
            'baud_rate': 115200,
            
            # Encoder Calibration (weighted load test)
            'counts_per_rev': 357.74,                 # Empirical: weighted load calibration
            'wheel_circumference': 0.38713,           # meters
            
            # ESC Deadband Configuration (Traxxas XL-5 HV 3S)
            'throttle_pwm_forward_start': 1565,       # Forward deadband end
            'throttle_pwm_reverse_start': 1455,       # Reverse deadband end
            'throttle_pwm_max': 1610,                 # Forward limit (conservative)
            'throttle_pwm_min': 1390,                 # Reverse limit (conservative)
            'throttle_pwm_neutral': 1500,
            
            # Steering Configuration
            'wheel_base': 0.254,                      # 10 inches
            'max_steering_angle': 0.785398,           # 45 degrees
            'steering_pwm_center': 1500,
            'steering_pwm_min': 1100,
            'steering_pwm_max': 1900,
            
            # PID Gains (tuned for narrow 45 PWM forward range)
            'kp': 1.0,                               # Proportional
            'ki': 1.5,                               # Integral (low to prevent windup)
            'kd': 0.2,                                # Derivative (dampening)
            'max_integral': 10.0,                     # Anti-windup limit
            
            # Control Rates
            'odom_publish_rate': 20.0,                # Hz - odometry publishing
            'pid_update_rate': 100.0,                 # Hz - PID control loop
            
            # Filtering
            'velocity_filter_alpha': 0.3,             # Low-pass filter coefficient
            
            # Frame IDs
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            
            'use_sim_time': use_sim_time
        }]
    )

    # ========== Launch Description ==========
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RPLidar C1'
        ),
        DeclareLaunchArgument(
            'lidar_baud',
            default_value='460800',
            description='Baud rate for RPLidar C1'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (false for hardware)'
        ),
        
        # Nodes (order matters for TF dependencies)
        robot_state_publisher,    # Must be first - publishes URDF TF tree
        joint_state_publisher,    # Publishes joint states for wheels/steering
        sllidar_node,             # Lidar driver
        scan_filter,              # Filter to front 180°
        unified_encoder_control,  # Unified encoder odometry + PID control with ESC deadband
        rf2o_laser_odom,          # Laser odometry
        ekf_node,                 # Sensor fusion (uses /mavros/imu/data + /wheel_odom)
        slam_toolbox,             # SLAM mapping
    ])
