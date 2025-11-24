import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('ugv_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    
    # File paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'ugv.sdf')
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'ugv.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    laser_filter_config_path = os.path.join(pkg_share, 'config', 'laser_filter_gz.yaml')
    world_path = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', default_urdf_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_path}'
        ],
        output='screen'
    )
    
    # Robot Localization - EKF Node for sensor fusion
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Laser Scan Filter - Front 180° arc for navigation
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[laser_filter_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('scan', 'scan_raw'),           # Input: raw 360° scan from Gazebo
            ('scan_filtered', 'scan')       # Output: filtered 180° scan for SLAM/Nav2
        ]
    )
    
    # Spawn Entity using ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', default_model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        
        # Launch Gazebo with GUI and custom world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),
        
        # Launch nodes
        robot_state_publisher_node,
        robot_localization_node,
        laser_filter_node,
        rviz_node,
        ros_gz_bridge,
        spawn_entity,
    ])
