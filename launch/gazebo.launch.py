from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the ros_gz_sim package
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    
    # Path to your package (replace 'example_package' with your package name)
    ugv_package_path = FindPackageShare('ugv_description')
    model_sdf_path = PathJoinSubstitution([ugv_package_path, 'urdf', 'ugv.sdf'])

    # Path to the Gazebo launch file
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        # Set environment variables for models and plugins
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([ugv_package_path, 'urdf'])
        ),

        # Include Gazebo launch file with the empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': 'empty.sdf',  # Launch the empty world
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        # --- Spawn your UGV model ---
        
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', model_sdf_path,   # <-- just pass the substitution
                '-name', 'ugv',
                '-x', '5.0',
                '-y', '5.0',
                '-z', '0.5'
            ],
            output='screen'
        )
    ])