from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'autorepeat_rate': 10.0,
            }]
        ),
        Node(
            package='ugv_description',
            executable='xbox_teleop',
            name='joy_teleop',
            output='screen'
        )
    ])
