from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maxarm',
            namespace='odom',
            executable='arm_pos_pub',
            output='screen'
        ),
        Node(
            package='maxarm',
            namespace='set_pos',
            executable='arm_set_pos',
            output='screen'
        ),
    ])
