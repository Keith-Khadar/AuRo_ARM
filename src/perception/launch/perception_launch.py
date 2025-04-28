import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_project_usb_cam = get_package_share_directory('usb_cam')

    perception = launch_ros.actions.Node(
            package='perception',
            executable='perception',
            output='screen',
            )

    usb_cam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_project_usb_cam, 'launch', 'camera.launch.py')),
            launch_arguments={}.items(),
    )

    return launch.LaunchDescription([
            usb_cam,
            perception
        ])
