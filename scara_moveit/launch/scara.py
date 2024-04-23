import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('scara_moveit_config'),
                        'launch/demo.launch.py')
        )
    )

    ld = LaunchDescription([
    ])

    ld.add_action(moveit_demo_launch)

    return ld