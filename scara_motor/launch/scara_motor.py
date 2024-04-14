import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('scara_motor'),
        'config',
        "scara_motor.yaml"
    )
    return LaunchDescription([
        Node(
            package='scara_motor',
            executable='scara_motor',
            name='scara_motor',
            parameters=[config],
        ),
    ])