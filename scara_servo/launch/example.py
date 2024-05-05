# modified from https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/launch/demo_ros_api.launch.py

import os
import yaml
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config").to_moveit_configs()

    # launch_as_standalone_node = LaunchConfiguration(
    #     "launch_as_standalone_node", default="true"
    # )

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "scara_arm"}

    # servo_params = {
    #     "scara_servo": ParameterBuilder("scara_servo")
    #     .yaml("config/servo_config.yaml")
    #     .to_dict()
    # }
    servo_yaml = load_yaml("scara_servo", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    scara_servo = launch_ros.actions.Node(
        package="scara_servo",
        executable="example_node",
        name="example_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        # condition=IfCondition(launch_as_standalone_node),
    )

    return launch.LaunchDescription([
        scara_servo,
    ])