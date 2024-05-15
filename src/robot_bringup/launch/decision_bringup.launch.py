# $ ros2 launch robot_bringup decision_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    control_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'robot_config.yaml'
    )

    decision = Node(
        package="decision_pkg_cpp",
        executable="robot_behavior",
        output = 'screen',
        parameters = [control_config]
    )

    ld.add_action(decision)
    return ld