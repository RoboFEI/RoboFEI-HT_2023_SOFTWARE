# $ ros2 launch robot_bringup decision_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import sys
sys.path.append("/home")
from robot_num import robot_number

def generate_launch_description():
    ld = LaunchDescription()

    config_file = 'robot_config'+str(robot_number)+'.yaml'
    control_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        config_file
    )

    decision = Node(
        package="decision_pkg_cpp",
        executable="robot_behavior",
        output = 'screen',
        parameters = [control_config]
    )

    ld.add_action(decision)
    return ld