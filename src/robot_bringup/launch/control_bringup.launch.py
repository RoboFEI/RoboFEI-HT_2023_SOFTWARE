# $ ros2 launch robot_bringup control_bringup.launch.py

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

    control = Node(
        package="control",
        executable="control",
        output = 'screen',
        parameters = [control_config]
    )

    gait = Node(
        package="control",
        executable="gait_publisher",
        output = 'screen',
        parameters = [control_config]
    )
    
    neck_control = Node(
        package="control",
        executable="neck_control",
        output = 'screen',
        parameters = [control_config]
    )

    ld.add_action(control)
    ld.add_action(gait)
    ld.add_action(neck_control)
    return ld