# $ ros2 launch robot_bringup decision_bringup.launch.py


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import sys
sys.path.append("/home")
from robot_num import robot_number

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))
    log_level = LaunchConfiguration('log_level')

    robot_config_file = 'robot_config'+str(robot_number)+'.yaml'
    control_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        robot_config_file
    )

    decision = Node(
        package="decision_pkg_cpp",
        executable="robot_behavior",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )

    ld.add_action(decision)
    return ld