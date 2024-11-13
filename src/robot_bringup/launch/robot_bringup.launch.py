#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import TextSubstitution

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import sys
sys.path.append("/home")
from robot_num import robot_number

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))
    log_level = LaunchConfiguration('log_level')
    
    vision_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('robot_bringup'), '/launch/vision_bringup.launch.py']
        ),
        launch_arguments = {
            'log_level': log_level
        }.items()     
    )

    ld.add_action(vision_launcher)

    gamecontroller_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('robot_bringup'), '/launch/gamecontroller_bringup.launch.py']
        )
    )

    ld.add_action(gamecontroller_launcher)

    imu_node = Node(
        package="um7",
        executable="um7_node",
        output = 'screen',
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )

    ld.add_action(imu_node)

    decision_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('robot_bringup'), '/launch/decision_bringup.launch.py']
        ),
        launch_arguments = {
            'log_level': log_level
        }.items()  
    )

    ld.add_action(decision_launcher)
    
    return ld   