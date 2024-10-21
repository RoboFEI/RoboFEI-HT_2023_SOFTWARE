# $ ros2 launch robot_bringup decision_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

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

    team_number_launch_arg = DeclareLaunchArgument(
        "team_number", default_value=TextSubstitution(text="7")
    )

    gc_node = Node(
        package="game_controller",
        executable="connect",
        output = 'screen',
        parameters = [control_config, {"team_number": LaunchConfiguration('team_number')}]
    )

    return LaunchDescription([
        team_number_launch_arg,
        gc_node
    ])