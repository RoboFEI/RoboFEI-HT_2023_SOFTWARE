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

    vision_node = Node(
        package="vision_pkg",
        executable="detect",
        output="screen"
    )

    gamecontroller = Node(
        package="game_controller",
        executable="connect",
        output="screen"
    )

    motors = Node(
        package="motors_pkg",
        executable="motors_communication",
        output="screen"
    )

    imu = Node(
        package="um7",
        executable="um7_node",
        output="screen"
    )


    ld.add_action(control)
    ld.add_action(gait)
    ld.add_action(neck_control)
    ld.add_action(vision_node)
    ld.add_action(gamecontroller)
    ld.add_action(motors)
    ld.add_action(imu)
    
    
    return ld