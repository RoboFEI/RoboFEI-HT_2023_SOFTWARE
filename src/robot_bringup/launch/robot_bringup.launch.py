#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    robot_config_file = 'robot_config'+str(robot_number)+'.yaml'
    vision_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        robot_config_file 
    )

    robot_config_file = 'robot_config'+str(robot_number)+'.yaml'
    control_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        robot_config_file
    )
    
    camera_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'camera_config.yaml'
    )
    
    control = Node(
        package="control",
        executable="control",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )

    gait = Node(
        package="control",
        executable="gait_publisher",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True
    )
    
    neck_control = Node(
        package="control",
        executable="neck_control",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output = 'screen',
        parameters = [camera_config],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=True,
        respawn_delay=0.5,
        emulate_tty=True
    )

    image_viwer = Node(
        package="vision_pkg",
        executable="detect",
        output = 'screen',
        parameters = [vision_config, camera_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )

    gc_node = Node(
        package="game_controller",
        executable="connect",
        output = 'screen'
    )

    motors = Node(
        package="motors_pkg",
        executable="motors_communication",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )

    imu = Node(
        package="um7",
        executable="um7_node",
        output = 'screen',
        parameters = [control_config],
        arguments=['--ros-args', '--log-level', log_level,
                   '--log-level',  'rcl:=info',
                   '--log-level',  'rmw_fastrtps_cpp:=info'],
        emulate_tty=True
    )


    ld.add_action(control)
    ld.add_action(gait)
    ld.add_action(neck_control)
    ld.add_action(camera)
    ld.add_action(image_viwer)
    ld.add_action(gc_node)
    ld.add_action(motors)
    ld.add_action(imu)
    
    
    return ld