# $ ros2 launch robot_bringup control_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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


    initial_position = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '/control_action', 'custom_interfaces/action/Control', '{action_number: 1}'],
        output='screen'
    ),

    initial_position = ExecuteProcess(
        cmd=['sleep', '0.2'],                 # cmd Befehl, warte 5 sec
        output = 'screen',                  # optional, Ausgabe im CMD                    
        on_exit = [ExecuteProcess(          # Am Ende (nach 5s) wird Aufgerufen
            cmd=['ros2', 'action', 'send_goal', '/control_action', 'custom_interfaces/action/Control', '{action_number: 1}'],
            output = 'screen',
            )]
    )


    ld.add_action(control)
    ld.add_action(gait)
    ld.add_action(neck_control)
    ld.add_action(initial_position)
    return ld