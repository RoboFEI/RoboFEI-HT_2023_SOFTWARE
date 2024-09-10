# $ ros2 launch robot_bringup vision_bringup.launch.py

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# import sys
# sys.path.append("/home")
# from robot_num import robot_number

# def generate_launch_description():
#     ld = LaunchDescription()

#     config_file = 'robot_config'+str(robot_number)+'.yaml'
#     vision_config = os.path.join(
#         get_package_share_directory('robot_bringup'),
#         'config',
#         config_file
#     )

#     vision = Node(
#         package="vision_pkg",
#         executable="detect",
#         output = 'screen',
#         parameters = [vision_config]
#     )

#     ld.add_action(vision)
#     return ld

# $ ros2 launch robot_bringup control_bringup.launch.py

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
    vision_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        robot_config_file 
    )
    
    camera_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'camera_config.yaml'
    )

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output = 'screen',
        parameters = [camera_config]
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

    ld.add_action(camera)
    ld.add_action(image_viwer)
    return ld