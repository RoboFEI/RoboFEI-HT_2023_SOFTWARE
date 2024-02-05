from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    open_camera = Node(
        package="vision_pkg",
        executable="open_cam"
    )

    start_detect = Node(
        package="vision_pkg",
        executable="detect"
    )

    ld.add_action(open_camera)
    ld.add_action(start_detect)
    
    return ld