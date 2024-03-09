# $ ros2 launch robot_bringup control_bringup.launch.py
 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    control = Node(
        package="control",
        executable="control",
        output = 'screen'
    )
    gait = Node(
        package="control",
        executable="gait_publisher",
        output = 'screen'
    )
    neck_control = Node(
        package="control",
        executable="neck_control",
        output = 'screen'
    )

    ld.add_action(control)
    ld.add_action(gait)
    ld.add_action(neck_control)
    return ld