from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/camera'),
        DeclareLaunchArgument('output_dir', default_value='~/dataset_images'),
        DeclareLaunchArgument('num_images', default_value='1000'),
        DeclareLaunchArgument('mode', default_value='auto'),
        DeclareLaunchArgument('interval', default_value='0.5'),

        Node(
            package='dataset_capture',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
            }],
        ),
        Node(
            package='dataset_capture',
            executable='image_saver',
            name='image_saver',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'num_images': LaunchConfiguration('num_images'),
                'mode': LaunchConfiguration('mode'),
                'interval': LaunchConfiguration('interval'),
            }],
        ),
    ])
