from setuptools import find_packages, setup

package_name = 'game_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboFEI',
    maintainer_email='thiago.tmoura01@gmail.com',
    description='The controller packages receives packets from the GameController and republishes them as msg/humanoid_league_msgs ROS2 messages. It sends response packetsback to the GameController',
    license='Apache License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connect = game_controller.connect:main'
        ],
    },
)
