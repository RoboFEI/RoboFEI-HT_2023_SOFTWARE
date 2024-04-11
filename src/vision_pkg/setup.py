from setuptools import find_packages, setup

package_name = 'vision_pkg'
submodules = 'vision_pkg/submodules'

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
    maintainer_email='rpj134@gmail.com',
    description='This package recive data from a camera and can recognize balls, robots and some fields landmarks',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect     = vision_pkg.detect:main'
        ],
    },
)
