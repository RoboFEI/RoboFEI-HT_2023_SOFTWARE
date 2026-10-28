import os
from glob import glob
from setuptools import setup

package_name = 'dataset_capture'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboFEI',
    maintainer_email='lucas@robofei.com.br',
    description='Pacote para captura automatica de imagens da camera para treino de dataset',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = dataset_capture.camera_publisher:main',
            'image_saver = dataset_capture.image_saver:main',
        ],
    },
)
