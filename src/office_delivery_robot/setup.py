import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'office_delivery_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@bue.edu.eg',
    description='Autonomous Office Document Delivery Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'corridor_safety_monitor = office_delivery_robot.corridor_safety_monitor:main',
            'office_drive_node = office_delivery_robot.office_drive_node:main',
        ],
    },
)
