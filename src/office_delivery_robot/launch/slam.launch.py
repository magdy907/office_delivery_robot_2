"""Start SLAM Toolbox (online async) and RViz for the office world.

Run AFTER the simulation is up:
    Terminal 1: ros2 launch office_delivery_robot office_world.launch.py
    Terminal 2: ros2 launch office_delivery_robot slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('office_delivery_robot')
    slam_share = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('rviz')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='true'),
        slam,
        rviz,
    ])
