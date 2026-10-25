import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('office_delivery_robot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'office_strip.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'office_bot.urdf.xacro')
    bridge_path = os.path.join(pkg_share, 'config', 'office_bridge.yaml')

    x_pose = LaunchConfiguration('x_pose', default='-1.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_office_delivery_bot',
        output='screen',
        arguments=[
            '-name', 'office_delivery_bot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.02',
            '-Y', yaw_pose,
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='office_ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_path,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='-1.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw_pose', default_value='0.0'),
        gz_sim,
        robot_state_pub,
        spawn_robot,
        ros_gz_bridge,
    ])
