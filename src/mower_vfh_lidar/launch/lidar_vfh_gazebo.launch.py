import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mower_description = get_package_share_directory('mower_description')
    mower_launch = os.path.join(pkg_mower_description, 'launch', 'mower_gazebo.launch.py')

    default_world = os.path.join(pkg_mower_description, 'worlds', 'room_world_open.world')
    default_rviz = os.path.join(pkg_mower_description, 'config', 'rviz', 'mower_gazebo.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rvizconfig = LaunchConfiguration('rvizconfig')

    include_mower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mower_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'gui': gui,
            'rviz': rviz,
            'rvizconfig': rvizconfig,
        }.items(),
    )

    lidar_vfh = Node(
        package='mower_vfh_lidar',
        executable='lidar_vfh_controller',
        name='lidar_vfh_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz),
        include_mower,
        lidar_vfh,
    ])
