import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
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
    vfh_type = LaunchConfiguration('vfh_type')  # 'depth' or 'lidar'

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

    # 深度相机VFH控制器（C++版本）
    depth_vfh = Node(
        package='mower_vfh',
        executable='depth_vfh_controller',
        name='depth_vfh_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(EqualsSubstitution(vfh_type, 'depth'))
    )

    # 激光雷达VFH控制器（C++版本）
    lidar_vfh = Node(
        package='mower_vfh_lidar',
        executable='lidar_vfh_controller',
        name='lidar_vfh_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(EqualsSubstitution(vfh_type, 'lidar'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz),
        DeclareLaunchArgument('vfh_type', default_value='lidar', description='VFH controller type: depth or lidar'),
        include_mower,
        depth_vfh,
        lidar_vfh,
    ])