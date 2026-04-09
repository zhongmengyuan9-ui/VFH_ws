import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取功能包的路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mower_description = get_package_share_directory('mower_description')
    
    # 使用命名空间来避免参数冲突
    robot_namespace = LaunchConfiguration('namespace', default='mower')

    # Gazebo 启动参数：是否使用GUI，是否暂停，世界文件路径
    default_world_path = os.path.join(pkg_mower_description, 'worlds', 'room_world_open.world')
    default_rviz_config = os.path.join(pkg_mower_description, 'config', 'rviz', 'mower_gazebo.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default=default_world_path)

    # 启动 Gazebo 服务器和客户端
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui', default='true'))
    )

    # 从 URDF 加载机器人描述
    urdf_path = os.path.join(pkg_mower_description, 'urdf', 'mower.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 启动 robot_state_publisher，发布 TF 和 joint states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
        #remappings=[('/robot_description', '/mower_robot_description')]
    )

    # Merge Gazebo joint states and publish defaults for joints that Gazebo doesn't report
    # (otherwise RobotStatePublisher won't publish TF for those joints, e.g. casters).
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 30,
            # Publish default positions for all joints so RobotStatePublisher can always
            # produce a complete TF tree (e.g. caster joints) even if Gazebo joint states
            # are missing / delayed after switching worlds.
            'publish_default_positions': True,
        }]
    )

    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mower', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('mower_rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('mower_rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Whether to run gzclient'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Whether to run gzclient'),
        DeclareLaunchArgument('world', default_value=default_world_path,
                              description='Gazebo world file'),
        # Backward-compatible RViz args for direct CLI usage
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Whether to run RViz2'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config,
                              description='Path to RViz2 config file'),
        # Namespaced RViz args to avoid collisions when included by other launches
        DeclareLaunchArgument('mower_rviz', default_value=LaunchConfiguration('rviz'),
                              description='(Internal) Whether to run RViz2 for mower_description'),
        DeclareLaunchArgument('mower_rvizconfig', default_value=LaunchConfiguration('rvizconfig'),
                              description='(Internal) RViz2 config path for mower_description'),
        gazebo,
        gazebo_client,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rviz2,
    ])