import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'two_wheels_core'

    declared_arguments = [
        DeclareLaunchArgument(
            "is_sim",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )]
    is_sim = LaunchConfiguration("is_sim")

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' is_sim:=', is_sim])

    params = {
        'robot_description': robot_description_config,
        'use_sim_time': is_sim
    }
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo_nodes = create_gazebo_nodes(package_name)
    controller_nodes = create_controller_nodes()
    rviz_node = create_rviz_node(package_name)
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'core.launch.py'
        )]), launch_arguments={
            'package_name': package_name,
            'is_sim': is_sim
        }.items()
    )

    return LaunchDescription(declared_arguments +
                             [
                                 rviz_node,
                                 robot_state_pub_node,
                                 core
                             ]
                             + gazebo_nodes
                             + controller_nodes
                             )


def create_rviz_node(package_name):
    package_share = FindPackageShare(package_name)
    rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "two_wheels.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )
    return rviz_node


def create_controller_nodes() -> list:
    """

    :rtype: list
    """
    robot_controller_names = ['joint_state_broadcaster', 'diff_drive_controller']
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller]
            )
        ]
    return robot_controller_spawners


def create_gazebo_nodes(package_name) -> list:
    """

    :rtype: list
    """
    package_dir = FindPackageShare(package_name)
    world = PathJoinSubstitution(
        [package_dir, 'worlds', 'many_walls.world']
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world}.items()
    )
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'two_wheels'],
                        output='screen')
    # launch.logging.launch_config.level = logging.WARN
    return [gazebo, spawn_entity]
