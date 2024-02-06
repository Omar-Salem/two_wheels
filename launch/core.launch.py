import os

import launch_ros
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            description="Package name",
        ), DeclareLaunchArgument(
            "is_sim",
            default_value="true",
        )]

    robot_nodes = create_robot_node()

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_robot_node() -> list:
    """

    :rtype: list
    """
    package_name = LaunchConfiguration("package_name")
    is_sim = LaunchConfiguration('is_sim')
    robot_localization = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "ekf.yaml"]
            ),
            {'use_sim_time': is_sim}]
    )
    bump_go = Node(
        package=LaunchConfiguration("package_name"),
        executable="bump_go",
        name="bump_go",
    )
    velocity_remapper = Node(
        package=LaunchConfiguration("package_name"),
        executable="velocity_remapper",
        name="velocity_remapper",
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("slam_toolbox"), 'launch', 'online_async_launch.py'
        )])
    )
    map_path = PathJoinSubstitution([FindPackageShare(package_name), "worlds", "traffic_cones_map.yaml"])
    navigation_launch_file_path = [os.path.join(
        get_package_share_directory("nav2_bringup"), 'launch', 'navigation_launch.py'
    )]
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file_path),
        launch_arguments={'map': map_path, 'use_sim_time': is_sim}.items()
    )
    return [
        # robot_localization,
        # bump_go,
        velocity_remapper,
        slam_toolbox,
        nav2_bringup
    ]
