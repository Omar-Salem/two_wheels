import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            description="Package name",
        ), DeclareLaunchArgument(
            "is_sim",
        )]

    robot_nodes = create_robot_node()

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_nav2_node(package_name, is_sim):
    map_yaml_file = PathJoinSubstitution([FindPackageShare(package_name), "maps", "apt.yaml"])
    navigation_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'bringup_launch.py'])
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file_path),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': is_sim,
            'package_name': package_name}.items()
    )
    return nav2_bringup


def create_slam_toolbox_node(package_name, is_sim):
    slam_toolbox_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'online_async_launch.py'])
    slam_params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "mapper_params_online_async.yaml"])
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file_path),
        launch_arguments={'use_sim_time': is_sim, 'slam_params_file': slam_params_file}.items()
    )
    return slam_toolbox


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
    slam_toolbox = create_slam_toolbox_node(package_name, is_sim)
    nav2_bringup = create_nav2_node(package_name, is_sim)
    return [
        # robot_localization,
        GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
                # slam_toolbox,
                # nav2_bringup,
                bump_go,
            ]
        )
    ]
