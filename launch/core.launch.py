import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    # remap_diff_drive_controller_cmd_vel = Node(
    #     package='turtlesim',
    #     executable='mimic',
    #     name='mimic',
    #     remappings=[
    #         ('/input/pose', '/turtlesim1/turtle1/pose'),
    #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    #     ]
    # )
    bump_go = Node(
        package=LaunchConfiguration("package_name"),
        executable="bump_go",
        name="bump_go",
    )
    return [robot_localization,

            # bump_go
            ]
