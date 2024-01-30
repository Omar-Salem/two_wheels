from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "package_name",
            description="Package name",
        )]

    robot_nodes = create_robot_node()

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_robot_node() -> list:
    """

    :rtype: list
    """
    bump_go = Node(
        package=LaunchConfiguration("package_name"),
        executable="bump_go",
        name="bump_go",
    )
    return [bump_go]
