# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    use_mock_hardware = 'true'

    # Get URDF via xacro
    package_share = FindPackageShare('two_wheels')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [package_share, "urdf", 'two_wheels.urdf.xacro']
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content,
                         "use_sim_time": True}

    rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "two_wheels.rviz"]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    gazebo_nodes = create_gazebo_nodes(package_share)
    return LaunchDescription(
        [
            robot_state_pub_node,
            joint_state_publisher_node,
            rviz_node,
        ]
        + gazebo_nodes
    )


def create_gazebo_nodes(package_dir: object) -> list:
    """

    :rtype: list
    """
    world_files = PathJoinSubstitution(
        [package_dir, 'worlds', 'my_world.world']
    )
    gazeboworld = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_files, '-s', 'libgazebo_ros_factory.so']
    )
    return [
        gazeboworld,
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "robot_name"])
    ]
