# Copyright 2025 Husarion sp. z o.o.
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),  # panther
        description="Add namespace to all launched nodes",
    )

    send_to_dock_config_path = LaunchConfiguration("send_to_dock_config_path")
    declare_send_to_dock_config_path = DeclareLaunchArgument(
        "send_to_dock_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("send_to_dock"),
                "config",
                "send_to_dock.yaml",
            ]
        ),
        description="Specify path to configuration file for docking",
    )

    send_to_dock_node = Node(
        package="send_to_dock",
        executable="send_to_dock_node",
        name="send_to_dock_node",
        parameters=[send_to_dock_config_path],
        namespace=namespace,
        # remappings=[("/diagnostics", "diagnostics")],
    )

    actions = [
        declare_namespace_arg,
        declare_send_to_dock_config_path,
        send_to_dock_node,
    ]

    return LaunchDescription(actions)
