# Copyright 2024 zuiyuki-ctrl
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

"""Launch file for system_monitor package."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for system monitor nodes."""
    return LaunchDescription([
        Node(
            package='system_monitor',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='system_monitor',
            executable='resource_logger_node',
            name='resource_logger_node',
            output='screen',
            parameters=[],
        ),
    ])
