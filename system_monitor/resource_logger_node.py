#!/usr/bin/env python3
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

"""
Resource Logger Node for ROS 2.

This node subscribes to system resource topics and logs them to a file.
"""

from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ResourceLoggerNode(Node):
    """Node that logs system resource data to a file."""

    def __init__(self):
        """Initialize the ResourceLoggerNode."""
        super().__init__('resource_logger_node')

        # Subscribe to system resources topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'system_resources',
            self.resource_callback,
            10
        )

        # Create log directory if it doesn't exist
        log_dir = os.path.expanduser('~/.ros/system_monitor_logs')
        os.makedirs(log_dir, exist_ok=True)

        # Create log file with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file_path = os.path.join(
            log_dir, f'resource_log_{timestamp}.csv')

        # Write header to log file
        with open(self.log_file_path, 'w') as f:
            f.write('timestamp,cpu_percent,memory_percent,disk_percent\n')

        self.get_logger().info(
            f'Resource Logger Node started. Logging to: {self.log_file_path}')
        self.log_count = 0

    def resource_callback(self, msg):
        """
        Handle incoming system resource messages.

        Parameters
        ----------
        msg : Float32MultiArray
            Message containing system resource data

        """
        if len(msg.data) >= 4:
            cpu_percent = msg.data[0]
            memory_percent = msg.data[1]
            disk_percent = msg.data[2]
            timestamp = msg.data[3]

            # Write to log file
            with open(self.log_file_path, 'a') as f:
                f.write(
                    f'{timestamp},{cpu_percent:.2f},'
                    f'{memory_percent:.2f},{disk_percent:.2f}\n')

            self.log_count += 1
            if self.log_count % 10 == 0:
                self.get_logger().info(
                    f'Logged {self.log_count} entries. '
                    f'CPU={cpu_percent:.1f}%, Memory={memory_percent:.1f}%, '
                    f'Disk={disk_percent:.1f}%')


def main(args=None):
    """Entry point for the resource_logger_node."""
    rclpy.init(args=args)
    node = ResourceLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Logging complete. Total entries: {node.log_count}')
        node.get_logger().info(
            f'Log file saved to: {node.log_file_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
