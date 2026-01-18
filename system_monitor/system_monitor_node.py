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
System Monitor Node for ROS 2.

This node monitors system resources (CPU, memory, disk usage) and publishes
them as ROS 2 topics.
"""

import time

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class SystemMonitorNode(Node):
    """Node that monitors and publishes system resource usage."""

    def __init__(self):
        """Initialize the SystemMonitorNode."""
        super().__init__('system_monitor_node')

        # Publisher for system resource data
        # Format: [cpu_percent, memory_percent, disk_percent, timestamp]
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'system_resources',
            10
        )

        # Timer to publish system resources every second
        self.timer = self.create_timer(1.0, self.publish_system_resources)

        self.get_logger().info('System Monitor Node started')

    def get_system_resources(self):
        """
        Collect current system resource usage.

        Returns
        -------
        tuple
            (cpu_percent, memory_percent, disk_percent)

        """
        cpu_percent = psutil.cpu_percent(interval=None)
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        disk = psutil.disk_usage('/')
        disk_percent = disk.percent

        return cpu_percent, memory_percent, disk_percent

    def publish_system_resources(self):
        """Publish current system resource usage as Float32MultiArray."""
        cpu_percent, memory_percent, disk_percent = self.get_system_resources()
        timestamp = time.time()

        msg = Float32MultiArray()
        msg.data = [
            float(cpu_percent),
            float(memory_percent),
            float(disk_percent),
            float(timestamp)
        ]

        # Set layout dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = 'resource_data'
        msg.layout.dim[0].size = len(msg.data)
        msg.layout.dim[0].stride = len(msg.data)

        self.publisher_.publish(msg)

        self.get_logger().debug(
            f'Published: CPU={cpu_percent:.1f}%, '
            f'Memory={memory_percent:.1f}%, '
            f'Disk={disk_percent:.1f}%'
        )


def main(args=None):
    """Entry point for the system_monitor_node."""
    rclpy.init(args=args)
    node = SystemMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
