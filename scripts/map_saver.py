#!/usr/bin/env python3
"""
Map saver utility — saves the current SLAM map to disk.

Usage:
  ros2 run amr_slam_navigation map_saver.py
  ros2 run amr_slam_navigation map_saver.py --ros-args -p map_name:=warehouse_v2
"""
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_utility')
        self.declare_parameter('map_name', 'warehouse_map')
        self.declare_parameter('output_dir', '')

    def save_map(self):
        map_name = self.get_parameter('map_name').value
        output_dir = self.get_parameter('output_dir').value

        if not output_dir:
            # Default to package maps directory
            from ament_index_python.packages import get_package_share_directory
            output_dir = str(Path(get_package_share_directory(
                'amr_slam_navigation')) / 'maps')

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        full_path = str(output_path / map_name)

        self.get_logger().info(f'Saving map to {full_path}...')

        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', full_path,
            '--ros-args', '-p', 'save_map_timeout:=10000',
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True,
                                    timeout=30)
            if result.returncode == 0:
                self.get_logger().info(f'Map saved: {full_path}.pgm + {full_path}.yaml')
            else:
                self.get_logger().error(f'Map save failed: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timed out after 30s')
        except FileNotFoundError:
            self.get_logger().error('nav2_map_server not found. '
                                   'Install: sudo apt install ros-jazzy-nav2-map-server')


def main():
    rclpy.init()
    node = MapSaverNode()
    try:
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
