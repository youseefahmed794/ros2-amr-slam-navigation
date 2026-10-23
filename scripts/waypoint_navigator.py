#!/usr/bin/env python3
"""
Waypoint navigator using Nav2 Simple Commander API.

Sends the robot through a sequence of waypoints, records the path and
ground truth, and writes navigation benchmarks to a JSON file.

Usage:
  ros2 run amr_slam_navigation waypoint_navigator.py
  ros2 run amr_slam_navigation waypoint_navigator.py --ros-args \
      -p waypoints_file:=/path/to/waypoints.yaml \
      -p output_file:=/tmp/nav_benchmark.json
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from nav_msgs.msg import Odometry, Path as NavPath

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None
    TaskResult = None

import yaml

from amr_slam_navigation.nav_benchmarks import (
    PoseStamped, NavigationMetrics, evaluate_navigation,
)
from amr_slam_navigation.waypoints import (
    DEFAULT_WAYPOINTS, euler_from_quaternion, load_waypoints_file,
)


def make_pose(x: float, y: float, yaw: float, frame: str = 'map') -> RosPoseStamped:
    """Create a ROS PoseStamped from x, y, yaw."""
    pose = RosPoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class WaypointNavigatorNode(Node):
    """ROS 2 node that navigates through waypoints and benchmarks performance."""

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('output_file', '/tmp/nav_benchmark.json')
        self.declare_parameter('xy_tolerance', 0.25)
        self.declare_parameter('yaw_tolerance', 0.30)

        # Recorded data
        self.estimated_path: list[PoseStamped] = []
        self.ground_truth_path: list[PoseStamped] = []

        # Subscribers
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(
            Odometry, '/wheel_odom', self._odom_callback, qos)
        self.create_subscription(
            Odometry, '/ground_truth', self._gt_callback, qos)

        self.get_logger().info('WaypointNavigator initialized')

    def _odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.estimated_path.append(PoseStamped(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw, stamp=stamp,
        ))

    def _gt_callback(self, msg: Odometry):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.ground_truth_path.append(PoseStamped(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw, stamp=stamp,
        ))

    def load_waypoints(self) -> list[tuple[float, float, float]]:
        """Load waypoints from YAML file or use defaults."""
        wp_file = self.get_parameter('waypoints_file').value
        if wp_file and Path(wp_file).is_file():
            self.get_logger().info(f'Loading waypoints from {wp_file}')
            return load_waypoints_file(wp_file)
        self.get_logger().info('Using default warehouse patrol waypoints')
        return DEFAULT_WAYPOINTS

    def run_mission(self):
        """Execute the waypoint mission and return results."""
        if BasicNavigator is None:
            raise RuntimeError(
                'nav2_simple_commander is required for autonomous mission execution. '
                'Install: sudo apt install ros-jazzy-nav2-simple-commander'
            )
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()

        waypoints_raw = self.load_waypoints()
        self.get_logger().info(f'Navigating through {len(waypoints_raw)} waypoints')

        results = []
        for i, (x, y, yaw) in enumerate(waypoints_raw):
            self.get_logger().info(
                f'[{i+1}/{len(waypoints_raw)}] Navigating to ({x:.1f}, {y:.1f})')

            # Clear path for this segment
            segment_start_idx = len(self.estimated_path)
            gt_start_idx = len(self.ground_truth_path)
            t_start = time.time()

            goal = make_pose(x, y, yaw)
            goal.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(goal)

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    remaining = feedback.distance_remaining
                    if remaining < 0.3:
                        self.get_logger().info(
                            f'  Almost there: {remaining:.2f}m remaining')
                rclpy.spin_once(self, timeout_sec=0.1)

            result = navigator.getResult()
            t_elapsed = time.time() - t_start

            segment_path = self.estimated_path[segment_start_idx:]
            segment_gt = self.ground_truth_path[gt_start_idx:]

            goal_pose = PoseStamped(x=x, y=y, yaw=yaw, stamp=0.0)
            xy_tol = self.get_parameter('xy_tolerance').value
            yaw_tol = self.get_parameter('yaw_tolerance').value
            metrics = evaluate_navigation(
                segment_path, segment_gt, goal_pose, xy_tol, yaw_tol)

            status = 'SUCCEEDED' if result == TaskResult.SUCCEEDED else \
                     'CANCELED' if result == TaskResult.CANCELED else 'FAILED'
            self.get_logger().info(
                f'  Result: {status} | '
                f'Path: {metrics.path_length:.2f}m | '
                f'Efficiency: {metrics.path_efficiency:.1%} | '
                f'Loc RMSE: {metrics.localization_rmse:.4f}m | '
                f'Time: {t_elapsed:.1f}s')

            results.append({
                'waypoint_index': i,
                'goal': {'x': x, 'y': y, 'yaw': yaw},
                'nav2_result': status,
                'wall_time_s': round(t_elapsed, 2),
                **metrics.to_dict(),
            })

        return results


def main():
    rclpy.init()
    node = WaypointNavigatorNode()

    try:
        results = node.run_mission()

        output_file = node.get_parameter('output_file').value
        report = {
            'total_waypoints': len(results),
            'succeeded': sum(1 for r in results
                             if r['nav2_result'] == 'SUCCEEDED'),
            'failed': sum(1 for r in results
                          if r['nav2_result'] == 'FAILED'),
            'total_path_length_m': round(
                sum(r['path_length_m'] for r in results), 3),
            'mean_path_efficiency': round(
                float(np.mean([r['path_efficiency'] for r in results])), 4),
            'mean_localization_rmse_m': round(
                float(np.mean([r['localization_rmse_m'] for r in results])), 4),
            'waypoints': results,
        }

        Path(output_file).parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)
        node.get_logger().info(f'Benchmark saved to {output_file}')
        node.get_logger().info(
            f'Summary: {report["succeeded"]}/{report["total_waypoints"]} '
            f'succeeded, mean efficiency {report["mean_path_efficiency"]:.1%}')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
