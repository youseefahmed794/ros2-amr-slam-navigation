"""
Navigation benchmark utilities.

Measures path efficiency, localization accuracy, and obstacle avoidance
performance by comparing Nav2-followed paths against ground truth and
optimal straight-line distances.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class PoseStamped:
    """Lightweight pose for benchmarking (avoids ROS dependency in tests)."""
    x: float
    y: float
    yaw: float
    stamp: float  # seconds


@dataclass
class NavigationMetrics:
    """Aggregate metrics for a navigation run."""
    # Goal-reaching
    goal_reached: bool = False
    goal_xy_error: float = float('inf')
    goal_yaw_error: float = float('inf')
    time_to_goal: float = float('inf')

    # Path quality
    path_length: float = 0.0
    optimal_distance: float = 0.0
    path_efficiency: float = 0.0  # optimal / actual (1.0 = perfect)

    # Smoothness
    mean_curvature: float = 0.0
    max_curvature: float = 0.0
    jerk_metric: float = 0.0

    # Localization
    mean_localization_error: float = 0.0
    max_localization_error: float = 0.0
    localization_rmse: float = 0.0
    rpe_trans_rmse: float = 0.0
    rpe_rot_rmse: float = 0.0

    # Safety
    min_obstacle_clearance: float = float('inf')
    collision_count: int = 0

    def to_dict(self) -> dict:
        """Serialize to JSON-friendly dictionary."""
        return {
            'goal_reached': self.goal_reached,
            'goal_xy_error_m': round(self.goal_xy_error, 4),
            'goal_yaw_error_rad': round(self.goal_yaw_error, 4),
            'time_to_goal_s': round(self.time_to_goal, 2),
            'path_length_m': round(self.path_length, 3),
            'optimal_distance_m': round(self.optimal_distance, 3),
            'path_efficiency': round(self.path_efficiency, 4),
            'mean_curvature': round(self.mean_curvature, 4),
            'max_curvature': round(self.max_curvature, 4),
            'jerk_metric': round(self.jerk_metric, 4),
            'mean_localization_error_m': round(self.mean_localization_error, 4),
            'max_localization_error_m': round(self.max_localization_error, 4),
            'localization_rmse_m': round(self.localization_rmse, 4),
            'rpe_trans_rmse_m': round(self.rpe_trans_rmse, 4),
            'rpe_rot_rmse_rad': round(self.rpe_rot_rmse, 4),
            'min_obstacle_clearance_m': round(self.min_obstacle_clearance, 4),
            'collision_count': self.collision_count,
        }


def compute_path_length(poses: List[PoseStamped]) -> float:
    """Total Euclidean path length through a sequence of poses."""
    if len(poses) < 2:
        return 0.0
    length = 0.0
    for i in range(1, len(poses)):
        dx = poses[i].x - poses[i - 1].x
        dy = poses[i].y - poses[i - 1].y
        length += math.hypot(dx, dy)
    return length


def compute_path_efficiency(actual_length: float, start: PoseStamped,
                            goal: PoseStamped) -> float:
    """Ratio of straight-line distance to actual path length."""
    optimal = math.hypot(goal.x - start.x, goal.y - start.y)
    if actual_length < 1e-6:
        return 0.0
    return min(optimal / actual_length, 1.0)


def compute_curvature(poses: List[PoseStamped]) -> Tuple[float, float]:
    """Mean and max curvature from heading changes per unit distance."""
    if len(poses) < 3:
        return 0.0, 0.0
    curvatures = []
    for i in range(1, len(poses) - 1):
        dx1 = poses[i].x - poses[i - 1].x
        dy1 = poses[i].y - poses[i - 1].y
        dx2 = poses[i + 1].x - poses[i].x
        dy2 = poses[i + 1].y - poses[i].y
        ds1 = math.hypot(dx1, dy1)
        ds2 = math.hypot(dx2, dy2)
        if ds1 < 1e-6 or ds2 < 1e-6:
            continue
        heading1 = math.atan2(dy1, dx1)
        heading2 = math.atan2(dy2, dx2)
        dtheta = abs(math.atan2(math.sin(heading2 - heading1),
                                math.cos(heading2 - heading1)))
        ds = (ds1 + ds2) / 2.0
        curvatures.append(dtheta / ds)
    if not curvatures:
        return 0.0, 0.0
    return float(np.mean(curvatures)), float(np.max(curvatures))


def compute_localization_error(estimated: List[PoseStamped],
                                ground_truth: List[PoseStamped]
                                ) -> Tuple[float, float, float]:
    """
    Compute mean, max, and RMSE of localization error.
    Matches poses by nearest timestamp.
    """
    if not estimated or not ground_truth:
        return 0.0, 0.0, 0.0

    gt_times = np.array([p.stamp for p in ground_truth])
    errors = []
    for est in estimated:
        idx = np.argmin(np.abs(gt_times - est.stamp))
        gt = ground_truth[idx]
        err = math.hypot(est.x - gt.x, est.y - gt.y)
        errors.append(err)

    errors_arr = np.array(errors)
    mean_err = float(np.mean(errors_arr))
    max_err = float(np.max(errors_arr))
    rmse = float(np.sqrt(np.mean(errors_arr ** 2)))
    return mean_err, max_err, rmse


def compute_jerk(poses: List[PoseStamped]) -> float:
    """Jerk metric: mean rate of acceleration change along the path."""
    if len(poses) < 4:
        return 0.0
    velocities = []
    for i in range(1, len(poses)):
        dt = poses[i].stamp - poses[i - 1].stamp
        if dt < 1e-6:
            continue
        ds = math.hypot(poses[i].x - poses[i - 1].x,
                        poses[i].y - poses[i - 1].y)
        velocities.append((ds / dt, poses[i].stamp))

    if len(velocities) < 3:
        return 0.0

    accels = []
    for i in range(1, len(velocities)):
        dt = velocities[i][1] - velocities[i - 1][1]
        if dt < 1e-6:
            continue
        accels.append(((velocities[i][0] - velocities[i - 1][0]) / dt,
                       velocities[i][1]))

    if len(accels) < 2:
        return 0.0

    jerks = []
    for i in range(1, len(accels)):
        dt = accels[i][1] - accels[i - 1][1]
        if dt < 1e-6:
            continue
        jerks.append(abs(accels[i][0] - accels[i - 1][0]) / dt)

    return float(np.mean(jerks)) if jerks else 0.0


def compute_relative_pose_error(
    estimated: List[PoseStamped],
    ground_truth: List[PoseStamped],
    delta_steps: int = 1,
) -> Tuple[float, float, float, float]:
    """Calculate Relative Pose Error (RPE) across delta_steps intervals.

    Measures odometry and SLAM drift per unit interval:
    Delta_T_gt = T_gt(t)^(-1) * T_gt(t+delta)
    Delta_T_est = T_est(t)^(-1) * T_est(t+delta)
    Error = Delta_T_gt^(-1) * Delta_T_est

    Returns:
        Tuple of (rpe_trans_mean, rpe_trans_rmse, rpe_rot_mean, rpe_rot_rmse)
    """
    n = min(len(estimated), len(ground_truth))
    if n <= delta_steps:
        return 0.0, 0.0, 0.0, 0.0

    trans_errors = []
    rot_errors = []

    for i in range(n - delta_steps):
        j = i + delta_steps
        dx_gt = ground_truth[j].x - ground_truth[i].x
        dy_gt = ground_truth[j].y - ground_truth[i].y
        dyaw_gt = ground_truth[j].yaw - ground_truth[i].yaw

        dx_est = estimated[j].x - estimated[i].x
        dy_est = estimated[j].y - estimated[i].y
        dyaw_est = estimated[j].yaw - estimated[i].yaw

        trans_err = math.hypot(dx_est - dx_gt, dy_est - dy_gt)
        trans_errors.append(trans_err)

        rot_err = abs(math.atan2(
            math.sin(dyaw_est - dyaw_gt), math.cos(dyaw_est - dyaw_gt)
        ))
        rot_errors.append(rot_err)

    trans_arr = np.array(trans_errors)
    rot_arr = np.array(rot_errors)

    trans_mean = float(np.mean(trans_arr))
    trans_rmse = float(np.sqrt(np.mean(trans_arr ** 2)))
    rot_mean = float(np.mean(rot_arr))
    rot_rmse = float(np.sqrt(np.mean(rot_arr ** 2)))

    return trans_mean, trans_rmse, rot_mean, rot_rmse


def evaluate_navigation(
    path: List[PoseStamped],
    ground_truth: List[PoseStamped],
    goal: PoseStamped,
    xy_tolerance: float = 0.25,
    yaw_tolerance: float = 0.30,
) -> NavigationMetrics:
    """Compute full navigation benchmark from recorded poses."""
    metrics = NavigationMetrics()

    if not path:
        return metrics

    start = path[0]
    end = path[-1]

    # Goal reaching
    metrics.goal_xy_error = math.hypot(end.x - goal.x, end.y - goal.y)
    metrics.goal_yaw_error = abs(math.atan2(
        math.sin(end.yaw - goal.yaw), math.cos(end.yaw - goal.yaw)))
    metrics.goal_reached = (metrics.goal_xy_error <= xy_tolerance and
                            metrics.goal_yaw_error <= yaw_tolerance)
    metrics.time_to_goal = end.stamp - start.stamp

    # Path quality
    metrics.path_length = compute_path_length(path)
    metrics.optimal_distance = math.hypot(goal.x - start.x, goal.y - start.y)
    metrics.path_efficiency = compute_path_efficiency(
        metrics.path_length, start, goal)

    # Smoothness
    metrics.mean_curvature, metrics.max_curvature = compute_curvature(path)
    metrics.jerk_metric = compute_jerk(path)

    # Localization accuracy
    if ground_truth:
        (metrics.mean_localization_error,
         metrics.max_localization_error,
         metrics.localization_rmse) = compute_localization_error(
            path, ground_truth)
        _, metrics.rpe_trans_rmse, _, metrics.rpe_rot_rmse = compute_relative_pose_error(
            path, ground_truth, delta_steps=1)

    return metrics
