import math
import sys
from pathlib import Path
import numpy as np
import pytest

# Ensure package root is importable standalone
pkg_root = Path(__file__).resolve().parent.parent
if str(pkg_root) not in sys.path:
    sys.path.insert(0, str(pkg_root))

from amr_slam_navigation.nav_benchmarks import (
    PoseStamped,
    NavigationMetrics,
    compute_path_length,
    compute_path_efficiency,
    compute_curvature,
    compute_localization_error,
    compute_relative_pose_error,
    compute_jerk,
    evaluate_navigation,
)


def test_empty_path():
    assert compute_path_length([]) == 0.0
    assert compute_curvature([]) == (0.0, 0.0)
    assert compute_jerk([]) == 0.0
    metrics = evaluate_navigation([], [], PoseStamped(0, 0, 0, 0))
    assert not metrics.goal_reached
    assert metrics.path_length == 0.0


def test_single_point_path():
    poses = [PoseStamped(0.0, 0.0, 0.0, 1.0)]
    assert compute_path_length(poses) == 0.0
    assert compute_curvature(poses) == (0.0, 0.0)
    assert compute_jerk(poses) == 0.0


def test_straight_line_path():
    poses = [
        PoseStamped(float(i), 0.0, 0.0, float(i))
        for i in range(11)
    ]
    # Length should be 10.0
    length = compute_path_length(poses)
    assert pytest.approx(length, rel=1e-5) == 10.0

    # Efficiency from (0,0) to (10,0) should be 1.0 (100%)
    efficiency = compute_path_efficiency(length, poses[0], poses[-1])
    assert pytest.approx(efficiency, rel=1e-5) == 1.0

    # Curvature of straight line should be 0.0
    mean_curv, max_curv = compute_curvature(poses)
    assert pytest.approx(mean_curv, abs=1e-5) == 0.0
    assert pytest.approx(max_curv, abs=1e-5) == 0.0

    # Constant velocity -> jerk should be 0.0
    jerk = compute_jerk(poses)
    assert pytest.approx(jerk, abs=1e-5) == 0.0


def test_detour_path_efficiency():
    # Robot goes (0,0) -> (5, 5) -> (10, 0)
    # Straight line distance is 10.0, actual length is 2 * sqrt(25+25) = 14.142
    poses = [
        PoseStamped(0.0, 0.0, 0.0, 0.0),
        PoseStamped(5.0, 5.0, 0.0, 5.0),
        PoseStamped(10.0, 0.0, 0.0, 10.0),
    ]
    length = compute_path_length(poses)
    expected_length = 2.0 * math.sqrt(50.0)
    assert pytest.approx(length, rel=1e-4) == expected_length

    efficiency = compute_path_efficiency(length, poses[0], poses[-1])
    expected_eff = 10.0 / expected_length
    assert pytest.approx(efficiency, rel=1e-4) == expected_eff
    assert efficiency < 1.0


def test_curvature_arc():
    # Circular arc with radius R = 5.0
    R = 5.0
    angles = np.linspace(0, math.pi / 2, 20)
    poses = [
        PoseStamped(R * math.cos(th), R * math.sin(th), th + math.pi / 2, float(i))
        for i, th in enumerate(angles)
    ]
    mean_curv, max_curv = compute_curvature(poses)
    # Theoretical curvature of circle radius R is 1/R = 0.20
    assert pytest.approx(mean_curv, rel=0.1) == 0.20
    assert pytest.approx(max_curv, rel=0.1) == 0.20


def test_localization_error():
    # Estimated poses slightly offset by 0.05m in X from ground truth
    gt = [
        PoseStamped(float(i), 0.0, 0.0, float(i))
        for i in range(10)
    ]
    est = [
        PoseStamped(float(i) + 0.05, 0.0, 0.0, float(i))
        for i in range(10)
    ]

    mean_err, max_err, rmse = compute_localization_error(est, gt)
    assert pytest.approx(mean_err, abs=1e-4) == 0.05
    assert pytest.approx(max_err, abs=1e-4) == 0.05
    assert pytest.approx(rmse, abs=1e-4) == 0.05


def test_evaluate_navigation_success():
    path = [
        PoseStamped(0.0, 0.0, 0.0, 0.0),
        PoseStamped(2.0, 0.0, 0.0, 2.0),
        PoseStamped(4.0, 0.0, 0.0, 4.0),
    ]
    gt = path
    goal = PoseStamped(4.0, 0.0, 0.0, 4.0)

    metrics = evaluate_navigation(path, gt, goal, xy_tolerance=0.25, yaw_tolerance=0.30)
    assert metrics.goal_reached
    assert metrics.goal_xy_error == pytest.approx(0.0, abs=1e-5)
    assert metrics.goal_yaw_error == pytest.approx(0.0, abs=1e-5)
    assert metrics.path_efficiency == pytest.approx(1.0, rel=1e-4)
    assert metrics.time_to_goal == 4.0

    d = metrics.to_dict()
    assert d['goal_reached'] is True
    assert 'path_efficiency' in d
    assert 'localization_rmse_m' in d
    assert 'rpe_trans_rmse_m' in d
    assert 'rpe_rot_rmse_rad' in d


def test_relative_pose_error():
    # Constant drift of 0.02m per step in translation
    gt = [
        PoseStamped(float(i), float(i), 0.1 * i, float(i))
        for i in range(10)
    ]
    # Each step adds 0.02m cumulative error in x
    est = [
        PoseStamped(float(i) + 0.02 * i, float(i), 0.1 * i, float(i))
        for i in range(10)
    ]

    t_mean, t_rmse, r_mean, r_rmse = compute_relative_pose_error(est, gt, delta_steps=1)
    assert pytest.approx(t_mean, abs=1e-4) == 0.02
    assert pytest.approx(t_rmse, abs=1e-4) == 0.02
    assert pytest.approx(r_mean, abs=1e-5) == 0.0
    assert pytest.approx(r_rmse, abs=1e-5) == 0.0
