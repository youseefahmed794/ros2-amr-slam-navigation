"""Unit tests for waypoint navigation logic, waypoint loading, and benchmark calculation."""
import math
import sys
import tempfile
from pathlib import Path
import pytest
import yaml

pkg_root = Path(__file__).resolve().parent.parent
if str(pkg_root) not in sys.path:
    sys.path.insert(0, str(pkg_root))

from amr_slam_navigation.waypoints import (
    DEFAULT_WAYPOINTS,
    euler_from_quaternion,
    load_waypoints_file,
)
from scripts.benchmark_navigation import summarize_results


class DummyQuat:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def test_euler_from_quaternion():
    # Zero rotation (identity quaternion)
    q0 = DummyQuat(0.0, 0.0, 0.0, 1.0)
    assert pytest.approx(euler_from_quaternion(q0), abs=1e-5) == 0.0

    # 90 degrees yaw (pi/2) -> z = sin(pi/4), w = cos(pi/4)
    q_90 = DummyQuat(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert pytest.approx(euler_from_quaternion(q_90), abs=1e-5) == math.pi / 2

    # 180 degrees yaw (pi) -> z = 1.0, w = 0.0
    q_180 = DummyQuat(0.0, 0.0, 1.0, 0.0)
    assert pytest.approx(abs(euler_from_quaternion(q_180)), abs=1e-5) == math.pi

    # -90 degrees yaw (-pi/2) -> z = sin(-pi/4), w = cos(-pi/4)
    q_neg90 = DummyQuat(0.0, 0.0, math.sin(-math.pi / 4), math.cos(-math.pi / 4))
    assert pytest.approx(euler_from_quaternion(q_neg90), abs=1e-5) == -math.pi / 2


def test_default_waypoints_bounds():
    assert len(DEFAULT_WAYPOINTS) >= 3

    # All waypoints must be well within warehouse bounds [-14, 14] x [-9, 9]
    for i, (x, y, yaw) in enumerate(DEFAULT_WAYPOINTS):
        assert -14.0 <= x <= 14.0, f"Waypoint {i} x={x} out of bounds"
        assert -9.0 <= y <= 9.0, f"Waypoint {i} y={y} out of bounds"
        assert -2 * math.pi <= yaw <= 2 * math.pi, f"Waypoint {i} yaw={yaw} invalid"

    # Adjacent waypoints distance should be reasonable (< 20m)
    for i in range(1, len(DEFAULT_WAYPOINTS)):
        x1, y1, _ = DEFAULT_WAYPOINTS[i - 1]
        x2, y2, _ = DEFAULT_WAYPOINTS[i]
        d = math.hypot(x2 - x1, y2 - y1)
        assert 0.1 < d < 20.0, f"Jump between waypoint {i-1} and {i} is {d}m"


def test_load_waypoints_file():
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.dump(
            {
                "waypoints": [
                    {"x": 1.5, "y": -2.0, "yaw": 0.5},
                    {"x": 3.0, "y": 4.0},  # default yaw 0.0
                ]
            },
            f,
        )
        temp_path = Path(f.name)

    try:
        wps = load_waypoints_file(temp_path)
        assert len(wps) == 2
        assert wps[0] == (1.5, -2.0, 0.5)
        assert wps[1] == (3.0, 4.0, 0.0)
    finally:
        temp_path.unlink(missing_ok=True)


def test_summarize_results_empty():
    assert summarize_results([]) == {}
    assert summarize_results([{'waypoints': []}]) == {}


def test_summarize_results_valid():
    trials = [
        {
            'waypoints': [
                {
                    'waypoint_index': 0,
                    'nav2_result': 'SUCCEEDED',
                    'path_efficiency': 0.95,
                    'localization_rmse_m': 0.04,
                    'wall_time_s': 12.0,
                    'path_length_m': 8.5,
                },
                {
                    'waypoint_index': 1,
                    'nav2_result': 'SUCCEEDED',
                    'path_efficiency': 0.85,
                    'localization_rmse_m': 0.06,
                    'wall_time_s': 18.0,
                    'path_length_m': 11.2,
                },
                {
                    'waypoint_index': 2,
                    'nav2_result': 'FAILED',
                    'path_efficiency': 0.40,
                    'localization_rmse_m': 0.20,
                    'wall_time_s': 30.0,
                    'path_length_m': 5.0,
                },
            ]
        }
    ]

    summary = summarize_results(trials)
    assert summary['total_trials'] == 1
    assert summary['total_waypoints'] == 3
    assert summary['succeeded'] == 2
    assert pytest.approx(summary['success_rate'], abs=1e-3) == 2 / 3

    # Efficiencies: mean of 0.95 and 0.85 is 0.90
    assert pytest.approx(summary['path_efficiency']['mean'], abs=1e-3) == 0.90
    # RMSE: mean of 0.04 and 0.06 is 0.05
    assert pytest.approx(summary['localization_rmse_m']['mean'], abs=1e-3) == 0.05
    assert summary['localization_rmse_m']['max'] == 0.06
    # Total distance: 8.5 + 11.2 = 19.7
    assert pytest.approx(summary['path_length_m']['total'], abs=1e-3) == 19.7
