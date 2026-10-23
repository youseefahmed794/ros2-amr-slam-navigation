"""
Waypoint definitions and geometric utility functions.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import List, Tuple, Any

import yaml


def euler_from_quaternion(q: Any) -> float:
    """Extract yaw from a quaternion (object with x, y, z, w attributes)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# Default warehouse patrol waypoints (x, y, yaw)
DEFAULT_WAYPOINTS: List[Tuple[float, float, float]] = [
    (-8.0, -5.0, 0.0),       # Left shelf aisle start
    (-8.0,  2.0, 1.571),     # Left shelf aisle end, turn north
    (-3.0,  5.0, 0.0),       # Through doorway
    ( 5.0,  6.0, -1.571),    # Right room, upper area
    ( 5.0, -5.0, 3.14),      # Right shelf aisle
    ( 0.0, -5.0, 1.571),     # Center
    ( 0.0,  0.0, 0.0),       # Return to origin
]


def load_waypoints_file(file_path: str | Path) -> List[Tuple[float, float, float]]:
    """Load waypoint tuples (x, y, yaw) from a YAML file."""
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"Waypoints file not found: {file_path}")

    with open(path) as f:
        data = yaml.safe_load(f)

    if not data or 'waypoints' not in data:
        raise ValueError(f"Invalid waypoints file format in {file_path}")

    return [(float(wp['x']), float(wp['y']), float(wp.get('yaw', 0.0)))
            for wp in data['waypoints']]
