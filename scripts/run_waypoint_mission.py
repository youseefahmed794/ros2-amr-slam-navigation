#!/usr/bin/env python3
"""
Standalone mission script — drives the robot through a series of waypoints
for demonstration or testing.

Usage:
  python3 scripts/run_waypoint_mission.py
  python3 scripts/run_waypoint_mission.py --waypoints config/patrol.yaml \
      --output /tmp/mission_result.json
"""
import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description='Run a waypoint navigation mission')
    parser.add_argument('--waypoints', type=str, default='',
                        help='Path to waypoints YAML file (default: built-in patrol)')
    parser.add_argument('--output', type=str, default='/tmp/nav_benchmark.json',
                        help='Output JSON file for benchmark results')
    args = parser.parse_args()

    cmd = [
        'ros2', 'run', 'amr_slam_navigation', 'waypoint_navigator.py',
        '--ros-args',
        '-p', f'output_file:={args.output}',
    ]
    if args.waypoints:
        cmd.extend(['-p', f'waypoints_file:={args.waypoints}'])

    print(f'Starting waypoint mission...')
    print(f'Output: {args.output}')
    print(f'Command: {" ".join(cmd)}')
    print()

    try:
        result = subprocess.run(cmd)
        sys.exit(result.returncode)
    except KeyboardInterrupt:
        print('\nMission interrupted')
        sys.exit(1)


if __name__ == '__main__':
    main()
