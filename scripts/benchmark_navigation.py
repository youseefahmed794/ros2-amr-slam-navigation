#!/usr/bin/env python3
"""
Navigation benchmark runner.

Runs multiple navigation trials with different configurations and produces
a comparison report with plots.

Usage:
  ros2 run amr_slam_navigation benchmark_navigation.py
  ros2 run amr_slam_navigation benchmark_navigation.py --ros-args \
      -p output_dir:=/tmp/nav_benchmarks \
      -p trials:=5
"""
from __future__ import annotations

import json
import math
import os
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def load_results(results_dir: Path) -> list[dict]:
    """Load all JSON benchmark results from a directory."""
    results = []
    for f in sorted(results_dir.glob('*.json')):
        with open(f) as fh:
            data = json.load(fh)
            data['_filename'] = f.name
            results.append(data)
    return results


def summarize_results(results: list[dict]) -> dict:
    """Aggregate metrics across multiple benchmark runs."""
    if not results:
        return {}

    all_waypoints = []
    for r in results:
        all_waypoints.extend(r.get('waypoints', []))

    if not all_waypoints:
        return {}

    succeeded = [w for w in all_waypoints
                 if w.get('nav2_result') == 'SUCCEEDED']
    efficiencies = [w['path_efficiency'] for w in succeeded
                    if 'path_efficiency' in w]
    loc_errors = [w['localization_rmse_m'] for w in succeeded
                  if 'localization_rmse_m' in w]
    times = [w['wall_time_s'] for w in succeeded if 'wall_time_s' in w]
    path_lens = [w['path_length_m'] for w in succeeded
                 if 'path_length_m' in w]

    return {
        'timestamp': datetime.now().isoformat(),
        'total_trials': len(results),
        'total_waypoints': len(all_waypoints),
        'succeeded': len(succeeded),
        'success_rate': round(len(succeeded) / max(len(all_waypoints), 1), 4),
        'path_efficiency': {
            'mean': round(float(np.mean(efficiencies)), 4) if efficiencies else 0,
            'std': round(float(np.std(efficiencies)), 4) if efficiencies else 0,
            'min': round(float(np.min(efficiencies)), 4) if efficiencies else 0,
            'max': round(float(np.max(efficiencies)), 4) if efficiencies else 0,
        },
        'localization_rmse_m': {
            'mean': round(float(np.mean(loc_errors)), 4) if loc_errors else 0,
            'std': round(float(np.std(loc_errors)), 4) if loc_errors else 0,
            'max': round(float(np.max(loc_errors)), 4) if loc_errors else 0,
        },
        'navigation_time_s': {
            'mean': round(float(np.mean(times)), 2) if times else 0,
            'total': round(float(np.sum(times)), 2) if times else 0,
        },
        'path_length_m': {
            'mean': round(float(np.mean(path_lens)), 3) if path_lens else 0,
            'total': round(float(np.sum(path_lens)), 3) if path_lens else 0,
        },
    }


def plot_benchmark(summary: dict, waypoint_data: list[dict],
                   output_dir: Path):
    """Generate benchmark visualization plots."""
    if not HAS_MPL or not waypoint_data:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Navigation Benchmark Results', fontsize=16, fontweight='bold')

    succeeded = [w for w in waypoint_data
                 if w.get('nav2_result') == 'SUCCEEDED']

    # 1. Path efficiency per waypoint
    ax = axes[0, 0]
    indices = range(len(succeeded))
    efficiencies = [w['path_efficiency'] for w in succeeded]
    colors = ['#2ecc71' if e > 0.7 else '#f39c12' if e > 0.4 else '#e74c3c'
              for e in efficiencies]
    ax.bar(indices, efficiencies, color=colors, alpha=0.8)
    ax.set_xlabel('Waypoint Index')
    ax.set_ylabel('Path Efficiency')
    ax.set_title('Path Efficiency per Waypoint')
    ax.set_ylim(0, 1.05)
    ax.axhline(y=float(np.mean(efficiencies)), color='#3498db',
               linestyle='--', label=f'Mean: {np.mean(efficiencies):.2%}')
    ax.legend()

    # 2. Localization RMSE
    ax = axes[0, 1]
    rmses = [w['localization_rmse_m'] for w in succeeded]
    ax.plot(range(len(rmses)), rmses, 'o-', color='#9b59b6',
            markersize=6, linewidth=2)
    ax.fill_between(range(len(rmses)), rmses, alpha=0.2, color='#9b59b6')
    ax.set_xlabel('Waypoint Index')
    ax.set_ylabel('Localization RMSE (m)')
    ax.set_title('Localization Accuracy')
    ax.axhline(y=float(np.mean(rmses)), color='#e74c3c', linestyle='--',
               label=f'Mean: {np.mean(rmses):.4f}m')
    ax.legend()

    # 3. Navigation time
    ax = axes[1, 0]
    times = [w['wall_time_s'] for w in succeeded]
    ax.bar(range(len(times)), times, color='#3498db', alpha=0.8)
    ax.set_xlabel('Waypoint Index')
    ax.set_ylabel('Time (s)')
    ax.set_title('Navigation Time per Waypoint')

    # 4. Path length vs optimal
    ax = axes[1, 1]
    actual = [w['path_length_m'] for w in succeeded]
    optimal = [w['optimal_distance_m'] for w in succeeded]
    x = np.arange(len(actual))
    width = 0.35
    ax.bar(x - width / 2, optimal, width, label='Optimal', color='#2ecc71',
           alpha=0.7)
    ax.bar(x + width / 2, actual, width, label='Actual', color='#e74c3c',
           alpha=0.7)
    ax.set_xlabel('Waypoint Index')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Actual vs Optimal Path Length')
    ax.legend()

    plt.tight_layout()
    plot_path = output_dir / 'benchmark_results.png'
    fig.savefig(str(plot_path), dpi=150, bbox_inches='tight')
    plt.close(fig)


def main():
    """Standalone summary from saved results (no ROS required)."""
    import argparse
    parser = argparse.ArgumentParser(
        description='Summarize navigation benchmark results')
    parser.add_argument('--results-dir', type=str, default='/tmp/nav_benchmarks',
                        help='Directory containing benchmark JSON files')
    parser.add_argument('--output-dir', type=str, default='',
                        help='Output directory for summary and plots')
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    output_dir = Path(args.output_dir) if args.output_dir else results_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    results = load_results(results_dir)
    if not results:
        print(f'No results found in {results_dir}')
        return

    summary = summarize_results(results)

    # Collect all waypoint data for plotting
    all_waypoints = []
    for r in results:
        all_waypoints.extend(r.get('waypoints', []))

    # Save summary
    summary_path = output_dir / 'benchmark_summary.json'
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'Summary saved to {summary_path}')

    # Generate plots
    plot_benchmark(summary, all_waypoints, output_dir)
    print(f'Plots saved to {output_dir}')

    # Print summary
    print(f'\n{"═" * 50}')
    print(f'Navigation Benchmark Summary')
    print(f'{"═" * 50}')
    print(f'Trials:          {summary["total_trials"]}')
    print(f'Waypoints:       {summary["total_waypoints"]}')
    print(f'Success rate:    {summary["success_rate"]:.1%}')
    print(f'Path efficiency: {summary["path_efficiency"]["mean"]:.1%} '
          f'± {summary["path_efficiency"]["std"]:.1%}')
    print(f'Loc RMSE:        {summary["localization_rmse_m"]["mean"]:.4f}m '
          f'(max {summary["localization_rmse_m"]["max"]:.4f}m)')
    print(f'Total distance:  {summary["path_length_m"]["total"]:.1f}m')
    print(f'Total time:      {summary["navigation_time_s"]["total"]:.1f}s')


if __name__ == '__main__':
    main()
