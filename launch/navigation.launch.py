"""
Launch Nav2 autonomous navigation with a pre-built map.

Loads the map, starts AMCL for localization, and brings up the full Nav2
stack (planner, controller, behaviors, smoother, velocity smoother).

Usage:
  ros2 launch amr_slam_navigation navigation.launch.py map:=/path/to/map.yaml
  ros2 launch amr_slam_navigation navigation.launch.py map:=$(ros2 pkg prefix amr_slam_navigation)/share/amr_slam_navigation/maps/warehouse_default.yaml
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    share = Path(get_package_share_directory('amr_slam_navigation'))
    try:
        gz_share = Path(get_package_share_directory('ros_gz_sim'))
        get_package_share_directory('ros_gz_bridge')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            'Gazebo integration requires ros-jazzy-ros-gz.'
        ) from exc

    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world_path = share / 'worlds' / 'warehouse_nav.sdf'
    gz_args = f'-r {world_path}' + ('' if gui else ' -s')
    sim_params = {'use_sim_time': True}

    urdf_text = (share / 'urdf' / 'amr_nav.urdf').read_text()
    map_file = LaunchConfiguration('map').perform(context)
    nav2_params = str(share / 'config' / 'nav2_params.yaml')

    nodes = [
        # ── Gazebo simulation ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        ),

        # ── Robot state publisher ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[sim_params, {'robot_description': urdf_text}],
        ),

        # ── Spawn robot ──
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-world', 'warehouse_nav', '-name', 'amr',
                       '-topic', '/robot_description', '-z', '0.01'],
            output='screen',
        ),

        # ── Gazebo ↔ ROS bridge ──
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': str(share / 'config' / 'gazebo_bridge.yaml')}],
            output='screen',
        ),

        # ── Map server ──
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[sim_params, {'yaml_filename': map_file}],
            output='screen',
        ),

        # ── AMCL localization ──
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Lifecycle manager for localization ──
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[sim_params, {
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen',
        ),

        # ── Nav2 controller ──
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Nav2 planner ──
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Nav2 behaviors (recovery) ──
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Nav2 BT navigator ──
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Nav2 smoother ──
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Velocity smoother ──
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Waypoint follower ──
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            parameters=[nav2_params, sim_params],
            output='screen',
        ),

        # ── Lifecycle manager for Nav2 ──
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[sim_params, {
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
            output='screen',
        ),

        # ── RViz ──
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[sim_params],
            arguments=['-d', str(share / 'rviz' / 'navigation.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('amr_slam_navigation'))
    default_map = str(share / 'maps' / 'warehouse_default.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ''),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz with Nav2 display'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to the map YAML file'),
        OpaqueFunction(function=launch_nodes),
    ])
