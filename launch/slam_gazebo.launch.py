"""
Launch Gazebo warehouse + SLAM Toolbox for online mapping.

Usage:
  ros2 launch amr_slam_navigation slam_gazebo.launch.py
  ros2 launch amr_slam_navigation slam_gazebo.launch.py gui:=false rviz:=true
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
            'Gazebo integration requires ros-jazzy-ros-gz. '
            'Install: sudo apt install ros-jazzy-ros-gz'
        ) from exc

    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world_path = share / 'worlds' / 'warehouse_nav.sdf'
    gz_args = f'-r {world_path}' + ('' if gui else ' -s')
    sim_params = {'use_sim_time': True}

    urdf_text = (share / 'urdf' / 'amr_nav.urdf').read_text()

    return [
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

        # ── Spawn robot in Gazebo ──
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

        # ── SLAM Toolbox (online async) ──
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                str(share / 'config' / 'slam_toolbox_params.yaml'),
                sim_params,
            ],
            output='screen',
        ),

        # ── RViz ──
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[sim_params],
            arguments=['-d', str(share / 'rviz' / 'slam.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ''),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz with SLAM display'),
        OpaqueFunction(function=launch_nodes),
    ])
