"""
bt_executor.launch.py
Defined Robotics — defined_runtime package

Launches the BT executor node which loads a BehaviorTree XML and ticks it.

Launch arguments:
  bt_xml_path  : path to BT XML file to load at startup (default: '' = wait for /task_command)
  tick_rate    : BT tick frequency in Hz (default: 100.0)
  enable_groot : enable Groot2 ZMQ publisher on port 1667 (default: true)
  use_sim_time : use simulation clock (default: true)

Usage:
  ros2 launch defined_runtime bt_executor.launch.py bt_xml_path:=/path/to/tree.xml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_runtime = get_package_share_directory('defined_runtime')

    bt_xml_path_arg = DeclareLaunchArgument(
        'bt_xml_path', default_value='',
        description='Path to BT XML file (empty = wait for /task_command)')

    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate', default_value='100.0',
        description='BT tick rate in Hz')

    enable_groot_arg = DeclareLaunchArgument(
        'enable_groot', default_value='true',
        description='Enable Groot2 ZMQ publisher on port 1667')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    run_explore_arg = DeclareLaunchArgument(
        'run_explore', default_value='true',
        description='Launch explore_lite for frontier exploration')

    bt_executor_node = Node(
        package='defined_runtime',
        executable='bt_executor',
        name='bt_executor',
        output='screen',
        parameters=[
            os.path.join(pkg_runtime, 'config', 'bt_executor_params.yaml'),
            {
                'bt_xml_path': LaunchConfiguration('bt_xml_path'),
                'tick_rate': LaunchConfiguration('tick_rate'),
                'enable_groot': LaunchConfiguration('enable_groot'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # explore_lite — frontier-based autonomous exploration (m-explore-ros2)
    # Runs as a standalone node alongside the BT executor.
    # The ExploreAction BT node controls it via /explore/resume.
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'robot_base_frame': 'base_link',
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('run_explore')),
    )

    return LaunchDescription([
        bt_xml_path_arg,
        tick_rate_arg,
        enable_groot_arg,
        use_sim_time_arg,
        run_explore_arg,
        bt_executor_node,
        explore_node,
    ])
