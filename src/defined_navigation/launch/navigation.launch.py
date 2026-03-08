"""
navigation.launch.py
Defined Robotics — defined_navigation package

MIDDLEWARE ADAPTER LAYER (ROS2 / Nav2 Humble)
----------------------------------------------
This file is ROS2-specific by design.  It lives inside the defined_platform
workspace — the sole location where Nav2, costmaps, and navigation action
servers are referenced.  Nothing outside defined_platform imports nav2_msgs,
geometry_msgs, or any other ROS2 navigation type.

The navigate primitive exposed to user verbs is:
  go_to(x, y, yaw)  →  NavigateToPose action  →  cmd_vel on /cmd_vel

If the navigation middleware changes (e.g. to a custom planner or a Zenoh-
based variant), only this package and its params need replacing.

WHAT THIS FILE DOES
-------------------
Launches the full Nav2 stack for localisation and path following.

Nodes started:
  map_server         — serves /map from room_10x10.pgm + room_10x10.yaml
  amcl               — particle-filter localisation (map → odom → base_footprint)
  controller_server  — DWB local planner → publishes /cmd_vel
  smoother_server    — path smoother (SimpleSmoother)
  planner_server     — NavFn global planner (Dijkstra)
  behavior_server    — recovery behaviours: Spin, BackUp, Wait
  bt_navigator       — NavigateToPose / NavigateThroughPoses action servers
  waypoint_follower  — sequential waypoint list execution
  velocity_smoother  — smooths controller /cmd_vel output
  lifecycle_manager  — configures + activates all of the above in order

ROS2 topics / actions owned here:
  /map               (nav_msgs/OccupancyGrid)           — from map_server
  /amcl_pose         (geometry_msgs/PoseWithCovarianceStamped) — from amcl
  /cmd_vel           (geometry_msgs/Twist)              — from velocity_smoother
  /navigate_to_pose  (nav2_msgs/NavigateToPose action)  — from bt_navigator
  /plan              (nav_msgs/Path)                    — from planner_server
  TF: map → odom                                        — from amcl (using /scan)

Launch arguments:
  map          : path to map yaml  (default: room_10x10.yaml)
  params_file  : Nav2 params yaml  (default: nav2_params.yaml)
  use_sim_time : bool              (default: true)
  autostart    : bool              (default: true)
  namespace    : string            (default: empty)

Usage:
  ros2 launch defined_navigation navigation.launch.py
  ros2 launch defined_navigation navigation.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_navigation = get_package_share_directory('defined_navigation')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_navigation, 'maps', 'maze_10x10.yaml'),
        description='Absolute path to the map yaml file',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
        description='Absolute path to the Nav2 params yaml file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-activate Nav2 lifecycle nodes on startup',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS2 node namespace (empty = global)',
    )

    # ---------------------------------------------------------------------------
    # RewrittenYaml — injects the resolved map path and sim time flag into the
    # params file at launch time, so the YAML itself does not hard-code paths.
    # From nav2_common (ros-humble-nav2-common package).
    # ---------------------------------------------------------------------------
    param_substitutions = {
        'use_sim_time':  LaunchConfiguration('use_sim_time'),
        'yaml_filename': LaunchConfiguration('map'),
        'autostart':     LaunchConfiguration('autostart'),
    }

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # ---------------------------------------------------------------------------
    # Global use_sim_time — applied to all nodes in this launch context.
    # ---------------------------------------------------------------------------
    set_sim_time = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time'),
    )

    # ---------------------------------------------------------------------------
    # Lifecycle node list — must match lifecycle_manager node_names parameter
    # in nav2_params.yaml exactly (order matters: map before localiser,
    # localiser before planners).
    # ---------------------------------------------------------------------------
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # ---------------------------------------------------------------------------
    # Nav2 nodes — launched individually for explicit per-node control.
    # All receive `configured_params` so runtime substitutions are applied.
    # ---------------------------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        # Explicit remapping documents the topic contract clearly.
        # cmd_vel here is the raw controller output; velocity_smoother
        # subscribes to cmd_vel_nav and re-publishes as cmd_vel.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        # Subscribes to raw controller output; publishes smoothed /cmd_vel.
        remappings=[
            ('cmd_vel',         'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # ---------------------------------------------------------------------------
    # lifecycle_manager — configures then activates each node in order.
    # node_names and autostart are passed directly (not via configured_params)
    # so the lifecycle manager's own startup is not gated on YAML injection.
    # ---------------------------------------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    LaunchConfiguration('autostart'),
            'node_names':   lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        # Arguments
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        namespace_arg,
        # Global param
        set_sim_time,
        # Nav2 nodes (AMCL provides map→odom TF using LiDAR /scan data)
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
