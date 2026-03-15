"""
navigation_slam.launch.py
Defined Robotics — defined_navigation package

SLAM variant of the navigation stack. Replaces map_server + amcl with
slam_toolbox (online async mode). The robot builds a map from /scan data
while simultaneously localising — no pre-built PGM map required.

Nodes started:
  slam_toolbox     — online async SLAM (publishes /map and map→odom TF)
  controller_server, smoother_server, planner_server, behavior_server,
  bt_navigator, waypoint_follower, velocity_smoother — same as navigation.launch.py
  lifecycle_manager — activates all Nav2 nodes in order

Usage:
  ros2 launch defined_navigation navigation_slam.launch.py
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

    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
        description='Nav2 params yaml (for controller, planner, costmaps, etc.)',
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_navigation, 'config', 'slam_params.yaml'),
        description='slam_toolbox params yaml',
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
        description='ROS2 node namespace',
    )

    # -----------------------------------------------------------------------
    # RewrittenYaml for Nav2 params (controller, planner, costmaps)
    # -----------------------------------------------------------------------
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'autostart':    LaunchConfiguration('autostart'),
    }

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # -----------------------------------------------------------------------
    # Global use_sim_time
    # -----------------------------------------------------------------------
    set_sim_time = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time'),
    )

    # -----------------------------------------------------------------------
    # Lifecycle nodes — slam_toolbox replaces map_server + amcl
    # -----------------------------------------------------------------------
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # -----------------------------------------------------------------------
    # slam_toolbox — online async SLAM
    # In Jazzy, slam_toolbox is a lifecycle node — managed by lifecycle_manager.
    # Publishes /map and map→odom TF from /scan.
    # -----------------------------------------------------------------------
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # -----------------------------------------------------------------------
    # Nav2 nodes (same as navigation.launch.py, minus map_server and amcl)
    # -----------------------------------------------------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
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
        remappings=[
            ('cmd_vel',         'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # Separate lifecycle manager for slam_toolbox — bond disabled because
    # slam_toolbox in Jazzy doesn't support Nav2 bond heartbeats.
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    LaunchConfiguration('autostart'),
            'node_names':   ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

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
        params_file_arg,
        slam_params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        namespace_arg,
        set_sim_time,
        # SLAM (replaces map_server + amcl)
        slam_toolbox,
        lifecycle_manager_slam,
        # Nav2 nodes
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
