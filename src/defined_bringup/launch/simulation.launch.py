"""
simulation.launch.py
Defined Robotics — defined_bringup package

MIDDLEWARE ADAPTER LAYER (ROS2 / Jazzy)
-----------------------------------------
This file is ROS2-specific by design.  It is the single entry point that
composes the full simulation stack.  Everything outside defined_platform
(defined_rdf, defined_compiler, dashboard) is middleware-agnostic and must
never import from here directly.

The adapter boundary is defined in:
  defined_bringup/defined_bringup/platform_interface.py
  (RobotCommand, RobotState, NavigationGoal — plain Python dataclasses,
   no ROS2 types)

HOW THIS FILE FITS THE MIDDLEWARE ISOLATION DESIGN
--------------------------------------------------
  defined_rdf        — no ROS2 dependency, describes robot capabilities
  defined_compiler   — no ROS2 dependency, compiles YAML → BT XML
  dashboard          — no ROS2 dependency, speaks platform_interface types
                         ↕ WebSocket (rosbridge) ↕
  defined_platform   ← this workspace (ROS2 / Nav2 / Gazebo adapter)
    defined_bringup/platform_interface.py  ← boundary types
    defined_bringup/launch/simulation.launch.py  ← YOU ARE HERE
    defined_gazebo    ← Gz Sim adapter
    defined_navigation ← Nav2 adapter
    defined_description ← URDF/xacro (impl detail of ROS2 layer)
    defined_runtime   ← BT executor (ROS2 action client)

If the middleware changes to Zenoh or a custom DDS, only defined_platform is
replaced.  defined_rdf, defined_compiler, and dashboard are unchanged.

WHAT THIS FILE DOES
-------------------
Boots the full simulation in three ordered stages:

  Stage 1 (t=0s):  Gz Sim
    gz sim + robot_state_publisher + spawn + ros_gz_bridge
    → defined_gazebo/launch/gazebo.launch.py

  Stage 2 (t=5s):  Nav2 stack
    map_server → amcl → controller → planner → bt_navigator → lifecycle_manager
    → defined_navigation/launch/navigation.launch.py
    Delayed to ensure /odom and /robot_description are live before Nav2 starts.

  Stage 3 (t=2s):  rosbridge WebSocket
    Lightweight bridge for the dashboard.  Delayed 2s only.
    ws://localhost:9090

Launch arguments (all optional — defaults produce a runnable sim):
  world        : Gz Sim world file       (default: maze_10x10.sdf)
  map          : Nav2 map yaml           (default: maze_10x10.yaml)
  params_file  : Nav2 params yaml        (default: nav2_params.yaml)
  use_gui      : launch Gz GUI           (default: true; false for headless)
  use_sim_time : use /clock              (default: true)
  autostart    : auto-activate Nav2      (default: true)
  x, y, z, yaw: robot spawn pose

Usage (host, ROS2 Jazzy sourced):
  ros2 launch defined_bringup simulation.launch.py

Usage (headless, Docker):
  ros2 launch defined_bringup simulation.launch.py use_gui:=false

Usage (custom map):
  ros2 launch defined_bringup simulation.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo     = get_package_share_directory('defined_gazebo')
    pkg_navigation = get_package_share_directory('defined_navigation')
    pkg_rosbridge  = get_package_share_directory('rosbridge_server')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'maze_10x10.sdf'),
        description='Gz Sim world file',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_navigation, 'maps', 'maze_10x10.yaml'),
        description='Nav2 map yaml file',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
        description='Nav2 params yaml file',
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gz Sim GUI (set false for headless/Docker)',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gz Sim /clock for simulation time',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-activate Nav2 lifecycle nodes',
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Use SLAM (slam_toolbox) instead of static map + AMCL',
    )

    x_arg   = DeclareLaunchArgument('x',   default_value='0.0',  description='Robot spawn X (m)')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0',  description='Robot spawn Y (m)')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.05', description='Robot spawn Z (m)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0',  description='Robot spawn yaw (rad)')

    # ---------------------------------------------------------------------------
    # Stage 1 — Gz Sim (t = 0 s)
    # Brings up Gz Sim, robot_state_publisher, spawns the robot, and bridges topics.
    # All ROS2 Gz-specific code lives in defined_gazebo.
    # ---------------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':        LaunchConfiguration('world'),
            'use_gui':      LaunchConfiguration('use_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x':            LaunchConfiguration('x'),
            'y':            LaunchConfiguration('y'),
            'z':            LaunchConfiguration('z'),
            'yaw':          LaunchConfiguration('yaw'),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # Stage 2 — Nav2 stack (t = 5 s)
    # Delayed slightly so Gz Sim has started. Nav2 will retry TF until the robot
    # is spawned.
    #
    # use_slam:=false (default) → static map + AMCL localisation
    # use_slam:=true            → slam_toolbox online async (no map needed)
    # ---------------------------------------------------------------------------
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('defined_navigation'),
                        'launch',
                        'navigation.launch.py',
                    )
                ),
                launch_arguments={
                    'map':          LaunchConfiguration('map'),
                    'params_file':  LaunchConfiguration('params_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart':    LaunchConfiguration('autostart'),
                }.items(),
                condition=UnlessCondition(LaunchConfiguration('use_slam')),
            )
        ],
    )

    navigation_slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('defined_navigation'),
                        'launch',
                        'navigation_slam.launch.py',
                    )
                ),
                launch_arguments={
                    'params_file':  LaunchConfiguration('params_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart':    LaunchConfiguration('autostart'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_slam')),
            )
        ],
    )

    # ---------------------------------------------------------------------------
    # Stage 3 — rosbridge WebSocket (t = 2 s)
    # Provides the WebSocket bridge consumed by the dashboard.
    # ws://localhost:9090  (port forwarded in docker-compose.yml)
    # This is the ONLY ROS2 entry point the dashboard uses — it speaks
    # rosbridge JSON, not rclpy.  The dashboard itself is middleware-agnostic.
    # ---------------------------------------------------------------------------
    rosbridge_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        pkg_rosbridge,
                        'launch',
                        'rosbridge_websocket_launch.xml',
                    )
                ),
            )
        ],
    )


    # ---------------------------------------------------------------------------
    # Stage 4 — foxglove_bridge WebSocket (t = 3 s)
    # Provides the Foxglove Studio WebSocket bridge on port 8765.
    # Connect Foxglove Studio to ws://localhost:8765
    # ---------------------------------------------------------------------------
    foxglove_launch = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[{
                    'port': 8765,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            )
        ],
    )

    # ---------------------------------------------------------------------------
    # Stage 5 — BT Executor (t = 15 s, optional)
    # Only launched when run_task:=/path/to/task.xml is provided.
    # Delayed to ensure Nav2 lifecycle nodes are active.
    # ---------------------------------------------------------------------------
    run_bt_arg = DeclareLaunchArgument(
        'run_bt',
        default_value='false',
        description='Launch BT executor (set true to enable)',
    )
    run_task_arg = DeclareLaunchArgument(
        'run_task',
        default_value='',
        description='Path to BT XML file to auto-execute (empty = wait for /task_command)',
    )

    bt_executor_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('defined_runtime'),
                        'launch',
                        'bt_executor.launch.py',
                    )
                ),
                launch_arguments={
                    'bt_xml_path':   LaunchConfiguration('run_task'),
                    'use_sim_time':  LaunchConfiguration('use_sim_time'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('run_bt')),
            )
        ],
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        map_arg,
        params_file_arg,
        use_gui_arg,
        use_sim_time_arg,
        use_slam_arg,
        autostart_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        run_bt_arg,
        run_task_arg,
        # Stages
        gazebo_launch,
        navigation_launch,
        navigation_slam_launch,
        rosbridge_launch,
        foxglove_launch,
        bt_executor_launch,
    ])
