"""
gazebo.launch.py
Defined Robotics — defined_gazebo package

MIDDLEWARE ADAPTER LAYER (ROS2 / Gazebo Classic 11)
-----------------------------------------------------
This file is ROS2-specific by design.  It lives inside the defined_platform
workspace — the sole location where Gazebo, gzserver, and the gazebo_ros
plugins are referenced.  Nothing outside defined_platform touches Gazebo
or any ROS2 type.

WHAT THIS FILE DOES
-------------------
Launches Gazebo Classic (gazebo11) and spawns the defined_mvp robot.

Boot sequence:
  1. gzserver  — physics engine, loads world file.
                 Loaded plugins:
                   libgazebo_ros_init.so    → publishes /clock (sim time)
                   libgazebo_ros_factory.so → provides /spawn_entity service
  2. gzclient  — GUI (optional; disabled when use_gui:=false for headless/Docker)
  3. robot_state_publisher.launch.py  — xacro → URDF + TF tree
  4. spawn_entity.py  — reads /robot_description, calls /spawn_entity,
                        then exits (expected behaviour)

ROS2 topics / services introduced here:
  /clock              (rosgraph_msgs/Clock)      — from libgazebo_ros_init
  /robot_description  (std_msgs/String)          — from robot_state_publisher
  /cmd_vel            (geometry_msgs/Twist)      — subscribed by diff_drive plugin
  /odom               (nav_msgs/Odometry)        — published by diff_drive plugin
  /joint_states       (sensor_msgs/JointState)   — published by joint_state plugin
  TF: odom → base_footprint                      — published by diff_drive plugin
  TF: base_footprint → base_link → wheel links   — published by robot_state_publisher

Launch arguments:
  world        : path to .world file  (default: room_10x10.world)
  use_gui      : launch gzclient      (default: true)
  use_sim_time : (default: true)
  x, y, z, yaw: robot spawn pose     (default: 0, 0, 0.05, 0)

Usage:
  ros2 launch defined_gazebo gazebo.launch.py
  ros2 launch defined_gazebo gazebo.launch.py use_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo      = get_package_share_directory('defined_gazebo')
    pkg_description = get_package_share_directory('defined_description')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'room_10x10.world'),
        description='Absolute path to the Gazebo .world file',
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch gzclient GUI. Set false for headless/Docker.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Pass use_sim_time=true to all nodes (required when using /clock)',
    )

    x_arg   = DeclareLaunchArgument('x',   default_value='0.0',  description='Spawn X (m)')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0',  description='Spawn Y (m)')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.05', description='Spawn Z (m) — slightly above ground to avoid collision on spawn')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0',  description='Spawn yaw (rad)')

    # ---------------------------------------------------------------------------
    # gzserver — physics engine
    # gazebo_ros/launch/gzserver.launch.py automatically loads:
    #   libgazebo_ros_init.so    (sim time → /clock)
    #   libgazebo_ros_factory.so (spawn_entity service)
    # ---------------------------------------------------------------------------
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'verbose': 'false',
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # gzclient — optional GUI
    # ---------------------------------------------------------------------------
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    # ---------------------------------------------------------------------------
    # robot_state_publisher — URDF + TF
    # ---------------------------------------------------------------------------
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # spawn_entity — waits for /spawn_entity service then spawns the robot.
    # Uses a retry loop because Gazebo Classic on Docker Desktop (no GPU/VM) can
    # take 2-5 minutes for the factory plugin to register the service.
    # -Y is the yaw argument (capital Y, not -yaw).
    # ---------------------------------------------------------------------------
    spawn_robot_delayed = ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'source /opt/ros/humble/setup.bash && '
                'source /ros2_ws/install/setup.bash && '
                'echo "[spawn] Waiting for /spawn_entity service..." && '
                'until ros2 service list 2>/dev/null | grep -q "^/spawn_entity$"; do sleep 5; done && '
                'echo "[spawn] Service found, spawning defined_mvp..." && '
                'ros2 run gazebo_ros spawn_entity.py '
                '  -entity defined_mvp '
                '  -topic /robot_description '
                '  -x 0.0 -y 0.0 -z 0.05 -Y 0.0'
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_gui_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot_delayed,
    ])
