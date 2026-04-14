"""
gazebo.launch.py
Defined Robotics — defined_gazebo package

MIDDLEWARE ADAPTER LAYER (ROS2 / Gz Sim — Ionic)
-----------------------------------------------------
This file is ROS2-specific by design.  It lives inside the defined_platform
workspace — the sole location where Gz Sim and the ros_gz bridge are
referenced.  Nothing outside defined_platform touches Gz or any ROS2 type.

WHAT THIS FILE DOES
-------------------
Launches Gz Sim and spawns the defined_mvp robot.

Boot sequence:
  1. gz_sim  — physics engine + scene broadcaster, loads world SDF.
               Gz system plugins (Physics, SceneBroadcaster, Sensors,
               UserCommands) are declared in the world SDF file.
  2. robot_state_publisher.launch.py  — xacro → URDF + TF tree
  3. ros_gz_sim create  — spawns robot from /robot_description into Gz
  4. ros_gz_bridge  — bridges Gz Transport topics ↔ ROS2 topics:
       /cmd_vel, /odom, /scan, /clock, /joint_states

Launch arguments:
  world        : path to .sdf file  (default: room_10x10.sdf)
  use_gui      : launch Gz GUI      (default: true)
  use_sim_time : (default: true)
  x, y, z, yaw: robot spawn pose   (default: 0, 0, 0.05, 0)

Usage:
  ros2 launch defined_gazebo gazebo.launch.py
  ros2 launch defined_gazebo gazebo.launch.py use_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo      = get_package_share_directory('defined_gazebo')
    pkg_description = get_package_share_directory('defined_description')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'room_10x10.sdf'),
        description='Absolute path to the Gz Sim .sdf world file',
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gz GUI. Set false for headless/Docker.',
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

    # Robot capability args — forwarded from simulation.launch.py (ROBOT_* env vars)
    wheel_separation_arg  = DeclareLaunchArgument('wheel_separation',  default_value='0.287')
    wheel_radius_arg      = DeclareLaunchArgument('wheel_radius',      default_value='0.033')
    has_lidar_arg         = DeclareLaunchArgument('has_lidar',         default_value='true')
    lidar_range_min_arg   = DeclareLaunchArgument('lidar_range_min',   default_value='0.20')
    lidar_range_max_arg   = DeclareLaunchArgument('lidar_range_max',   default_value='3.5')
    lidar_samples_arg     = DeclareLaunchArgument('lidar_samples',     default_value='720')
    lidar_update_rate_arg = DeclareLaunchArgument('lidar_update_rate', default_value='20')
    has_camera_arg        = DeclareLaunchArgument('has_camera',        default_value='false')
    camera_width_arg      = DeclareLaunchArgument('camera_width',      default_value='640')
    camera_height_arg     = DeclareLaunchArgument('camera_height',     default_value='480')
    camera_fps_arg        = DeclareLaunchArgument('camera_fps',        default_value='30')

    # ---------------------------------------------------------------------------
    # Gz Sim — physics engine + scene broadcaster
    # Uses ros_gz_sim's gz_sim.launch.py which wraps the `gz sim` command.
    # -r = run immediately, -s = server only (no GUI) when use_gui is false.
    # ---------------------------------------------------------------------------
    gz_args = PythonExpression([
        "'-r -s ' + '", LaunchConfiguration('world'), "' if '",
        LaunchConfiguration('use_gui'), "' == 'false' else '-r ' + '",
        LaunchConfiguration('world'), "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # robot_state_publisher — URDF + TF
    # ---------------------------------------------------------------------------
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time':     LaunchConfiguration('use_sim_time'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'wheel_radius':     LaunchConfiguration('wheel_radius'),
            'has_lidar':        LaunchConfiguration('has_lidar'),
            'lidar_range_min':  LaunchConfiguration('lidar_range_min'),
            'lidar_range_max':  LaunchConfiguration('lidar_range_max'),
            'lidar_samples':    LaunchConfiguration('lidar_samples'),
            'lidar_update_rate': LaunchConfiguration('lidar_update_rate'),
            'has_camera':       LaunchConfiguration('has_camera'),
            'camera_width':     LaunchConfiguration('camera_width'),
            'camera_height':    LaunchConfiguration('camera_height'),
            'camera_fps':       LaunchConfiguration('camera_fps'),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # Spawn robot — uses ros_gz_sim's create node to spawn from /robot_description
    # ---------------------------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'defined_mvp',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    # ---------------------------------------------------------------------------
    # ros_gz_bridge — bridges Gz Transport ↔ ROS2 topics
    # ---------------------------------------------------------------------------
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', ['config_file:=', os.path.join(pkg_gazebo, 'config', 'gz_bridge.yaml')],
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Publish odom→base_footprint TF from /odom nav_msgs/Odometry topic.
    # We do NOT use Pose_V→TFMessage bridge because it produces empty
    # frame_id/child_frame_id (known ros_gz issue #172/#410).
    odom_tf_node = Node(
        package='defined_gazebo',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        world_arg,
        use_gui_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        # Robot capability args
        wheel_separation_arg,
        wheel_radius_arg,
        has_lidar_arg,
        lidar_range_min_arg,
        lidar_range_max_arg,
        lidar_samples_arg,
        lidar_update_rate_arg,
        has_camera_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        # Stages
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        odom_tf_node,
    ])
