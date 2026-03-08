"""
robot_state_publisher.launch.py
Defined Robotics — defined_description package

MIDDLEWARE ADAPTER LAYER (ROS2 / Humble)
-----------------------------------------
This file is ROS2-specific by design.  It lives inside the defined_platform
workspace, which is the sole boundary between Defined Robotics middleware-
agnostic code (defined_rdf, defined_compiler, dashboard) and ROS2.

If the middleware changes (e.g. to Zenoh or a custom DDS profile), this file
and its siblings in defined_platform are the only things that need replacing.
Nothing outside defined_platform imports rclpy or any ROS2 type.

WHAT THIS FILE DOES
-------------------
Launches:
  - xacro  : processes defined_mvp.urdf.xacro → URDF XML string at launch time
  - robot_state_publisher : subscribes /joint_states, publishes /robot_description
                            and the full TF tree (base_footprint → base_link →
                            wheel links, caster)

This launch file is included by defined_gazebo/launch/gazebo.launch.py.
It can also be run standalone for URDF inspection:

  ros2 launch defined_description robot_state_publisher.launch.py

Launch arguments:
  use_sim_time : bool   (default true) — use Gazebo /clock
  xacro_file   : str    (default: installed defined_mvp.urdf.xacro)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_description = get_package_share_directory('defined_description')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(
            pkg_description, 'urdf', 'defined_mvp.urdf.xacro'
        ),
        description='Absolute path to the robot xacro file',
    )

    # ---------------------------------------------------------------------------
    # Process xacro → URDF string at launch time.
    # ParameterValue(value_type=str) is required with Command substitution;
    # without it robot_state_publisher receives bytes, not str.
    # ---------------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('xacro_file')]),
        value_type=str,
    )

    # ---------------------------------------------------------------------------
    # robot_state_publisher
    # ROS2 topics owned by this node:
    #   Subscribes : /joint_states  (sensor_msgs/JointState) — from Gazebo plugin
    #   Publishes  : /robot_description (std_msgs/String)
    #   Broadcasts : TF frames for all fixed + revolute joints
    # ---------------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        xacro_file_arg,
        robot_state_publisher_node,
    ])
