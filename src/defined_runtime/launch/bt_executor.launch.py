import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        bt_xml_path_arg,
        tick_rate_arg,
        enable_groot_arg,
        use_sim_time_arg,
        bt_executor_node,
    ])
