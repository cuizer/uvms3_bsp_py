from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='192.168.1.100',
        description='IP address of the local PC running the teleop sender',
    )

    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='5005',
        description='TCP port of the local PC teleop sender',
    )

    reconnect_base_delay_arg = DeclareLaunchArgument(
        'reconnect_base_delay',
        default_value='1.0',
        description='Initial reconnect delay in seconds',
    )

    reconnect_max_delay_arg = DeclareLaunchArgument(
        'reconnect_max_delay',
        default_value='30.0',
        description='Maximum reconnect delay in seconds',
    )

    teleop_receiver_node = Node(
        package='bsp',
        executable='teleop_receiver',
        name='teleop_receiver',
        output='screen',
        parameters=[{
            'server_host': LaunchConfiguration('server_host'),
            'server_port': LaunchConfiguration('server_port'),
            'reconnect_base_delay': LaunchConfiguration('reconnect_base_delay'),
            'reconnect_max_delay': LaunchConfiguration('reconnect_max_delay'),
        }],
    )

    return LaunchDescription([
        server_host_arg,
        server_port_arg,
        reconnect_base_delay_arg,
        reconnect_max_delay_arg,
        teleop_receiver_node,
    ])
