from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Declare arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value=EnvironmentVariable('ROBOT_ID', default_value='robot'),
        description='Robot ID'
    )

    desired_freq_arg = DeclareLaunchArgument(
        'desired_freq',
        default_value='50.0',
        description='Desired frequency'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value=EnvironmentVariable('ROBOT_PAD_DEV', default_value='/dev/ttyUSB_PAD'),
        description='Serial port'
    )

    link_watchdog_timeout_arg = DeclareLaunchArgument(
        'link_watchdog_timeout',
        default_value='3.0',
        description='Link watchdog timeout'
    )

    # LaunchConfigurations to retrieve argument values
    robot_id = LaunchConfiguration('robot_id')
    desired_freq = LaunchConfiguration('desired_freq')
    port = LaunchConfiguration('port')
    link_watchdog_timeout = LaunchConfiguration('link_watchdog_timeout')

    # GroupAction with namespace and node
    load_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace=robot_id),
            Node(
                package='itowa_safe_pad',
                executable='itowa_safe_pad_node',
                name='itowa_safe_pad',
                output='screen',
                parameters=[{
                    'port': port,
                    'desired_freq': desired_freq,
                    'link_watchdog_timeout': link_watchdog_timeout
                }]
            )
        ]
    )

    return LaunchDescription([
        robot_id_arg,
        desired_freq_arg,
        port_arg,
        link_watchdog_timeout_arg,
        load_nodes
    ])