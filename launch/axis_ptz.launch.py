from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
import os

def generate_launch_description():
    args = []
    desired_freq_arg = DeclareLaunchArgument(
        'desired_freq',
        default_value='20.0'
    )
    args.append(desired_freq_arg)
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='axis_camera'
    )
    args.append(node_name_arg)
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.185'
    )
    args.append(ip_address_arg)
    camera_number_arg = DeclareLaunchArgument(
        'camera_number',
        default_value='1'
    )
    args.append(camera_number_arg)
    connection_timeout_arg = DeclareLaunchArgument(
        'connection_timeout',
        default_value='5.0'
    )
    args.append(connection_timeout_arg)
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='axis_m5525'
    )
    args.append(camera_model_arg)
    camera_not_moving_timeout_arg = DeclareLaunchArgument(
        'camera_not_moving_timeout',
        default_value='3.0'
    )
    args.append(camera_not_moving_timeout_arg)
    last_position_command_timeout_arg = DeclareLaunchArgument(
        'last_position_command_timeout',
        default_value='10.0'
    )
    args.append(last_position_command_timeout_arg)
    last_velocity_command_timeout_arg = DeclareLaunchArgument(
        'last_velocity_command_timeout',
        default_value='0.5'
    )
    args.append(last_velocity_command_timeout_arg)
    reject_new_goal_arg = DeclareLaunchArgument(
        'reject_new_goal',
        default_value='false'
    )
    args.append(reject_new_goal_arg)
    
    desired_freq = LaunchConfiguration('desired_freq')
    node_name = LaunchConfiguration('node_name')
    ip_address = LaunchConfiguration('ip_address')
    camera_number = LaunchConfiguration('camera_number')
    connection_timeout = LaunchConfiguration('connection_timeout')
    camera_model = LaunchConfiguration('camera_model')
    camera_not_moving_timeout = LaunchConfiguration('camera_not_moving_timeout')
    last_position_command_timeout = LaunchConfiguration('last_position_command_timeout')
    last_velocity_command_timeout = LaunchConfiguration('last_velocity_command_timeout')
    reject_new_goal = LaunchConfiguration('reject_new_goal')

    param_file = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare('axis_camera'),
            'config',
            PythonExpression(["'", camera_model, "' + '.yaml'"])
        ]),
        allow_substs=True
    )

    load_nodes = GroupAction([
        Node(
            package='axis_camera',
            executable='axis_ptz_node',
            output='screen',
            name=PythonExpression(["'", node_name, "' + '_ptz'"]),
            parameters=[{
                'desired_freq': desired_freq,
                'hostname': ip_address,
                'camera_number': camera_number,
                'connection_timeout': connection_timeout,
                'camera_not_moving_timeout': camera_not_moving_timeout,
                'last_position_command_timeout': last_position_command_timeout,
                'last_velocity_command_timeout': last_velocity_command_timeout,
                'reject_new_goal' : reject_new_goal
            },
            param_file],
        )
    ])

    return LaunchDescription(args + [load_nodes])