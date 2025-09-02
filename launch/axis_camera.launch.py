from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    ptz_arg = DeclareLaunchArgument(
        'ptz',
        default_value='true',
        description='Flag to enable PTZ control (true/false)'
    )

    stream_arg = DeclareLaunchArgument(
        'stream',
        default_value='true',
        description='Flag to enable image streaming (true/false)'
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='axis_m5525',
        description='Camera model (e.g., axis_m5525, axis_p5635)'
    )

    camera_number_arg = DeclareLaunchArgument(
        'camera_number',
        default_value='1',
        description='Camera number'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='axis_camera',
        description='Name of the camera nodes'
    )

    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.185',
        description='IP address of the camera'
    )

    username_arg = DeclareLaunchArgument(
        'username',
        default_value='root',
        description='Username for camera access'
    )

    password_arg = DeclareLaunchArgument(
        'password',
        default_value='R0b0tn1K',
        description='Password for camera access'
    )

    enable_auth_arg = DeclareLaunchArgument(
        'enable_auth',
        default_value='false',
        description='Enable authentication (true/false)'
    )

    # LaunchConfigurations to retrieve argument values
    ptz = LaunchConfiguration('ptz')
    stream = LaunchConfiguration('stream')
    camera_model = LaunchConfiguration('camera_model')
    camera_number = LaunchConfiguration('camera_number')
    node_name = LaunchConfiguration('node_name')
    ip_address = LaunchConfiguration('ip_address')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    enable_auth = LaunchConfiguration('enable_auth')

    # GroupAction with namespace and node
    load_nodes = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('axis_camera'), 'launch', 'axis_stream.launch.py')
                ),
                launch_arguments={
                    'ip_address': ip_address,
                    'username': username,
                    'password': password,
                    'enable_auth': enable_auth,
                    'node_name': node_name,
                    'camera_number': camera_number
                }.items(),
                condition=IfCondition(stream)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('axis_camera'), 'launch', 'axis_ptz.launch.py')
                ),
                launch_arguments={
                    'camera_model': camera_model,
                    'ip_address': ip_address,
                    'username': username,
                    'password': password,
                    'enable_auth': enable_auth,
                    'node_name': node_name,
                    'camera_number': camera_number
                }.items(),
                condition=IfCondition(ptz)
            ),
        ]
    )

    return LaunchDescription([
        ptz_arg,
        stream_arg,
        camera_model_arg,
        camera_number_arg,
        node_name_arg,
        ip_address_arg,
        username_arg,
        password_arg,
        enable_auth_arg,
        load_nodes
    ])