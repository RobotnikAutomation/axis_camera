from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='axis_camera'
    )
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.185'
    )
    username_arg = DeclareLaunchArgument(
        'username',
        default_value='root'
    )
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='pass'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera'
    )
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value=''
    )
    run_republisher_arg = DeclareLaunchArgument(
        'run_republisher',
        default_value='true'
    )
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='camera'
    )
    camera_number_arg = DeclareLaunchArgument(
        'camera_number',
        default_value='0'
    )
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30'
    )
    compression_arg = DeclareLaunchArgument(
        'compression',
        default_value='0'
    )
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='high'
    )
    enable_auth_arg = DeclareLaunchArgument(
        'enable_auth',
        default_value='false'
    )
    initialization_delay_arg = DeclareLaunchArgument(
        'initialization_delay',
        default_value='0.0'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='5.0'
    )
    videocodec_arg = DeclareLaunchArgument(
        'videocodec',
        default_value='mjpeg'
    )
    reconection_time_arg = DeclareLaunchArgument(
        'reconection_time',
        default_value='5.0'
    )
    desired_freq_arg = DeclareLaunchArgument(
        'desired_freq',
        default_value='20.0'
    )
 
    node_name = LaunchConfiguration('node_name')
    ip_address = LaunchConfiguration('ip_address')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    frame_id = LaunchConfiguration('frame_id')
    camera_info_url = LaunchConfiguration('camera_info_url')
    run_republisher = LaunchConfiguration('run_republisher')
    camera_id = LaunchConfiguration('camera_id')
    camera_number = LaunchConfiguration('camera_number')
    fps = LaunchConfiguration('fps')
    compression = LaunchConfiguration('compression')
    profile = LaunchConfiguration('profile')
    enable_auth = LaunchConfiguration('enable_auth')
    initialization_delay = LaunchConfiguration('initialization_delay')
    timeout = LaunchConfiguration('timeout')
    videocodec = LaunchConfiguration('videocodec')
    reconection_time = LaunchConfiguration('reconection_time')
    desired_freq = LaunchConfiguration('desired_freq')

    load_nodes = GroupAction([
        Node(
            package='axis_camera',
            executable='axis_stream_node',
            output='screen',
            name=PythonExpression(["'", node_name, "' + '_stream'"]),
            parameters=[{
                'hostname': ip_address,
                'username': username,
                'password': password,
                'axis_frame_id': frame_id,
                'camera_info_url': camera_info_url,
                'camera_id': camera_id,
                'camera_number': camera_number,
                'fps': fps,
                'compression': compression,
                'profile': profile,
                'enable_auth': enable_auth,
                'initialization_delay': initialization_delay,
                'timeout': timeout,
                'videocodec': videocodec,
                'reconnection_time' : reconection_time,
                'desired_freq': desired_freq,
            }],
        )
    ])

    return LaunchDescription([
        node_name_arg,
        ip_address_arg,
        username_arg,
        password_arg,
        frame_id_arg,
        camera_info_url_arg,
        run_republisher_arg,
        camera_id_arg,
        camera_number_arg,
        fps_arg,
        compression_arg,
        profile_arg,
        enable_auth_arg,
        initialization_delay_arg,
        timeout_arg,
        videocodec_arg,
        reconection_time_arg,
        desired_freq_arg,
        load_nodes])