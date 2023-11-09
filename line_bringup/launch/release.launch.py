import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory('line_bringup'), 'config', 'bringup.yaml')
    params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')
    camera_info_url = 'package://hik_camera/config/camera_info.yaml'

    line_detector_node = Node(
        package='line_detector',
        executable='line_detector_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[{"debug":False}],
    )
    gimbal_control_node = Node(
        package='gimbal_control',
        executable='gimbal_control_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[node_params],
    )
    serial_node = Node(
        package='serial',
        executable='serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[node_params],
    )
    camera_node = Node(
        package='hik_camera',
        executable='hik_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file'), {
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),
        line_detector_node,
        gimbal_control_node,
        serial_node,
        camera_node,
    ])