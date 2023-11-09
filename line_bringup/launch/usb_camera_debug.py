import os
import argparse
import sys

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

    # usb_cam_start

    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    args, unknown = parser.parse_known_args(sys.argv[4:])

    usb_cam_dir = get_package_share_directory('usb_cam')

    # get path to params file
    params_path = os.path.join(
        usb_cam_dir,
        'config',
        'params.yaml'
    )

    node_name = args.node_name
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name=node_name,
        # namespace=ns,
        parameters=[params_path]
    ))
    ld.add_action(Node(
        package='usb_cam', executable='show_image.py', output='screen',
        # namespace=ns,
        # arguments=[image_manip_dir + "/data/mosaic.jpg"])
        # remappings=[('image_in', 'image_raw')]
    ))

    #usb_cam_end

    line_detector_node = Node(
        package='line_detector',
        executable='line_detector_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[{"debug":True}],
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
    return LaunchDescription([
        line_detector_node,
        gimbal_control_node,
        serial_node,
        ld,
    ])