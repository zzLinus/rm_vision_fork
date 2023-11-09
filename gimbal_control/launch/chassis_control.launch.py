import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gimbal_control'), 'config', 'gimbal_control.yaml')

    chassis_control_node = Node(
        package='gimbal_control',
        executable='chassis_control_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([chassis_control_node])