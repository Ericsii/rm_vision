import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory(
        'auto_aim_bringup'), 'config/default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='detect_color',
                              default_value='1', description='0-Red 1-Blue'),
        DeclareLaunchArgument(name='debug',
                              default_value='true'),
        DeclareLaunchArgument(name='params_file',
                              default_value=default_params_file),

        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'debug': LaunchConfiguration('debug'),
                'detect_color': LaunchConfiguration('detect_color'),
            }],
        ),

        Node(
            package='armor_processor',
            executable='armor_processor_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'debug': LaunchConfiguration('debug'),
            }],
        ),
    ])
