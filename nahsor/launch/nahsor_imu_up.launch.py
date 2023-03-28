import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = 'infantry4'
    imu_node = Node(
        name='wit_imu_node',
        namespace=robot_name,
        package='wit_imu_driver',
        executable='wit_imu_node',
        parameters=[{'device':'/dev/ttyIMU',
                     'baud':460800}],
        output='screen',
        respawn=True,
        respawn_delay=1
    )
    
    ld = LaunchDescription()
    ld.add_action(imu_node)
    return ld