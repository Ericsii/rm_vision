import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("rm_nahsor"), "config/nahsor_param.yaml")
    with open(param_path, 'r') as f:
        nahsor_params = yaml.safe_load(f)['nahsor']['ros__parameters']
    nahsor_node = Node(
        namespace="infantry4",
        package="rm_nahsor",
        executable="nahsor",
        name="nahsor",
        parameters=[nahsor_params]
    )
    ld = LaunchDescription()
    ld.add_action(nahsor_node)

    return ld