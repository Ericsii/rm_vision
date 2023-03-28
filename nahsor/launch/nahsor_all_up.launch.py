import imp
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    param_path = os.path.join(get_package_share_directory(
        "rm_nahsor"), "config/all_param.yaml")
    
    robot_name = 'infantry4'
    with open(param_path, 'r') as f:
        mvcam_params = yaml.safe_load(f)['mindvision_camera']['ros__parameters']
        print(mvcam_params)
    with open(param_path, 'r') as f:
        base_params = yaml.safe_load(f)['base']['ros__parameters']
    with open(param_path, 'r') as f:
        nahsor_params = yaml.safe_load(f)['nahsor_node']['ros__parameters']
    

    # 创建容器
    rm_container = Node(
        name='rm_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )   
    

    load_compose = LoadComposableNodes(
        target_container='rm_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rm_entity_cam',
                plugin='rm_cam::MindVisionCamNode',
                name='mindvision_camera',
                namespace=robot_name,
                parameters=[mvcam_params],
            ),
            ComposableNode(
                package='rm_base',
                plugin='rm_base::RobotBaseNode',
                name='robot_base',
                namespace=robot_name,
                parameters=[base_params],
            ),
            ComposableNode(
                package="rm_nahsor",
                plugin='nahsor::NahsorNode',
                name='nahsor',
                namespace=robot_name,
                parameters=[nahsor_params],
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(load_compose)
    ld.add_action(rm_container)
    return ld
