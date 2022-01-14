import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config_sim = os.path.join(
        get_package_share_directory('elastica_sim'),
        'config',
        'params.yaml'
        )
    config_ctrl = os.path.join(
        get_package_share_directory('elastica_controllers'),
        'config',
        'params.yaml'
        )
        
    node_sim=Node(
        package = 'elastica_sim',
        name = 'elastica_pub_sub',
        executable = 'continuum_snake_ros2',
        output='screen',
        parameters = [config_sim]
    )
    
    node_rand_control=Node(
        package = 'elastica_controllers',
        name = 'elastica_random_control',
        executable = 'elastica_random_controller',
        output='screen',
        parameters = [config_ctrl]
    )
    ld.add_action(node_sim)
    ld.add_action(node_rand_control)
    return ld
