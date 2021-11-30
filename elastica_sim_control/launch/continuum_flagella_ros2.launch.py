import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('elastica_sim_control'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'elastica_sim_control',
        name = 'elastica_pub_sub',
        executable = 'continuum_flagella_ros2',
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
