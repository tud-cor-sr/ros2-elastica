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
    
    config_kin = os.path.join(
        get_package_share_directory('elastica_kinematics'),
        'config',
        'params.yaml'
        )

        
    node_sim=Node(
        package = 'elastica_sim',
        name = 'elastica_pub_sub',
        executable = 'continuum_flagella_ros2',
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
    
    node_rods_state_to_pcc =Node(
        package = 'elastica_kinematics',
        name = 'rods_state_to_pcc',
        executable = 'rods_state_to_pcc',
        output='screen',
        parameters = [config_kin]
    )
    
    node_forward_kinematics =Node(
        package = 'elastica_kinematics',
        name = 'forward_kinematics',
        executable = 'forward_kinematics',
        output='screen',
        parameters = [config_kin]
    )
    
    ld.add_action(node_sim)
    ld.add_action(node_rand_control)
    ld.add_action(node_rods_state_to_pcc)
    ld.add_action(node_forward_kinematics)
    return ld
