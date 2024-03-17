import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')
    config_path = os.path.join(get_package_share_directory('chassis'), 'params', 'lubancat_ubuntu_zero_w.json')

    return LaunchDescription([
        Node(namespace='/', package='chassis', executable='chassis_node', parameters=[chassis_cfg, {'driver_conf': config_path}], output='screen'),
    ])
