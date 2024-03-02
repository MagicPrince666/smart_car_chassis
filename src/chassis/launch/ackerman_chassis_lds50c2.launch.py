import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')
    lidar_cfg = os.path.join(get_package_share_directory('bluesea2'), 'params', 'LDS-50C-2.yaml')
    urdf_name = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'ackerman.urdf')

    return LaunchDescription([
        Node(namespace='/', package='chassis', executable='chassis_node', parameters=[chassis_cfg], output='screen'),
        Node(namespace='/', package='robot_state_publisher', executable='robot_state_publisher',  arguments=[urdf_name]),
        Node(namespace='/', package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui', arguments=[urdf_name]),
        Node(namespace='/', package='bluesea2',executable='bluesea_node', name='bluesea_node', output='screen', parameters=[lidar_cfg]),
    ])
