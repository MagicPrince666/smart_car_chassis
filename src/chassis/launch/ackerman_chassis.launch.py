import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')

    ROS_DISTRO=''
    ROS_DISTRO = os.getenv('ROS_DISTRO')
    print("Current ROS2 Version: ",ROS_DISTRO)
    if ROS_DISTRO == 'humble' or ROS_DISTRO == 'galactic' or ROS_DISTRO == 'foxy' or ROS_DISTRO == 'iron':
        return LaunchDescription([
            Node(namespace='/', package='chassis', executable='chassis_node', parameters=[chassis_cfg], output='screen'),
        ])
    else:
        return LaunchDescription([
            Node(node_namespace='/', package='chassis', node_executable='chassis_node', parameters=[chassis_cfg], output='screen'),
        ])
