import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo

import lifecycle_msgs.msg

def generate_launch_description():

    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')
    urdf_model_path = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'ackerman.urdf')
    ldlidar_cfg = os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'params', 'ldlidar_ld19.yaml')

    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='laser_ld19',
        output='screen',
        parameters=[ldlidar_cfg]
        )

    chassis_node = Node(
        package='chassis',
        executable='chassis_node',
        name='chassis_node',
        output='screen',
        parameters=[chassis_cfg],
        namespace='/',
        )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
        )

    return LaunchDescription([
        chassis_node,
        lidar_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
