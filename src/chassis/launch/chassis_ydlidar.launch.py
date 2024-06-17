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

    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'mqquad_cfg.yaml')
    parameter_file=os.path.join(get_package_share_directory('ydlidar'), 'params', 'ydlidar.yaml')
    urdf_model_path = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'fishbot_base.urdf')

    lidar_node = Node(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
        )

    chassis_node = Node(
        namespace='/',
        package='chassis',
        executable='chassis_node',
        output='screen',
        parameters=[chassis_cfg],
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
