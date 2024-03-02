import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')
    ldlidar_cfg = os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'params', 'ldlidar_ld06.yaml')
    urdf_name = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'ackerman.urdf')

    return LaunchDescription([
        Node(namespace='/', package='chassis', executable='chassis_node', parameters=[chassis_cfg], output='screen'),
        Node(namespace='/', package='robot_state_publisher', executable='robot_state_publisher',  arguments=[urdf_name]),
        Node(namespace='/', package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui', arguments=[urdf_name]),
        Node(namespace='/', package='ldlidar_stl_ros2',executable='ldlidar_stl_ros2_node', name='ldlaser', output='screen', parameters=[ldlidar_cfg]),
    ])
