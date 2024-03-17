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
    config_path = os.path.join(get_package_share_directory('chassis'), 'params', 'mqquad_two_wheel_diff_model.json')
    parameter_file = os.path.join(get_package_share_directory('ydlidar'), 'params', 'ydlidar.yaml')
    urdf_model_path = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'fishbot_base.urdf')
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(get_package_share_directory('fishbot_cartographer'), 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_2d.lua')
    rviz_config_dir = os.path.join(get_package_share_directory('fishbot_cartographer'), 'config', "cartographer.rviz")

    lidar_node = LifecycleNode(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
        )

    chassis_node = Node(
        package='chassis',
        executable='chassis_node',
        output='screen',
        parameters=[chassis_cfg, {'driver_conf': config_path}],
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
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        chassis_node,
        lidar_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node
    ])
