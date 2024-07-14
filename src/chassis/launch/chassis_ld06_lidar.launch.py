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
    # 获取底盘参数文件，搜索底盘节点包（chassis）的params目录下base_cfg.yaml配置文件
    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')
    # 获取雷达参数文件，搜索雷达节点包（ldlidar_stl_ros2）的params目录下ldlidar_ld06.yaml配置文件
    parameter_file=os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'params', 'ldlidar_ld06.yaml')
    # 获取TF参数文件，搜索TF节点包（fishbot_description）的urdf目录下ackerman.urdf配置文件
    urdf_model_path = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'ackerman.urdf')

    lidar_node = Node( # 雷达节点
        package='ldlidar_stl_ros2', # 包名，参考源码目录下 package.xml 文件
        executable='ldlidar_stl_ros2_node', # 可执行文件，参考cmake编译出来的可执行文件名称
        name='ldlaser', # 重命名节点名称，非必须
        output='screen', # 输出调试信息到控制台
        emulate_tty=True,
        parameters=[parameter_file], # 配置参数，此处传入yaml格式配置文件，参考配置文件的获取
        namespace='/', # 命名空间，非必须
        ) # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html

    chassis_node = Node( # 底盘节点
        namespace='/', # 命名空间，非必须
        package='chassis', # 包名，参考源码目录下 package.xml 文件
        executable='chassis_node', # 可执行文件，参考cmake编译出来的可执行文件名称
        output='screen', # 输出调试信息到控制台
        parameters=[chassis_cfg], # 配置参数，此处传入yaml格式配置文件，参考配置文件的获取
        ) # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html
    
    robot_state_publisher_node = Node( # urdf发布，即机器人各关节的位姿发布
        package='robot_state_publisher', # 包名，robot_state_publisher安装包 ，ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-robot-state-publisher
        executable='robot_state_publisher', # 可执行文件，参考robot_state_publisher包可执行文件名称
        arguments=[urdf_model_path] # 导入参数，此处传入yaml格式参数文件，参考配置文件的获取
        )

    joint_state_publisher_node = Node( # urdf发布，即机器人各关节的位姿发布
        package='joint_state_publisher_gui',  # 包名，joint-state-publisher-gui安装包 ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui
        executable='joint_state_publisher_gui', # 可执行文件，参考joint-state-publisher-gui包可执行文件名称
        name='joint_state_publisher_gui', # 重命名节点名称，非必须
        arguments=[urdf_model_path] # 导入参数，此处传入yaml格式参数文件，参考配置文件的获取
        )

    return LaunchDescription([
        chassis_node, # 运行底盘节点
        lidar_node, # 运行雷达节点
        robot_state_publisher_node, # 运行TF节点
        joint_state_publisher_node, # 运行可视化TF节点
    ])
