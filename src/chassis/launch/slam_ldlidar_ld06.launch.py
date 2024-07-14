import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo

import lifecycle_msgs.msg

# 这是一键建图脚本，给小白中的小白中又不愿学习和思考的人用，必须务必一定要在小车上运行
def generate_launch_description():
    # 获取底盘参数文件，搜索底盘节点包（chassis）的params目录下base_cfg.yaml配置文件
    chassis_cfg = os.path.join(get_package_share_directory('chassis'), 'params', 'base_cfg.yaml')
    # 获取雷达参数文件，搜索雷达节点包（ldlidar_stl_ros2）的params目录下ldlidar_ld06.yaml配置文件
    ldlidar_cfg = os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'params', 'ldlidar_ld06.yaml')
    # 获取TF参数文件，搜索TF节点包（fishbot_description）的urdf目录下fishbot_base.urdf配置文件
    urdf_model_path = os.path.join(get_package_share_directory('fishbot_description'), 'urdf', 'ackerman.urdf')
    # 是否使用仿真时间，我们用真实机器，这里设置成false
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率,0.05米
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期 1Hz,即1秒发布一次地图
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default=os.path.join(get_package_share_directory('fishbot_cartographer'), 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_2d.lua')
    # 启动可视化界面的配置文件位置
    rviz_config_dir = os.path.join(get_package_share_directory('fishbot_cartographer'), 'config', "cartographer.rviz")
    ROS_DISTRO=''
    ROS_DISTRO = os.getenv('ROS_DISTRO') # 获取当前ros版本
    print("Current ROS2 Version: ",ROS_DISTRO) # 打印当前ros版本

    lidar_node = Node( # 雷达节点
        namespace='/', # 命名空间，非必须
        package='ldlidar_stl_ros2', # 包名，参考源码目录下 package.xml 文件
        executable='ldlidar_stl_ros2_node', # 可执行文件，参考cmake编译出来的可执行文件名称
        name='ldlaser', # 重命名节点名称，非必须
        output='screen', # 输出调试信息到控制台
        parameters=[ldlidar_cfg] # 配置参数，此处传入yaml格式配置文件，参考配置文件的获取
        ) # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html

    chassis_node = Node( # 底盘节点
        package='chassis', # 包名，参考源码目录下 package.xml 文件
        executable='chassis_node', # 可执行文件，参考cmake编译出来的可执行文件名称
        output='screen', # 输出调试信息到控制台
        parameters=[chassis_cfg], # 配置参数，此处传入yaml格式配置文件，参考配置文件的获取
        namespace='/', # 命名空间，非必须
        ) # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html
    
    robot_state_publisher_node = Node( # urdf发布，即机器人各关节的位姿发布
        package='robot_state_publisher', # 包名，robot_state_publisher安装包 ，ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-robot-state-publisher
        executable='robot_state_publisher',  # 可执行文件，参考robot_state_publisher包可执行文件名称
        arguments=[urdf_model_path] # 启动参数，导入配置文件
        ) # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html

    joint_state_publisher_node = Node( # urdf发布，即机器人各关节的位姿发布
        package='joint_state_publisher_gui', # 包名，joint-state-publisher-gui安装包 ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui
        executable='joint_state_publisher_gui',  # 可执行文件，参考joint_state_publisher_gui包可执行文件名称
        name='joint_state_publisher_gui', # 重命名节点名称，非必须
        arguments=[urdf_model_path] # 启动参数，导入配置文件
        ) # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html
    
    cartographer_node = Node( # 建图节点
        package='cartographer_ros', # 包名，cartographer安装包 ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-cartographer
        executable='cartographer_node',  # 可执行文件，参考cartographer_ros包可执行文件名称
        name='cartographer_node', # 重命名节点名称，非必须
        output='screen', # 输出调试信息到控制台
        parameters=[{'use_sim_time': use_sim_time}], # 配置参数，此处传入yaml格式配置文件，参考配置文件的获取
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename]) # 启动参数，导入配置文件
        # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html

    if ROS_DISTRO == 'humble' or ROS_DISTRO == 'iron': # 判断当前ros版本，如果是 humble 或者 iron则执行以下操作 
        cartographer_occupancy_grid_node = Node( # 建图节点
            package='cartographer_ros', # 包名，cartographer安装包 ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-cartographer
            executable='cartographer_occupancy_grid_node',  # 可执行文件，参考cartographer_ros包可执行文件名称
            name='cartographer_occupancy_grid_node', # 重命名节点名称，非必须
            output='screen', # 输出调试信息到控制台
            parameters=[{'use_sim_time': use_sim_time}], # 启动参数，此处传入是否使用仿真时间
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]) # 启动参数，导入配置文件

            # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
            # https://www.robotsfan.com/posts/7a5950c4.html
     elif ROS_DISTRO == 'galactic' or ROS_DISTRO == 'foxy': # 判断当前ros版本，如果是 galactic 或者 foxy则执行以下操作 
        cartographer_occupancy_grid_node = Node( # 建图节点
            package='cartographer_ros', # 包名，cartographer安装包 ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-cartographer
            executable='occupancy_grid_node', # 可执行文件，参考cartographer_ros包可执行文件名称
            name='occupancy_grid_node', # 重命名节点名称，非必须
            output='screen', # 输出调试信息到控制台
            parameters=[{'use_sim_time': use_sim_time}], # 启动参数，此处传入是否使用仿真时间
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]) # 启动参数，导入配置文件
    
    rviz_node = Node( # 可视化节点
        package='rviz2', # 包名，rviz2安装包 ubuntu可以通过命令安装 sudo apt install -y ros-$ROS_DISTRO-rviz2
        executable='rviz2',  # 可执行文件，参考rviz2包可执行文件名称
        name='rviz2', # 重命名节点名称，非必须
        arguments=['-d', rviz_config_dir], # 启动参数，导入配置文件
        parameters=[{'use_sim_time': use_sim_time}], # 配置参数，此处传入yaml格式配置文件，参考配置文件的获取
        output='screen') # 输出调试信息到控制台
        # 更多启动脚本信息参考： https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch
        # https://www.robotsfan.com/posts/7a5950c4.html

    return LaunchDescription([
        chassis_node, # 运行底盘节点
        lidar_node, # 运行雷达节点
        robot_state_publisher_node, # 运行TF节点
        joint_state_publisher_node, # 运行可视化TF节点
        cartographer_node, # 启动建图节点
        cartographer_occupancy_grid_node, # 运行代价地图节点
        rviz_node # 启动可视化
    ])
