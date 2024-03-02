/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node Client
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#include <cmath>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
using LaserScanMsg = sensor_msgs::LaserScan;
#define SharedPtr ConstPtr
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
using LaserScanMsg = sensor_msgs::msg::LaserScan;
#endif

#define RAD2DEG(x) ((x) * 180. / M_PI)

static void scanCb(LaserScanMsg::SharedPtr scan)
{
    int count = scan->scan_time / scan->time_increment;
    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
           RAD2DEG(scan->angle_max));

    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    }
}

int main(int argc, char **argv)
{
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros::init(argc, argv, "ydlidar_client");
    auto ros_node = std::make_shared<ros::NodeHandle>();
    auto lidar_info_sub = ros_node->subscribe<LaserScanMsg>(
        "scan", 10, scanCb);

    ros::spin();
    ros::shutdown();
#else
    rclcpp::init(argc, argv);
    auto node           = rclcpp::Node::make_shared("ydlidar_client");
    auto lidar_info_sub = node->create_subscription<LaserScanMsg>(
        "scan", rclcpp::SensorDataQoS(), scanCb);
    rclcpp::spin(node);
    rclcpp::shutdown();
#endif

    return 0;
}
