/**
 * @file chassis_service.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */

#ifndef __CHASSIS_SERVICE_H__
#define __CHASSIS_SERVICE_H__

#include <memory>

#include "Kinematics.h"
#include "RecordData.h"
#include "imu_interface.h"
#include "keyboard.h"
#include "srf04.h"
#include "vl53l0x.h"

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#define SharedPtr ConstPtr
#else
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#endif

class ChassisSrv
{
public:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ChassisSrv(std::shared_ptr<ros::NodeHandle> node);
#else
    ChassisSrv(std::shared_ptr<rclcpp::Node> node);
#endif
    ~ChassisSrv();

private:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    using TwistMsg    = geometry_msgs::Twist;
    using ImuMsg      = sensor_msgs::Imu;
    using RangeMsg    = sensor_msgs::Range;
    using OdometryMsg = nav_msgs::Odometry;
    std::shared_ptr<ros::Publisher> imu_pub_;
    std::shared_ptr<ros::Publisher> ultrasonic_pub_;
    std::shared_ptr<ros::Publisher> tof_pub_;
    std::shared_ptr<ros::Publisher> odom_pub_;
    std::shared_ptr<ros::Subscriber> cmd_vel_sub_;
    std::shared_ptr<ros::NodeHandle> ros_node_;

    // ros::WallTimer imu_timer_;
    // ros::WallTimer loop_timer_;
    ros::Timer imu_timer_;
    ros::Timer loop_timer_;

    ros::CallbackQueue callback_group_sub1_;
    ros::CallbackQueue callback_group_sub2_;
#else
    using TwistMsg    = geometry_msgs::msg::Twist;
    using ImuMsg      = sensor_msgs::msg::Imu;
    using RangeMsg    = sensor_msgs::msg::Range;
    using OdometryMsg = nav_msgs::msg::Odometry;
    rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;
    rclcpp::Publisher<RangeMsg>::SharedPtr ultrasonic_pub_;
    rclcpp::Publisher<RangeMsg>::SharedPtr tof_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
#if defined(USE_HUMBLE_VERSION) || defined(USE_IRON_VERSION)
    rclcpp::CallbackGroup::SharedPtr callback_group_sub1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub2_;
#else
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub2_;
#endif
    rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    std::shared_ptr<rclcpp::Node> ros_node_;
#endif
    void CmdVelCallback(const TwistMsg::SharedPtr msg);
    /**imu_publisher_
     * @brief IMU回调
     */
    void FastTimerCallback();

    void LoopCallback();

    /**
     * @brief 里程计发布
     */
    void PubOdom();

    /**
     * @brief 获取毫秒时间戳
     * @return uint32_t
     */
    uint32_t GetCurrentMsTime();

    bool car_is_ackerman_;
    std::shared_ptr<Kinematics> motion_ctl_;
    std::shared_ptr<ImuInterface> imu_data_ptr_;
    std::shared_ptr<Pwm> lidar_speed_;
    std::shared_ptr<Srf04> ultrasonic_;
    std::shared_ptr<Vl53l0x> back_distance_;
    std::shared_ptr<RemoteProduct> remote_;

    RemoteState rc_data_;
    RemoteConfig_t config_;
    bool driver_enable_;
    bool avoid_obstacles_;

    std::shared_ptr<RecordData> record_data_;
    float recording_input_ = 0.0;

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros::Time GetTimeNow();
#else
    rclcpp::Time GetTimeNow();
#endif
};

#endif
