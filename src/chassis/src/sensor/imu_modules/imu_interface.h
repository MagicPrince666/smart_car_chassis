/**
 * @file imu_interface.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 串口IMU
 * @version 0.1
 * @date 2023-10-21
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __IMU_INTERFACE_H__
#define __IMU_INTERFACE_H__

#include <iostream>
#include <mutex>
#include <spdlog/spdlog.h>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
} Quaternion;

typedef struct
{
    float roll;  /* 横滚角，单位：rad */
    float pitch; /* 俯仰角，单位：rad */
    float yaw;   /* 航向角，单位：rad */
} Eular;

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
} Vector3;

typedef struct {
    Quaternion orientation;      // 姿态
    Vector3 angular_velocity;    // 角速度
    Vector3 linear_acceleration; // 线加速度
    Eular eular;                 // 欧拉角
} Imu;

class ImuInterface
{
public:
    ImuInterface(std::string port, uint32_t rate)
        : imu_port_(port), baud_rate_(rate) {}
    virtual ~ImuInterface() {}

    virtual bool Init() = 0;

    virtual Imu GetImuData()
    {
        std::lock_guard<std::mutex> mylock_guard(data_lock_);
        return imu_data_;
    }

    virtual double GetImuYaw()
    {
        std::lock_guard<std::mutex> mylock_guard(data_lock_);
        return imu_data_.eular.yaw;
    }

protected:
    std::string imu_port_;
    uint32_t baud_rate_;
    Imu imu_data_;
    std::mutex data_lock_;
};

#endif
