/**
 * @file speed_pid.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 电机PID控制
 * @version 0.1
 * @date 2023-02-05
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <memory>
#include <vector>
#include <mutex>

#include "PidController.h"
#include "moto.h"
#include "mpu6050.h"
#include "pwm.h"
#include "rotary_encoder.h"
#include "servo_moto.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

struct DriverParams {
    float LeftWheelRadius;                 // 轮子半径，单位m
    float RightWheelRadius;                // 轮子半径，单位m
    float TrendLength;                     // 轮距，单位m（左轮中线到右轮中线的1/2）
    float WheelBase;                       // 前轮到后左右轮中心点轴距，单位m
    float Ticks1Roll;                      // 编码器线数值（轮子正向转一圈增长的Tick数）
    float MaxXVel;                         // 最大行进速度，单位m/s
    float MaxWVel;                         // 最大角速度，单位rad/s
    float MaxAngle;                        // 最大角度 单位rad
    float ZeroAngle;                       // 0度偏移值 单位rad
    float MaxXAcc;                         // 线加速度限制，单位m/s2
    float MaxWAcc;                         // 角加速度限制，单位rad/s2
    float MaxAngerLimit;                   // 最大转角限制
    float MinAngerLimit;                   // 最小转角限制
    std::vector<std::string> RotaryChanel; // 编码器通道
    float Proportion;                      // 比例系数
    float Integration;                     // 积分系数
    float Differentiation;                 // 微分系数
};

typedef struct
{
    float x   = 0.0;           // 坐标x
    float y   = 0.0;           // 坐标y
    float yaw = 0.0;           // yaw
    Quaternion quaternion;   // 姿态四元数
    float linear_speed  = 0.0; // 线速度
    float angular_speed = 0.0; // 角速度
} odom_t;

class Kinematics
{
public:
    Kinematics(DriverParams car_param, MotoInfo moto_ctrl, PwmPram servo_ctrl);

    Kinematics(DriverParams car_param, MotoInfo moto1_ctrl, MotoInfo moto2_ctrl);
    ~Kinematics();

    /**
     * @brief 运动控制
     * @param xvel 线速度
     * @param wvel 角速度
     * @return true
     * @return false
     */
    bool CmdVel(float xvel, float wvel);

    /**
     * @brief 运动控制
     * @param v_d 电机速度
     * @param delta 转向角
     * @return true
     * @return false
     */
    bool DriverCtrl(float v_d, float delta);

    /**
     * @brief 获取里程计
     * @return odom_t
     */
    odom_t GetOdom();

    /**
     * @brief 设置IMU数据获取接口
     * @param imu_ptr 
     * @return true 
     * @return false 
     */
    bool SetImuPtr(std::shared_ptr<ImuInterface> imu_ptr);

private:
    /**
     * @brief 初始化定时器
     * @return bool
     */
    bool InitTimer(void);

    /**
     * @brief 定时器回调
     * @return int
     */
    int timeOutCallBack();

    /**
     * @brief 运动学逆解
     * @param line_speed 输入线速度
     * @param angle_speed 输入角速度
     * @param out_wheel1_speed 输出电机1速度
     * @param out_wheel2_speed 输出电机2速度
     */
    void TwoWheelInverse(float line_speed, float angle_speed, float &out_wheel1_speed, float &out_wheel2_speed);

    /**
     * @brief 双轮差速运动学正解
     * @param wheel1_speed 输入左轮速度
     * @param wheel2_speed 输入右轮速度
     * @param line_speed 小车线速度
     * @param angle_speed 小车角速度
     */
    void TwoWheelForward(float wheel1_speed, float wheel2_speed, float &line_speed, float &angle_speed);

    /**
     * @brief 阿克曼底盘运动学逆解
     * @param v 输入线速度
     * @param w 输入角速度
     * @param v_d 输出电机速度
     * @param delta 输出转向角
     */
    void AckermanInverse(float v, float w, float &v_d, float &delta);

    /**
     * @brief 阿克曼底盘运动学正解
     * @param wheel_speed 轮子速度
     * @param angle 转向角
     * @param line_speed 小车线速度
     * @param angle_speed 小车角速度
     */
    void AckermanForward(float wheel_speed, float angle, float &line_speed, float &angle_speed);

    /**
     * @brief
     * @param angle
     * @param out_angle
     */
    void TransAngleInPI(float angle, float &out_angle);

    /**
     * @brief 欧拉角转四元数
     * @param roll
     * @param pitch
     * @param yaw
     * @param q
     */
    void Euler2Quaternion(float roll, float pitch, float yaw, Quaternion &q);

    /**
     * @brief 里程计更新
     * @param linear_speed 当前线速度
     * @param angular_speed 当前角速度
     * @param dt deta_t
     */
    void UpdateOdom(float linear_speed, float angular_speed, float dt);

    void TestOpenCl();

    std::shared_ptr<RotaryEncoder> rotary_encoder_[2]; // 编码器通道
    uint64_t last_update_time_;
    float out_motor_speed_[2];                         // 用于保存输出电机速度
    float current_speeds_[2];                          // 用于保存当前电机速度 两个电机的情况下0代表右电机 1代表左电机
    std::shared_ptr<Moto> moto_ctrl_[2];               // 电机控制通道
    std::shared_ptr<PidController> pid_controller_[2]; // PidController的对象
    std::shared_ptr<ServoMoto> servo_ctrl_;            // 舵机控制
    std::shared_ptr<ImuInterface> imu_data_ptr_;

    DriverParams car_param_;
    int timer_fd_;
    // uint32_t dispaly_count_;
    bool model_is_ackerman_;
    // 轮子转动一圈距离
    float distan_1circle_[2];

    float transient_theta_; // 转向角

    std::mutex odom_lock_;
    odom_t odom_; // 里程计数据
};

#endif
