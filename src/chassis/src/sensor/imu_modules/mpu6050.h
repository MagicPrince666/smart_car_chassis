/**
 * @file mpu6050.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 陀螺仪数据获取及解算
 * @version 0.1
 * @date 2023-02-02
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "driver_mpu6050_dmp.h"
#include "gpio.h"
#include "imu_interface.h"
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <thread>

class Mpu6050 : public ImuInterface
{
public:
    Mpu6050(ImuConf conf);
    ~Mpu6050();

    bool Init();

private:
    std::shared_ptr<Gpio> mpu_int_;

    std::thread imu_thread_;
    std::string module_name_;

    void Euler2Quaternion(float roll, float pitch, float yaw, Quaternion &quat);

    int GpioInterruptInit();

    void GpioInterruptDeinit();

    void Mpu6050Loop();

    void ReadHander(const bool val, const uint64_t timestamp);
};

#endif
