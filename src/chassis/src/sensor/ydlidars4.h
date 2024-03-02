/**
 * @file ydlidars4.h
 * @author 黄李全 (846863428@qq.com)
 * @brief YD雷达S4
 * @version 0.1
 * @date 2023-04-05
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __YDLIDAR_S4_H__
#define __YDLIDAR_S4_H__

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <memory>
#include "pwm.h"
#include "xepoll.h"

class YdLidarS4
{
public:
    YdLidarS4(std::string dev, PwmPram lidar_pwm);
    ~YdLidarS4();

    int Speed(int speed);
private:
    int OpenSerial();
    int Posttioning();

    std::string lidar_port_;
    int lidar_port_fd_;
    std::shared_ptr<Pwm> lidar_speed_;
};

#endif
