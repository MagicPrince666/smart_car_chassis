/**
 * @file vl53l0x.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 距离传感器
 * @version 0.1
 * @date 2023-04-05
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __VL5310X_H__
#define __VL5310X_H__

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>

class Vl53l0x {
public:
    Vl53l0x(std::string dev = "vl53l0x");
    ~Vl53l0x();
    int GetDistance();

private:
    std::string device_dir_;
};

#endif
