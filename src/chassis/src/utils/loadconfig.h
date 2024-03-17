/**
 * @file loadconfig.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 载入配置文件
 * @version 0.1
 * @date 2023-07-08
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __LOAD_CONFIG_H__
#define __LOAD_CONFIG_H__

#include <iostream>
#include <string>

#include "json/json.h"
#include "pwm.h"
#include "Kinematics.h"

class LoadConfig
{
public:
    LoadConfig(std::string file_path);
    ~LoadConfig();

    bool LoadPwmConfig(std::string key, PwmPram &pwm);
    bool LoadPinConfig(std::string key, int &pin);
    bool LoadPinConfig(std::string key, std::string &pin);
    bool LoadDistanceConfig(std::string key, bool &enable);
    bool LoadRataryConfig(std::string key, std::string &device, float &reduction_ratio, float &precision);
    bool LoadCarConfig(std::string key, DriverParams &car);

private:
    Json::Value conf_json_;
};

#endif
