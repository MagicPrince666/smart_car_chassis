/**
 * @file gpio_sysfs.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-01-05
 * @copyright Copyright (c) {2021} 个人版权所有
 */
#ifndef __GPIO_SYSFS_H__
#define __GPIO_SYSFS_H__

#include <iostream>
#include <unordered_map>
#include "gpio.h"

class GpioSysfs : public Gpio
{
public:
    GpioSysfs(int pin, bool io = false);
    ~GpioSysfs();

    bool Init();
    int OpenGpio();
    void SetValue(bool value);
    bool GetValue();
    // 修改输入输出方向
    bool SetDirection(bool io);

    // 配置极性
    void SetActiveLow(bool act_low);
    // 设置中断方式
    void SetEdge(EdgeType type);

private:
    char setpin[64] = {0};
    int gpio_pin_;
    int gpio_fd_;
    std::unordered_map<EdgeType, std::string> interrupt_type_map_ = {
        {IRQ_TYPE_EDGE_RISING, "rising"},
        {IRQ_TYPE_EDGE_FALLING, "falling"},
        {IRQ_TYPE_EDGE_BOTH, "both"},
        {IRQ_TYPE_LEVEL_HIGH, "high"},
        {IRQ_TYPE_LEVEL_LOW, "low"},
    };

    bool UnExportGpio();
    bool SetupGpio();
    void CloseGpio();
    std::string ReadFileIntoString(const std::string& path);
};

#endif
