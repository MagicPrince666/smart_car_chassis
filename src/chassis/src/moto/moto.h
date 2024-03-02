/**
 * @file moto.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 通用电机控制
 * @version 0.1
 * @date 2023-01-05
 * @copyright Copyright (c) {2021} 个人版权所有
 */
#ifndef __MOTO_H__
#define __MOTO_H__

#include "gpio.h"
#include "led.h"
#include "pwm.h"
#include <memory>

struct MotoInfo {
    int i_moto_ena = -1;
    int i_moto_enb = -1;
    std::string s_moto_ena = "";
    std::string s_moto_enb = "";
    PwmPram moto_pwm;
};

class Moto
{
public:
    Moto(MotoInfo info);
    virtual ~Moto(void);

    /**
     * @brief 前进
     * @param percent 速度百分比
     * @return int 
     */
    virtual int GoForward(uint32_t pwm_val);

    /**
     * @brief 后退
     * @param percent 速度百分比
     * @return int 
     */
    virtual int GoBack(uint32_t pwm_val);

    /**
     * @brief 电机速度
     * @param speed 负数为后退，正数为前进 -1 - 1
     * @return int 当前速度
     */
    virtual int MotoSpeed(int32_t pwm_val);

    /**
     * @brief 最高速度设置值
     * @return float 
     */
    virtual int32_t MaxPwm();

    /**
     * @brief 电机停转
     * @return int 
     */
    virtual int Stop(void);

private:
    /**
     * @brief Set the Speed object
     * @param pwm_val PWM值
     * @return int 
     */
    int SetSpeed(uint32_t pwm_val);

    char setpin[64] = {0};

protected:
    std::shared_ptr<Gpio> moto_gpio_ena_;
    std::shared_ptr<Gpio> moto_gpio_enb_;
    std::shared_ptr<Led> moto_led_ena_;
    std::shared_ptr<Led> moto_led_enb_;
    std::shared_ptr<Pwm> pwm_moto_;
    uint32_t moto_val_min_;  // 占空比最小值
    uint32_t moto_val_max_;  // 占空比最大值
    int32_t moto_val_rang_; // 占空比变化范围
};

#endif
