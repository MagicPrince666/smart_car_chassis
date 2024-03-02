/**
 * @file servo_moto.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 舵机控制
 * @version 0.1
 * @date 2023-04-03
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __SERVO_MOTO_H__
#define __SERVO_MOTO_H__

#include <memory>
#include "pwm.h"

class ServoMoto {
public:
    ServoMoto(PwmPram servo_pwm);
    ~ServoMoto();

    /**
     * @brief 舵机控制
     * @param angle 弧度 -PI/4 ~ PI/4 向左为正
     * @return int 
     */
    virtual int SetServo(float angle);
private:
    std::shared_ptr<Pwm> pwm_servo_;
};

#endif
