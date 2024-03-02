#include <cmath>
#include <iostream>
#include "servo_moto.h"

ServoMoto::ServoMoto(PwmPram servo_pwm)
{
    pwm_servo_ = std::make_shared<Pwm>(servo_pwm); // pwm1 20ms周期
    SetServo(0.0);                                 // 回中
    std::cout << "Init servo moto" << std::endl;
}

ServoMoto::~ServoMoto() {}

int ServoMoto::SetServo(float angle)
{
    int32_t duty_cycle = 1500000 - (angle * 2000000 / M_PI);
    if (pwm_servo_) {
        pwm_servo_->PwmDutyCycle(duty_cycle);
    }
    return 0;
}
