#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "moto.h"
#include "gpio_sysfs.h"

Moto::Moto(MotoInfo info) {
    moto_val_max_ = info.moto_pwm.period;
    moto_val_min_ = moto_val_max_/2;
    moto_val_rang_ = moto_val_max_ - moto_val_min_;

    if (info.i_moto_ena != -1) {
        moto_gpio_ena_ = std::make_shared<GpioSysfs>(info.i_moto_ena, true);
        moto_gpio_ena_->SetValue(true);
    }

    if (info.i_moto_enb != -1) {
        moto_gpio_enb_ = std::make_shared<GpioSysfs>(info.i_moto_enb, true);
        moto_gpio_enb_->SetValue(true);
    }

    if (!info.s_moto_ena.empty()) {
        moto_led_ena_ = std::make_shared<Led>(info.s_moto_ena);
        moto_led_ena_->SetGpioValue(true);
    }

    if (!info.s_moto_enb.empty()) {
        moto_led_enb_ = std::make_shared<Led>(info.s_moto_enb);
        moto_led_enb_->SetGpioValue(true);
    }

    pwm_moto_  = std::make_shared<Pwm>(info.moto_pwm);   // pwm0 25KHz 1/25000 * 1000000000 = 40000
}

Moto::~Moto(void)
{
    Stop();
    std::cout << "Disable moto" << std::endl;
}

int32_t Moto::MaxPwm()
{
    return moto_val_max_;
}

int Moto::SetSpeed(uint32_t pwm_val)
{
    uint32_t val = pwm_val;
    if(val > moto_val_max_) {
        val = moto_val_max_;
    }
    return pwm_moto_->PwmDutyCycle(val);
}

int Moto::GoForward(uint32_t pwm_val)
{
    // SetSpeed(0);
    if(moto_gpio_ena_) {
        moto_gpio_ena_->SetValue(false);
    }
    if(moto_led_ena_) {
        moto_led_ena_->SetGpioValue(false);
    }
    if(moto_gpio_enb_) {
        moto_gpio_enb_->SetValue(true);
    }
    if(moto_led_enb_) {
        moto_led_enb_->SetGpioValue(true);
    }
    return SetSpeed(pwm_val);
}

int Moto::GoBack(uint32_t pwm_val)
{
    // SetSpeed(0);
    if(moto_gpio_ena_) {
        moto_gpio_ena_->SetValue(true);
    }
    if(moto_led_ena_) {
        moto_led_ena_->SetGpioValue(true);
    }
    if(moto_gpio_enb_) {
        moto_gpio_enb_->SetValue(false);
    }
    if(moto_led_enb_) {
        moto_led_enb_->SetGpioValue(false);
    }
    return SetSpeed(pwm_val);
}

int Moto::MotoSpeed(int32_t pwm_val)
{
    if(pwm_val >= 0) {
        return GoForward(pwm_val);
    } else {
        return GoBack(-pwm_val);
    }
    return 0;
}

int Moto::Stop(void)
{
    if(moto_gpio_ena_) {
        moto_gpio_ena_->SetValue(true);
    }
    if(moto_led_ena_) {
        moto_led_ena_->SetGpioValue(true);
    }
    if(moto_gpio_enb_) {
        moto_gpio_enb_->SetValue(true);
    }
    if(moto_led_enb_) {
        moto_led_enb_->SetGpioValue(true);
    }
    return SetSpeed(0);
}

