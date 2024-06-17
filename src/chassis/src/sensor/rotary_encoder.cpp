#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#ifndef __APPLE__
#include <linux/input.h>
#include <sys/timerfd.h>
#endif

#include "rotary_encoder.h"
#include "utils.h"
#include "xepoll.h"

RotaryEncoder::RotaryEncoder(std::string dev, float reduction_ratio, float precision)
    : encoder_dev_name_(dev), reduction_ratio_(reduction_ratio), precision_(precision)
{
    rotary_encoder_fd_ = -1;
    velocity_          = 0;

    Init();
}

RotaryEncoder::~RotaryEncoder()
{

    if (rotary_encoder_fd_ > 0) {
        MY_EPOLL.EpollDel(rotary_encoder_fd_);
        close(rotary_encoder_fd_);
    }
}

int RotaryEncoder::ReadTicks(void)
{
    struct input_event key;
    int ret = read(rotary_encoder_fd_, &key, sizeof(key));
    if (ret > 0) {
        // 使用旋转编码器相对轴， key.value只有 1/-1
        if (key.type != 0) {
            // std::cout << rotary_encoder_dev_ << " type = " << key.type << " Code = " << key.code << " Value = " << key.value << std::endl;
            std::lock_guard<std::mutex> mylock_guard(counter_lock_);
            if (key.value == 1) { // 正转
                velocity_++;
            } else {              // -1 反转
                velocity_--;
            }
        }
    }
    return ret;
}

int32_t RotaryEncoder::GetTicksRoll()
{
    std::lock_guard<std::mutex> mylock_guard(counter_lock_);
    int32_t tmp_velocity = velocity_;
    velocity_            = 0;
    return tmp_velocity;
}

bool RotaryEncoder::Init()
{
    rotary_encoder_dev_ = Utils::ScanInputDevice(encoder_dev_name_);
    rotary_encoder_fd_ = open(rotary_encoder_dev_.c_str(), O_RDONLY, 0);

    assert(rotary_encoder_fd_ >= 0);
    // 绑定回调函数
    if (rotary_encoder_fd_ > 0) {
        std::cout << encoder_dev_name_ << " bind epoll" << std::endl;
        MY_EPOLL.EpollAddRead(rotary_encoder_fd_, std::bind(&RotaryEncoder::ReadTicks, this));
    }
    return true;
}
