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
    : rotary_encoder_dev_(dev), reduction_ratio_(reduction_ratio), precision_(precision)
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
    int fd = -1;
    if ((fd = open(rotary_encoder_dev_.c_str(), O_RDONLY, 0)) >= 0) {
        char buf[256] = {0};
        ioctl(fd, EVIOCGNAME(sizeof(buf)), buf);
        std::string device(buf);
        std::cout << rotary_encoder_dev_ << " info " << device << std::endl;
        if (device.find("rotary") != std::string::npos) {
            rotary_encoder_fd_ = fd;
        }
    }

    assert(rotary_encoder_fd_ >= 0);
    // 绑定回调函数
    if (rotary_encoder_fd_ > 0) {
        std::cout << "Bind epoll" << std::endl;
        MY_EPOLL.EpollAddRead(rotary_encoder_fd_, std::bind(&RotaryEncoder::ReadTicks, this));
    }
    return true;
}
