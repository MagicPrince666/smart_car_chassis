/**
 * @file rotary_encoder.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 旋转编码器
 * @version 0.1
 * @date 2023-02-02
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 *
 * 选用的减速电机减速比1:34,编码器精度 12CPR，及电机一圈有12个脉冲（不包含减速）
 */
#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

#include <iostream>
#include <mutex>
#include <string>

class RotaryEncoder
{
public:
    RotaryEncoder(std::string dev = "rotary@0", float reduction_ratio = 34.0, float precision = 12.0);
    ~RotaryEncoder();

    /**
     * @brief 获取编码器计数
     * @return int32_t
     */
    int32_t GetTicksRoll();

private:
    bool Init();

    /**
     * @brief 脉冲计数
     * @return int
     */
    int ReadTicks(void);

    std::mutex counter_lock_;
    std::string rotary_encoder_dev_;
    std::string encoder_dev_name_;
    int rotary_encoder_fd_;
    float reduction_ratio_; // 减速比
    float precision_;       // 编码器精度
    int32_t velocity_;
};

#endif
