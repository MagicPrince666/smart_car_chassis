/**
 * @file comm_factory.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __COMM_FACTORY_H__
#define __COMM_FACTORY_H__

#include <iostream>
#include <functional>

// 通讯产品基类
class Communication
{
public:
    Communication(std::string dev, const int rate, bool debug) : device_(dev), comm_rate_(rate), debug_(debug) {}
    virtual ~Communication() {}

    /**
     * @brief 发送接口
     * @param buffer
     * @param length
     * @return int
     */
    virtual int SendBuffer(const uint8_t *const buffer, const int length) = 0;

    /**
     * @brief 设置通讯参数
     * @param opt 参数值
     * @param filterType 类型
     * @return true 设置成功
     * @return false 设置失败
     */
    virtual bool SetOption(int32_t opt, int32_t filterType)
    {
        if (!opt && !filterType) {
            return false;
        }
        return true;
    }

    /**
     * @brief 回调注册
     * @param handler
     */
    virtual void AddCallback(std::function<void(const uint8_t *, const uint32_t)> handler)
    {
        read_function_ = handler;
    }

    /**
     * 清除回调
     */
    virtual void RemoveCallback()
    {
        read_function_ = nullptr;
    }

protected:
    std::string device_;  // 设备名
    const int comm_rate_; // 通讯速率
    bool debug_;          // 调试开关
    std::function<void(const uint8_t *, const uint32_t)> read_function_; // 接收回调

private:
    /**
     * @brief 收到数据产生的回调
     * @return int
     */
    virtual int ReadCallback() = 0;

    /**
     * @brief 写回调
     * @return int
     */
    virtual int WriteCallback() = 0;
};

// 通讯工厂基类
class CommFactory
{
public:
    virtual Communication *CreateCommTarget(std::string dev, const int rate, bool debug) = 0;
};

#endif
