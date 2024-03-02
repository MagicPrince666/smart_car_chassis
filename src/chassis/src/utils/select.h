/**
 * @file select.h
 * @author 黄李全 (846863428@qq.com)
 * @brief select io多路复用
 * @version 0.1
 * @date 2023-04-12
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __SELECT_H__
#define __SELECT_H__

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <thread>

#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#define MY_SELECT Select::Instance()

class Select
{
public:
    static Select& Instance() {
        static Select instance;
        return instance;
    }

    ~Select();

    int SelectAddRead(int fd, std::function<void()> read_handler);

private:
    Select();

    void SelectWait();

    std::thread loop_thread_;

    // 读回调列表
    std::unordered_map<int, std::function<void()>> readable_map_;
    // 写回调列表
    std::unordered_map<int, std::function<void()>> witable_map_;
};

#endif
