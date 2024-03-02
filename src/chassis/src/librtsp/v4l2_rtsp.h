/**
 * @file v4l2_rtsp.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-06-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __V4L2_RTSP_H__
#define __V4L2_RTSP_H__

#include "h264.h"
#include "net.h"
#include "rtp.h"
#include "rtsp_handler.h"
#include "rtsp_parse.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <netdb.h> /* struct hostent */
#include <arpa/inet.h> /* inet_ntop */
#include <thread>
#include <atomic>
#include <iostream>

class V4l2Rtsp
{
private:
    std::thread live_thread_;
    std::atomic<bool> g_pause_;
    ip_t g_ip_;
    std::string video_device_;

    int RtspStart();

    std::string GetHostIpAddress();

    void RtpThread();

    uint32_t RtspGetReltime(void);

    const char *Rfc822DatetimeFormat(time_t time, char *datetime);

public:
    V4l2Rtsp(std::string dev);
    ~V4l2Rtsp();

    bool Init();
};

#endif
