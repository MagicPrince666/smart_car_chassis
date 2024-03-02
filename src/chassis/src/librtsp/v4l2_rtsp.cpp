/*
 * @Author: cijliu
 * @Date: 2021-02-04 16:04:16
 * @LastEditTime: 2021-02-26 17:01:59
 */

#include "v4l2_rtsp.h"
#include "H264_UVC_Cap.h"
#include "h264_camera.h"
#include "video_source.h"
#include "xepoll.h"
#include <spdlog/spdlog.h>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

V4l2Rtsp::V4l2Rtsp(std::string dev)
    : video_device_(dev)
{
}

V4l2Rtsp::~V4l2Rtsp()
{
    if (live_thread_.joinable()) {
        live_thread_.join();
    }
}

bool V4l2Rtsp::Init()
{
    live_thread_ = std::thread([](V4l2Rtsp *p_this) { p_this->RtspStart(); }, this);
    return true;
}

const char *V4l2Rtsp::Rfc822DatetimeFormat(time_t time, char *datetime)
{
    int32_t r;
    char *date = asctime(gmtime(&time));
    char mon[8], week[8];
    int32_t year, day, hour, min, sec;
    sscanf(date, "%s %s %d %d:%d:%d %d", week, mon, &day, &hour, &min, &sec, &year);
    r = sprintf(datetime, "%s, %02d %s %04d %02d:%02d:%02d GMT",
                week, day, mon, year, hour, min, sec);
    return r > 0 && r < 32 ? datetime : NULL;
}

uint32_t V4l2Rtsp::RtspGetReltime(void)
{
    struct timespec tp;
    uint64_t ts;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    ts = (tp.tv_sec * 1000000 + tp.tv_nsec / 1000);
    return ts;
}

void V4l2Rtsp::RtpThread()
{
    ip_t *ipaddr = &g_ip_;
    udp_t udp, rtcp;

    UdpServer udp_server;
    if (udp_server.Init(&udp, 45504)) {
        spdlog::error("udp server init fail.");
        return;
    }
    if (udp_server.Init(&rtcp, 45505)) {
        spdlog::error("udp server init fail.");
        return;
    }

    uint32_t rtptime = 0;
    int32_t idr      = 0;

    Rtp rtp;   // rtp通讯类
    H264 h264; // 好64通讯类

    spdlog::info("rtp server init.");

    // 创建具体的视频流工厂
    std::shared_ptr<VideoFactory> video_stream_factory = std::make_shared<UvcH264Camera>();
    // 通过工厂方法创建视屏流产品
    std::shared_ptr<VideoStream> h264_video_ptr(video_stream_factory->createVideoStream(video_device_, 1280, 720, 30));

    uint32_t fMaxSize = 1843200;
    uint8_t *h264data = new uint8_t[fMaxSize];
    uint32_t fFrameSize;         // out
    uint32_t fNumTruncatedBytes; // out

    while (g_pause_) {
        unsigned len = h264_video_ptr->getData(h264data, fMaxSize, fFrameSize, fNumTruncatedBytes);
        if (len <= 0) {
            continue;
        }
        h264_nalu_t *nalu = h264.NalPacketMalloc(h264data, fFrameSize);

        h264_nalu_t *h264_nal = nalu;

        while (h264_nal && g_pause_) {
            if (h264_nal->type == H264_NAL_IDR || h264_nal->type == H264_NAL_PFRAME) {
                if (rtptime == 0) {
                    rtptime = RtspGetReltime();
                } else {
                    while ((rtptime + 40000) > RtspGetReltime()) {
                        usleep(100);
                    }
                    // printf("sleep:%d us\n", RtspGetReltime() - rtptime);
                    rtptime = RtspGetReltime();
                }
                idr                   = 1;
                rtp_packet_t *rtp_ptk = rtp.PacketMalloc(h264_nal->data, h264_nal->len);
                rtp_packet_t *cur     = rtp_ptk;
                while (cur) {
                    udp_server.SendMsg(&udp, ipaddr->ip, ipaddr->port, (uint8_t *)cur->data, cur->len);
                    cur = cur->next;
                }
                rtp.PacketFree(rtp_ptk);
            } else if ((h264_nal->type == H264_NAL_SPS || h264_nal->type == H264_NAL_PPS) && !idr) {
                rtp_packet_t *cur = rtp.PacketMalloc(h264_nal->data, h264_nal->len);
                udp_server.SendMsg(&udp, ipaddr->ip, ipaddr->port, (uint8_t *)cur->data, cur->len);
                rtp.PacketFree(cur);
            }
            h264_nal = h264_nal->next;
        }
        h264.NalPacketFree(nalu);
    }
    delete[] h264data;
    udp_server.Deinit(&udp);
    udp_server.Deinit(&rtcp);
    spdlog::info("rtp exit");
}

std::string V4l2Rtsp::GetHostIpAddress()
{
    std::string Ip;
    char name[256];
    gethostname(name, sizeof(name));

    struct hostent *host = gethostbyname(name);
    char ipStr[32];
    const char *ret = inet_ntop(host->h_addrtype, host->h_addr_list[0], ipStr, sizeof(ipStr));
    if (nullptr == ret) {
        spdlog::info("hostname transform to ip failed");
        return "";
    }
    Ip = ipStr;
    return Ip;
}

int V4l2Rtsp::RtspStart()
{
    ip_t *ipaddr = &g_ip_;
    tcp_t tcp;
    int32_t client = 0;
    RtspHandler rtsp_handler;
    TcpServer tcp_server;

    spdlog::info("Use commad: rtsp://{}:8554/live", GetHostIpAddress().c_str());

    if (tcp_server.Init(&tcp, 8554)) {
        spdlog::error("tcp server init fail.");
        return -1;
    }

    char msg[2048];
    char sdp[2048] = "v=0\n"
                     "o=- 16409863082207520751 16409863082207520751 IN IP4 0.0.0.0\n"
                     "c=IN IP4 0.0.0.0\n"
                     "t=0 0\n"
                     "a=range:npt=0-1.4\n"
                     "a=recvonly\n"
                     "m=video 0 RTP/AVP 97\n"
                     "a=rtpmap:97 H264/90000\n";
    std::thread rtp_thread_create;

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        if (client == 0) {
            fd_set fds;
            struct timeval tv;
            int32_t r;

            FD_ZERO(&fds);
            FD_SET(tcp.sock, &fds);
            tv.tv_sec  = 2;
            tv.tv_usec = 0;
            r          = select(tcp.sock + 1, &fds, NULL, NULL, &tv);
            if (-1 == r || 0 == r) {
                continue;
            }
            client = tcp_server.WaitClient(&tcp);
            sprintf(ipaddr->ip, "%s", inet_ntoa(tcp.addr.sin_addr));
            ipaddr->port = ntohs(tcp.addr.sin_port);
            spdlog::info("rtsp client ip:{} port:{}", inet_ntoa(tcp.addr.sin_addr), ntohs(tcp.addr.sin_port));
        }
        char recvbuffer[2048];
        tcp_server.ReceiveMsg(&tcp, client, (uint8_t *)recvbuffer, sizeof(recvbuffer));
        rtsp_msg_t rtsp = rtsp_handler.RtspMsgLoad(recvbuffer);
        char datetime[30];
        Rfc822DatetimeFormat(time(NULL), datetime);
        rtsp_rely_t rely = rtsp_handler.GetRely(rtsp);
        memcpy(rely.datetime, datetime, strlen(datetime));
        switch (rtsp.request.method) {
        case SETUP:
            rely.tansport.server_port = 45504;
            rtsp_handler.RtspRelyDumps(rely, msg, 2048);
            sprintf(g_ip_.ip, "%s", ipaddr->ip);
            g_ip_.port        = rtsp.tansport.client_port;
            g_pause_          = true;
            rtp_thread_create = std::thread(std::bind(&V4l2Rtsp::RtpThread, this));
            rtp_thread_create.detach();
            // if (rtp_thread_create.joinable()) {
            //     rtp_thread_create.join();
            // }
            spdlog::info("rtp client ip:{} port:{}", g_ip_.ip, g_ip_.port);
            break;
        case DESCRIBE:
            rely.sdp_len = strlen(sdp);
            memcpy(rely.sdp, sdp, rely.sdp_len);
            rtsp_handler.RtspRelyDumps(rely, msg, 2048);
            break;
        case TEARDOWN:
            rtsp_handler.RtspRelyDumps(rely, msg, 2048);
            tcp_server.SendMsg(&tcp, client, msg, strlen(msg));
            tcp_server.CloseClient(&tcp, client);
            client   = 0;
            g_pause_ = false;
            continue;
        default:
            rtsp_handler.RtspRelyDumps(rely, msg, 2048);
            break;
        }
        tcp_server.SendMsg(&tcp, client, msg, strlen(msg));
        usleep(1000);
    }
    tcp_server.Deinit(&tcp);

    return 0;
}
