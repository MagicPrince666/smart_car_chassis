#include "select.h"

Select::Select()
{
    readable_map_.clear();
    witable_map_.clear();
    loop_thread_ = std::thread([](Select *p_this) { p_this->SelectWait(); }, this);
}

Select::~Select()
{
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

int Select::SelectAddRead(int fd, std::function<void()> read_handler)
{
    readable_map_[fd] = read_handler;
    return 0;
}

void Select::SelectWait()
{
    fd_set recv_fds;

    struct timeval tv;
    tv.tv_sec  = 10; // 设定超时时间
    tv.tv_usec = 0;  // 10000us = 10ms

    while (true) {
        int maxfd = 0;
        FD_ZERO(&recv_fds);
        for (auto fds : readable_map_) {
            FD_SET(fds.first, &recv_fds);
            if (fds.first > maxfd) { /* maxfd 为最大值  */
                maxfd = fds.first;
            }
        }

        int fd_result = select(maxfd + 1, &recv_fds, NULL, NULL, &tv); // 返回有事件发生的个数
        if (fd_result < 0) {
            // select函数出错
            printf("select err");
            usleep(10000);
            continue;
        } else if (fd_result == 0) {
            // 超时了
            continue;
        } else {
            for (auto fds : readable_map_) {
                if (FD_ISSET(fds.first, &recv_fds)) {
                    fds.second();
                }
            }
        }
    }
}