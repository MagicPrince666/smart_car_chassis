#include <arpa/inet.h>
#include <ctype.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "remote.h"
#include "socket.h"
#include "xepoll.h"
#include "utils.h"

UdpSocket::UdpSocket(RemoteConfig_t config, bool debug) : RemoteProduct(config, debug) {
    std::cout << "UdpSocket init" << std::endl;
    Init();
}

UdpSocket::~UdpSocket()
{
    if (socket_fd_ > 0) {
        MY_EPOLL.EpollDel(socket_fd_);
        close(socket_fd_);
    }
}

int UdpSocket::Init()
{
    struct sockaddr_in servaddr;

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        perror("Create socket error");
        return -1;
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(SERV_PORT);

    bind(socket_fd_, (struct sockaddr *)&servaddr, sizeof(servaddr));

    MY_EPOLL.EpollAddRead(socket_fd_, std::bind(&UdpSocket::GetClientData, this));
    return 0;
}

int UdpSocket::GetClientData()
{
    char buf[MAXLINE];
    struct sockaddr_in cliaddr;
    socklen_t cliaddr_len;
    cliaddr_len = sizeof(cliaddr);
    int btyes   = recvfrom(socket_fd_, buf, MAXLINE, 0, (struct sockaddr *)&cliaddr, &cliaddr_len);
    if (btyes == -1) {
        perror("recvfrom error");
    }
    // char str[INET_ADDRSTRLEN];
    // printf("Recvfrom %s port %d\n",
    //        inet_ntop(AF_INET, &cliaddr.sin_addr, str, sizeof(str)), ntohs(cliaddr.sin_port));

    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    last_update_time_ = Utils::GetCurrentMsTime();
    memcpy((char *)&rc_data_, buf, sizeof(rc_data_));

    return 0;
}

bool UdpSocket::Request(struct RemoteState &data)
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    uint64_t current  = Utils::GetCurrentMsTime();
    if ((current - last_update_time_) > 1000) {
        rc_data_.lose_signal = true;
    } else {
        rc_data_.lose_signal = false;
    }
    data = rc_data_;
    return true;
}
