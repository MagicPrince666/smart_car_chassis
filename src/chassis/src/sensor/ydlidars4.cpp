#include "ydlidars4.h"

YdLidarS4::YdLidarS4(std::string dev, PwmPram lidar_pwm)
:lidar_port_(dev)
{
    printf("Lidar port %s\n", dev.c_str());
    lidar_speed_ = std::make_shared<Pwm>(lidar_pwm);
    lidar_port_fd_ = OpenSerial();
    assert(lidar_port_fd_ > 0);
    MY_EPOLL.EpollAddRead(lidar_port_fd_, std::bind(&YdLidarS4::Posttioning, this));
}

YdLidarS4::~YdLidarS4()
{
    if (lidar_port_fd_ > 0) {
        MY_EPOLL.EpollDel(lidar_port_fd_);
        close(lidar_port_fd_);
    }
}

int YdLidarS4::Speed(int speed)
{
    return lidar_speed_->PwmDutyCycle(speed);
}

int YdLidarS4::OpenSerial()
{
    struct termios opt;

    int iFd = open(lidar_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (iFd < 0) {
        perror(lidar_port_.c_str());
        return -1;
    }

    tcgetattr(iFd, &opt);

    cfsetispeed(&opt, B115200);
    cfsetospeed(&opt, B115200);

    /*
     * raw mode
     */
    opt.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    opt.c_oflag &= ~(OPOST);
    opt.c_cflag &= ~(CSIZE | PARENB);
    opt.c_cflag |= CS8;

    /*
     * 'DATA_LEN' bytes can be read by serial
     */
    opt.c_cc[VMIN]  = 0xFF;
    opt.c_cc[VTIME] = 150;

    if (tcsetattr(iFd, TCSANOW, &opt) < 0) {
        return -1;
    }

    return iFd;
}

int YdLidarS4::Posttioning()
{
    uint8_t uart_buf[1024];
    int len = read(lidar_port_fd_, uart_buf, sizeof(uart_buf));
    if (len > 0) {
        uart_buf[len] = 0;
        // std::string text((char *)uart_buf);
        // printf("lidar:%s size :%d\n", spdlog::to_hex(text).c_str(), text.length());
    }
    return 0;
}
