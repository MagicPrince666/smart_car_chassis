#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <unordered_map>

#include "gpio_sysfs.h"
#include "xepoll.h"

GpioSysfs::GpioSysfs(int pin, bool io)
{
    gpio_fd_ = -1;
    gpio_pin_ = pin;
    if (SetupGpio()) {
        std::cout << "Init user gpio: " << pin << std::endl;
        SetDirection(io);
        SetActiveLow(true);
        OpenGpio();
    } else {
        gpio_pin_ = -1;
    }
}

GpioSysfs::~GpioSysfs()
{
    CloseGpio();
    UnExportGpio();
    std::cout << "Close user gpio: " << gpio_pin_ << std::endl;
}

bool GpioSysfs::SetupGpio()
{
    FILE *set_export = fopen("/sys/class/gpio/export", "w");
    if (set_export == nullptr) {
        printf("Can't open /sys/class/gpio/export!\n");
        return false;
    } else {
        fprintf(set_export, "%d", gpio_pin_);
    }
    assert(set_export != nullptr);
    fclose(set_export);
    return true;
}

bool GpioSysfs::UnExportGpio()
{
    if (gpio_fd_ < 0) {
        return false;
    }
    FILE *set_export = fopen("/sys/class/gpio/unexport", "w");
    if (set_export == nullptr) {
        printf("Can't open /sys/class/gpio/unexport!\n");
        return false;
    } else {
        fprintf(set_export, "%d", gpio_pin_);
    }
    fclose(set_export);
    return true;
}

bool GpioSysfs::SetDirection(bool io)
{
    if (gpio_pin_ < 0) {
        return false;
    }
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/gpio/gpio%d/direction", gpio_pin_);
    setpin[len] = 0;
    for (uint32_t i = 0; i < 5; i++) {
        std::ofstream direction(setpin);
        if (!direction.is_open()) {
            printf("open %s error\n", setpin);
            usleep(20000);
            continue;
        } else {
            if (io) {
                direction << "out";
                is_output_ = true;
            } else {
                direction << "in";
                is_output_ = false;
            }
            direction.close();
            break;
        }
    }
    return io;
}

int GpioSysfs::OpenGpio()
{
    if (gpio_pin_ < 0) {
        return -1;
    }
    CloseGpio();
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/gpio/gpio%d/value", gpio_pin_);
    setpin[len] = 0;
    for (uint32_t i = 0; i < 5; i++) {
        gpio_fd_ = open(setpin, O_RDWR);
        if (gpio_fd_ < 0) {
            printf("can not open %s\n", setpin);
            usleep(20000);
            continue;
        } else {
            break;
        }
    }
    if (!is_output_) {
        // SetEdge(IRQ_TYPE_EDGE_BOTH);
        // MY_EPOLL.EpollAddRead(gpio_fd_, std::bind(&GpioSysfs::GetValue, this));
    }
    return gpio_fd_;
}

bool GpioSysfs::Init()
{
    return true;
}

void GpioSysfs::CloseGpio()
{
    if (gpio_fd_ > 0) {
        if (!is_output_) {
            // MY_EPOLL.EpollDel(gpio_fd_);
        }
        close(gpio_fd_);
    }
}

void GpioSysfs::SetValue(bool value)
{
    if (gpio_fd_ > 0) {
        write(gpio_fd_, std::to_string(value).c_str(), 1);
    }
}

std::string GpioSysfs::ReadFileIntoString(const std::string& path) {
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        return "";
    }
    return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
}

bool GpioSysfs::GetValue()
{
    bool value = false;
    if (gpio_pin_ < 0) {
        return false;
    }
#if 1
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/gpio/gpio%d/value", gpio_pin_);
    setpin[len] = 0;

    std::string data = ReadFileIntoString(setpin);
    // std::cout << "value = " << data;
    if (data.find("0") != std::string::npos) {
        value  = false;
    } else if (data.find("1") != std::string::npos) {
        value = true;
    }
#else
    char val;
    lseek(gpio_fd_, 0, SEEK_SET); // 重置文件指针位置
	int len = read(gpio_fd_, &val, sizeof(val));
    std::cout << "value = " << (int)val << std::endl;
    if(len > 0) {
        if (val == '1') {
            value = true;
        } else {
            value = false;
        }
    }
#endif

    return value;
}

void GpioSysfs::SetActiveLow(bool act_low)
{
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/gpio/gpio%d/active_low", gpio_pin_);
    setpin[len] = 0;
    FILE *set_dir = fopen(setpin, "w");
    if (set_dir == nullptr) {
        printf("open %s error\n", setpin);
        return;
    } else {
        if (act_low) {
            fprintf(set_dir, "0");
        } else {
            fprintf(set_dir, "1");
        }
        fclose(set_dir);
    }
}

void GpioSysfs::SetEdge(EdgeType type)
{
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/gpio/gpio%d/edge", gpio_pin_);
    setpin[len] = 0;
    std::ofstream edgeFile(setpin);
    if (!edgeFile.is_open()) {
        printf("open %s error\n", setpin);
        return;
    }
    if (interrupt_type_map_.count(type)) {
        edgeFile << interrupt_type_map_[type];
    }
    edgeFile.close();
}