/**
 * @file wifi.h
 * @author 黄李全 (846863428@qq.com)
 * @brief wifi操作
 * @version 0.1
 * @date 2023-03-26
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __WIFI_H__
#define __WIFI_H__

class WifiCtrl {
public:
    WifiCtrl(std::string ssid, std::string password);
    ~WifiCtrl();

    bool WpaCliConfigWifi();
    bool SaveWifiConfig();
    int WpaSupplicantConfigWifi();

private:
    std::string wifi_ssid_;
    std::string wifi_password_;
    void Execute(std::string cmdline, std::string &recv);
};

#endif
