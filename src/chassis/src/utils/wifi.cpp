#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include "wifi.h"

const char *WIFI_CONFIG_FORMAT = "ctrl_interface=/var/run/wpa_supplicant\n"
                                 "ap_scan=1\n\nnetwork={\nssid=\"%s\"\n"
                                 "psk=\"%s\"\npriority=1\n}\n";

WifiCtrl::WifiCtrl(std::string ssid, std::string password) :
wifi_ssid_(ssid), wifi_password_(password) {}

WifiCtrl::~WifiCtrl() {}

void WifiCtrl::Execute(std::string cmdline, std::string &recv)
{
    printf("consule_run: %s\n", cmdline.c_str());

    FILE *stream = NULL;
    char buff[1024];
    char recv_buff[256]      = {0};

    memset(recv_buff, 0, sizeof(recv_buff));

    if ((stream = popen(cmdline.c_str(), "r")) != NULL) {
        while (fgets(buff, 1024, stream)) {
            strcat(recv_buff, buff);
        }
    }
    recv = recv_buff;
    pclose(stream);
}

// wpa_supplicant
int WifiCtrl::WpaSupplicantConfigWifi()
{
    FILE *fp = NULL;

    if ((fp = fopen("/data/cfg/wpa_supplicant.conf", "w+")) == NULL) {
        printf("open cfg wpa_supplicant.conf failed\n");
        return -1;
    }

    fprintf(fp, "%s\n", "ctrl_interface=/var/run/wpa_supplicant");
    fprintf(fp, "%s\n", "ap_scan=1");
    fprintf(fp, "%s\n", "network={");
    fprintf(fp, "%s%s%s\n", "ssid=\"", wifi_ssid_.c_str(), "\"");
    fprintf(fp, "%s%s%s\n", "psk=\"", wifi_password_.c_str(), "\"");
    fprintf(fp, "%s\n", "key_mgmt=WPA-PSK");
    fprintf(fp, "%s\n", "}");

    fclose(fp);

    if (-1 == system("killall wpa_supplicant; dhcpcd -k wlan0; killall dhcpcd;"
                     "ifconfig wlan0 0.0.0.0")) {
        printf("killall wpa_supplicant dhcpcd failed\n");
        return -1;
    }

    if (-1 == system("wpa_supplicant -Dnl80211 -i wlan0 "
                     "-c /data/cfg/wpa_supplicant.conf &")) {
        printf("start wpa_supplicant failed\n");
        return -1;
    }

    if (-1 == system("dhcpcd wlan0 -t 0 &")) {
        printf("dhcpcd failed\n");
        return -1;
    }

    return 0;
}

// wpa_cli
bool  WifiCtrl::WpaCliConfigWifi()
{
    printf("start config_wifi\n");
    char cmdline[256]   = {0};
    int id              = -1;
    bool execute_result = false;
    std::string recv_buff;

    // 1. add network
    Execute("wpa_cli -iwlan0 add_network", recv_buff);
    id = atoi(recv_buff.c_str());
    if (id < 0) {
        perror("add_network failed.\n");
        return execute_result;
    }

    // 2. setNetWorkSSID
    memset(cmdline, 0, sizeof(cmdline));
    sprintf(cmdline, "wpa_cli -iwlan0 set_network %d ssid \\\"%s\\\"", id, wifi_ssid_.c_str());
    printf("%s\n", cmdline);

    Execute(cmdline, recv_buff);
    execute_result = !strncmp(recv_buff.c_str(), "OK", 2);

    if (!execute_result) {
        perror("setNetWorkSSID failed.\n");
        return execute_result;
    }

    // 3. setNetWorkPWD
    memset(cmdline, 0, sizeof(cmdline));
    sprintf(cmdline, "wpa_cli -iwlan0 set_network %d psk \\\"%s\\\"", id, wifi_password_.c_str());
    printf("%s\n", cmdline);
    Execute(cmdline, recv_buff);
    execute_result = !strncmp(recv_buff.c_str(), "OK", 2);

    if (!execute_result) {
        perror("setNetWorkPWD failed.\n");
        return execute_result;
    }

    // 4. selectNetWork
    memset(cmdline, 0, sizeof(cmdline));
    sprintf(cmdline, "wpa_cli -iwlan0 select_network %d", id);
    printf("%s\n", cmdline);
    Execute(cmdline, recv_buff);
    execute_result = !strncmp(recv_buff.c_str(), "OK", 2);

    if (!execute_result) {
        perror("setNetWorkPWD failed.\n");
        return execute_result;
    }

    return execute_result;
}

bool WifiCtrl::SaveWifiConfig()
{
    FILE *fp;
    char body[256];
    int fd;
    fp = fopen("/data/cfg/wpa_supplicant.conf", "w");

    if (fp == NULL) {
        return -1;
    }

    snprintf(body, sizeof(body), WIFI_CONFIG_FORMAT, wifi_ssid_.c_str(), wifi_password_.c_str());
    fputs(body, fp);
    fflush(fp);
    fd = fileno(fp);
    if (fd >= 0) {
        fsync(fd);
        printf("save wpa_supplicant.conf sucecees.\n");
    }
    fclose(fp);

    return 0;
}
