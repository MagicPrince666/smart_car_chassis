#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "pwm.h"

// 辅助函数：尝试写入PWM属性，支持权限重试
int WritePwmWithRetry(const char* path, const char* value) {
    const int maxRetries = 5;
    const int retryDelayMs = 10;
    
    for (int retry = 0; retry < maxRetries; retry++) {
        FILE* fp = fopen(path, "w");
        if (fp == nullptr) {
            if (errno == EACCES || errno == EPERM) {
                // 权限不足，等待后重试
                usleep(retryDelayMs * 1000);  // 转换为微秒
                continue;
            }
            printf("open %s error: %s\n", path, strerror(errno));
            return -1;
        }
        
        int written = fprintf(fp, "%s", value);
        fclose(fp);
        
        if (written < 0) {
            printf("write %s error: %s\n", path, strerror(errno));
            return -1;
        }
        return 0;  // 成功写入
    }
    
    printf("Failed to write %s after %d retries: permission denied\n", path, maxRetries);
    return -1;
}

// 重载：用于写入数值
int WritePwmWithRetry(const char* path, int value) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", value);
    return WritePwmWithRetry(path, buf);
}

Pwm::Pwm(PwmPram parm) {
    pwm_chip_ = parm.chip;
    pwm_channel_ = parm.channel;
    SetupPwm();
    PwmPeriod(parm.period);
    PwmDutyCycle(parm.dutycycle);
    PwmPolarity(parm.polarity);
    PwmEnable(true);
}

Pwm::~Pwm() {
    // PwmDutyCycle(0);
    // PwmEnable(false);
    // UnExportPwm();
}

int Pwm::SetupPwm()
{
    char setpin[64] = {0};
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/pwm/pwmchip%d/export", pwm_chip_);
    setpin[len] = 0;
    FILE *set_export = fopen(setpin, "w");
    if (set_export == nullptr) {
        std::cout << "Can\'t open " << setpin << std::endl;
        return -1;
    } else {
        std::cout << "Open " << setpin << std::endl;
        fprintf(set_export, "%d", pwm_channel_);
    }
    assert(set_export != nullptr);
    fclose(set_export);
    return 0;
}

int Pwm::UnExportPwm()
{
    char setpin[64] = {0};
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/pwm/pwmchip%d/unexport", pwm_chip_);
    setpin[len] = 0;
    FILE *set_export = fopen(setpin, "w");
    if (set_export == nullptr) {
        std::cout << "Can\'t open " << setpin << std::endl;
    } else {
        fprintf(set_export, "%d", pwm_channel_);
    }
    fclose(set_export);
    return 0;
}

int Pwm::PwmEnable(bool enable)
{
    char setpin[64] = {0};
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/pwm/pwmchip%d/pwm%d/enable", pwm_chip_, pwm_channel_);
    setpin[len] = 0;
    
    std::cout << "Open " << setpin << std::endl;
    
    const char* value = enable ? "1" : "0";
    return WritePwmWithRetry(setpin, value);
}

int Pwm::PwmPolarity(bool value)
{
    char setpin[64] = {0};
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/pwm/pwmchip%d/pwm%d/polarity", pwm_chip_, pwm_channel_);
    setpin[len] = 0;
    
    std::cout << "Open " << setpin << std::endl;
    
    const char* polarity = value ? "normal" : "inversed";
    return WritePwmWithRetry(setpin, polarity);
}

int Pwm::PwmPeriod(uint32_t value)
{
    char setpin[64] = {0};
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/pwm/pwmchip%d/pwm%d/period", pwm_chip_, pwm_channel_);
    setpin[len] = 0;
    
    return WritePwmWithRetry(setpin, (int)value);
}

int Pwm::PwmDutyCycle(uint32_t value)
{
    char setpin[64] = {0};
    int len = snprintf(setpin, sizeof(setpin) - 1, "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", pwm_chip_, pwm_channel_);
    setpin[len] = 0;
    
    return WritePwmWithRetry(setpin, (int)value);
}
