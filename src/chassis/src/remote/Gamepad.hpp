/**
 * @file Gamepad.hpp
 * @author 黄李全 (846863428@qq.com)
 * @brief 通用joystick数据获取
 * @version 0.1
 * @date 2023-01-08
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */

#include "RemoteFactory.h"
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

class Gamepad : public RemoteProduct
{
public:
    Gamepad(RemoteConfig_t config, bool debug = false);
    ~Gamepad();

    int Init();

    virtual bool Request(struct RemoteState &data);

private:
    int ReadJoystick();

    typedef enum {
        JOYSTICK_SKY    = 0x00,
        JOYSTICK_DR     = 0x01,
        JOYSTICK_PS4    = 0x02,
        JOYSTICK_PS4_WIRELESS = 0x03,
        JOYSTICK_UNKNOW
    } JoyType;
    typedef enum { // 通道映射
        ADS_LX = 0x00, // 左摇杆x
        ADS_LY = 0x01, // 左摇杆y
        ADS_RX = 0x02, // 右摇杆x
        ADS_RY = 0x03, // 右摇杆yy
        ADS_FB = 0x04, // 前后按钮
        ADS_LR = 0x05, // 左右按钮
        ADS_R2 = 0x06, // 右上方按钮
        ADS_L2 = 0x07, // 左上方按钮
        ADS_END
    } JoyChanel;

    int js_fd_;
    struct RemoteState rc_data_;
    JoyType joy_type_;
    std::mutex rc_data_lock_;
    std::unordered_map<int, JoyChanel> joystick_chanel_map_;
};

// 生产game pad 遥控工厂
class GamePadRemote : public RemoteFactory
{
public:
    RemoteProduct *CreateRemoteProduct(RemoteConfig_t config, bool debug)
    {
        return new Gamepad(config, debug);
    }
};
