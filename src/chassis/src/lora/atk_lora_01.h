/**
 * @file atk_lora_01.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 正点原子atk lora 01
 * @version 0.1
 * @date 2023-11-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __ATK_LORA_01_H__
#define __ATK_LORA_01_H__

#include "comm_factory.h"
#include "station_comm.h"
#include "gpio.h"
#include <condition_variable>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

typedef enum {
    ATK_BAUD_RATE_1200   = 0x00,
    ATK_BAUD_RATE_2400   = 0x01,
    ATK_BAUD_RATE_4800   = 0x02,
    ATK_BAUD_RATE_9600   = 0x03,
    ATK_BAUD_RATE_19200  = 0x04,
    ATK_BAUD_RATE_38400  = 0x05,
    ATK_BAUD_RATE_57600  = 0x06,
    ATK_BAUD_RATE_115200 = 0x07
} ATKSerialBaudRate;

typedef enum {
    ATK_PARITY_NONE = 0x00,
    ATK_PARITY_ODD  = 0x01,
    ATK_PARITY_EVEN = 0x02
} ATKSerialParity;

typedef enum {
    ATK_MODE_STANDARD  = 0x00,
    ATK_MODE_WAKE_UP   = 0x01,
    ATK_MODE_LOW_POWER = 0x02,
    ATK_MODE_SIGNED    = 0x03
} ATKMode;

typedef enum {
    ATK_RF_TRANSMIT_POWER_11DB = 0x00,
    ATK_RF_TRANSMIT_POWER_14DB = 0x01,
    ATK_RF_TRANSMIT_POWER_17DB = 0x02,
    ATK_RF_TRANSMIT_POWER_20DB = 0x03,
} ATKRFTransmitPower;

typedef enum {
    ATK_RF_BANDWIDTH_300   = 0x00,
    ATK_RF_BANDWIDTH_1200  = 0x01,
    ATK_RF_BANDWIDTH_24000 = 0x02,
    ATK_RF_BANDWIDTH_48000 = 0x03,
    ATK_RF_BANDWIDTH_96000 = 0x04,
    ATK_RF_BANDWIDTH_19200 = 0x05,
} ATKRFBandwidth;

typedef enum {
    ATK_WAKE_TIME_1S = 0x00,
    ATK_WAKE_TIME_2S = 0x01,
} ATKBreathTime;

class AtkLora01 : public LoraWan
{
public:
    AtkLora01(std::string port, uint32_t rate);
    virtual ~AtkLora01();

    void Init();

    /**
     * @brief 进入对频模式
     */
    void EnterPairing();

    /**
     * @brief 传输lora控制指令
     * @param cmd
     * @return int
     */
    int SendToLoraModule(int cmd);

    /**
     * @brief 设置正常通讯
     */
    void SetLoraNormal();

private:
    std::thread lora_thread_;
    LoraAllParameters lora_config_;              // lora模块参数
    std::shared_ptr<Gpio> mdo_pin_; // 配置脚
    std::shared_ptr<Gpio> aux_pin_; // 模块状态指示灯

    uint32_t time_counter_;
    uint32_t lora_frequency_[32] = {
        410000000, 411000000, 412000000, 413000000, 414000000, 415000000, 416000000, 417000000, 418000000, 419000000,
        420000000, 421000000, 422000000, 423000000, 424000000, 425000000, 426000000, 427000000, 428000000, 429000000,
        430000000, 431000000, 432000000, 433000000, 434000000, 435000000, 436000000, 437000000, 438000000, 439000000,
        440000000, 441000000};

    struct
    {
        uint8_t rx_buffer[255];
        uint8_t size; // buf长度
    } lora_buffer_;

    typedef enum {
        ATK_LORA_01_MODEL          = 0x01, // 查询设备型号
        ATK_LORA_01_CGMR           = 0x02, // 查询软件版本号
        ATK_LORA_01_UPDATE         = 0x03, // 查询设备是否处于固件升级模式
        ATK_LORA_01_ATE1           = 0x04, // 打开指令回显
        ATK_LORA_01_ATE0           = 0x05, // 关闭指令回显
        ATK_LORA_01_RESET          = 0x06, // 模块软复位
        ATK_LORA_01_SAVE_FLASH     = 0x09, // 模块参数保存设置
        ATK_LORA_01_NOT_SAVE_FLASH = 0x0A, // 模块参数不保存设置
        ATK_LORA_01_DEFAULT        = 0x0B, // 模块参数恢复默认设置
        ATK_LORA_01_ADDR           = 0x0C, // 查询设备配置地址范围
        ATK_LORA_01_TPOWER         = 0x0D, // 查询设备配置的发射功率范围 0：11dbm 1：14dbm 2：17dbm 3：20dbm（默认）
        ATK_LORA_01_CWMODE         = 0x0E, // 查询工作模式配置范围 0：一般模式（默认） 1：唤醒模式 2：省电模式 3：信号强度模式
        ATK_LORA_01_TMODE          = 0x0F, // 查询设备发送状态配置范围 0：透明传输（默认） 1：定向传输
        ATK_LORA_01_WLRATE         = 0x10, // 查询设备信道和无线速率设置范围 0：0.3kbps 1：1.2kbps 2：2.4kbps 3：4.8kbps 4：9.6kbps 5：19.2kbps（默认）
        ATK_LORA_01_WLTIME         = 0x11, // 查询设备休眠/唤醒时间设置范围 0：1 秒 1：2 秒
        ATK_LORA_01_UART           = 0x12, // 查询设备串口波特率和数据奇偶校验位设置范围 0：1200 1：2400 2：4800 3：9600（默认） 4：19200 5：38400 6：57600 7：115200
                                           // 数据奇偶校验位配置“0-2”，共 3 种。 0：无校验（默认） 1：偶校验 2：奇校验

    } ATKCmdCode;

    std::unordered_map<int, std::string> atk_at_commd_ = {
        {ATK_LORA_01_MODEL, "AT+MODEL?"},
        {ATK_LORA_01_CGMR, "AT+CGMR?"},
        {ATK_LORA_01_UPDATE, "AT+UPDATE"},
        {ATK_LORA_01_ATE1, "ATE1"},
        {ATK_LORA_01_ATE0, "ATE0"},
        {ATK_LORA_01_RESET, "AT+RESET"},
        {ATK_LORA_01_SAVE_FLASH, "AT+FLASH=1"},
        {ATK_LORA_01_NOT_SAVE_FLASH, "AT+FLASH=0"},
        {ATK_LORA_01_DEFAULT, "AT+DEFAULT"},
        {ATK_LORA_01_ADDR, "AT+ADDR"}, // 地址范围为 0-65536
        {ATK_LORA_01_TPOWER, "AT+TPOWER"},
        {ATK_LORA_01_CWMODE, "AT+CWMODE"},
        {ATK_LORA_01_TMODE, "AT+TMODE"},
        {ATK_LORA_01_WLRATE, "AT+WLRATE"},
        {ATK_LORA_01_WLTIME, "AT+WLTIME"},
        {ATK_LORA_01_UART, "AT+UART"},
    };

    /**
     * @brief 串口接收回调
     * @param buffer
     * @param length
     */
    void ReadBuffer(const uint8_t *buffer, const int length);

    /**
     * @brief lora数据包解析线程
     */
    void LoraReader();

    /**
     * @brief 查找模块参数包
     * @param data
     * @param len
     * @param index
     * @return uint16_t 返回头部信息
     */
    uint16_t SearchHearLE(uint8_t *data, uint32_t len, uint32_t &index);

    /**
     * @brief 移除旧数据包
     * @param length 
     */
    void RemoveOlderBuffer(uint8_t *ros_rx_buffer_ptr, uint32_t buf_len);
};
#endif
