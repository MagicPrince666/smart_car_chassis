/**
 * @file yl-800t.h
 * @author Leo Huang (846863428@qq.com)
 * @brief yl-800t AT命令设置
 * @version 0.1
 * @date 2023-11-16
 * @copyright 个人版权所有 Copyright (c) 2023
 */

#ifndef _YL_800T_H
#define _YL_800T_H

#include "comm_factory.h"
#include "gpio.h"
#include "station_comm.h"
#include <inttypes.h>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#define PARAMETER_INDEX 8

#define COMMAND_WRITE_PARAMETERS 0x01
#define COMMAND_READ_PARAMETERS 0x02
#define PARAMETER_LENGTH_READ_WRITE 14

#define COMMAND_READ_SIGNAL_STRENGTH 0x06
#define PARAMETER_LENGTH_SIGNAL_STRENGTH 2

#define LORA_AT_CMD_SYNC_HEAD 0xAFAF

typedef enum {
    YL_800T_BAUD_RATE_1200   = 0x01,
    YL_800T_BAUD_RATE_2400   = 0x02,
    YL_800T_BAUD_RATE_4800   = 0x03,
    YL_800T_BAUD_RATE_9600   = 0x04,
    YL_800T_BAUD_RATE_19200  = 0x05,
    YL_800T_BAUD_RATE_38400  = 0x06,
    YL_800T_BAUD_RATE_57600  = 0x07,
    YL_800T_BAUD_RATE_115200 = 0x08
} YL800TSerialBaudRate;

typedef enum {
    YL_800T_PARITY_NONE = 0x00,
    YL_800T_PARITY_ODD  = 0x01,
    YL_800T_PARITY_EVEN = 0x02
} YL800TSerialParity;

typedef enum {
    YL_800T_RF_SPREADING_FACTOR_128  = 0x07,
    YL_800T_RF_SPREADING_FACTOR_256  = 0x08,
    YL_800T_RF_SPREADING_FACTOR_512  = 0x09,
    YL_800T_RF_SPREADING_FACTOR_1024 = 0x0A,
    YL_800T_RF_SPREADING_FACTOR_2048 = 0x0B,
    YL_800T_RF_SPREADING_FACTOR_4096 = 0x0C
} YL800TRFSpreadingFactor;

typedef enum {
    YL_800T_RF_MODE_STANDARD = 0x00,
    YL_800T_RF_MODE_CENTRAL  = 0x01,
    YL_800T_RF_MODE_NODE     = 0x02
} YL800TMode;

typedef enum {
    YL_800T_RF_BANDWIDTH_62_5K = 0x06,
    YL_800T_RF_BANDWIDTH_125K  = 0x07,
    YL_800T_RF_BANDWIDTH_250K  = 0x08,
    YL_800T_RF_BANDWIDTH_500K  = 0x09
} YL800TRFBandwidth;

typedef enum {
    YL_800T_RF_TRANSMIT_POWER_4DB  = 0x01,
    YL_800T_RF_TRANSMIT_POWER_7DB  = 0x02,
    YL_800T_RF_TRANSMIT_POWER_10DB = 0x03,
    YL_800T_RF_TRANSMIT_POWER_13DB = 0x04,
    YL_800T_RF_TRANSMIT_POWER_14DB = 0x05,
    YL_800T_RF_TRANSMIT_POWER_17DB = 0x06,
    YL_800T_RF_TRANSMIT_POWER_20DB = 0x07,
} YL800TRFTransmitPower;

typedef enum {
    YL_800T_BREATH_CYCLE_2S  = 0x00,
    YL_800T_BREATH_CYCLE_4S  = 0x01,
    YL_800T_BREATH_CYCLE_6S  = 0x02,
    YL_800T_BREATH_CYCLE_8S  = 0x03,
    YL_800T_BREATH_CYCLE_10S = 0x04
} YL800TBreathCycle;

typedef enum {
    YL_800T_BREATH_TIME_2MS  = 0x00,
    YL_800T_BREATH_TIME_4MS  = 0x01,
    YL_800T_BREATH_TIME_8MS  = 0x02,
    YL_800T_BREATH_TIME_16MS = 0x03,
    YL_800T_BREATH_TIME_32MS = 0x04,
    YL_800T_BREATH_TIME_64MS = 0x05
} YL800TBreathTime;

typedef enum {
    YL_800T_WRITE_PARAMETER               = 0x01, // 写参数
    YL_800T_READ_PARAMETER                = 0x02, // 读参数
    YL_800T_ENTER_NORMAL                  = 0x03, // 进入标准模式
    YL_800T_ENTER_CENTER                  = 0x04, // 进入中心模式
    YL_800T_ENTER_NODE                    = 0x05, // 进入节点模式
    YL_800T_LAST_SIGNED                   = 0x06, // 读取上一包接收强度
    YL_800T_SET_UART                      = 0x09, // 写串口参数
    YL_800T_GET_UART                      = 0x0A, // 读串口参数
    YL_800T_SET_FREQUECY                  = 0x0B, // 写载波频率
    YL_800T_GET_FREQUECY                  = 0x0C, // 读载波频率
    YL_800T_SET_SPREAD_SPECTRUM_FACTOR    = 0x0D, // 写扩频因子
    YL_800T_GET_SPREAD_SPECTRUM_FACTOR    = 0x0E, // 读扩频因子
    YL_800T_SET_SPREAD_SPECTRUM_BANDWIDTH = 0x0F, // 写扩频带宽
    YL_800T_GET_SPREAD_SPECTRUM_BANDWIDTH = 0x10, // 读扩频带宽
    YL_800T_SET_WORK_MODE                 = 0x11, // 写工作模式
    YL_800T_GET_WORK_MODE                 = 0x12, // 读工作模式
    YL_800T_SET_NODE_ID                   = 0x13, // 写节点ID
    YL_800T_GET_NODE_ID                   = 0x14, // 读节点ID
    YL_800T_SET_NETWORK_ID                = 0x15, // 写网络ID
    YL_800T_GET_NETWORK_ID                = 0x16, // 读网络ID
    YL_800T_SET_TRANSMISSION_POWER        = 0x17, // 写发射功率
    YL_800T_GET_TRANSMISSION_POWER        = 0x18, // 读发射功率
    YL_800T_SET_BREATHING_CYCLE           = 0x19, // 写呼吸周期
    YL_800T_GET_BREATHING_CYCLE           = 0x1A, // 读呼吸周期
    YL_800T_SET_BREATHING_TIME            = 0x1B, // 写呼吸时间
    YL_800T_GET_BREATHING_TIME            = 0x1C, // 读呼吸时间
    YL_800T_SET_OTA_MODE                  = 0x1F, // 设置OTA模式
} YL800TCmdCode;

#pragma pack(push)
#pragma pack(1)
// lora AT 命令消息
typedef struct
{
    uint16_t sync_head                       = 0;   /* 同步帧头 0xAFAF*/
    uint16_t id                              = 0;   /* 帧ID */
    uint8_t head                             = 0;   /* 头 */
    uint8_t rwc                              = 0;   /* 读写码 */
    uint8_t cmd                              = 0;   /* 功能码 */
    uint8_t len                              = 0;   /* 数据长度 */
    uint8_t dat[PARAMETER_LENGTH_READ_WRITE] = {0}; /* 数据 */
    uint8_t check_sum                        = 0;   /* 校验和 */
    uint16_t end_code                        = 0;   /* 结束码 固定为0x0D0A */
} lora_at_frame_t;

#pragma pack(pop)
class YL800t : public LoraWan
{
public:
    YL800t(std::string port, uint32_t rate);
    ~YL800t();

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
     * @brief 异步处理指令返回
     * @param cmd
     * @return int
     */
    int AsyncSendToLoraCmd(int cmd);

    /**
     * @brief 设置正常通讯
     */
    void SetLoraNormal();

private:
    std::thread lora_thread_;
    LoraAllParameters lora_config_; // lora模块参数
    std::shared_ptr<Gpio> en_pin_;  // 低电平工作 高电平休眠
    std::shared_ptr<Gpio> aux_pin_; // 模块状态指示灯
    std::shared_ptr<Gpio> set_pin_; // 低电平进入快速通道，中心节点模式下有效

    uint32_t time_counter_;
    uint32_t lora_frequency_[24] = {
        440200000,
        441700000,
        443200000,
        444700000,
        446500000,
        451600000,
        453100000,
        454600000,
        456100000,
        457600000,
        459100000,
        460600000,
        462100000,
        463600000,
        466100000,
        468100000,
        469600000,
        471100000,
        472600000,
        474100000,
        475600000,
        477100000,
        478600000,
        483500000,
    };

    uint32_t serial_baud_rate_[10]                          = {0, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    std::unordered_map<uint8_t, int> spreading_factor_map_= {
        {YL_800T_RF_SPREADING_FACTOR_128, 128},
        {YL_800T_RF_SPREADING_FACTOR_256, 256},
        {YL_800T_RF_SPREADING_FACTOR_512, 512},
        {YL_800T_RF_SPREADING_FACTOR_1024, 1024},
        {YL_800T_RF_SPREADING_FACTOR_2048, 2048},
        {YL_800T_RF_SPREADING_FACTOR_4096, 4096},
    };

    std::unordered_map<uint8_t, int> bandwidth_map_ = {
        {YL_800T_RF_BANDWIDTH_62_5K, 62500},
        {YL_800T_RF_BANDWIDTH_125K, 125000},
        {YL_800T_RF_BANDWIDTH_250K, 250000},
        {YL_800T_RF_BANDWIDTH_500K, 500000},
    };
    
    std::unordered_map<uint8_t, int> transmit_power_map_ = {
        {YL_800T_RF_TRANSMIT_POWER_4DB, 4},
        {YL_800T_RF_TRANSMIT_POWER_7DB, 7},
        {YL_800T_RF_TRANSMIT_POWER_10DB, 10},
        {YL_800T_RF_TRANSMIT_POWER_13DB, 13},
        {YL_800T_RF_TRANSMIT_POWER_14DB, 14},
        {YL_800T_RF_TRANSMIT_POWER_17DB, 17},
        {YL_800T_RF_TRANSMIT_POWER_20DB, 20},
    };

    std::unordered_map<uint8_t, std::string> serial_parity_map_ = {
        {YL_800T_PARITY_NONE, "NONE"},
        {YL_800T_PARITY_ODD, "ODD"},
        {YL_800T_PARITY_EVEN, "EVEN"},
    };

    std::unordered_map<uint8_t, std::string> lora_mode_map_ = {
        {YL_800T_RF_MODE_STANDARD, "STANDARD"},
        {YL_800T_RF_MODE_CENTRAL, "CENTRAL"},
        {YL_800T_RF_MODE_NODE, "NODE"},
    };

    struct
    {
        uint8_t rx_buffer[255];
        uint8_t size; // buf长度
    } lora_buffer_;

    typedef struct
    {
        int cmd;       // 命令字
        bool has_send; // 是否已发送
    } CmdMsg;

    std::list<CmdMsg> cmd_queue_; // 命令队列
    std::unordered_map<int, bool> cmd_has_result_map_;

    /**
     * @brief 查找AT 命令头，模块参数包
     * @param data
     * @param len
     * @param index
     * @return uint16_t 返回头部信息
     */
    uint16_t SearchHearLE(uint8_t *data, uint32_t len, uint32_t &index);

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
     * @brief 处理AT命令
     * @param lora_frame
     */
    void HandleLoraMsg(lora_at_frame_t lora_frame);

    /**
     * @brief lora模块数据校验
     * @param message
     * @param length
     * @return uint8_t
     */
    uint8_t checksum(const uint8_t *message, uint8_t length);

    /**
     * @brief 移除旧数据包
     * @param length
     */
    void RemoveOlderBuffer(uint8_t *ros_rx_buffer_ptr, uint32_t buf_len);

    /**
     * @brief 处理指令发送和返回
     * @return 处理完成 true 未完成 false
     */
    bool HandleCmd();
};

#endif
