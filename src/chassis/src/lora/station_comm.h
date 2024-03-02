/**
 * @file station_comm.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 水电站通信父类
 * @version 0.1
 * @date 2023-11-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __LORA_MODULE_H__
#define __LORA_MODULE_H__

#include <iostream>
#include <mutex>
#include <condition_variable>
#include "comm_factory.h"
#include "json/json.h"
#include <spdlog/spdlog.h>

#define WS_MAX_LENGTH_READ_WRITE 4 // 水电站通信最大数据长度
#define WORK_STATION_SYNC_HEAD 0x5AA5

typedef enum {
    READ_WES_HW_VERSION = 0x01, // 读取水电站硬件版本号
    READ_WES_FW_VERSION = 0x02, // 读取水电站软件版本号
    REPORT_WES_STATE    = 0x03, // 反馈水电站状态
    REPORT_WES_FAULT    = 0x04, // 反馈水电站故障码
    CMD_REQUEST_CHG_VOL = 0x20, // SC80主机向WES请求工作电压
    CMD_REQUEST_CHG_CUR = 0x21, // SC80主机向WES请求工作电流
    CMD_LORA_PAIRING    = 0xFA, // SC80主机发出对频对码指令
    CMD_LORA_PAIR_ACK   = 0xFB, // 水电站回应对码对频指令
    CMD_LORA_PAIR_OK    = 0xFC, // SC80主机发送对频对码成功
} WESCmdCode;

typedef struct {
    uint8_t serialBaudRate;
    uint8_t serialParity;
    uint32_t rfFrequency;
    uint8_t rfSpreadingFactor;
    uint8_t rfBandwidth;
    uint8_t mode;
    uint16_t nodeId;
    uint8_t netId;
    uint8_t rfTransmitPower;
    uint8_t breathCycle;
    uint8_t breathTime;
} LoraAllParameters;

typedef struct {
    uint8_t source_id  = 0;     // 源ID
    uint8_t target_id  = 0;     // 目标ID
    uint8_t network_id = 0;     // 网络ID
    uint8_t freq_index = 0;     // 频率索引
    uint16_t charge_vol       = 4000;  // 充电电压
    uint16_t charge_cur       = 10000; // 充电电流

    Json::Value ToJson() {
        Json::Value config;
        config["source_id"] = source_id;
        config["target_id"] = target_id;
        config["network_id"] = network_id;
        config["freq_index"] = freq_index;
        config["charge_vol"] = charge_vol;
        config["charge_cur"] = charge_cur;
        return config;
    }
} WorkStationAllParameters;

#pragma pack(push)
#pragma pack(1)
// 水电站消息定义
typedef struct
{
    uint16_t sync_head                    = 0;   /* 同步帧头 0xA55A*/
    uint8_t source                        = 0;   /* 源ID */
    uint8_t target                        = 0;   /* 目标ID */
    uint8_t cmd                           = 0;   /* 功能码 */
    uint8_t len                           = 0;   /* 数据长度 */
    uint8_t dat[WS_MAX_LENGTH_READ_WRITE] = {0}; /* 数据 */
    uint16_t check_sum                     = 0;   /* 校验和 */
} lora_ws_frame_t;
#pragma pack(pop)

class LoraWan
{
public:
    LoraWan(std::string port, uint32_t rate);
    virtual ~LoraWan();

    virtual void Init() = 0;

    /**
     * @brief 进入对频模式
     */
    virtual void EnterPairing() = 0;

    /**
     * @brief 设置lora正常通讯
     */
    virtual void SetLoraNormal() = 0;

    /**
     * @brief 传输lora控制指令
     * @param cmd
     * @return int
     */
    virtual int SendToLoraModule(int cmd) = 0;

    /**
     * @brief 设置充电电压
     * @param vol 0.01V
     */
    void SetWSChargeVol(uint16_t vol)
    {
        ws_config_.charge_vol = vol;
    }

    /**
     * @brief 设置充电电流
     * @param cur 0.01A
     */
    void SetWSChargeCur(uint16_t cur)
    {
        ws_config_.charge_cur = cur;
    }

    /**
     * @brief 传输水电站指令
     * @param cmd
     * @return int
     */
    int SendToWorkStation(WESCmdCode cmd);

    /**
     * @brief 处理水电站消息
     * @param ws_frame 
     */
    void HandleWorkStationMsg(lora_ws_frame_t ws_frame);

    /**
     * @brief 保存配置文件
     * @return true 
     * @return false 
     */
    bool SaveConfig();

    /**
     * @brief 加载配置文件
     * @return true 
     * @return false 
     */
    bool LoadConfig();

protected:
    std::string lora_port_;
    uint32_t baud_rate_;
    std::condition_variable lora_cv_; // 全局条件变量
    std::mutex lora_mtx_;             // 全局互斥锁.
    WorkStationAllParameters ws_config_;       // 水电站参数
    std::shared_ptr<Communication> serial_comm_; // 通讯端口
    bool b_connect_;
    bool b_wait_ack_;

    /**
     * @brief 水电站数据校验
     * @param message
     * @param length
     * @return uint16_t
     */
    uint16_t wschecksum(const uint8_t *message, uint8_t length);

private:
};

#endif
