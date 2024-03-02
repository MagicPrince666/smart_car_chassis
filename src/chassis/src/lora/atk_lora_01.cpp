#include "atk_lora_01.h"
#include "serial.h"
#include "utils.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <random>
#include <unistd.h>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

AtkLora01::AtkLora01(std::string port, uint32_t rate)
    : LoraWan(port, rate)
{
}

AtkLora01::~AtkLora01()
{
    if (lora_thread_.joinable()) {
        lora_thread_.join();
    }
    spdlog::info("exit");
}

void AtkLora01::Init()
{
    // 创建通讯部件工厂,这一步可以优化到从lunch配置文件选择初始化不同的通讯部件工厂
    std::unique_ptr<CommFactory> factory(new SerialComm());
    // 通过工厂方法创建通讯产品
    std::shared_ptr<Communication> serial(factory->CreateCommTarget(lora_port_, baud_rate_, false));
    serial_comm_ = serial;

    // mdo_pin_ = std::make_shared<Gpio>(123, true); // 引脚 123 输出
    // aux_pin_ = std::make_shared<Gpio>(123, false); // 引脚 123 输入

    b_connect_ = LoadConfig(); // 加载配置 没有找到配置则进入对码流程

    if (b_connect_) {
        SetLoraNormal(); // 切换到正常通讯
    } else {
        EnterPairing(); //  进入配对模式
    }

    lora_thread_ = std::thread([](AtkLora01 *p_this) { p_this->LoraReader(); }, this);
}

void AtkLora01::ReadBuffer(const uint8_t *buffer, const int length)
{
    std::unique_lock<std::mutex> lck(lora_mtx_);
    int buf_size = sizeof(lora_buffer_.rx_buffer) - lora_buffer_.size;
    if (buf_size >= length) {
        memcpy(lora_buffer_.rx_buffer + lora_buffer_.size, buffer, length);
        lora_buffer_.size += length; // 更新buff长度
    } else {
        memcpy(lora_buffer_.rx_buffer + lora_buffer_.size, buffer, buf_size);
        lora_buffer_.size += buf_size; // 更新buff长度
    }
    lora_cv_.notify_all(); // 唤醒所有线程.

    return;
}

uint16_t AtkLora01::SearchHearLE(uint8_t *data, uint32_t len, uint32_t &index)
{
    uint16_t *head = nullptr;
    for (uint32_t i = 0; i < len; i++) {
        // 剩余长度大于一个包长度，说明还有协议数据包可以处理
        if ((len - i) >= 8) {
            // 一个字节一个字节的偏移，直到查找到协议头
            head = (uint16_t *)(data + i);
            // 找到协议头
            if (*head == WORK_STATION_SYNC_HEAD) {
                index = i; // 记录偏移地址
                break;
            }
        } else {
            return 0;
        }
    }
    return *head;
}

void AtkLora01::LoraReader()
{
    serial_comm_->AddCallback(std::bind(&AtkLora01::ReadBuffer, this, std::placeholders::_1, std::placeholders::_2));

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        std::unique_lock<std::mutex> lck(lora_mtx_);
        lora_cv_.wait_for(lck, std::chrono::milliseconds(100));
        if (!b_connect_) { // 进入对频模式
            if (++time_counter_ >= 10 * 60 * 10) {
                // 10分钟后退出对频模式
                break;
            } else {
                // 对码
                for (uint32_t i = 0; i < 10; i++) {
                    ws_config_.source_id = std::rand();
                    if (!ws_config_.source_id) {
                        break;
                    }
                }
                ws_config_.target_id  = 0;
                ws_config_.network_id = 0xAA;
                std::random_device rd;                      // 用于获取随机数的种子
                std::mt19937 gen(rd());                     // 使用Mersenne Twister算法生成随机数
                std::uniform_int_distribution<> dis(1, 24); // 定义随机数的范围为1-24
                ws_config_.freq_index = dis(gen);
                this->SendToWorkStation(CMD_LORA_PAIRING);
            }
        }

        for (uint32_t i = 0; lora_buffer_.size >= 8; i++) {
            uint8_t *ros_rx_buffer_ptr = lora_buffer_.rx_buffer;
            uint32_t index             = 0;
            uint16_t head              = SearchHearLE(ros_rx_buffer_ptr, lora_buffer_.size, index);
            if (head == WORK_STATION_SYNC_HEAD) {
                // 工作站消息
                lora_ws_frame_t *res_tmp;
                // 重置指针位置指向包头位置和更新长度
                uint8_t lenght = lora_buffer_.size - index;
                ros_rx_buffer_ptr += index;
                res_tmp = (lora_ws_frame_t *)ros_rx_buffer_ptr;
                if (lenght >= (res_tmp->len + 8)) {
                    lora_buffer_.size = lenght;
                } else {
                    spdlog::warn("not a full buffer size = {}", lora_buffer_.size);
                    break;
                }

                lora_ws_frame_t ws_frame = *res_tmp;
                ws_frame.check_sum       = *(uint16_t *)(res_tmp->dat + res_tmp->len);
                uint32_t sum             = ws_frame.sync_head + ws_frame.source + ws_frame.target + ws_frame.cmd + ws_frame.len;
                for (uint32_t i = 0; i < ws_frame.len; i++) {
                    sum += ws_frame.dat[i];
                }
                sum = ~sum;

                // 移除已处理的数据
                uint32_t buf_len = ws_frame.len + 8;
                RemoveOlderBuffer(ros_rx_buffer_ptr, buf_len);

                if ((sum & 0xFFFF) != ws_frame.check_sum) {
                    spdlog::warn("Ws check sum fail");
                    continue;
                }

                this->HandleWorkStationMsg(ws_frame);
            }
        }
    }
    serial_comm_->RemoveCallback();
}

void AtkLora01::EnterPairing()
{
    b_connect_                     = false;
    time_counter_                  = 0;
    lora_config_.serialBaudRate    = ATK_BAUD_RATE_9600;
    lora_config_.serialParity      = ATK_PARITY_NONE;
    if(ws_config_.freq_index > sizeof(lora_frequency_)) {
        ws_config_.freq_index = 23;
    }
    lora_config_.rfFrequency       = ws_config_.freq_index; // 设置载波频率
    lora_config_.rfSpreadingFactor = 0;
    lora_config_.rfBandwidth       = ATK_RF_BANDWIDTH_19200;
    lora_config_.mode              = ATK_MODE_STANDARD;          // 一般模式
    lora_config_.netId             = ws_config_.network_id;      // 设置网络ID
    lora_config_.nodeId            = ws_config_.source_id;       // 设置节点ID
    lora_config_.rfTransmitPower   = ATK_RF_TRANSMIT_POWER_11DB; // 对码时设置最低功率
    lora_config_.breathTime        = ATK_WAKE_TIME_1S;

    if(mdo_pin_) {
        mdo_pin_->SetGpioValue(true); // 高电平进入配置模式
    }
    serial_comm_->SetOption(115200, 0); // 切换到配置模式下的波特率
    SendToLoraModule(ATK_LORA_01_NOT_SAVE_FLASH);
    SendToLoraModule(ATK_LORA_01_UART);
    SendToLoraModule(ATK_LORA_01_ADDR);
    SendToLoraModule(ATK_LORA_01_TPOWER);
    SendToLoraModule(ATK_LORA_01_CWMODE);
    SendToLoraModule(ATK_LORA_01_TMODE);
    SendToLoraModule(ATK_LORA_01_WLRATE);
    SendToLoraModule(ATK_LORA_01_WLTIME);
    serial_comm_->SetOption(9600, 0); // 切换到透传模式下的波特率
    if(mdo_pin_) {
        mdo_pin_->SetGpioValue(false); // 低电平进入透传模式
    }
}

void AtkLora01::SetLoraNormal()
{
    lora_config_.serialBaudRate    = ATK_BAUD_RATE_9600;
    lora_config_.serialParity      = ATK_PARITY_NONE;
    if(ws_config_.freq_index > sizeof(lora_frequency_)) {
        ws_config_.freq_index = 23;
    }
    lora_config_.rfFrequency       = ws_config_.freq_index; // 设置载波频率
    lora_config_.rfSpreadingFactor = 0;
    lora_config_.rfBandwidth       = ATK_RF_BANDWIDTH_19200;
    lora_config_.mode              = ATK_MODE_STANDARD;     // 一般模式
    lora_config_.netId             = ws_config_.network_id; // 设置网络ID
    lora_config_.nodeId            = ws_config_.source_id;  // 设置节点ID
    lora_config_.rfTransmitPower   = ATK_RF_TRANSMIT_POWER_20DB;
    lora_config_.breathTime        = ATK_WAKE_TIME_1S;

    if(mdo_pin_) {
        mdo_pin_ ->SetGpioValue(true); // 高电平进入配置模式
    }
    serial_comm_->SetOption(115200, 0);
    SendToLoraModule(ATK_LORA_01_NOT_SAVE_FLASH);
    SendToLoraModule(ATK_LORA_01_UART);
    SendToLoraModule(ATK_LORA_01_ADDR);
    SendToLoraModule(ATK_LORA_01_TPOWER);
    SendToLoraModule(ATK_LORA_01_CWMODE);
    SendToLoraModule(ATK_LORA_01_TMODE);
    SendToLoraModule(ATK_LORA_01_WLRATE);
    SendToLoraModule(ATK_LORA_01_WLTIME);
    serial_comm_->SetOption(9600, 0);
    if(mdo_pin_) {
        mdo_pin_ ->SetGpioValue(false); // 低电平进入透传模式
    }
}

int AtkLora01::SendToLoraModule(int cmd)
{
    std::string cmd_str = "";
    if (atk_at_commd_.count(cmd)) {
        cmd_str = atk_at_commd_[cmd];
        switch (cmd) {
        case ATK_LORA_01_MODEL /* 查询设备型号 */:
            break;
        case ATK_LORA_01_CGMR /* 查询软件版本号 */:
            break;
        case ATK_LORA_01_UPDATE /* 查询设备是否处于固件升级模式 */:
            break;
        case ATK_LORA_01_ATE1 /* 打开指令回显 */:
            break;
        case ATK_LORA_01_ATE0 /* 关闭指令回显 */:
            break;
        case ATK_LORA_01_RESET /* 模块软复位 */:
            break;
        case ATK_LORA_01_SAVE_FLASH /* 模块参数保存设置 */:
            break;
        case ATK_LORA_01_NOT_SAVE_FLASH /* 模块参数不保存设置 */:
            break;
        case ATK_LORA_01_DEFAULT /* 模块参数恢复默认设置 */:
            break;
        case ATK_LORA_01_ADDR /* 查询设备配置地址范围 */:
            cmd_str += "=" + std::to_string((uint8_t)(lora_config_.nodeId >> 8)) + "," + std::to_string((uint8_t)(lora_config_.nodeId));
            break;
        case ATK_LORA_01_TPOWER /* 查询设备配置的发射功率范围 */:
            cmd_str += "=" + std::to_string(lora_config_.rfTransmitPower);
            break;
        case ATK_LORA_01_CWMODE /* 查询工作模式配置范围 */:
            cmd_str += "=" + std::to_string(lora_config_.mode);
            break;
        case ATK_LORA_01_TMODE /* 查询设备发送状态配置范围 */:
            cmd_str += "=" + std::to_string(0);
            break;
        case ATK_LORA_01_WLRATE /* 查询设备信道和无线速率设置范围 */:
            cmd_str += "=" + std::to_string(lora_config_.rfFrequency) + "," + std::to_string(lora_config_.rfBandwidth);
            break;
        case ATK_LORA_01_WLTIME /* 查询设备休眠/唤醒时间设置范围 */:
            cmd_str += "=" + std::to_string(lora_config_.breathTime);
            break;
        case ATK_LORA_01_UART /* 查询设备串口波特率和数据奇偶校验位设置范围 */:
            cmd_str += "=" + std::to_string(lora_config_.serialBaudRate) + "," + std::to_string(lora_config_.serialParity);
            break;

        default:
            break;
        }
    }
    cmd_str += "\r\n";

    return serial_comm_->SendBuffer((const uint8_t *)cmd_str.c_str(), cmd_str.size());
}

void AtkLora01::RemoveOlderBuffer(uint8_t *ros_rx_buffer_ptr, uint32_t buf_len)
{
    spdlog::info("remove buff = {}", Utils::Bytes2String(ros_rx_buffer_ptr, buf_len).c_str());
    lora_buffer_.size -= buf_len;
    uint8_t buffer[sizeof(lora_buffer_.rx_buffer)];
    // 剩余未处理数据拷贝到临时变量
    memcpy(buffer, ros_rx_buffer_ptr + buf_len, lora_buffer_.size);
    // 覆盖掉原来的buff
    memcpy(lora_buffer_.rx_buffer, buffer, lora_buffer_.size);
}

