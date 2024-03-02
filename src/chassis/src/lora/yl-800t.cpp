#include "yl-800t.h"
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

YL800t::YL800t(std::string port, uint32_t rate)
    : LoraWan(port, rate), time_counter_(0)
{
}

YL800t::~YL800t()
{
    if (lora_thread_.joinable()) {
        lora_thread_.join();
    }
    spdlog::info("exit");
}

void YL800t::Init()
{
    // 创建通讯部件工厂,这一步可以优化到从lunch配置文件选择初始化不同的通讯部件工厂
    std::unique_ptr<CommFactory> factory(new SerialComm());
    // 通过工厂方法创建通讯产品
    std::shared_ptr<Communication> serial(factory->CreateCommTarget(lora_port_, baud_rate_, false));
    serial_comm_ = serial;

    // en_pin_ = std::make_shared<Gpio>(123, true); // 引脚 123 输出
    // aux_pin_ = std::make_shared<Gpio>(123, false); // 引脚 123 输入
    // set_pin_ = std::make_shared<Gpio>(123, true); // 引脚 123 输出
    // en_pin_->SetGpioValue(false); // 低电平进入工作模式

    b_connect_ = LoadConfig(); // 加载配置 没有找到配置则进入对码流程

    if (b_connect_) {
        spdlog::info("enter in normal");
        SetLoraNormal(); // 切换到正常通讯
    } else {
        spdlog::info("enter in pairing");
        for (uint32_t i = 0; i < 10; i++) {
            ws_config_.source_id = std::rand();
            if (!ws_config_.source_id) {
                break;
            }
        }
        EnterPairing(); //  进入配对模式
    }

    lora_thread_ = std::thread([](YL800t *p_this) { p_this->LoraReader(); }, this);
}

void YL800t::ReadBuffer(const uint8_t *buffer, const int length)
{
    std::unique_lock<std::mutex> lck(lora_mtx_);
    int buf_size = sizeof(lora_buffer_.rx_buffer) - lora_buffer_.size;
    if (buf_size < length) {
        lora_buffer_.size = 0;
    }
    memcpy(lora_buffer_.rx_buffer + lora_buffer_.size, buffer, length);
    lora_buffer_.size += length; // 更新buff长度
    lora_cv_.notify_all();       // 唤醒所有线程.

    return;
}

uint16_t YL800t::SearchHearLE(uint8_t *data, uint32_t len, uint32_t &index)
{
    uint16_t head = 0;
    for (uint32_t i = 0; i < len; i++) {
        // 剩余长度大于一个包长度，说明还有协议数据包可以处理
        if ((len - i) >= 8) {
            // 一个字节一个字节的偏移，直到查找到协议头
            head = *(uint16_t *)(data + i);
            // 找到协议头
            if (head == LORA_AT_CMD_SYNC_HEAD || head == WORK_STATION_SYNC_HEAD) {
                index = i; // 记录偏移地址
                break;
            }
        } else {
            return 0;
        }
    }
    return head;
}

void YL800t::LoraReader()
{
    serial_comm_->AddCallback(std::bind(&YL800t::ReadBuffer, this, std::placeholders::_1, std::placeholders::_2));

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        std::unique_lock<std::mutex> lck(lora_mtx_);
        lora_cv_.wait_for(lck, std::chrono::milliseconds(500));

        HandleCmd();

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
                    spdlog::warn("not a full buffer size = %d", lora_buffer_.size);
                    break;
                }

                lora_ws_frame_t ws_frame = *res_tmp;
                ws_frame.check_sum       = *(uint16_t *)(res_tmp->dat + res_tmp->len);
                uint16_t sum             = this->wschecksum(ros_rx_buffer_ptr, ws_frame.len + 6);

                // 移除已处理的数据
                uint32_t buf_len = ws_frame.len + 8;
                RemoveOlderBuffer(ros_rx_buffer_ptr, buf_len);

                if (sum != ws_frame.check_sum) {
                    spdlog::warn( "Ws check sum fail sum = {}, check_sum = {}", sum, ws_frame.check_sum);
                    continue;
                }

                this->HandleWorkStationMsg(ws_frame);

            } else if (head == LORA_AT_CMD_SYNC_HEAD) {
                // AT 命令返回
                lora_at_frame_t *res_tmp;
                // 重置指针位置指向包头位置和更新长度
                uint8_t lenght = lora_buffer_.size - index;
                ros_rx_buffer_ptr += index;
                res_tmp = (lora_at_frame_t *)ros_rx_buffer_ptr;
                if (lenght >= (res_tmp->len + 11)) {
                    lora_buffer_.size = lenght;
                } else {
                    spdlog::info("not a full buffer size = {}", lora_buffer_.size);
                    break;
                }

                lora_at_frame_t lora_frame = *res_tmp;
                lora_frame.check_sum       = res_tmp->dat[res_tmp->len];
                lora_frame.end_code        = *(uint16_t *)(res_tmp->dat + res_tmp->len + 1);

                uint8_t sum = checksum(ros_rx_buffer_ptr, lora_frame.len + 8);

                // 移除已处理的数据
                uint32_t buf_len = lora_frame.len + 11;
                RemoveOlderBuffer(ros_rx_buffer_ptr, buf_len);

                if (sum != lora_frame.check_sum) {
                    spdlog::warn("AT CMD check sum fail");
                    continue;
                }

                HandleLoraMsg(lora_frame);
            } else {
                break;
            }
        }
    }
    serial_comm_->RemoveCallback();
    spdlog::info("YL800t pthread quilt");
}

void YL800t::RemoveOlderBuffer(uint8_t *ros_rx_buffer_ptr, uint32_t buf_len)
{
    // spdlog::info("remove buff = {}", Utils::Bytes2String(ros_rx_buffer_ptr, buf_len).c_str());
    lora_buffer_.size -= buf_len;
    uint8_t buffer[sizeof(lora_buffer_.rx_buffer)];
    // 剩余未处理数据拷贝到临时变量
    memcpy(buffer, ros_rx_buffer_ptr + buf_len, lora_buffer_.size);
    // 覆盖掉原来的buff
    memcpy(lora_buffer_.rx_buffer, buffer, lora_buffer_.size);
}

void YL800t::EnterPairing()
{
    b_connect_                  = false;
    b_wait_ack_                 = false;
    time_counter_               = 0;
    lora_config_.serialBaudRate = YL_800T_BAUD_RATE_9600;
    lora_config_.serialParity   = YL_800T_PARITY_NONE;
    lora_config_.rfFrequency    = 465100000.0 / 61.035; // 设置配对用的载波频率
    spdlog::info("rfFrequency = {:04X}", lora_config_.rfFrequency);
    lora_config_.rfSpreadingFactor = YL_800T_RF_SPREADING_FACTOR_2048;
    lora_config_.rfBandwidth       = YL_800T_RF_BANDWIDTH_250K;
    lora_config_.mode              = YL_800T_RF_MODE_STANDARD;
    lora_config_.netId             = 0xAA;                          // 设置网络ID
    lora_config_.rfTransmitPower   = YL_800T_RF_TRANSMIT_POWER_4DB; // 对码时设置最低功率
    lora_config_.nodeId            = ws_config_.source_id;          // 设置节点ID

    AsyncSendToLoraCmd(YL_800T_SET_UART);
    AsyncSendToLoraCmd(YL_800T_SET_FREQUECY);
    AsyncSendToLoraCmd(YL_800T_SET_SPREAD_SPECTRUM_FACTOR);
    AsyncSendToLoraCmd(YL_800T_SET_SPREAD_SPECTRUM_BANDWIDTH);
    AsyncSendToLoraCmd(YL_800T_SET_WORK_MODE);
    AsyncSendToLoraCmd(YL_800T_SET_NODE_ID);
    AsyncSendToLoraCmd(YL_800T_SET_NETWORK_ID);
    AsyncSendToLoraCmd(YL_800T_SET_TRANSMISSION_POWER);
}

void YL800t::SetLoraNormal()
{
    lora_config_.serialBaudRate    = YL_800T_BAUD_RATE_9600;
    lora_config_.serialParity      = YL_800T_PARITY_NONE;
    lora_config_.rfFrequency       = (double)(lora_frequency_[ws_config_.freq_index]) / 61.035; // 设置载波频率
    lora_config_.rfSpreadingFactor = YL_800T_RF_SPREADING_FACTOR_2048;
    lora_config_.rfBandwidth       = YL_800T_RF_BANDWIDTH_250K;
    lora_config_.mode              = YL_800T_RF_MODE_STANDARD;
    lora_config_.netId             = ws_config_.network_id;          // 设置网络ID
    lora_config_.nodeId            = ws_config_.source_id;           // 设置节点ID
    lora_config_.rfTransmitPower   = YL_800T_RF_TRANSMIT_POWER_14DB; // 设置正常通信功率

    AsyncSendToLoraCmd(YL_800T_SET_UART);
    AsyncSendToLoraCmd(YL_800T_SET_FREQUECY);
    AsyncSendToLoraCmd(YL_800T_SET_SPREAD_SPECTRUM_FACTOR);
    AsyncSendToLoraCmd(YL_800T_SET_SPREAD_SPECTRUM_BANDWIDTH);
    AsyncSendToLoraCmd(YL_800T_SET_WORK_MODE);
    AsyncSendToLoraCmd(YL_800T_SET_NODE_ID);
    AsyncSendToLoraCmd(YL_800T_SET_NETWORK_ID);
    AsyncSendToLoraCmd(YL_800T_SET_TRANSMISSION_POWER);
    b_wait_ack_ = false;
}

int YL800t::AsyncSendToLoraCmd(int cmd)
{
    CmdMsg commad;
    commad.cmd      = cmd;
    commad.has_send = false;
    cmd_queue_.push_back(commad);
    cmd_has_result_map_[cmd] = false;
    return cmd_queue_.size();
}

bool YL800t::HandleCmd()
{
    if (cmd_queue_.empty()) {
        return true;
    }

    std::list<CmdMsg>::iterator itList;
    for (itList = cmd_queue_.begin(); itList != cmd_queue_.end();) {
        if (!itList->has_send) {
            SendToLoraModule(itList->cmd);
            itList->has_send = true;
            break;
        } else {
            if (cmd_has_result_map_[itList->cmd]) {
                // log_info(TAG, "YL800t get %d result", itList->cmd);
                cmd_queue_.erase(itList++);
                continue;
            }
            break;
        }
        itList++;
    }
    return false;
}

int YL800t::SendToLoraModule(int cmd)
{
    uint8_t buffer[20] = {0};
    uint32_t len       = 0;

    buffer[0] = 0xAF;
    buffer[1] = 0xAF;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0xAF;
    buffer[5] = 0x80;
    buffer[6] = cmd;

    switch (cmd) {
    case YL_800T_ENTER_NORMAL /* 进入标准模式 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_ENTER_CENTER /* 进入中心模式 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_ENTER_NODE /* 进入节点模式 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_LAST_SIGNED /* 读取上一包接收强度 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_UART /* 写串口参数 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.serialBaudRate;
        buffer[9] = lora_config_.serialParity;
    } break;

    case YL_800T_GET_UART /* 读串口参数 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_FREQUECY /* 写载波频率 */: {
        buffer[7]  = 3;
        buffer[8]  = (lora_config_.rfFrequency >> 16) & 0xFF;
        buffer[9]  = (lora_config_.rfFrequency >> 8) & 0xFF;
        buffer[10] = lora_config_.rfFrequency & 0xFF;
    } break;

    case YL_800T_GET_FREQUECY /* 读载波频率 */: {
        buffer[7]  = 3;
        buffer[8]  = 0;
        buffer[9]  = 0;
        buffer[10] = 0;
    } break;

    case YL_800T_SET_SPREAD_SPECTRUM_FACTOR /* 写扩频因子 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.rfSpreadingFactor;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_SPREAD_SPECTRUM_FACTOR /* 读扩频因子 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_SPREAD_SPECTRUM_BANDWIDTH /* 写扩频带宽 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.rfBandwidth;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_SPREAD_SPECTRUM_BANDWIDTH /* 读扩频带宽 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_WORK_MODE /* 写工作模式 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.mode;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_WORK_MODE /* 读工作模式 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_NODE_ID /* 写节点ID */: {
        buffer[7]                 = 2;
        *(uint16_t *)(buffer + 8) = lora_config_.nodeId;
    } break;

    case YL_800T_GET_NODE_ID /* 读节点ID */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_NETWORK_ID /* 写网络ID */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.netId;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_NETWORK_ID /* 读网络ID */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_TRANSMISSION_POWER /* 写发射功率 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.rfTransmitPower;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_TRANSMISSION_POWER /* 读发射功率 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_BREATHING_CYCLE /* 写呼吸周期 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.breathCycle;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_BREATHING_CYCLE /* 读呼吸周期 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_BREATHING_TIME /* 写呼吸时间 */: {
        buffer[7] = 2;
        buffer[8] = lora_config_.breathTime;
        buffer[9] = 0;
    } break;

    case YL_800T_GET_BREATHING_TIME /* 读呼吸时间 */: {
        buffer[7] = 2;
        buffer[8] = 0;
        buffer[9] = 0;
    } break;

    case YL_800T_SET_OTA_MODE /* 设置OTA模式 */: {
        buffer[7]  = 8;
        buffer[8]  = 0x64;
        buffer[9]  = 0x6F;
        buffer[10] = 0x77;
        buffer[11] = 0x6E;
        buffer[12] = 0x6C;
        buffer[13] = 0x6F;
        buffer[14] = 0x61;
        buffer[15] = 0x64;
    } break;

    default: {
        spdlog::error("unkonw lora commad");
        return -1;
    } break;
    }

    len             = buffer[7] + PARAMETER_INDEX;
    buffer[len]     = checksum(buffer, len);
    buffer[len + 1] = 0x0D;
    buffer[len + 2] = 0x0A;

    spdlog::info("send cmd[{}]", Utils::Bytes2String(buffer, len + 3).c_str());

    return serial_comm_->SendBuffer(buffer, len + 3);
}

void YL800t::HandleLoraMsg(lora_at_frame_t lora_frame)
{
    cmd_has_result_map_[lora_frame.cmd] = true;

    switch (lora_frame.cmd) {

    case YL_800T_ENTER_NORMAL /* 进入标准模式 */: {
        spdlog::info("Enter normal mode");
    } break;

    case YL_800T_ENTER_CENTER /* 进入中心模式 */: {
        spdlog::info("Enter center mode");
    } break;

    case YL_800T_ENTER_NODE /* 进入节点模式 */: {
        spdlog::info("Enter node mode");
    } break;

    case YL_800T_LAST_SIGNED /* 读取上一包接收强度 */: {
        spdlog::info("Sigenal = {} db", lora_frame.dat[0] - 164);
    } break;

    case YL_800T_SET_UART /* 写串口参数 */: {
        spdlog::info("Set uart baud rate = {} parity = {}",
                 serial_baud_rate_[lora_frame.dat[0]],
                 serial_parity_map_[lora_frame.dat[1]].c_str());
    } break;

    case YL_800T_GET_UART /* 读串口参数 */: {
        spdlog::info("Get uart baud rate = {} parity = {}",
                 serial_baud_rate_[lora_frame.dat[0]],
                 serial_parity_map_[lora_frame.dat[1]].c_str());
    } break;

    case YL_800T_SET_FREQUECY /* 写载波频率 */: {
        uint32_t frequecy = (double)(lora_frame.dat[0] << 16 | lora_frame.dat[1] << 8 | lora_frame.dat[3]) * 61.035;
        spdlog::info("Set frequecy = {}", frequecy);
    } break;

    case YL_800T_GET_FREQUECY /* 读载波频率 */: {
        uint32_t frequecy = (double)(lora_frame.dat[0] << 16 | lora_frame.dat[1] << 8 | lora_frame.dat[3]) * 61.035;
        spdlog::info("Get frequecy = {}", frequecy);
    } break;

    case YL_800T_SET_SPREAD_SPECTRUM_FACTOR /* 写扩频因子 */: {
        spdlog::info("Set spread spectrum factor = {} k", spreading_factor_map_[lora_frame.dat[0]]);
    } break;

    case YL_800T_GET_SPREAD_SPECTRUM_FACTOR /* 读扩频因子 */: {
        spdlog::info("Get spread spectrum factor = {} k", spreading_factor_map_[lora_frame.dat[0]]);
    } break;

    case YL_800T_SET_SPREAD_SPECTRUM_BANDWIDTH /* 写扩频带宽 */: {
        spdlog::info("Set spread spectrum bandeidth = {}", bandwidth_map_[lora_frame.dat[0]]);
    } break;

    case YL_800T_GET_SPREAD_SPECTRUM_BANDWIDTH /* 读扩频带宽 */: {
        spdlog::info("Get spread spectrum bandeidth = {:X}", bandwidth_map_[lora_frame.dat[0]]);
    } break;

    case YL_800T_SET_WORK_MODE /* 读工作模式 */: {
        spdlog::info("Set work mode = {}", lora_mode_map_[lora_frame.dat[0]].c_str());
    } break;

    case YL_800T_GET_WORK_MODE /* 读工作模式 */: {
        spdlog::info("Get work mode = {}", lora_mode_map_[lora_frame.dat[0]].c_str());
    } break;

    case YL_800T_SET_NODE_ID /* 设置节点ID */: {
        uint16_t clinent = *(uint16_t *)(lora_frame.dat);
        spdlog::info("Set node id = {}", clinent);
    } break;

    case YL_800T_GET_NODE_ID /* 读节点ID */: {
        uint16_t clinent = *(uint16_t *)(lora_frame.dat);
        spdlog::info("Get node id = {}", clinent);
    } break;

    case YL_800T_SET_NETWORK_ID /* 写网络ID */: {
        spdlog::info("Set network id = {}", lora_frame.dat[0]);
    } break;

    case YL_800T_GET_NETWORK_ID /* 读网络ID */: {
        spdlog::info("Get network id = {}", lora_frame.dat[0]);
    } break;

    case YL_800T_SET_TRANSMISSION_POWER /* 写发射功率 */: {
        spdlog::info("Set transmit power = {} db", transmit_power_map_[lora_frame.dat[0]]);
    } break;

    case YL_800T_GET_TRANSMISSION_POWER /* 读发射功率 */: {
        spdlog::info("Get transmit power = {} db ", transmit_power_map_[lora_frame.dat[0]]);
    } break;

    case YL_800T_SET_BREATHING_CYCLE /* 写呼吸周期 */: {
        spdlog::info("Set breathing cycle = {}", lora_frame.dat[0]);
    } break;

    case YL_800T_GET_BREATHING_CYCLE /* 读呼吸周期 */: {
        spdlog::info("Get breathing cycle = {}", lora_frame.dat[0]);
    } break;

    case YL_800T_SET_BREATHING_TIME /* 写呼吸时间 */: {
        spdlog::info("Set breathing time = {}", lora_frame.dat[0]);
    } break;

    case YL_800T_GET_BREATHING_TIME /* 读呼吸时间 */: {
        spdlog::info("Get breathing time = {}", lora_frame.dat[0]);
    } break;

    default:
        spdlog::info("recv buff[{}]", Utils::Bytes2String((uint8_t *)&lora_frame, lora_frame.len + 8).c_str());
        break;
    }
}

uint8_t YL800t::checksum(const uint8_t *message, uint8_t length)
{
    uint8_t result = 0;
    for (uint8_t i = 0; i < length; ++i) {
        result += message[i];
    }
    return result;
}

