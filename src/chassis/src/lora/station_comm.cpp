#include "station_comm.h"
#include "utils.h"
#include <fstream>
#include <iostream>
#include <unistd.h>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

// 水电站配置文件位置
#if defined __aarch64__
static const char *conf_file = "/userdata/xz_robot/config/ws_config.json";
#elif defined __x86_64__
static const char *conf_file = "ws_config.json";
#endif

LoraWan::LoraWan(std::string port, uint32_t rate)
    : lora_port_(port), baud_rate_(rate), b_connect_(false)
{
}

LoraWan::~LoraWan()
{
    spdlog::info("exit");
}

int LoraWan::SendToWorkStation(WESCmdCode cmd)
{
    uint8_t buffer[8 + WS_MAX_LENGTH_READ_WRITE] = {0};
    buffer[0]                                    = 0xA5;
    buffer[1]                                    = 0x5A;
    buffer[2]                                    = ws_config_.source_id;
    buffer[3]                                    = ws_config_.target_id;
    buffer[4]                                    = cmd;

    switch (cmd) {
    case READ_WES_HW_VERSION /* 读取水电站硬件版本号 */: {
        buffer[5] = 0;
    } break;

    case READ_WES_FW_VERSION /* 读取水电站软件版本号 */: {
        buffer[5] = 0;
    } break;

    case REPORT_WES_STATE /* 反馈水电站状态 */: {
    } break;

    case REPORT_WES_FAULT /* 反馈水电站故障码 */: {
    } break;

    case CMD_REQUEST_CHG_VOL /* SC80主机向WES请求工作电压 */: {
        buffer[5]                 = 2;
        *(uint16_t *)(buffer + 6) = ws_config_.charge_vol; // 40V
    } break;

    case CMD_REQUEST_CHG_CUR /* SC80主机向WES请求工作电流 */: {
        buffer[5]                 = 2;
        *(uint16_t *)(buffer + 6) = ws_config_.charge_cur; // 100A
    } break;

    case CMD_LORA_PAIRING /* SC80主机发出对频对码指令 */: {
        buffer[5] = 2;
        buffer[6] = ws_config_.freq_index;
        buffer[7] = ws_config_.network_id;
    } break;

    case CMD_LORA_PAIR_ACK /* 水电站回应对码对频指令 */: {
        buffer[5] = 2;
        buffer[6] = ws_config_.freq_index;
        buffer[7] = ws_config_.network_id;
    } break;

    case CMD_LORA_PAIR_OK /* SC80主机发送对频对码成功 */: {
        buffer[5] = 0;
    } break;

    default: {
        spdlog::error("unkonw work station commad");
        return -1;
    } break;
    }
    uint8_t len                 = buffer[5] + 6;
    *(uint16_t *)(buffer + len) = wschecksum(buffer, len);

    spdlog::info("send buff[{}]", Utils::Bytes2String(buffer, len + 2).c_str());

    if (serial_comm_) {
        return serial_comm_->SendBuffer(buffer, len + 2);
    }

    return 0;
}

void LoraWan::HandleWorkStationMsg(lora_ws_frame_t ws_frame)
{
    switch (ws_frame.cmd) {
    case READ_WES_HW_VERSION /* 读取水电站硬件版本号 */: {
        uint16_t hardware_version = *(uint16_t *)(ws_frame.dat);
        spdlog::info("work station hw = {}", hardware_version);

    } break;

    case READ_WES_FW_VERSION /* 读取水电站软件版本号 */: {
        uint16_t software_version = *(uint16_t *)(ws_frame.dat);
        spdlog::info("work station fw = {}", software_version);
    } break;

    case REPORT_WES_STATE /* 反馈水电站状态 */: {
        uint16_t states = ws_frame.dat[0];
        spdlog::info("work station status = {}", states);
    } break;

    case REPORT_WES_FAULT /* 反馈水电站故障码 */: {
        int16_t error_code = *(int16_t *)(ws_frame.dat);
        spdlog::info("work station error code = {}", error_code);
    } break;

    case CMD_REQUEST_CHG_VOL /* SC80主机向WES请求工作电压 */: {
        int16_t var            = *(int16_t *)(ws_frame.dat);
        double voltage = var * 0.01;
        spdlog::info("work station vol = {} V", voltage);
    } break;

    case CMD_REQUEST_CHG_CUR /* SC80主机向WES请求工作电流 */: {
        int16_t var            = *(int16_t *)(ws_frame.dat);
        double current = var * 0.01;
        spdlog::info("work station cur = {} A", current);
    } break;

    case CMD_LORA_PAIRING /* SC80主机发出对频对码指令 */: {
        if (!b_wait_ack_) {
            spdlog::info("work station right code with freq = {} network id = {}", ws_frame.dat[0], ws_frame.dat[1]);
            ws_config_.freq_index = ws_frame.dat[0];
            ws_config_.network_id = ws_frame.dat[1];
            b_wait_ack_           = true;
        } else {
            spdlog::info("work station wait master ack");
        }
        for (uint32_t i = 0; i < 10; i++) {
            SendToWorkStation(CMD_LORA_PAIR_ACK);
            usleep(500000);
        }
    } break;

    case CMD_LORA_PAIR_ACK /* 水电站回应对码对频指令 */: {
        ws_config_.freq_index = ws_frame.dat[0];
        ws_config_.network_id  = ws_frame.dat[1];
        spdlog::info("work station ack with freq = {} network id = {}", ws_config_.freq_index, ws_config_.target_id);
        for (uint32_t i = 0; i < 10; i++) {
            SendToWorkStation(CMD_LORA_PAIR_OK);
            usleep(500000);
        }
        SetLoraNormal(); // 切换到正常通讯
        SaveConfig();    // 保存到配置文件
    } break;

    case CMD_LORA_PAIR_OK /* SC80主机发送对频对码成功 */: {
        spdlog::info("work station connected");
        b_connect_             = true;
        SetLoraNormal(); // 切换到正常通讯
        SaveConfig();    // 保存到配置文件
    } break;

    default:
        break;
    }
}

bool LoraWan::SaveConfig()
{
    std::ofstream station_config(conf_file, std::ios::out);
    station_config << ws_config_.ToJson().toStyledString() << std::endl;
    station_config.close();
    return true;
}

bool LoraWan::LoadConfig()
{
    Json::Value des_json;
    std::ifstream in_file(conf_file, std::ios::binary);

    if (!in_file.is_open()) {
        spdlog::error("config file open fail");
        return false;
    }
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING errs;
    bool result = parseFromStream(builder, in_file, &des_json, &errs);
    in_file.close();
    if (result) {
        if (des_json.isMember("source_id") && des_json["source_id"].isInt()) {
            ws_config_.source_id = des_json["source_id"].asInt();
        }
        if (des_json.isMember("target_id") && des_json["target_id"].isInt()) {
            ws_config_.target_id = des_json["target_id"].asInt();
        }
        if (des_json.isMember("network_id") && des_json["network_id"].isInt()) {
            ws_config_.network_id = des_json["network_id"].asInt();
        }
        if (des_json.isMember("freq_index") && des_json["freq_index"].isInt()) {
            ws_config_.freq_index = des_json["freq_index"].asInt();
        }
        if (des_json.isMember("charge_vol") && des_json["charge_vol"].isInt()) {
            ws_config_.charge_vol = des_json["charge_vol"].asInt();
        }
        if (des_json.isMember("charge_cur") && des_json["charge_cur"].isInt()) {
            ws_config_.charge_cur = des_json["charge_cur"].asInt();
        }
    } else {
        spdlog::error("read json fail");
        return false;
    }
    return true;
}

uint16_t LoraWan::wschecksum(const uint8_t *message, uint8_t length)
{
    uint16_t result = 0;
    for (uint8_t i = 0; i < length; ++i) {
        result += message[i];
    }
    return ~result;
}
