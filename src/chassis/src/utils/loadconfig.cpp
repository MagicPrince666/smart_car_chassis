#include "loadconfig.h"
#include "jsonparse.h"
#include "interface.h"

#include <stdio.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

LoadConfig::LoadConfig(std::string file_path) {
    JsoncppParseRead::ReadFileToJson(file_path, conf_json_);
}

LoadConfig::~LoadConfig() {}

bool LoadConfig::LoadPwmConfig(std::string key, PwmPram &pwm)
{
    if (conf_json_.isMember(key) && conf_json_[key].isObject()) {
        if (conf_json_[key].isMember("chip") && conf_json_[key]["chip"].isInt()) {
            pwm.chip = conf_json_[key]["chip"].asInt();
        }
        if (conf_json_[key].isMember("channel") && conf_json_[key]["channel"].isInt()) {
            pwm.channel = conf_json_[key]["channel"].asInt();
        }
        if (conf_json_[key].isMember("polarity") && conf_json_[key]["polarity"].isBool()) {
            pwm.polarity = conf_json_[key]["polarity"].asBool();
        }
        if (conf_json_[key].isMember("period") && conf_json_[key]["period"].isInt()) {
            pwm.period = conf_json_[key]["period"].asInt();
        }
        if (conf_json_[key].isMember("dutycycle") && conf_json_[key]["dutycycle"].isInt()) {
            pwm.dutycycle = conf_json_[key]["dutycycle"].asInt();
        }
        return true;
    }
    return false;
}

bool LoadConfig::LoadPinConfig(std::string key, int &pin)
{
    if (conf_json_.isMember(key) && conf_json_[key].isInt()) {
        pin = conf_json_[key].asInt();
        return true;
    }
    return false;
}

bool LoadConfig::LoadPinConfig(std::string key, std::string &pin)
{
    if (conf_json_.isMember(key) && conf_json_[key].isString()) {
        pin = conf_json_[key].asString();
        return true;
    }
    return false;
}

bool LoadConfig::LoadDistanceConfig(std::string key, bool &enable)
{
    if (conf_json_.isMember("distance") && conf_json_["distance"].isObject()) {
        if (conf_json_["distance"].isMember(key) && conf_json_["distance"][key].isBool()) {
            enable = conf_json_["distance"][key].asBool();
            return true;
        }
    }
    return false;
}

bool LoadConfig::LoadRataryConfig(std::string key, std::string &device, float &reduction_ratio, float &precision)
{
    if (conf_json_.isMember(key) && conf_json_[key].isObject()) {
        if (conf_json_[key].isMember("device") && conf_json_[key]["device"].isString()) {
            device = conf_json_[key]["device"].asString();
        }
        if (conf_json_[key].isMember("reduction_ratio") && conf_json_[key]["reduction_ratio"].isDouble()) {
            reduction_ratio = conf_json_[key]["reduction_ratio"].asDouble();
        }
        if (conf_json_[key].isMember("precision") && conf_json_[key]["precision"].isDouble()) {
            precision = conf_json_[key]["precision"].asDouble();
        }
        return true;
    }
    return false;
}

bool LoadConfig::LoadCarConfig(std::string key, DriverParams &car)
{
    if (conf_json_.isMember(key) && conf_json_[key].isObject()) {
        if (conf_json_[key].isMember("RotaryEncoder") && conf_json_[key]["RotaryEncoder"].isArray()) {
            for (uint32_t i = 0; i < conf_json_[key]["RotaryEncoder"].size(); i++) {
                car.RotaryChanel.push_back(conf_json_[key]["RotaryEncoder"][i].asString());
            }
        }
        if (conf_json_[key].isMember("LeftWheelRadius") && conf_json_[key]["LeftWheelRadius"].isDouble()) {
            car.LeftWheelRadius = conf_json_[key]["LeftWheelRadius"].asDouble();
        }
        if (conf_json_[key].isMember("RightWheelRadius") && conf_json_[key]["RightWheelRadius"].isDouble()) {
            car.RightWheelRadius = conf_json_[key]["RightWheelRadius"].asDouble();
        }
        if (conf_json_[key].isMember("TrendLength") && conf_json_[key]["TrendLength"].isDouble()) {
            car.TrendLength = conf_json_[key]["TrendLength"].asDouble();
        }
        if (conf_json_[key].isMember("WheelBase") && conf_json_[key]["WheelBase"].isDouble()) {
            car.WheelBase = conf_json_[key]["WheelBase"].asDouble();
        }
        if (conf_json_[key].isMember("Ticks1Roll") && conf_json_[key]["Ticks1Roll"].isDouble()) {
            car.Ticks1Roll = conf_json_[key]["Ticks1Roll"].asDouble();
        }
        if (conf_json_[key].isMember("MaxXVel") && conf_json_[key]["MaxXVel"].isDouble()) {
            car.MaxXVel = conf_json_[key]["MaxXVel"].asDouble();
        }
        if (conf_json_[key].isMember("MaxWVel") && conf_json_[key]["MaxWVel"].isDouble()) {
            car.MaxWVel = conf_json_[key]["MaxWVel"].asDouble();
        }
        if (conf_json_[key].isMember("MaxAngle") && conf_json_[key]["MaxAngle"].isDouble()) {
            car.MaxAngle = conf_json_[key]["MaxAngle"].asDouble();
        }
        if (conf_json_[key].isMember("ZeroAngle") && conf_json_[key]["ZeroAngle"].isDouble()) {
            car.ZeroAngle = conf_json_[key]["ZeroAngle"].asDouble();
        }
        if (conf_json_[key].isMember("MaxXAcc") && conf_json_[key]["MaxXAcc"].isDouble()) {
            car.MaxXAcc = conf_json_[key]["MaxXAcc"].asDouble();
        }
        if (conf_json_[key].isMember("MaxWAcc") && conf_json_[key]["MaxWAcc"].isDouble()) {
            car.MaxWAcc = conf_json_[key]["MaxWAcc"].asDouble();
        }
        if (conf_json_[key].isMember("MaxAngerLimit") && conf_json_[key]["MaxAngerLimit"].isDouble()) {
            car.MaxAngerLimit = conf_json_[key]["MaxAngerLimit"].asDouble();
        }
        if (conf_json_[key].isMember("MinAngerLimit") && conf_json_[key]["MinAngerLimit"].isDouble()) {
            car.MinAngerLimit = conf_json_[key]["MinAngerLimit"].asDouble();
        }
        if (conf_json_[key].isMember("Proportion") && conf_json_[key]["Proportion"].isDouble()) {
            car.Proportion = conf_json_[key]["Proportion"].asDouble();
        }
        if (conf_json_[key].isMember("Integration") && conf_json_[key]["Integration"].isDouble()) {
            car.Integration = conf_json_[key]["Integration"].asDouble();
        }
        if (conf_json_[key].isMember("Differentiation") && conf_json_[key]["Differentiation"].isDouble()) {
            car.Differentiation = conf_json_[key]["Differentiation"].asDouble();
        }
        return true;
    }
    return false;
}