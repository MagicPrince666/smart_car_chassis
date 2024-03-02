#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>

#include "vl53l0x.h"
#include "interface.h"
#include "utils.h"

Vl53l0x::Vl53l0x(std::string device) 
{
    device_dir_ = Utils::ScanIioDevice(device);
    assert(device_dir_ != "");
    std::cout << BOLDGREEN << "vl53l0x iio bus path " << device_dir_ << std::endl;
}

Vl53l0x::~Vl53l0x() {
    std::cout << BOLDGREEN << "Close vl53l0x device!" << std::endl;
}

int Vl53l0x::GetDistance()
{
    std::string distance_str = device_dir_ + "in_distance_raw";
    std::string buf = Utils::ReadFileIntoString(distance_str);
    int distance = atoi(buf.c_str());

    return distance;
}