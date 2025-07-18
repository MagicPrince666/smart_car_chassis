#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include "utils.h"

std::string Utils::ScanIioDevice(std::string name) {
    std::string device = "";
    if(!name.empty()) { //扫描设备
        for(int i = 0; i < 10; i++) {
            std::string filename = "/sys/bus/iio/devices/iio:device" + std::to_string(i) + "/name";
            // std::cout << "iio bus name" << filename << std::endl;
            struct stat buffer;   
            if(stat(filename.c_str(), &buffer) == 0) {
                std::ifstream iio_name;
                iio_name.open(filename, std::ios::in);
                char buff[64] = {0};
                iio_name.read(buff, sizeof(buff));
                std::string dev_name(buff);

                if (dev_name.find(name) != std::string::npos) {
                    // std::cout << "find " << name << " device\n";
                    device = "/sys/bus/iio/devices/iio:device" + std::to_string(i) +"/";
                    break;
                }
            } else {
                // 搜索完了，没有找到对应设备
                std::cout << "iio bus scan end\n";
                break;
            }
        }
    }

    return device;
}

std::string Utils::ScanInputDevice(std::string name)
{
    std::string device = "";
    if(!name.empty()) { //扫描设备
        for(int i = 0; i < 10; i++) {
            std::string filename = "/sys/class/input/event" + std::to_string(i) + "/device/name";
            // std::cout << "iio bus name" << filename << std::endl;
            struct stat buffer;   
            if(stat(filename.c_str(), &buffer) == 0) {
                std::ifstream iio_name;
                iio_name.open(filename, std::ios::in);
                char buff[64] = {0};
                iio_name.read(buff, sizeof(buff));
                std::string dev_name(buff);

                if (dev_name.find(name) != std::string::npos) {
                    // std::cout << "find " << name << " device\n";
                    device = "/dev/input/event" + std::to_string(i);
                    break;
                }
            } else {
                // 搜索完了，没有找到对应设备
                // std::cout << "input device scan end\n";
                break;
            }
        }
    }

    return device;
}

std::string Utils::ReadFileIntoString(const std::string& path) {
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        return "";
    }
    std::stringstream buffer;
    buffer << input_file.rdbuf();
    std::string contents(buffer.str());
    input_file.close();
    return contents;
}

void Utils::getFiles(std::string path, std::vector<std::string>& files)
{
	// check the parameter !
	if( path.empty() ) {
		return;
	}
	// check if dir_name is a valid dir
	struct stat s;
	lstat( path.c_str(), &s );
	if( ! S_ISDIR( s.st_mode ) ) {
		return;
	}

	struct dirent * filename;    // return value for readdir()
	DIR * dir;                   // return value for opendir()
	dir = opendir( path.c_str() );
	if( NULL == dir ) {
		return;
	}

	/* read all the files in the dir ~ */
	while( ( filename = readdir(dir) ) != NULL ) {
		// get rid of "." and ".."
		if( strcmp( filename->d_name , "." ) == 0 ||
			strcmp( filename->d_name , "..") == 0 )
			continue;
        std::string full_path = path + filename->d_name;
        struct stat s;
        lstat( full_path.c_str(), &s );
        if( S_ISDIR( s.st_mode ) ) {
            continue;
        }
        files.push_back(full_path);
	}
}

std::string Utils::Bytes2String(uint8_t *data, uint32_t len)
{
    char temp[512];
    std::string str("");
    for (size_t i = 0; i < len; i++) {
        snprintf(temp, sizeof(temp) - 1, "%02x ", data[i]);
        str.append(temp);
    }
    return str;
}


// CRC表格, 采用查表法实现CRC计算,为了减少工作量, 首先高位与低位互换,
// 这样只要CRC结果与原先CRC High的高位异或,即可得出正确的CRC结果.
const uint16_t g_usCRCTable[] =
{
	0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601,
	0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0,
	0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81,
	0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 0X18C0, 0X1980, 0XD941,
	0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01,
	0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0,
	0X1680, 0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081,
	0X1040, 0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
	0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441, 0X3C00,
	0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 0XFA01, 0X3AC0,
	0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
	0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41,
	0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700,
	0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0,
	0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281,
	0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
	0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01,
	0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1,
	0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80,
	0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 0XB401, 0X74C0, 0X7580, 0XB541,
	0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101,
	0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0,
	0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481,
	0X5440, 0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
	0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841, 0X8801,
	0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40, 0X4E00, 0X8EC1,
	0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 0X8581,
	0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341,
	0X4100, 0X81C1, 0X8081, 0X4040 };


/**
 *@brief  工具函数，用于计算循环冗余校验码
 *@param  pContent	指向需要计算校验码的数据
 *@param  usLength	数据的字节长度
 */
uint16_t Utils::CheckCRC(uint8_t *pContent, uint16_t usLength)
{
    uint16_t usCRCVal = 0xffff;                     // CRC初始值为0XFFFF
    uint16_t usCRCTemp;                             // CRC计算的临时变量
    uint16_t usCounter = 0;                         // 当前的计数个数
    uint8_t *pCurChar = pContent;                   // 当前的CRC指针
    uint8_t *pucLow = (uint8_t *)(&usCRCVal);       // 当前CRC的低字
    uint8_t *pucHigh = pucLow + 1;                  // 当前CRC的高字.

    // 从0字节开始遍历pContent所有内容
    while (usCounter < usLength)
    {
        // 与CRC的值异或.
        usCRCTemp = (*pucLow) ^ (*pCurChar);
        // 异或结果的低8位作为索引, 查g_usCRCTable表.
        usCRCTemp = g_usCRCTable[usCRCTemp];
        // 查表结果与CRC值的高8位(放到0~7位)异或, 结果作为CRC的结果
        usCRCVal = (uint16_t)((*pucHigh) ^ usCRCTemp);
        // 对于偶数字节的下一个字节,与checkSum的低32位字求和
        pCurChar++;
        usCounter++;
    }
    // 返回CRC结果
    return usCRCVal;
}

int64_t Utils::GetCurrentSecTime()
{
    auto current_time            = std::chrono::high_resolution_clock::now();
    auto duration_in_nanoseconds = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch());
    return duration_in_nanoseconds.count();
}

int64_t Utils::GetCurrentMsTime()
{
    auto current_time            = std::chrono::high_resolution_clock::now();
    auto duration_in_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch());
    return duration_in_milliseconds.count();
}

bool Utils::FileExists(const std::string name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

std::string Utils::getCurrentTime(int32_t zones)
{
    std::time_t result = std::time(nullptr) + zones * 3600;
    auto sec           = std::chrono::seconds(result);
    std::chrono::time_point<std::chrono::system_clock> now(sec);
    auto timet     = std::chrono::system_clock::to_time_t(now);
    auto localTime = *std::gmtime(&timet);

    std::stringstream ss;
    std::string str;
    ss << std::put_time(&localTime, "%H:%M");
    ss >> str;

    return str;
}

std::string Utils::getCurrentUser()
{
#if defined(__unix__)
    uid_t userid;
    struct passwd* pwd;
    userid = getuid();
    pwd = getpwuid(userid);
    return pwd->pw_name;
#elif defined(_WIN32)
    TCHAR name[UNLEN + 1];
    DWORD size = UNLEN + 1;
    if (GetUserName(name, &size)) {
        return std::string(name, size);
    }
#endif
    return "";
}

void Utils::Execute(std::string cmdline, std::string &recv)
{
#if defined(__unix__)
    FILE *stream = NULL;
    char buff[1024];
    char recv_buff[256]      = {0};

    memset(recv_buff, 0, sizeof(recv_buff));

    if ((stream = popen(cmdline.c_str(), "r")) != NULL) {
        while (fgets(buff, 1024, stream)) {
            strcat(recv_buff, buff);
        }
    }
    recv = recv_buff;
    pclose(stream);
#endif
}

void Utils::SuperExecute(std::string cmdline, std::string passwd)
{
#if defined(__unix__)
    char cmd[256] = {0};
    int len = snprintf(cmd, sizeof(cmd), "echo %s | sudo -S %s", passwd.c_str(), cmdline.c_str());
    cmd[len] = 0;
    system(cmd);
#endif
}

void Utils::Addr2Line(std::string exe, std::vector<std::string>& strs)
{
#if defined(__unix__)
    char str[1024] = {0};
    for (uint32_t i = 0; i < strs.size(); i++) {
        std::string line = strs[i];
        std::string::size_type index = line.find("(+"); // found start stuck
        line = line.substr(index + 1, line.size() - index - 1);
        if (index != std::string::npos) {
            index = line.find(")"); // foud end
            if (index != std::string::npos) {
                line = line.substr(0, index);
                int len = snprintf(str, sizeof(str), "addr2line -e %s %s", exe.c_str(), line.c_str());
                str[len] = 0;
                // std::cout << "Run " << str << std::endl;
                std::string recv;
                Execute(str, recv);
                std::ofstream outfile;
                if (recv.find("??") == std::string::npos) {
                    outfile.open("coredump.log", std::ios::out | std::ios::app);
                    if (outfile.is_open()) {
                        outfile << recv;
                        outfile.close();
                    }
                }
            }
        }
    }
#endif
}
