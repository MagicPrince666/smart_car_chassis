/**
 * @file RecordData.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-20
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __RECORD_DATA_H__
#define __RECORD_DATA_H__

#include <fstream>
#include <iostream>
#include <string>

class RecordData
{
public:
    RecordData(std::string file);
    ~RecordData();

    template <typename T1, typename T2, typename T3>
    bool PushToFile(T1 key, T2 data1, T3 data2)
    {
        if (!output_file_.is_open()) {
            std::cout << "Can not open file!!" << std::endl;
            return false;
        }

        // 写入数据到文件
        output_file_ << key << "," << data1 << "," << data2 << std::endl;
        return true;
    }

private:
    std::ofstream output_file_;
};

#endif
