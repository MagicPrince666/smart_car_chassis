#include "RecordData.h"

RecordData::RecordData(std::string file)
{
    //std::ofstream output_file_("turn_moto_test.csv");  
    // 创建一个输出文件，可以使用.csv格式
    output_file_.open(file, std::ios::out | std::ios::binary | std::ios::app);
}

RecordData::~RecordData()
{
    output_file_.close();
}
