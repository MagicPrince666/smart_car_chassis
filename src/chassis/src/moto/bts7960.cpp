#include "bts7960.h"
#include <iostream>

Bts7960::Bts7960(int l_is, int r_is, MotoInfo info) : Moto(info)
{
    // 报警接口设置为输入
    if (l_is != -1) {
        moto_l_is_ = std::make_shared<Gpio>(l_is, false);
    }
    if (r_is != -1) {
        moto_r_is_ = std::make_shared<Gpio>(r_is, false);
    }
}

Bts7960::~Bts7960() {}

