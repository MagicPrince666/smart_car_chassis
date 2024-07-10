/**
 * @file vl53l0x.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 距离传感器
 * @version 0.1
 * @date 2023-04-05
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __VL5310X_H__
#define __VL5310X_H__

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

#define SEQUENCE_ENABLE_FINAL_RANGE 0x80
#define SEQUENCE_ENABLE_PRE_RANGE 0x40
#define SEQUENCE_ENABLE_TCC 0x10
#define SEQUENCE_ENABLE_DSS 0x08
#define SEQUENCE_ENABLE_MSRC 0x04

typedef enum vcselperiodtype { VcselPeriodPreRange,
                               VcselPeriodFinalRange } vcselPeriodType;

typedef struct tagSequenceStepTimeouts {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} SequenceStepTimeouts;

// VL53L0X internal registers
#define REG_IDENTIFICATION_MODEL_ID 0xc0
#define REG_IDENTIFICATION_REVISION_ID 0xc2
#define REG_SYSRANGE_START 0x00

#define REG_RESULT_INTERRUPT_STATUS 0x13
#define RESULT_RANGE_STATUS 0x14
#define ALGO_PHASECAL_LIM 0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT 0x30

#define GLOBAL_CONFIG_VCSEL_WIDTH 0x32
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW 0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48

#define PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW 0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x57

#define REG_MSRC_CONFIG_CONTROL 0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define MSRC_CONFIG_TIMEOUT_MACROP 0x46
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define SYSRANGE_START 0x00
#define SYSTEM_SEQUENCE_CONFIG 0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define RESULT_INTERRUPT_STATUS 0x13
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define SYSTEM_INTERRUPT_CLEAR 0x0B

class Vl53l0x
{
public:
    Vl53l0x(std::string dev = "vl53l0x");
    ~Vl53l0x();
    int GetDistance();

    int tofGetModel(int *model, int *revision);

    int tofReadDistance(void);

    int tofInit(int iChan, int iAddr, int bLongRange);

private:
    std::string device_dir_;
    int file_i2c = 0;
    unsigned char stop_variable;
    uint32_t measurement_timing_budget_us;

    unsigned char readReg(unsigned char ucAddr);
    unsigned short readReg16(unsigned char ucAddr);
    void writeReg16(unsigned char ucAddr, unsigned short usValue);
    void writeReg(unsigned char ucAddr, unsigned char ucValue);
    void writeRegList(unsigned char *ucList);
    int initSensor(int);
    int performSingleRefCalibration(uint8_t vhv_init_byte);
    int setMeasurementTimingBudget(uint32_t budget_us);
    int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
    void readMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount);
    void writeMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount);
    int getSpadInfo(unsigned char *pCount, unsigned char *pTypeIsAperture);
    uint16_t decodeTimeout(uint16_t reg_val);
    uint16_t encodeTimeout(uint16_t timeout_mclks);
    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
    void getSequenceStepTimeouts(uint8_t enables, SequenceStepTimeouts *timeouts);
    uint32_t getMeasurementTimingBudget(void);
    uint16_t readRangeContinuousMillimeters(void);
};

#endif
