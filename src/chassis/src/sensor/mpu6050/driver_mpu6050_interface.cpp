/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_mpu6050_interface_template.c
 * @brief     driver mpu6050 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-06-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "driver_mpu6050_interface.h"
#include "iic.h"

IicBus *g_iic_bus = nullptr;
const char *g_iic_dev;

void mpu6050_interface_set(const char *dev)
{
    g_iic_dev = dev;
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t mpu6050_interface_iic_init(void)
{
    g_iic_bus = new IicBus(g_iic_dev);
    return g_iic_bus->IicInit();
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t mpu6050_interface_iic_deinit(void)
{
    delete g_iic_bus;
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6050_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{

    return g_iic_bus->IicRead(addr, reg, buf, len);
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6050_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return g_iic_bus->IicWrite(addr, reg, buf, len);
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void mpu6050_interface_delay_ms(uint32_t ms)
{
    usleep(ms * 1000);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void mpu6050_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    uint16_t len;
    va_list args;

    memset((char *)str, 0, sizeof(char) * 256);
    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);

    len = strlen((char *)str);
    (void)printf((const char *)str, len);
}

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
void mpu6050_interface_receive_callback(uint8_t type)
{
    switch (type) {
    case MPU6050_INTERRUPT_MOTION: {
        mpu6050_interface_debug_print("mpu6050: irq motion.\n");

        break;
    }
    case MPU6050_INTERRUPT_FIFO_OVERFLOW: {
        mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");

        break;
    }
    case MPU6050_INTERRUPT_I2C_MAST: {
        mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");

        break;
    }
    case MPU6050_INTERRUPT_DMP: {
        mpu6050_interface_debug_print("mpu6050: irq dmp\n");

        break;
    }
    case MPU6050_INTERRUPT_DATA_READY: {
        mpu6050_interface_debug_print("mpu6050: irq data ready\n");

        break;
    }
    default: {
        mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");

        break;
    }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
void mpu6050_interface_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction) {
    case MPU6050_DMP_TAP_X_UP: {
        mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);

        break;
    }
    case MPU6050_DMP_TAP_X_DOWN: {
        mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);

        break;
    }
    case MPU6050_DMP_TAP_Y_UP: {
        mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);

        break;
    }
    case MPU6050_DMP_TAP_Y_DOWN: {
        mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);

        break;
    }
    case MPU6050_DMP_TAP_Z_UP: {
        mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);

        break;
    }
    case MPU6050_DMP_TAP_Z_DOWN: {
        mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);

        break;
    }
    default: {
        mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");

        break;
    }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orientation is the dmp orientation
 * @note      none
 */
void mpu6050_interface_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation) {
    case MPU6050_DMP_ORIENT_PORTRAIT: {
        mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");

        break;
    }
    case MPU6050_DMP_ORIENT_LANDSCAPE: {
        mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");

        break;
    }
    case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT: {
        mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");

        break;
    }
    case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE: {
        mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");

        break;
    }
    default: {
        mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");

        break;
    }
    }
}
