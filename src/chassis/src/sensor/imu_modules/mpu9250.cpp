#include "mpu9250.h"
#include <spdlog/spdlog.h>
#include "driver_mpu9250_interface.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

static void a_receive_callback(uint8_t type)
{
    switch (type) {
    case MPU9250_INTERRUPT_MOTION: {
        mpu9250_interface_debug_print("mpu9250: irq motion.\n");

        break;
    }
    case MPU9250_INTERRUPT_FIFO_OVERFLOW: {
        mpu9250_interface_debug_print("mpu9250: irq fifo overflow.\n");

        break;
    }
    case MPU9250_INTERRUPT_FSYNC_INT: {
        mpu9250_interface_debug_print("mpu9250: irq fsync int.\n");

        break;
    }
    case MPU9250_INTERRUPT_DMP: {
        mpu9250_interface_debug_print("mpu9250: irq dmp\n");

        break;
    }
    case MPU9250_INTERRUPT_DATA_READY: {
        mpu9250_interface_debug_print("mpu9250: irq data ready\n");

        break;
    }
    default: {
        mpu9250_interface_debug_print("mpu9250: irq unknown code.\n");

        break;
    }
    }
}

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction) {
    case MPU9250_DMP_TAP_X_UP: {
        mpu9250_interface_debug_print("mpu9250: tap irq x up with %d.\n", count);

        break;
    }
    case MPU9250_DMP_TAP_X_DOWN: {
        mpu9250_interface_debug_print("mpu9250: tap irq x down with %d.\n", count);

        break;
    }
    case MPU9250_DMP_TAP_Y_UP: {
        mpu9250_interface_debug_print("mpu9250: tap irq y up with %d.\n", count);

        break;
    }
    case MPU9250_DMP_TAP_Y_DOWN: {
        mpu9250_interface_debug_print("mpu9250: tap irq y down with %d.\n", count);

        break;
    }
    case MPU9250_DMP_TAP_Z_UP: {
        mpu9250_interface_debug_print("mpu9250: tap irq z up with %d.\n", count);

        break;
    }
    case MPU9250_DMP_TAP_Z_DOWN: {
        mpu9250_interface_debug_print("mpu9250: tap irq z down with %d.\n", count);

        break;
    }
    default: {
        mpu9250_interface_debug_print("mpu9250: tap irq unknown code.\n");

        break;
    }
    }
}

static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation) {
    case MPU9250_DMP_ORIENT_PORTRAIT: {
        mpu9250_interface_debug_print("mpu9250: orient irq portrait.\n");

        break;
    }
    case MPU9250_DMP_ORIENT_LANDSCAPE: {
        mpu9250_interface_debug_print("mpu9250: orient irq landscape.\n");

        break;
    }
    case MPU9250_DMP_ORIENT_REVERSE_PORTRAIT: {
        mpu9250_interface_debug_print("mpu9250: orient irq reverse portrait.\n");

        break;
    }
    case MPU9250_DMP_ORIENT_REVERSE_LANDSCAPE: {
        mpu9250_interface_debug_print("mpu9250: orient irq reverse landscape.\n");

        break;
    }
    default: {
        mpu9250_interface_debug_print("mpu9250: orient irq unknown code.\n");

        break;
    }
    }
}

Mpu9250::Mpu9250(std::string dev, uint32_t rate) : ImuInterface(dev, rate), module_name_("mpu9250")
{
    mpu9250_interface_set(imu_port_.c_str());
}

Mpu9250::~Mpu9250()
{
    mpu9250_dmp_deinit();
    g_gpio_irq_ = nullptr;
    GpioInterruptDeinit();
}

int Mpu9250::GpioInterruptInit()
{
    // mpu_int_ = new GpioKey;
    // if (mpu_int_) {
    //     return 0;
    // }
    return 0;
}

void Mpu9250::GpioInterruptDeinit()
{
    // delete mpu_int_;
}

bool Mpu9250::Init()
{
    /* init */
    if (GpioInterruptInit() != 0) {
        return false;
    }
    g_gpio_irq_ = mpu9250_dmp_irq_handler;

    /* init */
    if (mpu9250_dmp_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW, a_receive_callback,
                         a_dmp_tap_callback, a_dmp_orient_callback) != 0) {
        g_gpio_irq_ = nullptr;
        GpioInterruptDeinit();

        return false;
    }

    mpu9250_interface_delay_ms(500);
    return true;
}

void Mpu9250::Mpu9250Loop()
{
    uint32_t cnt;
    uint16_t len = 128;
    int16_t gs_accel_raw[128][3];
    float gs_accel_g[128][3];
    int16_t gs_gyro_raw[128][3];
    float gs_gyro_dps[128][3];
    int32_t gs_quat[128][4];
    float gs_pitch[128];
    float gs_roll[128];
    float gs_yaw[128];

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        /* read */
        if (mpu9250_dmp_read_all(gs_accel_raw, gs_accel_g,
                                 gs_gyro_raw, gs_gyro_dps,
                                 gs_quat,
                                 gs_pitch, gs_roll, gs_yaw,
                                 &len) != 0) {
            mpu9250_dmp_deinit();
            g_gpio_irq_ = nullptr;
            GpioInterruptDeinit();
            spdlog::error("dmp read all fail!!");
            return;
        }

        /* output */
        spdlog::info("fifo size {}.", len);
        spdlog::info("pitch[0] is {}dps.", gs_pitch[0]);
        spdlog::info("roll[0] is {}dps.", gs_roll[0]);
        spdlog::info("yaw[0] is {}dps.", gs_yaw[0]);
        // spdlog::info("acc x[0] is {}g.", gs_accel_g[0][0]);
        // spdlog::info("acc y[0] is {}g.", gs_accel_g[0][1]);
        // spdlog::info("acc z[0] is {}g.", gs_accel_g[0][2]);
        // spdlog::info("gyro x[0] is {}dps.", gs_gyro_dps[0][0]);
        // spdlog::info("gyro y[0] is {}dps.", gs_gyro_dps[0][1]);
        // spdlog::info("gyro z[0] is {}dps.", gs_gyro_dps[0][2]);

        /* delay 500 ms */
        mpu9250_interface_delay_ms(500);

        /* get the pedometer step count */
        int res = mpu9250_dmp_get_pedometer_counter(&cnt);
        if (res != 0) {
            mpu9250_dmp_deinit();
            g_gpio_irq_ = nullptr;
            GpioInterruptDeinit();
            spdlog::error("dmp get pedometer counter fail!!");
            return;
        }
    }
}