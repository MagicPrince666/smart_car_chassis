#include "zyf176ex.h"
#include <cmath>
#include <unistd.h>

Zyf176ex::Zyf176ex(std::string port, uint32_t rate)
    : ImuInterface(port, rate) {}

Zyf176ex::~Zyf176ex()
{
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
}

bool Zyf176ex::Init()
{
    // 创建通讯部件工厂,这一步可以优化到从lunch配置文件选择初始化不同的通讯部件工厂
    std::shared_ptr<CommFactory> factory(new SerialComm());
    // 通过工厂方法创建通讯产品
    std::shared_ptr<Communication> serial(factory->CreateCommTarget(imu_port_, baud_rate_, false));
    serial_comm_ = serial;

    imu_thread_ = std::thread([](Zyf176ex *p_this) { p_this->ImuReader(); }, this);
    return true;
}

void Zyf176ex::ReadBuffer(const uint8_t *buffer, const uint32_t length)
{
    std::unique_lock<std::mutex> lck(g_mtx_);
    uint32_t buf_size = sizeof(zyz_176ex_buffer_.rx_buffer) - zyz_176ex_buffer_.size;
    if (buf_size >= length) {
        memcpy(zyz_176ex_buffer_.rx_buffer + zyz_176ex_buffer_.size, buffer, length);
        zyz_176ex_buffer_.size += length; // 更新buff长度
    } else {
        memcpy(zyz_176ex_buffer_.rx_buffer + zyz_176ex_buffer_.size, buffer, buf_size);
        zyz_176ex_buffer_.size += buf_size; // 更新buff长度
    }
    g_cv_.notify_all(); // 唤醒所有线程.

    return;
}

void Zyf176ex::ImuReader()
{
    serial_comm_->AddCallback(std::bind(&Zyf176ex::ReadBuffer, this, std::placeholders::_1, std::placeholders::_2));
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        std::unique_lock<std::mutex> lck(g_mtx_);
        g_cv_.wait_for(lck, std::chrono::milliseconds(100));
        if (zyz_176ex_buffer_.size < sizeof(zyz_data_t)) {
            spdlog::warn("buffer size = {} not a full protocol", zyz_176ex_buffer_.size);
            usleep(10000);
            continue;
        }

        uint32_t loop_times = zyz_176ex_buffer_.size / sizeof(zyz_data_t);
        for (uint32_t i = 0; i < loop_times; i++) {
            uint8_t *ros_rx_buffer_ptr = zyz_176ex_buffer_.rx_buffer;
            uint32_t index                  = 0;
            zyz_data_t *res_tmp        = SearchHearLE(ros_rx_buffer_ptr, zyz_176ex_buffer_.size, index);
            if (res_tmp != nullptr) {
                // 有数据包需要处理
                if (index) {
                    ros_rx_buffer_ptr += index;
                    zyz_176ex_buffer_.size -= index;
                }

                zyz_data_t get_data = *res_tmp;
                zyz_176ex_buffer_.size -= sizeof(zyz_data_t);
                std::unique_ptr<uint8_t[]> buffer(new uint8_t[zyz_176ex_buffer_.size]);
                // 剩余未处理数据拷贝到临时变量
                memcpy(buffer.get(), ros_rx_buffer_ptr + sizeof(zyz_data_t), zyz_176ex_buffer_.size);
                // 覆盖掉原来的buff
                memcpy(zyz_176ex_buffer_.rx_buffer, buffer.get(), zyz_176ex_buffer_.size);

                std::lock_guard<std::mutex> mylock_guard(data_lock_);
                // 加速度
                imu_data_.linear_acceleration.x = get_data.acc_x * 9.8 / 10920;
                imu_data_.linear_acceleration.y = get_data.acc_y * 9.8 / 10920;
                imu_data_.linear_acceleration.z = get_data.acc_z * 9.8 / 10920;
                // 角速度
                imu_data_.angular_velocity.x = get_data.gyro_x * 0.01 * M_PI / 180;
                imu_data_.angular_velocity.y = get_data.gyro_y * 0.01 * M_PI / 180;
                imu_data_.angular_velocity.z = get_data.gyro_z * 0.01 * M_PI / 180;
                // 欧拉角
                imu_data_.eular.roll  = get_data.roll * 0.01 * M_PI / 180;
                imu_data_.eular.pitch = get_data.pitch * 0.01 * M_PI / 180;
                imu_data_.eular.yaw   = get_data.yaw * 0.01 * M_PI / 180;

                // spdlog::info("roll = %lf  pitch = %lf yaw = %lf", roll, pitch, yaw);
                Euler2Quaternion(imu_data_.eular.roll, imu_data_.eular.pitch, imu_data_.eular.yaw, imu_data_.orientation);
            }
        }
    }
    serial_comm_->RemoveCallback();
}

zyz_data_t *Zyf176ex::SearchHearLE(uint8_t *data, uint32_t total_len, uint32_t &index)
{
    zyz_data_t *res_tmp = nullptr;
    for (uint32_t i = 0; i < total_len; i++) {
        // 剩余长度大于一个包长度，说明还有协议数据包可以处理
        if ((total_len - i) >= sizeof(zyz_data_t)) {
            // 一个字节一个字节的偏移，直到查找到协议头
            res_tmp = (zyz_data_t *)(data + i);
            // 找到协议头
            if (res_tmp->head == 0xA5A5) {
                index = i; // 记录偏移地址
                break;
            }
        } else {
            return nullptr;
        }
    }
    return res_tmp;
}

void Zyf176ex::Euler2Quaternion(double roll, double pitch, double yaw, Quaternion &q)
{
    // 传入机器人的欧拉角 roll、pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中
    // https://blog.csdn.net/xiaoma_bk/article/details/79082629
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

