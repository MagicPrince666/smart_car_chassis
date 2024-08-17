#include <assert.h>
#include <cmath>
#include <cstring>
#include <unistd.h>
#ifndef __APPLE__
#include <linux/input.h>
#include <sys/timerfd.h>
#endif

#define USE_OPENCL 0

#if USE_OPENCL
#define CL_TARGET_OPENCL_VERSION 220
// #define CL_NO_EXTENSION_PROTOTYPES
#include <CL/opencl.h>
#endif

#include "Kinematics.h"
#include "xepoll.h"
#include "utils.h"

Kinematics::Kinematics(DriverParams car_param, MotoInfo moto_ctrl, PwmPram servo_ctrl)
    : car_param_(car_param)
{
    TestOpenCl();
    model_is_ackerman_ = true;
    timer_fd_          = -1;
    moto_ctrl_[0]      = std::make_shared<Moto>(moto_ctrl); // 电机控制
    moto_ctrl_[0]->Stop();
    pid_controller_[0] = std::make_shared<PidController>(car_param.Proportion, car_param.Integration, car_param.Differentiation);
    pid_controller_[0]->out_limit(-moto_ctrl_[0]->MaxPwm(), moto_ctrl_[0]->MaxPwm());
    pid_controller_[0]->update_target(0);
    servo_ctrl_ = std::make_shared<ServoMoto>(servo_ctrl); // 舵机控制
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "Rotary input %s", car_param.RotaryChanel[0].c_str());
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "MaxAngle %f", car_param.MaxAngle);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "ZeroAngle %f", car_param.ZeroAngle);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "Proportion: %f\t Integration %f\t Differentiation: %f", car_param.Proportion, car_param.Integration, car_param.Differentiation);
    rotary_encoder_[0]  = std::make_shared<RotaryEncoder>(car_param.RotaryChanel[0]); // 编码器
    out_motor_speed_[0] = 0.0;
    current_speeds_[0]  = 0.0;
    distan_1circle_[0]  = 2 * (car_param_.LeftWheelRadius + car_param_.RightWheelRadius) / 2 * M_PI / car_param_.Ticks1Roll; // 计算转动一圈行进距离
    // dispaly_count_            = 0;
    InitTimer(); // 初始化定时器
}

Kinematics::Kinematics(DriverParams car_param, MotoInfo moto1_ctrl, MotoInfo moto2_ctrl)
    : car_param_(car_param)
{
    TestOpenCl();
    model_is_ackerman_ = false;
    timer_fd_          = -1;
    moto_ctrl_[1]      = std::make_shared<Moto>(moto1_ctrl);
    moto_ctrl_[0]      = std::make_shared<Moto>(moto2_ctrl);
    moto_ctrl_[1]->Stop();
    moto_ctrl_[0]->Stop();
    pid_controller_[1] = std::make_shared<PidController>(car_param.Proportion, car_param.Integration, car_param.Differentiation);
    pid_controller_[1]->out_limit(-moto_ctrl_[1]->MaxPwm(), moto_ctrl_[1]->MaxPwm());
    pid_controller_[1]->update_target(0);
    pid_controller_[0] = std::make_shared<PidController>(car_param.Proportion, car_param.Integration, car_param.Differentiation);
    pid_controller_[0]->out_limit(-moto_ctrl_[0]->MaxPwm(), moto_ctrl_[0]->MaxPwm());
    pid_controller_[0]->update_target(0);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "RotaryChanel1: %s\tRotaryChanel2: %s", car_param.RotaryChanel[0].c_str(), car_param.RotaryChanel[1].c_str());
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "MaxXVel %f", car_param.MaxXVel);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "MaxWVel %f", car_param.MaxWVel);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "Proportion: %f\t Integration %f\t Differentiation: %f", car_param.Proportion, car_param.Integration, car_param.Differentiation);
    rotary_encoder_[1]  = std::make_shared<RotaryEncoder>(car_param.RotaryChanel[0]);
    rotary_encoder_[0]  = std::make_shared<RotaryEncoder>(car_param.RotaryChanel[1]);
    out_motor_speed_[1] = 0.0;
    current_speeds_[1]  = 0.0;
    out_motor_speed_[0] = 0.0;
    current_speeds_[0]  = 0.0;
    distan_1circle_[1]  = 2 * car_param_.LeftWheelRadius * M_PI / car_param_.Ticks1Roll;  // 计算转动一圈行进距离
    distan_1circle_[0]  = 2 * car_param_.RightWheelRadius * M_PI / car_param_.Ticks1Roll; // 计算转动一圈行进距离
    // dispaly_count_                  = 0;
    InitTimer(); // 初始化定时器
}

Kinematics::~Kinematics()
{
    if (timer_fd_ > 0) {
        MY_EPOLL.EpollDel(timer_fd_);
        close(timer_fd_);
    }
}

bool Kinematics::InitTimer(void)
{
    // 创建1s定时器fd
    if ((timer_fd_ =
             timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC)) < 0) {
        std::cout << "create timer fd fail" << std::endl;
        return false;
    }
    assert(timer_fd_ > 0);
    // 设置1s定时器
    struct itimerspec time_intv;
    time_intv.it_value.tv_sec     = 0; // 设定超时 10ms
    time_intv.it_value.tv_nsec    = 20000000;
    time_intv.it_interval.tv_sec  = time_intv.it_value.tv_sec; // 间隔超时
    time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;
    // 启动定时器
    timerfd_settime(timer_fd_, 0, &time_intv, NULL);
    // 绑定回调函数
    MY_EPOLL.EpollAddRead(timer_fd_, std::bind(&Kinematics::timeOutCallBack, this));
    return true;
}

// M法测速
int Kinematics::timeOutCallBack()
{
    uint64_t value;
    bool show         = false;
    int ret           = read(timer_fd_, &value, sizeof(uint64_t));
    uint64_t current  = Utils::GetCurrentMsTime();
    float dt          = (float)(current - last_update_time_) / 1000.0; // 计算两次读取之间的时间差
    last_update_time_ = current;                                       // 更新上一次更新时间
    if (dt == 0) {
        return -1;
    }

    if (rotary_encoder_[1]) {
        int32_t encoder = rotary_encoder_[1]->GetTicksRoll();
        float speeds    = encoder * distan_1circle_[1] / dt; // 计算左电机的速度 单位m/s
        if (pid_controller_[1]->get_target() == 0) {
            out_motor_speed_[1] = 0;
        } else {
            float out_target = pid_controller_[1]->update(speeds);
            if (out_motor_speed_[1] != out_target) {
                out_motor_speed_[1] = out_target;
                // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "target_left = %f", out_motor_speed_[1]);
            }
        }
        if (speeds != current_speeds_[1]) {
            current_speeds_[1] = speeds;
            // show               = true;
        }
        moto_ctrl_[1]->MotoSpeed(out_motor_speed_[1]);
    }

    if (rotary_encoder_[0]) {
        int32_t encoder = rotary_encoder_[0]->GetTicksRoll();
        float speeds    = encoder * distan_1circle_[0] / dt; // 计算右电机的速度 单位m/s
        if (pid_controller_[0]->get_target() == 0) {
            out_motor_speed_[0] = 0;
        } else {
            float out_target = pid_controller_[0]->update(speeds);
            if (out_motor_speed_[0] != out_target) {
                out_motor_speed_[0] = out_target;
                // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "target_right = %f", out_motor_speed_[0]);
            }
        }
        if (speeds != current_speeds_[0]) {
            current_speeds_[0] = speeds;
            // show               = true;
        }
        moto_ctrl_[0]->MotoSpeed(out_motor_speed_[0]);
    }

    float linear_speed, angular_speed;
    if (model_is_ackerman_) {
        if (show) {
            RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "Current speed = %fm/s", current_speeds_[0]);
        }
        // 阿克曼底盘运动学正解
        AckermanForward(current_speeds_[0], transient_theta_, linear_speed, angular_speed);
    } else {
        if (show) {
            RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "left = %fm/s\tright = %fm/s", current_speeds_[1], current_speeds_[0]);
        }
        // 双轮差速运动学正解
        TwoWheelForward(current_speeds_[1], current_speeds_[0], linear_speed, angular_speed);
    }
    // 更新里程计
    UpdateOdom(linear_speed, angular_speed, dt);

    return ret;
}

void Kinematics::TwoWheelInverse(float line_speed, float angle_speed, float &out_wheel1_speed, float &out_wheel2_speed)
{
    out_wheel1_speed = line_speed - (angle_speed * car_param_.TrendLength) / 2.0;
    out_wheel2_speed = line_speed + (angle_speed * car_param_.TrendLength) / 2.0;
}

void Kinematics::TwoWheelForward(float wheel1_speed, float wheel2_speed, float &line_speed, float &angle_speed)
{

    line_speed  = (wheel1_speed + wheel2_speed) / 2;                      // 运动学逆解得到线速度
    angle_speed = (wheel2_speed - wheel1_speed) / car_param_.TrendLength; // 运动学逆解得到角速度
}

void Kinematics::AckermanInverse(float v, float w, float &v_d, float &delta)
{
    float v_L = v - w * car_param_.WheelBase / 2;
    float v_R = v + w * car_param_.WheelBase / 2;
    float w_L = v_L / car_param_.LeftWheelRadius;
    float w_R = v_R / car_param_.RightWheelRadius;
    v_d       = (w_L * car_param_.LeftWheelRadius + w_R * car_param_.RightWheelRadius) / 2;
    delta     = (w_R * car_param_.RightWheelRadius - w_L * car_param_.LeftWheelRadius) / car_param_.WheelBase;
}

void Kinematics::AckermanForward(float v_d, float delta, float &v, float &w)
{
    v = v_d;
    w = v / car_param_.WheelBase * tan(delta);
}

bool Kinematics::CmdVel(float xvel, float wvel)
{
    if (model_is_ackerman_) {
        float wheel_speed = xvel;
        float angle       = wvel;
        if (xvel > car_param_.MaxXVel) {
            wheel_speed = car_param_.MaxXVel;
        } else if (xvel < -car_param_.MaxXVel) {
            wheel_speed = -car_param_.MaxXVel;
        }

        if (wvel > car_param_.MaxWVel) {
            angle = car_param_.MaxWVel + car_param_.ZeroAngle;
        } else if (wvel < -car_param_.MaxWVel) {
            angle = -car_param_.MaxWVel + car_param_.ZeroAngle;
        }

        float speed, target_angle;
        AckermanInverse(wheel_speed, angle, speed, target_angle);
        pid_controller_[0]->update_target(speed);
        // moto_ctrl_[0]->MotoSpeed(wheel_speed);
        servo_ctrl_->SetServo(target_angle);
        return true;
    } else {
        float vl = 0.0, vr = 0.0;
        float line_speed  = xvel;
        float angle_speed = wvel;
        if (xvel > car_param_.MaxXVel) {
            line_speed = car_param_.MaxXVel;
        } else if (xvel < -car_param_.MaxXVel) {
            line_speed = -car_param_.MaxXVel;
        }

        if (wvel > car_param_.MaxWVel) {
            angle_speed = car_param_.MaxWVel + car_param_.ZeroAngle;
        } else if (wvel < -car_param_.MaxWVel) {
            angle_speed = -car_param_.MaxWVel + car_param_.ZeroAngle;
        }

        TwoWheelInverse(line_speed, angle_speed, vl, vr);
        // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "target_left = %f\ttarget_right = %f", vl, vr);
        pid_controller_[1]->update_target(vl * 1000); // mm/s作为单位
        pid_controller_[0]->update_target(vr * 1000);
        // moto_ctrl_[1]->MotoSpeed(vl);
        // moto_ctrl_[0]->MotoSpeed(vr);
        return true;
    }

    return false;
}

bool Kinematics::DriverCtrl(float v_d, float delta)
{
    float speed = v_d;
    if (v_d > car_param_.MaxXVel) {
        speed = car_param_.MaxXVel;
    } else if (v_d < -car_param_.MaxXVel) {
        speed = -car_param_.MaxXVel;
    }

    float angle = delta + car_param_.ZeroAngle;
    if (delta > car_param_.MaxAngle) {
        angle = car_param_.MaxAngle + car_param_.ZeroAngle;
    } else if (delta < -car_param_.MaxAngle) {
        angle = -car_param_.MaxAngle + car_param_.ZeroAngle;
    }
    transient_theta_ = angle;
    angle            = angle / car_param_.MaxAngle;

    // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "speed = %f\tangle = %f", speed, angle);
    pid_controller_[0]->update_target(speed * 1000);
    // moto_ctrl_[0]->MotoSpeed(speed);
    servo_ctrl_->SetServo(angle);
    return true;
}

void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    if (angle > M_PI) {
        out_angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        out_angle += 2 * M_PI;
    }
}

bool Kinematics::SetImuPtr(std::shared_ptr<ImuInterface> imu_ptr)
{
    if (imu_ptr) {
        imu_data_ptr_ = imu_ptr;
        return true;
    }
    return false;
}

void Kinematics::UpdateOdom(float linear_speed, float angular_speed, float dt)
{
    std::lock_guard<std::mutex> mylock_guard(odom_lock_);
    odom_.linear_speed  = linear_speed; // 速度 m/s
    odom_.angular_speed = angular_speed;

    if (model_is_ackerman_ && imu_data_ptr_) {
        // 用IMU来校准yaw
        odom_.yaw = imu_data_ptr_->GetImuYaw();
    } else {
        odom_.yaw += odom_.angular_speed * dt;
        TransAngleInPI(odom_.yaw, odom_.yaw);
    }

    /*更新x和y轴上移动的距离*/
    float delta_distance = odom_.linear_speed * dt; // 单位m
    odom_.x += delta_distance * std::cos(odom_.yaw);
    odom_.y += delta_distance * std::sin(odom_.yaw);
    // 调用 Euler2Quaternion 函数，将机器人的欧拉角 yaw 转换为四元数 quaternion。
    Kinematics::Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
}

odom_t Kinematics::GetOdom()
{
    std::lock_guard<std::mutex> mylock_guard(odom_lock_);
    return odom_;
}

// 用于将欧拉角转换为四元数。
void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw, Quaternion &q)
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

void Kinematics::TestOpenCl()
{
#if USE_OPENCL
    cl_platform_id *platform;
    cl_uint num_platform;
    cl_int err;
    err = clGetPlatformIDs(0, NULL, &num_platform);
    if (err != CL_SUCCESS) {
        printf("CL fail!!\n");
        return;
    }
    platform = (cl_platform_id *)malloc(sizeof(cl_platform_id) * num_platform);
    err      = clGetPlatformIDs(num_platform, platform, NULL);
    if (err != CL_SUCCESS) {
        printf("CL fail!!\n");
        return;
    }

    for (uint i = 0; i < num_platform; i++) {
        size_t size;
        err = clGetPlatformInfo(platform[i], CL_PLATFORM_NAME, 0, NULL, &size);
        if (err != CL_SUCCESS) {
            printf("CL fail!!\n");
            return;
        }
        char *PName = (char *)malloc(size);
        err         = clGetPlatformInfo(platform[i], CL_PLATFORM_NAME, size, PName, NULL);
        if (err != CL_SUCCESS) {
            printf("CL fail!!\n");
            return;
        }
        printf("\nCL_PLATFORM_NAME:%s\n", PName);
        free(PName);
    }
#endif
}
