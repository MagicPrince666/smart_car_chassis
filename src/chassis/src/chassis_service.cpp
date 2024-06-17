#include <atomic>
#include <cmath>
#include <csignal>
#include <thread>
#include <unistd.h>

#include "chassis/chassis_service.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

#include "atk_ms901m.h"
#include "bts7960.h"
#include "interface.h"
#include "jsonparse.h"
#include "loadconfig.h"
#include "mpu6050.h"
#include "sbus.h"
#include "zyf176ex.h"

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
ChassisSrv::ChassisSrv(std::shared_ptr<ros::NodeHandle> node)
#else
ChassisSrv::ChassisSrv(std::shared_ptr<rclcpp::Node> node)
#endif
    : ros_node_(node)
{
    GetRemoteConfig();
    GetDriverConfig();
    GetImuConfig();
    GetOtherConfig();

    memset((uint8_t *)&rc_data_, 0, sizeof(rc_data_));

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    if (imu_data_ptr_) {
        ROS_INFO("%s imu start", imu_conf_.imu_module.c_str());
        imu_data_ptr_->Init();
        imu_pub_ = std::make_shared<ros::Publisher>(ros_node_->advertise<ImuMsg>("imu_data", 10));
    }
    odom_pub_       = std::make_shared<ros::Publisher>(ros_node_->advertise<OdometryMsg>("odom", 10));
    ultrasonic_pub_ = std::make_shared<ros::Publisher>(ros_node_->advertise<RangeMsg>("ultrasonic", 10));
    tof_pub_        = std::make_shared<ros::Publisher>(ros_node_->advertise<RangeMsg>("tof", 10));

    // ros_node_->setCallbackQueue(&callback_group_sub1_);
    cmd_vel_sub_ = std::make_shared<ros::Subscriber>(ros_node_->subscribe("cmd_vel", 10, &ChassisSrv::CmdVelCallback, this));

    imu_timer_ = ros_node_->createTimer(ros::Duration(0.01), std::bind(&ChassisSrv::FastTimerCallback, this));

    loop_timer_ = ros_node_->createTimer(ros::Duration(0.1), std::bind(&ChassisSrv::LoopCallback, this));

#else
    if (imu_data_ptr_) {
        RCLCPP_INFO(ros_node_->get_logger(), "%s imu start", imu_conf_.imu_module.c_str());
        imu_data_ptr_->Init();
        imu_pub_ = ros_node_->create_publisher<ImuMsg>("imu_data", 10);
    }
    // 使用回调组来并行接收回调
#if defined(USE_GALACTIC_VERSION) || defined(USE_HUMBLE_VERSION) || defined(USE_IRON_VERSION)
    callback_group_sub1_ = ros_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
#else
    callback_group_sub1_ = ros_node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
#endif
    auto sub1_opt           = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_sub1_;

    cmd_vel_sub_ = ros_node_->create_subscription<TwistMsg>(
        "cmd_vel", rclcpp::QoS(10), std::bind(&ChassisSrv::CmdVelCallback, this, std::placeholders::_1),
        sub1_opt);

    odom_pub_       = ros_node_->create_publisher<OdometryMsg>("odom", rclcpp::QoS(10));
    ultrasonic_pub_ = ros_node_->create_publisher<RangeMsg>("ultrasonic", rclcpp::QoS(10));
    tof_pub_        = ros_node_->create_publisher<RangeMsg>("tof", rclcpp::QoS(10));

    imu_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ChassisSrv::FastTimerCallback, this));
    loop_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ChassisSrv::LoopCallback, this));
#endif

    driver_enable_   = true;
    avoid_obstacles_ = true;
}

ChassisSrv::~ChassisSrv()
{
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    loop_timer_.stop();
    imu_timer_.stop();
#else
    imu_timer_->cancel();
    loop_timer_->cancel();
#endif
}

void ChassisSrv::GetImuConfig()
{
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros_node_->getParam("chassis/imu.module", imu_conf_.imu_module);
    ros_node_->getParam("chassis/imu.port", imu_conf_.imu_port);
    ros_node_->getParam("chassis/imu.baudrate", imu_conf_.imu_baudrate);
    ROS_INFO("imu_module = %s", imu_module.c_str());
    ROS_INFO("imu_port = %s", imu_port.c_str());
    ROS_INFO("imu_baudrate = %d", imu_baudrate);
#else
    ros_node_->declare_parameter("imu.module", "");
    ros_node_->get_parameter("imu.module", imu_conf_.imu_module);

    ros_node_->declare_parameter("imu.port", "");
    ros_node_->get_parameter("imu.port", imu_conf_.imu_port);

    ros_node_->declare_parameter("imu.baudrate", 115200);
    ros_node_->get_parameter("imu.baudrate", imu_conf_.imu_baudrate);
#endif

    if (!imu_conf_.imu_port.empty()) {
        if (imu_conf_.imu_module == "atk") {
            imu_data_ptr_ = std::make_shared<AtkMs901m>(imu_conf_.imu_port, imu_conf_.imu_baudrate);
            if (motion_ctl_) {
                motion_ctl_->SetImuPtr(imu_data_ptr_);
            }
        } else if (imu_conf_.imu_module == "zyz") {
            imu_data_ptr_ = std::make_shared<Zyf176ex>(imu_conf_.imu_port, imu_conf_.imu_baudrate);
        } else if (imu_conf_.imu_module == "mpu6050") {
            imu_data_ptr_ = std::make_shared<Mpu6050>(imu_conf_.imu_port, imu_conf_.imu_baudrate);
        } else {
            RCLCPP_ERROR(ros_node_->get_logger(), "%s imu is not support yet", imu_conf_.imu_module.c_str());
        }
    }
}

void ChassisSrv::GetRemoteConfig()
{
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros_node_->getParam("chassis/remote.type", config_.type);
    ros_node_->getParam("chassis/remote.port", config_.port);
    ros_node_->getParam("chassis/remote.baudrate", config_.baudrate);
    ros_node_->getParam("chassis/remote.data_len", config_.data_len);
    ros_node_->getParam("chassis/remote.joy_var_max", config_.joy_var_max);
    ros_node_->getParam("chassis/remote.joy_var_min", config_.joy_var_min);
    ros_node_->getParam("chassis/remote.max_x_vel", config_.max_x_vel);
    ros_node_->getParam("chassis/remote.max_w_vel", config_.max_w_vel);
    ros_node_->getParam("chassis/remote.max_angle", config_.max_angle);
#else
    ros_node_->declare_parameter("remote.type", "");
    ros_node_->get_parameter("remote.type", config_.type);

    ros_node_->declare_parameter("remote.port", "/dev/ttyUSB0");
    ros_node_->get_parameter("remote.port", config_.port);

    ros_node_->declare_parameter("remote.baudrate", 100000);
    ros_node_->get_parameter("remote.baudrate", config_.baudrate);

    ros_node_->declare_parameter("remote.data_len", 25);
    ros_node_->get_parameter("remote.data_len", config_.data_len);

    ros_node_->declare_parameter("remote.joy_var_max", 1800);
    ros_node_->get_parameter("remote.joy_var_max", config_.joy_var_max);

    ros_node_->declare_parameter("remote.joy_var_min", 200);
    ros_node_->get_parameter("remote.joy_var_min", config_.joy_var_min);

    ros_node_->declare_parameter("remote.max_x_vel", 1.0);
    ros_node_->get_parameter("remote.max_x_vel", config_.max_x_vel);

    ros_node_->declare_parameter("remote.max_w_vel", 1.0);
    ros_node_->get_parameter("remote.max_w_vel", config_.max_w_vel);

    ros_node_->declare_parameter("remote.max_angle", 1.0);
    ros_node_->get_parameter("remote.max_angle", config_.max_angle);
#endif
    if (config_.type == "sbus") {
        // 创建遥控工厂
        std::unique_ptr<RemoteFactory> factory(new SbusRemote());
        // 通过工厂方法创建sbus遥控产品
        std::shared_ptr<RemoteProduct> sbus(factory->CreateRemoteProduct(config_, false));
        remote_ = sbus;
        remote_type_ = REMOTE_SBUS;
    }
    // else if (config_.type == "gamepad") {
    //     std::unique_ptr<RemoteFactory> factory(new GamePadRemote());
    //     std::shared_ptr<RemoteProduct> gamepad(factory->CreateRemoteProduct(config_, false));
    //     remote_ = gamepad;
    //     remote_type_ = REMOTE_GAMEPAD;
    // } else if (config_.type == "keyboard") {
    //     // 创建遥控工厂
    //     std::unique_ptr<RemoteFactory> factory(new KeyBoardRemote());
    //     // 通过工厂方法创建键盘遥控产品
    //     std::shared_ptr<RemoteProduct> key(factory->CreateRemoteProduct(config_, false));
    //     remote_ = key;
    //     remote_type_ = REMOTE_KEYBOARD;
    // } else if (config_.type == "socket") {
    //     std::unique_ptr<RemoteFactory> factory(new UdpRemote());
    //     std::shared_ptr<RemoteProduct> udp_server(factory->CreateRemoteProduct(config_, false));
    //     remote_ = udp_server;
    //     remote_type_ = REMOTE_SOCKET;
    // } else if (config_.type == "sonnyps2") {
    //     std::unique_ptr<RemoteFactory> factory(new SonnyRemote());
    //     std::shared_ptr<RemoteProduct> ps2(factory->CreateRemoteProduct(config_, false));
    //     remote_ = ps2;
    //     remote_type_ = REMOTE_SONY_PS2;
    // } else {
    //     RCLCPP_ERROR(ros_node_->get_logger(), "please use an avlable remote");
    // }
}

void ChassisSrv::GetDriverConfig()
{
    std::string chip = "";
    PwmPram lidar_pwm;
    DriverParams car_param;
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros_node_->getParam("chassis/driver.chip", chip);

    ros_node_->getParam("chassis/driver.lidar_pwm.chip", lidar_pwm.chip);
    ros_node_->getParam("chassis/driver.lidar_pwm.channel", lidar_pwm.channel);
    ros_node_->getParam("chassis/driver.lidar_pwm.polarity", lidar_pwm.polarity);
    ros_node_->getParam("chassis/driver.lidar_pwm.period", lidar_pwm.period);
    ros_node_->getParam("chassis/driver.lidar_pwm.dutycycle", lidar_pwm.dutycycle);
    ros_node_->getParam("chassis/driver.RotaryEncoder", car_param.RotaryChanel);

    // 雷达转速控制
    lidar_speed_ = std::make_shared<Pwm>(lidar_pwm);

    ros_node_->getParam("chassis/car_param.LeftWheelRadius", car_param.LeftWheelRadius);
    ros_node_->getParam("chassis/car_param.RightWheelRadius", car_param.RightWheelRadius);
    ros_node_->getParam("chassis/car_param.TrendLength", car_param.TrendLength);
    ros_node_->getParam("chassis/car_param.WheelBase", car_param.WheelBase);
    ros_node_->getParam("chassis/car_param.Ticks1Roll", car_param.Ticks1Roll);
    ros_node_->getParam("chassis/car_param.MaxXVel", car_param.MaxXVel);
    ros_node_->getParam("chassis/car_param.MaxWVel", car_param.MaxWVel);
    ros_node_->getParam("chassis/car_param.MaxAngle", car_param.MaxAngle);
    ros_node_->getParam("chassis/car_param.ZeroAngle", car_param.ZeroAngle);
    ros_node_->getParam("chassis/car_param.MaxXAcc", car_param.MaxXAcc);
    ros_node_->getParam("chassis/car_param.MaxWAcc", car_param.MaxWAcc);
    ros_node_->getParam("chassis/car_param.MaxAngerLimit", car_param.MaxAngerLimit);
    ros_node_->getParam("chassis/car_param.MinAngerLimit", car_param.MinAngerLimit);
    ros_node_->getParam("chassis/car_param.Proportion", car_param.Proportion);
    ros_node_->getParam("chassis/car_param.Integration", car_param.Integration);
    ros_node_->getParam("chassis/car_param.Differentiation", car_param.Differentiation);

    if (chip == "Ackerman") {
        MotoInfo moto_info;
        PwmPram servo_pwm;
        ros_node_->getParam("driver.moto_pwm.chip", moto_info.moto_pwm.chip);
        ros_node_->getParam("driver.moto_pwm.channel", moto_info.moto_pwm.channel);
        ros_node_->getParam("driver.moto_pwm.polarity", moto_info.moto_pwm.polarity);
        ros_node_->getParam("driver.moto_pwm.period", moto_info.moto_pwm.period);
        ros_node_->getParam("driver.moto_pwm.dutycycle", moto_info.moto_pwm.dutycycle);

        ros_node_->getParam("driver.servo_pwm.chip", servo_pwm.chip);
        ros_node_->getParam("driver.servo_pwm.channel", servo_pwm.channel);
        ros_node_->getParam("driver.servo_pwm.polarity", servo_pwm.polarity);
        ros_node_->getParam("driver.servo_pwm.period", servo_pwm.period);
        ros_node_->getParam("driver.servo_pwm.dutycycle", servo_pwm.dutycycle);

        ros_node_->getParam("driver.moto_ena", moto_info.s_moto_ena);
        ros_node_->getParam("driver.moto_enb", moto_info.s_moto_enb);

        car_is_ackerman_ = true;
        motion_ctl_      = std::make_shared<Kinematics>(car_param, moto_info, servo_pwm);
    } else if (chip == "Two-wheel") {
        MotoInfo moto_left_info, moto_right_info;

        ros_node_->getParam("driver.moto_pwm_left.chip", moto_left_info.moto_pwm.chip);
        ros_node_->getParam("driver.moto_pwm_left.channel", moto_left_info.moto_pwm.channel);
        ros_node_->getParam("driver.moto_pwm_left.polarity", moto_left_info.moto_pwm.polarity);
        ros_node_->getParam("driver.moto_pwm_left.period", moto_left_info.moto_pwm.period);
        ros_node_->getParam("driver.moto_pwm_left.dutycycle", moto_left_info.moto_pwm.dutycycle);

        ros_node_->getParam("driver.moto_pwm_right.chip", moto_right_info.moto_pwm.chip);
        ros_node_->getParam("driver.moto_pwm_right.channel", moto_right_info.moto_pwm.channel);
        ros_node_->getParam("driver.moto_pwm_right.polarity", moto_right_info.moto_pwm.polarity);
        ros_node_->getParam("driver.moto_pwm_right.period", moto_right_info.moto_pwm.period);
        ros_node_->getParam("driver.moto_pwm_right.dutycycle", moto_right_info.moto_pwm.dutycycle);

        ros_node_->getParam("driver.moto2_ena", moto_left_info.s_moto_ena);
        ros_node_->getParam("driver.moto2_enb", moto_left_info.s_moto_enb);

        ros_node_->getParam("driver.moto1_ena", moto_right_info.s_moto_ena);
        ros_node_->getParam("driver.moto1_enb", moto_right_info.s_moto_enb);

        car_is_ackerman_ = false;
        motion_ctl_      = std::make_shared<Kinematics>(car_param, moto_left_info, moto_right_info);
    } else {
        RCLCPP_ERROR(ros_node_->get_logger(), "Not support yet!!");
    }
#else
    ros_node_->declare_parameter("driver.chip", "");
    ros_node_->get_parameter("driver.chip", chip);

    ros_node_->declare_parameter("driver.lidar_pwm.chip", 0);
    ros_node_->get_parameter("driver.lidar_pwm.chip", lidar_pwm.chip);
    ros_node_->declare_parameter("driver.lidar_pwm.channel", 0);
    ros_node_->get_parameter("driver.lidar_pwm.channel", lidar_pwm.channel);
    ros_node_->declare_parameter("driver.lidar_pwm.polarity", false);
    ros_node_->get_parameter("driver.lidar_pwm.polarity", lidar_pwm.polarity);
    ros_node_->declare_parameter("driver.lidar_pwm.period", 0);
    ros_node_->get_parameter("driver.lidar_pwm.period", lidar_pwm.period);
    ros_node_->declare_parameter("driver.lidar_pwm.dutycycle", 0);
    ros_node_->get_parameter("driver.lidar_pwm.dutycycle", lidar_pwm.dutycycle);
    ros_node_->declare_parameter("driver.RotaryEncoder", car_param.RotaryChanel);
    ros_node_->get_parameter("driver.RotaryEncoder", car_param.RotaryChanel);

    // 雷达转速控制
    lidar_speed_ = std::make_shared<Pwm>(lidar_pwm);

    ros_node_->declare_parameter("car_param.LeftWheelRadius", 0.0);
    ros_node_->get_parameter("car_param.LeftWheelRadius", car_param.LeftWheelRadius);
    ros_node_->declare_parameter("car_param.RightWheelRadius", 0.0);
    ros_node_->get_parameter("car_param.RightWheelRadius", car_param.RightWheelRadius);
    ros_node_->declare_parameter("car_param.TrendLength", 0.0);
    ros_node_->get_parameter("car_param.TrendLength", car_param.TrendLength);
    ros_node_->declare_parameter("car_param.WheelBase", 0.0);
    ros_node_->get_parameter("car_param.WheelBase", car_param.WheelBase);
    ros_node_->declare_parameter("car_param.Ticks1Roll", 0.0);
    ros_node_->get_parameter("car_param.Ticks1Roll", car_param.Ticks1Roll);
    ros_node_->declare_parameter("car_param.MaxXVel", 0.0);
    ros_node_->get_parameter("car_param.MaxXVel", car_param.MaxXVel);
    ros_node_->declare_parameter("car_param.MaxWVel", 0.0);
    ros_node_->get_parameter("car_param.MaxWVel", car_param.MaxWVel);
    ros_node_->declare_parameter("car_param.MaxAngle", 0.0);
    ros_node_->get_parameter("car_param.MaxAngle", car_param.MaxAngle);
    ros_node_->declare_parameter("car_param.ZeroAngle", 0.0);
    ros_node_->get_parameter("car_param.ZeroAngle", car_param.ZeroAngle);
    ros_node_->declare_parameter("car_param.MaxXAcc", 0.0);
    ros_node_->get_parameter("car_param.MaxXAcc", car_param.MaxXAcc);
    ros_node_->declare_parameter("car_param.MaxWAcc", 0.0);
    ros_node_->get_parameter("car_param.MaxWAcc", car_param.MaxWAcc);
    ros_node_->declare_parameter("car_param.MaxAngerLimit", 0.0);
    ros_node_->get_parameter("car_param.MaxAngerLimit", car_param.MaxAngerLimit);
    ros_node_->declare_parameter("car_param.MinAngerLimit", 0.0);
    ros_node_->get_parameter("car_param.MinAngerLimit", car_param.MinAngerLimit);
    ros_node_->declare_parameter("car_param.Proportion", 0.0);
    ros_node_->get_parameter("car_param.Proportion", car_param.Proportion);
    ros_node_->declare_parameter("car_param.Integration", 0.0);
    ros_node_->get_parameter("car_param.Integration", car_param.Integration);
    ros_node_->declare_parameter("car_param.Differentiation", 0.0);
    ros_node_->get_parameter("car_param.Differentiation", car_param.Differentiation);

    if (chip == "Ackerman") {
        MotoInfo moto_info;
        PwmPram servo_pwm;
        ros_node_->declare_parameter("driver.moto_pwm.chip", 0);
        ros_node_->get_parameter("driver.moto_pwm.chip", moto_info.moto_pwm.chip);
        ros_node_->declare_parameter("driver.moto_pwm.channel", 0);
        ros_node_->get_parameter("driver.moto_pwm.channel", moto_info.moto_pwm.channel);
        ros_node_->declare_parameter("driver.moto_pwm.polarity", false);
        ros_node_->get_parameter("driver.moto_pwm.polarity", moto_info.moto_pwm.polarity);
        ros_node_->declare_parameter("driver.moto_pwm.period", 0);
        ros_node_->get_parameter("driver.moto_pwm.period", moto_info.moto_pwm.period);
        ros_node_->declare_parameter("driver.moto_pwm.dutycycle", 0);
        ros_node_->get_parameter("driver.moto_pwm.dutycycle", moto_info.moto_pwm.dutycycle);

        ros_node_->declare_parameter("driver.servo_pwm.chip", 0);
        ros_node_->get_parameter("driver.servo_pwm.chip", servo_pwm.chip);
        ros_node_->declare_parameter("driver.servo_pwm.channel", 0);
        ros_node_->get_parameter("driver.servo_pwm.channel", servo_pwm.channel);
        ros_node_->declare_parameter("driver.servo_pwm.polarity", false);
        ros_node_->get_parameter("driver.servo_pwm.polarity", servo_pwm.polarity);
        ros_node_->declare_parameter("driver.servo_pwm.period", 0);
        ros_node_->get_parameter("driver.servo_pwm.period", servo_pwm.period);
        ros_node_->declare_parameter("driver.servo_pwm.dutycycle", 0);
        ros_node_->get_parameter("driver.servo_pwm.dutycycle", servo_pwm.dutycycle);

        ros_node_->declare_parameter("driver.moto_ena", "");
        ros_node_->get_parameter("driver.moto_ena", moto_info.s_moto_ena);
        ros_node_->declare_parameter("driver.moto_enb", "");
        ros_node_->get_parameter("driver.moto_enb", moto_info.s_moto_enb);

        car_is_ackerman_ = true;
        motion_ctl_      = std::make_shared<Kinematics>(car_param, moto_info, servo_pwm);
    } else if (chip == "Two-wheel") {
        MotoInfo moto_left_info, moto_right_info;

        ros_node_->declare_parameter("driver.moto_pwm_left.chip", 0);
        ros_node_->get_parameter("driver.moto_pwm_left.chip", moto_left_info.moto_pwm.chip);
        ros_node_->declare_parameter("driver.moto_pwm_left.channel", 0);
        ros_node_->get_parameter("driver.moto_pwm_left.channel", moto_left_info.moto_pwm.channel);
        ros_node_->declare_parameter("driver.moto_pwm_left.polarity", false);
        ros_node_->get_parameter("driver.moto_pwm_left.polarity", moto_left_info.moto_pwm.polarity);
        ros_node_->declare_parameter("driver.moto_pwm_left.period", 0);
        ros_node_->get_parameter("driver.moto_pwm_left.period", moto_left_info.moto_pwm.period);
        ros_node_->declare_parameter("driver.moto_pwm_left.dutycycle", 0);
        ros_node_->get_parameter("driver.moto_pwm_left.dutycycle", moto_left_info.moto_pwm.dutycycle);

        ros_node_->declare_parameter("driver.moto_pwm_right.chip", 0);
        ros_node_->get_parameter("driver.moto_pwm_right.chip", moto_right_info.moto_pwm.chip);
        ros_node_->declare_parameter("driver.moto_pwm_right.channel", 0);
        ros_node_->get_parameter("driver.moto_pwm_right.channel", moto_right_info.moto_pwm.channel);
        ros_node_->declare_parameter("driver.moto_pwm_right.polarity", false);
        ros_node_->get_parameter("driver.moto_pwm_right.polarity", moto_right_info.moto_pwm.polarity);
        ros_node_->declare_parameter("driver.moto_pwm_right.period", 0);
        ros_node_->get_parameter("driver.moto_pwm_right.period", moto_right_info.moto_pwm.period);
        ros_node_->declare_parameter("driver.moto_pwm_right.dutycycle", 0);
        ros_node_->get_parameter("driver.moto_pwm_right.dutycycle", moto_right_info.moto_pwm.dutycycle);

        ros_node_->declare_parameter("driver.moto2_ena", "");
        ros_node_->get_parameter("driver.moto2_ena", moto_left_info.s_moto_ena);
        ros_node_->declare_parameter("driver.moto2_enb", "");
        ros_node_->get_parameter("driver.moto2_enb", moto_left_info.s_moto_enb);

        ros_node_->declare_parameter("driver.moto1_ena", "");
        ros_node_->get_parameter("driver.moto1_ena", moto_right_info.s_moto_ena);
        ros_node_->declare_parameter("driver.moto1_enb", "");
        ros_node_->get_parameter("driver.moto1_enb", moto_right_info.s_moto_enb);

        car_is_ackerman_ = false;
        motion_ctl_      = std::make_shared<Kinematics>(car_param, moto_left_info, moto_right_info);
    } else {
        RCLCPP_ERROR(ros_node_->get_logger(), "Not support yet!!");
    }
#endif
}

void ChassisSrv::GetOtherConfig()
{
    bool sonar = false;
    bool tof = false;
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    bool live_video;
    ros_node_->getParam("chassis/video_device.live_video", live_video);

    std::string video_device;
    ros_node_->getParam("chassis/video_device.dev", video_device);

    ros_node_->getParam("chassis/srf04", sonar);
    ros_node_->getParam("chassis/vl53l0x", tof);
#else
    ros_node_->declare_parameter("srf04", false);
    ros_node_->get_parameter("srf04", sonar);
    ros_node_->declare_parameter("vl53l0x", false);
    ros_node_->get_parameter("vl53l0x", tof);
#endif
    if (sonar) {
        ultrasonic_ = std::make_shared<Srf04>();
    }

    if (tof) {
        back_distance_ = std::make_shared<Vl53l0x>();
    }
}

void ChassisSrv::CmdVelCallback(const TwistMsg::SharedPtr msg)
{
    // RCLCPP_INFO(ros_node_->get_logger(), "linear: [%f]\tangular : [%f]", msg->linear.x, msg->angular.z);
    if (car_is_ackerman_) {
        motion_ctl_->DriverCtrl(msg->linear.x, msg->angular.z);
    } else {
        motion_ctl_->CmdVel(msg->linear.x, msg->angular.z);
    }
}

void ChassisSrv::FastTimerCallback()
{
    if (imu_data_ptr_) {
        // 定义IMU数据
        ImuMsg imu_msg;
        imu_msg.header.stamp    = GetTimeNow();
        imu_msg.header.frame_id = "imu_link";
        Imu get_imu             = imu_data_ptr_->GetImuData();

        imu_msg.orientation.w         = get_imu.orientation.w;
        imu_msg.orientation.x         = get_imu.orientation.x;
        imu_msg.orientation.y         = get_imu.orientation.y;
        imu_msg.orientation.z         = get_imu.orientation.z;
        imu_msg.angular_velocity.x    = get_imu.angular_velocity.x;
        imu_msg.angular_velocity.y    = get_imu.angular_velocity.y;
        imu_msg.angular_velocity.z    = get_imu.angular_velocity.z;
        imu_msg.linear_acceleration.x = get_imu.linear_acceleration.x;
        imu_msg.linear_acceleration.y = get_imu.linear_acceleration.y;
        imu_msg.linear_acceleration.z = get_imu.linear_acceleration.z;

        for (size_t i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i]         = 0;
            imu_msg.angular_velocity_covariance[i]    = 0;
            imu_msg.linear_acceleration_covariance[i] = 0;
        }

        imu_pub_->publish(imu_msg);
    }
}

void ChassisSrv::LoopCallback()
{
    int font_dis = 1000;
    int back_dis = 1000;

    PubOdom();

    if (remote_) {
        remote_->Request(rc_data_);
    } else {
        return;
    }

    if (ultrasonic_) {
        font_dis = ultrasonic_->GetDistance();
        RangeMsg sonar_msg;
        sonar_msg.header.stamp    = GetTimeNow();
        sonar_msg.header.frame_id = "sonar_link";
        sonar_msg.radiation_type  = RangeMsg::ULTRASOUND;
        sonar_msg.field_of_view   = 0.52;
        sonar_msg.min_range       = 0.02;
        sonar_msg.max_range       = 4.5;
        sonar_msg.range           = (double)font_dis / 100.0;
        ultrasonic_pub_->publish(sonar_msg);
    }

    if (back_distance_) {
        back_dis = back_distance_->GetDistance();
        RangeMsg tof_msg;
        tof_msg.header.stamp    = GetTimeNow();
        tof_msg.header.frame_id = "tof_link";
        tof_msg.radiation_type  = RangeMsg::INFRARED;
        tof_msg.field_of_view   = 0.01;
        tof_msg.min_range       = 0.1;
        tof_msg.max_range       = 2.0;
        tof_msg.range           = (double)back_dis / 100.0;
        tof_pub_->publish(tof_msg);
    }

    if (!rc_data_.adslx || !rc_data_.adsly || !rc_data_.adsrx || !rc_data_.adsry) {
        RCLCPP_WARN(ros_node_->get_logger(), "Driver lx = %f, ly = %f, rx = %f, ry = %f",
                     rc_data_.adslx, rc_data_.adsly, rc_data_.adsrx, rc_data_.adsry);
        return;
    }

    if (rc_data_.ads[4] > 0.5) { // 使能开关
        motion_ctl_->DriverCtrl(0.0, 0.0);
        if (driver_enable_) {
            driver_enable_ = false;
            RCLCPP_WARN(ros_node_->get_logger(), "Driver disable");
        }
        return;
    } else {
        if (!driver_enable_) {
            driver_enable_ = true;
            RCLCPP_INFO(ros_node_->get_logger(), "Driver enable");
        }
    }

    if (rc_data_.ads[5] > 0.5) { // 避障开关
        font_dis = 1000;
        back_dis = 1000;
        if (avoid_obstacles_) {
            avoid_obstacles_ = false;
            RCLCPP_WARN(ros_node_->get_logger(), "Avoid obstacles disable");
        }
    } else {
        if (!avoid_obstacles_) {
            avoid_obstacles_ = true;
            RCLCPP_INFO(ros_node_->get_logger(), "Avoid obstacles enable");
        }
    }

    float vspeed = (1 - 2.0 * rc_data_.adsry) * config_.max_x_vel; // 右摇杆y轴 线速度
    if (rc_data_.adsry == 0) {
        vspeed = 0.0;
    }

    if (font_dis > 0 && font_dis <= 400 && vspeed > 0) { // 前避障
        RCLCPP_WARN(ros_node_->get_logger(), "Obstacle ahead distance = %d mm", font_dis);
        vspeed = 0.0;
    }

    if (back_dis > 20 && back_dis <= 100 && vspeed < 0) { // 后避障
        RCLCPP_WARN(ros_node_->get_logger(), "Obstacle behind distance = %d mm", back_dis);
        vspeed = 0.0;
    }
    if (vspeed > -0.01 && vspeed < 0.01) {
        vspeed = 0.0;
    }

    if (car_is_ackerman_) {
        float angle = (1 - 2.0 * rc_data_.adsrx) * config_.max_angle; // 右摇杆x轴 转向角度
        motion_ctl_->DriverCtrl(vspeed, angle);
    } else {
        float angle = (1 - 2.0 * rc_data_.adsrx) * config_.max_w_vel; // 右摇杆x轴 角速度
        if (angle > -0.01 && angle < 0.01) {
            angle = 0.0;
        }
        motion_ctl_->CmdVel(vspeed, angle);
    }
}

void ChassisSrv::PubOdom()
{
    odom_t odom = motion_ctl_->GetOdom();
    OdometryMsg message;
    message.header.stamp    = GetTimeNow();
    message.header.frame_id = "odom";
    message.child_frame_id  = "base_link";

    // position
    message.pose.pose.position.x    = odom.x;
    message.pose.pose.position.y    = odom.y;
    message.pose.pose.position.z    = 0.0;
    message.pose.pose.orientation.w = odom.quaternion.w;
    message.pose.pose.orientation.x = odom.quaternion.x;
    message.pose.pose.orientation.y = odom.quaternion.y;
    message.pose.pose.orientation.z = odom.quaternion.z;

    // velocity
    message.twist.twist.linear.x  = odom.linear_speed;
    message.twist.twist.linear.y  = 0.0;
    message.twist.twist.linear.z  = 0.0;
    message.twist.twist.angular.x = 0.0;
    message.twist.twist.angular.y = 0.0;
    message.twist.twist.angular.z = odom.angular_speed;

    odom_pub_->publish(message);
}

uint32_t ChassisSrv::GetCurrentMsTime()
{
    auto current_time            = std::chrono::high_resolution_clock::now();
    auto duration_in_nanoseconds = std::chrono::duration_cast<std::chrono::microseconds>(current_time.time_since_epoch());
    return duration_in_nanoseconds.count();
}

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
ros::Time ChassisSrv::GetTimeNow()
{
    return ros::Time::now();
}
#else
rclcpp::Time ChassisSrv::GetTimeNow()
{
    return ros_node_->get_clock()->now();
}
#endif
