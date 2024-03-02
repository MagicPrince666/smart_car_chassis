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

#ifdef USE_LIVE555
#include "OnDemandRTSPServer.h"
#else
#include "v4l2_rtsp.h"
#endif
#include "Gamepad.hpp"
#include "atk_ms901m.h"
#include "bts7960.h"
#include "interface.h"
#include "jsonparse.h"
#include "loadconfig.h"
#include "mpu6050.h"
#include "mpu9250.h"
#include "sbus.h"
#include "socket.h"
#include "sonnyps2.h"
#include "zyf176ex.h"

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
ChassisSrv::ChassisSrv(std::shared_ptr<ros::NodeHandle> node)
#else
ChassisSrv::ChassisSrv(std::shared_ptr<rclcpp::Node> node)
#endif
    : ros_node_(node)
{
    Json::Value conf_json;
    JsoncppParseRead::ReadFileToJson(CONFIG_FILE_PATH, conf_json);
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    std::string imu_module;
    ros_node_->getParam("chassis/imu_module", imu_module);
    spdlog::info("imu_module = {}", imu_module.c_str());

    std::string imu_port;
    ros_node_->getParam("chassis/imu_port", imu_port);
    spdlog::info("imu_port = {}", imu_port.c_str());

    int imu_baudrate;
    ros_node_->getParam("chassis/imu_baudrate", imu_baudrate);
    spdlog::info("imu_baudrate = {}", imu_baudrate);

    ros_node_->getParam("type", config_.type);
    spdlog::info("type = {}", config_.type.c_str());

    ros_node_->getParam("chassis/remote_port", config_.port);
    spdlog::info("remote_port = {}", config_.port.c_str());

    ros_node_->getParam("chassis/baudrate", config_.baudrate);
    spdlog::info("baudrate = {}", config_.baudrate);

    ros_node_->getParam("chassis/data_len", config_.data_len);
    spdlog::info("data_len = {}", config_.data_len);

    ros_node_->getParam("chassis/joy_var_max", config_.joy_var_max);
    spdlog::info("joy_var_max = {}", config_.joy_var_max);

    ros_node_->getParam("chassis/joy_var_min", config_.joy_var_min);
    spdlog::info("joy_var_min = {}", config_.joy_var_min);

    ros_node_->getParam("chassis/max_x_vel", config_.max_x_vel);
    spdlog::info("max_x_vel = {}", config_.max_x_vel);

    ros_node_->getParam("chassis/max_w_vel", config_.max_w_vel);
    spdlog::info("max_w_vel = {}", config_.max_w_vel);

    ros_node_->getParam("chassis/max_angle", config_.max_angle);
    spdlog::info("max_angle = {}", config_.max_angle);

    bool record_data;
    ros_node_->getParam("chassis/record_data", record_data);
    spdlog::info("record_data = {}", record_data);

    bool live_video;
    ros_node_->getParam("chassis/live_video", live_video);
    spdlog::info("live_video = {}", live_video);

    std::string video_device;
    ros_node_->getParam("chassis/video_device", video_device);
    spdlog::info("video_device = {}", video_device.c_str());
#else
    std::string imu_module;
    ros_node_->declare_parameter("imu_module", "");
    ros_node_->get_parameter("imu_module", imu_module);
    spdlog::info("imu_module = {}", imu_module.c_str());

    std::string imu_port;
    ros_node_->declare_parameter("imu_port", "");
    ros_node_->get_parameter("imu_port", imu_port);
    spdlog::info("imu_port = {}", imu_port.c_str());

    int imu_baudrate;
    ros_node_->declare_parameter("imu_baudrate", 115200);
    ros_node_->get_parameter("imu_baudrate", imu_baudrate);
    spdlog::info("imu_baudrate = {}", imu_baudrate);

    ros_node_->declare_parameter("type", "");
    ros_node_->get_parameter("type", config_.type);
    spdlog::info("type = {}", config_.type.c_str());

    ros_node_->declare_parameter("remote_port", "/dev/ttyUSB0");
    ros_node_->get_parameter("remote_port", config_.port);
    spdlog::info("remote_port = {}", config_.port.c_str());

    ros_node_->declare_parameter("baudrate", 100000);
    ros_node_->get_parameter("baudrate", config_.baudrate);
    spdlog::info("baudrate = {}", config_.baudrate);

    ros_node_->declare_parameter("data_len", 25);
    ros_node_->get_parameter("data_len", config_.data_len);
    spdlog::info("data_len = {}", config_.data_len);

    ros_node_->declare_parameter("joy_var_max", 1800);
    ros_node_->get_parameter("joy_var_max", config_.joy_var_max);
    spdlog::info("joy_var_max = {}", config_.joy_var_max);

    ros_node_->declare_parameter("joy_var_min", 200);
    ros_node_->get_parameter("joy_var_min", config_.joy_var_min);
    spdlog::info("joy_var_min = {}", config_.joy_var_min);

    ros_node_->declare_parameter("max_x_vel", 1.0);
    ros_node_->get_parameter("max_x_vel", config_.max_x_vel);
    spdlog::info("max_x_vel = {}", config_.max_x_vel);

    ros_node_->declare_parameter("max_w_vel", 1.0);
    ros_node_->get_parameter("max_w_vel", config_.max_w_vel);
    spdlog::info("max_w_vel = {}", config_.max_w_vel);

    ros_node_->declare_parameter("max_angle", 1.0);
    ros_node_->get_parameter("max_angle", config_.max_angle);
    spdlog::info("max_angle = {}", config_.max_angle);

    bool record_data;
    ros_node_->declare_parameter("record_data", false);
    ros_node_->get_parameter("record_data", record_data);
    spdlog::info("record_data = {}", record_data);

    bool live_video;
    ros_node_->declare_parameter("live_video", false);
    ros_node_->get_parameter("live_video", live_video);
    spdlog::info("live_video = {}", live_video);

    std::string video_device;
    ros_node_->declare_parameter("video_device", "");
    ros_node_->get_parameter("video_device", video_device);
    spdlog::info("video_device = {}", video_device.c_str());
#endif

    if (record_data) {
        record_data_ = std::make_shared<RecordData>("speed_ctrl.csv");
        record_data_->PushToFile("time", "action", "response");
    }

    std::string chip = "";
    if (conf_json.isMember("chip") && conf_json["chip"].isString()) {
        chip = conf_json["chip"].asString();
    }
    std::unique_ptr<LoadConfig> config(new LoadConfig);

    // 雷达转速控制
    PwmPram lidar_pwm;
    config->LoadPwmConfig("lidar_pwm", lidar_pwm);
    lidar_speed_ = std::make_shared<Pwm>(lidar_pwm);

    if (config_.type == "sbus") {
        // 创建遥控工厂
        std::unique_ptr<RemoteFactory> factory(new SbusRemote());
        // 通过工厂方法创建sbus遥控产品
        std::shared_ptr<RemoteProduct> sbus(factory->CreateRemoteProduct(config_, false));
        remote_ = sbus;
    } else if (config_.type == "gamepad") {
        std::unique_ptr<RemoteFactory> factory(new GamePadRemote());
        std::shared_ptr<RemoteProduct> gamepad(factory->CreateRemoteProduct(config_, false));
        remote_ = gamepad;
    } else if (config_.type == "keyboard") {
        // 创建遥控工厂
        std::unique_ptr<RemoteFactory> factory(new KeyBoardRemote());
        // 通过工厂方法创建键盘遥控产品
        std::shared_ptr<RemoteProduct> key(factory->CreateRemoteProduct(config_, false));
        remote_ = key;
    } else if (config_.type == "socket") {
        std::unique_ptr<RemoteFactory> factory(new UdpRemote());
        std::shared_ptr<RemoteProduct> udp_server(factory->CreateRemoteProduct(config_, false));
        remote_ = udp_server;
    } else if (config_.type == "sonnyps2") {
        std::unique_ptr<RemoteFactory> factory(new SonnyRemote());
        std::shared_ptr<RemoteProduct> ps2(factory->CreateRemoteProduct(config_, false));
        remote_ = ps2;
    } else {
        spdlog::error("please use an avlable remote");
    }

    bool sonar = false;
    config->LoadDistanceConfig("srf04", sonar);
    if (sonar) {
        spdlog::info("Enable srf04");
        ultrasonic_ = std::make_shared<Srf04>();
    }

    bool tof = false;
    config->LoadDistanceConfig("vl53l0x", tof);
    if (tof) {
        spdlog::info("Enable vl53l0x");
        back_distance_ = std::make_shared<Vl53l0x>();
    }

    if (chip == "Ackerman") {
        MotoInfo moto_info;
        config->LoadPwmConfig("moto_pwm", moto_info.moto_pwm);
        config->LoadPinConfig("moto_ena", moto_info.s_moto_ena);
        config->LoadPinConfig("moto_enb", moto_info.s_moto_enb);
        config->LoadPinConfig("moto_ena", moto_info.i_moto_ena);
        config->LoadPinConfig("moto_enb", moto_info.i_moto_enb);

        PwmPram servo_pwm;
        config->LoadPwmConfig("servo_pwm", servo_pwm);

        DriverParams car_param;
        config->LoadCarConfig("yaml", car_param);

        car_is_ackerman_ = true;
        motion_ctl_      = std::make_shared<Kinematics>(car_param, moto_info, servo_pwm);
    } else if (chip == "Two-wheel") {
        MotoInfo moto_left_info, moto_right_info;
        config->LoadPwmConfig("moto_pwm_left", moto_left_info.moto_pwm);
        config->LoadPwmConfig("moto_pwm_right", moto_right_info.moto_pwm);

        config->LoadPinConfig("moto2_ena", moto_left_info.s_moto_ena);
        config->LoadPinConfig("moto2_enb", moto_left_info.s_moto_enb);

        config->LoadPinConfig("moto1_ena", moto_right_info.s_moto_ena);
        config->LoadPinConfig("moto1_enb", moto_right_info.s_moto_enb);

        DriverParams car_param;
        config->LoadCarConfig("yaml", car_param);

        car_is_ackerman_ = false;
        motion_ctl_      = std::make_shared<Kinematics>(car_param, moto_left_info, moto_right_info);
    } else {
        spdlog::error("Not support yet!!");
    }

    memset((uint8_t *)&rc_data_, 0, sizeof(rc_data_));

    if (!imu_port.empty()) {
        if (imu_module == "atk") {
            imu_data_ptr_ = std::make_shared<AtkMs901m>(imu_port, imu_baudrate);
            motion_ctl_->SetImuPtr(imu_data_ptr_);
        } else if (imu_module == "zyz") {
            imu_data_ptr_ = std::make_shared<Zyf176ex>(imu_port, imu_baudrate);
        } else if (imu_module == "mpu6050") {
            imu_data_ptr_ = std::make_shared<Mpu6050>(imu_port, imu_baudrate);
        } else if (imu_module == "mpu9250") {
            imu_data_ptr_ = std::make_shared<Mpu9250>(imu_port, imu_baudrate);
        } else {
            spdlog::error("{} imu is not support yet", imu_module.c_str());
        }
    }

    if (live_video) {
        live_video_ = std::make_shared<V4l2Rtsp>(video_device);
        live_video_->Init();
    }

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    if (imu_data_ptr_) {
        spdlog::info("{} imu start", imu_module.c_str());
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
        spdlog::info("{} imu start", imu_module.c_str());
        imu_data_ptr_->Init();
        imu_pub_ = ros_node_->create_publisher<ImuMsg>("imu_data", 10);
    }
    // 使用回调组来并行接收回调
#if defined(USE_HUMBLE_VERSION) || defined(USE_IRON_VERSION)
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

    // music_player_ = std::make_shared<Music>("default");
    // music_player_->Init();
    // music_player_->PushToPlayList("12345.mp3");
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

void ChassisSrv::CmdVelCallback(const TwistMsg::SharedPtr msg)
{
    // spdlog::info("linear: [{}]\tangular : [{}]", msg->linear.x, msg->angular.z);
    if (record_data_) {
        recording_input_ = msg->linear.x;
    }
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
        spdlog::warn("Driver lx = {}, ly = {}, rx = {}, ry = {}",
                     rc_data_.adslx, rc_data_.adsly, rc_data_.adsrx, rc_data_.adsry);
        return;
    }

    if (rc_data_.ads[4] > 0.5) { // 使能开关
        motion_ctl_->DriverCtrl(0.0, 0.0);
        if (driver_enable_) {
            driver_enable_ = false;
            spdlog::warn("Driver disable");
        }
        return;
    } else {
        if (!driver_enable_) {
            driver_enable_ = true;
            spdlog::info("Driver enable");
        }
    }

    if (rc_data_.ads[5] > 0.5) { // 避障开关
        font_dis = 1000;
        back_dis = 1000;
        if (avoid_obstacles_) {
            avoid_obstacles_ = false;
            spdlog::warn("Avoid obstacles disable");
        }
    } else {
        if (!avoid_obstacles_) {
            avoid_obstacles_ = true;
            spdlog::info("Avoid obstacles enable");
        }
    }

    float vspeed = (1 - 2.0 * rc_data_.adsry) * config_.max_x_vel; // 右摇杆y轴 线速度
    if (rc_data_.adsry == 0) {
        vspeed = 0.0;
    }

    if (font_dis > 0 && font_dis <= 400 && vspeed > 0) { // 前避障
        spdlog::warn("Obstacle ahead distance = {} mm", font_dis);
        vspeed = 0.0;
    }

    if (back_dis > 20 && back_dis <= 100 && vspeed < 0) { // 后避障
        spdlog::warn("Obstacle behind distance = {} mm", back_dis);
        vspeed = 0.0;
    }
    if (vspeed > -0.01 && vspeed < 0.01) {
        vspeed = 0.0;
    }

    if (record_data_) {
        recording_input_ = vspeed;
    }

    if (car_is_ackerman_) {
        float angle = (1 - 2.0 * rc_data_.adsrx) * config_.max_angle; // 右摇杆x轴 转向角度
        // spdlog::info("speed = {}ms\tangle = {}", vspeed, angle);
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

    if (record_data_) {
        record_data_->PushToFile(GetCurrentMsTime(), recording_input_, odom.linear_speed);
    }
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
