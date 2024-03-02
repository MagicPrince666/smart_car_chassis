# 芒果派D1 小车
双轮差速模型
## 状态
进行中...

## 硬件连接
| 连接设备       | name          | PIN | PIN | name         | 连接设备       |
|---------------|---------      |-----|-----|----------    |---------------|
| NC            | 3.3v          | 1   | 2   | 5v           | NC            |
| 电机2使能脚2   | PG13/SDA      | 3   | 4   | 5v           | NC            |
| 电机2使能脚1   | PG12/SCL      | 5   | 6   | GND          | NC            |
| 电机1使能脚1   | PB7           | 7   | 8   | TX0          | 调试串口tx     |
| NC            | GND           | 9   | 10  | RX0          | 调试串口rx     |
| NC            | PD21/TX1      | 11  | 12  | PB5/PWM0/RX5 | 雷达速度控制   |
| PWM7          | PD22/RX1/PWM7 | 13  | 14  | GND          | NC            |
| 电机2调速      | PB0/PWM3      | 15  | 16  | PB1/PWM4     | 电机1调速      |
| NC            | 3.3v          | 17  | 18  | PD14         | 电机1编码器A相 |
| MOSI          | MOSI          | 19  | 20  | GND          | NC            |
| MISO          | MISO          | 21  | 22  | PC1/RX2      | Lidar RX      |
| SCLK          | SCLK          | 23  | 24  | CS0          | Lcd Cs        |
| NC            | GND           | 25  | 26  | CS1          | Touch Cs      |
| MPU6050 SDA   | PE17/SDA3     | 27  | 28  | PE16/SCL3    | MPU6050 SCL   |
| Lcd Dc        | PB10/PWM7     | 29  | 30  | GND          | NC            |
| PWM2          | PB11/PWM2     | 31  | 32  | PC0/TX2      | TX2           |
| Touch Int     | PB12/PWM0     | 33  | 34  | GND          | NC            |
| PWM1          | PB6/PWM1      | 35  | 36  | PB2/TX4      | 电机1编码器B相 |
| MPU6050 Int   | PD17/TX4      | 37  | 38  | PB3/RX4      | 电机2编码器A相 |
| NC            | GND           | 39  | 40  | PB4/TX5      | 电机2编码器B相 |

## BTS7960驱动说明
* 方法1：
VCC接单片机的5V电源，GND接单片机的GND。

REN与LEN短路井接5V电平，驱动器可以工作。

LPWM,输入PWM信号或高电平电机正转。

RPWM，输入PWM信号或高电平电机反转。

* 方法2(推荐)：

vCC接单片机的5V电源，GND接单片机的GND。

REN与LEN短路并接输入PWM信号调速。

LPWM，脚输入5V电平电机正转。

RPWM，脚输入5V电平电机反转。

## 编译MQ PRO
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_mqpro.cmake ..
$ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_mqpro.cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## 设备树配置
开发需要更新设备树配置

[MangoPi MQ PRO devicetree](https://github.com/MagicPrince666/Tina-Linux/blob/main/device/config/chips/d1/configs/mq_pro/linux-5.4/board.dts)

## 已完成
1. 遥控接收机sbus解码
2. 陀螺仪数据获取
3. 电机正反转IO控制
4. 获取USB摄像头数据
5. rtsp 视频直播
6. 超声波传感器数据获取
7. 通过配置文件（json）适配硬件，与软件代码无关
8. 监控文件发生变化
9. 支持GPS定位模块数据获取
10. 更换带码盘电机
11. 支持USB手柄
12. IMU位姿解算

## TO DO
1. 电机支持PID控制
2. 图像识别算法引入
3. 支持udp控制
4. 新增双轮差速底盘

## 网络连接
```
network={
    ssid="OpenWrt_R619ac_2.4G"
    psk="67123236"
}

wifi_connect_ap_test OpenWrt_R619ac_2.4G 67123236

softap_up <ssid> <pwd> <broadcast/hidden>
softap_down
```

```
tftp -g -r SmartCar 192.168.3.184
```