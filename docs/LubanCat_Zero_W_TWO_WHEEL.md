# 鲁班猫zero w 智能小车

## 底盘模型
双轮差速底盘

## 状态
进行中...

## 硬件连接
| 连接设备       | name          | PIN | PIN | name          | 连接设备       |
|---------------|---------      |-----|-----|----------     |---------------|
| NC            | 3.3v          | 1   | 2   | 5v            | NC            |
| 右电机编码器A相| GPIO1_A0/RX3  | 3   | 4   | 5v            | NC            |
| 右电机编码器B相| GPIO1_A1/TX3  | 5   | 6   | GND           | NC            |
| 左电机使能脚2  | GPIO1_A2      | 7   | 8   | GPIO2_C5/TX8  | NC            |
| NC            | GND           | 9   | 10  | GPIO2_C6/RX8  | 雷达数据接收脚 |
| NC            | GPIO1_A3      | 11  | 12  | GPIO0_C2/PWM3 | 左电机使能脚1  |
| 遥控SBUS接收   | GPIO1_A4      | 13  | 14  | GND           | NC            |
| MPU中断脚      | GPIO1_A5      | 15  | 16  | GPIO2_C3      | 触摸屏中断脚   |
| NC            | 3.3v          | 17  | 18  | GPIO2_C4      | 右电机使能脚1  |
| MOSI          | GPIO4_C3/MOSI | 19  | 20  | GND           | NC            |
| MISO          | GPIO4_C5/MISO | 21  | 22  | GPIO1_B1      | NC            |
| SCLK          | GPIO4_C2/SCLK | 23  | 24  | GPIO4_C6/CS0  | Touch Cs      |
| NC            | GND           | 25  | 26  | GPIO1_B3/CS1  | Lcd Cs        |
| MPU6050 SDA   | GPIO3_B4/SDA5 | 27  | 28  | GPIO3_B3/SCL5 | MPU6050 SCL   |
| NC            | GPIO1_A7      | 29  | 30  | GND           | NC            |
| Lcd Bl        | GPIO1_B0      | 31  | 32  | GPIO3_B6/PWM11| 雷达调速       |
| 左电机调速     | GPIO3_B1/PWM8 | 33  | 34  | GND           | NC            |
| 右电机调速     | GPIO3_B2/PWM9 | 35  | 36  | GPIO3_A5      | 右电机使能脚2  |
| LCD DC        | GPIO1_B1      | 37  | 38  | GPIO3_A6      | 左电机编码器A相|
| NC            | GND           | 39  | 40  | GPIO3_A7      | 左电机编码器B相|

## 编译Lubancat
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_lubancat.cmake ..
$ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_lubancat.cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## 设备树配置
开发需要更新设备树配置

[LubanCat Zero W devicetree](https://github.com/MagicPrince666/LubanCatWrt/blob/master/target/linux/rockchip/files-6.1/arch/arm64/boot/dts/rockchip/rk3566-lubancat-zero-w.dts)

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
10. 带码盘电机
11. 支持USB手柄
12. IMU位姿解算
13. 机关距离传感器VL5310x
14. 新增双轮差速底盘
15. 激光雷达电机速度控制
16. 激光雷达数据获取

## TO DO
1. 电机支持PID控制
2. 图像识别算法引入
3. 支持udp控制

## 调试笔记
右电机
```sh
echo 0 > /sys/class/leds/moto1-ena/brightness
echo 1 > /sys/class/leds/moto1-enb/brightness
echo 0 > /sys/class/pwm/pwmchip2/export
echo 40000 > /sys/class/pwm/pwmchip2/pwm0/period
echo 30000 > /sys/class/pwm/pwmchip2/pwm0/duty_cycle
echo normal > /sys/class/pwm/pwmchip2/pwm0/polarity
echo 1 > /sys/class/pwm/pwmchip2/pwm0/enable
```

左电机
```sh
echo 0 > /sys/class/leds/moto2-ena/brightness
echo 1 > /sys/class/leds/moto2-enb/brightness
echo 0 > /sys/class/pwm/pwmchip1/export
echo 40000 > /sys/class/pwm/pwmchip1/pwm0/period
echo 30000 > /sys/class/pwm/pwmchip1/pwm0/duty_cycle
echo normal > /sys/class/pwm/pwmchip1/pwm0/polarity
echo 1 > /sys/class/pwm/pwmchip1/pwm0/enable
```