# 泰山派智能小车

## 底盘模型
双轮差速底盘

## 状态
进行中...

## 硬件连接
| 连接设备       | name          | PIN | PIN | name          | 连接设备       |
|---------------|---------      |-----|-----|----------     |---------------|
| NC            | 3.3v          | 1   | 2   | 5v            | NC            |
| 右电机编码器A相| GPIO0_B6/sda2 | 3   | 4   | 5v            | NC            |
| 右电机编码器B相| GPIO0_B5/scl2 | 5   | 6   | GND           | NC            |
| 左电机使能脚2  | GPIO1_A4      | 7   | 8   | GPIO3_B7/TX3  | NC            |
| NC            | GND           | 9   | 10  | GPIO3_C0/RX3  | 雷达数据接收脚 |
| NC            | GPIO3_A1      | 11  | 12  | GPIO3_C4/PWM14| 左电机使能脚1  |
| 遥控SBUS接收   | GPIO3_A2      | 13  | 14  | GND           | NC            |
| MPU中断脚      | GPIO3_A3      | 15  | 16  | GPIO3_A4      | 触摸屏中断脚   |
| NC            | 3.3v          | 17  | 18  | GPIO3_A5      | 右电机使能脚1  |
| MOSI          | GPIO4_C3/MOSI | 19  | 20  | GND           | NC            |
| MISO          | GPIO4_C5/MISO | 21  | 22  | GPIO3_A6      | NC            |
| SCLK          | GPIO4_C2/SCLK | 23  | 24  | GPIO4_C6/CS0  | Touch Cs      |
| NC            | GND           | 25  | 26  | GPIO3_A7/CS1  | Lcd Cs        |
| MPU6050 SDA   | GPIO3_B6/SDA3 | 27  | 28  | GPIO3_B5/SCL3 | MPU6050 SCL   |
| NC            | GPIO3_B0      | 29  | 30  | GND           | NC            |
| Lcd Bl        | GPIO3_C2      | 31  | 32  | GPIO3_C5/PWM15| 雷达调速       |
| 左电机调速     | GPIO3_B1/PWM8 | 33  | 34  | GND           | NC            |
| 右电机调速     | GPIO3_B2/PWM9 | 35  | 36  | GPIO3_C3      | 右电机使能脚2  |
| LCD DC        | GPIO0_B7      | 37  | 38  | GPIO3_B3      | 左电机编码器A相|
| NC            | GND           | 39  | 40  | GPIO3_B4      | 左电机编码器B相|

## 编译Lubancat
```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_tspi.cmake ..
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/build_for_tspi.cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## 设备树配置
开发需要更新设备树配置

[tspi devicetree](https://github.com/MagicPrince666/LubanCatWrt/blob/master/target/linux/rockchip/files-6.1/arch/arm64/boot/dts/rockchip/rk3566-lubancat-zero-w.dts)

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

雷达调速 乐动雷达占空比45% - 55%
```bash
echo 0 > /sys/class/pwm/pwmchip3/export
echo 50000000 > /sys/class/pwm/pwmchip3/pwm0/period
echo 20000000 > /sys/class/pwm/pwmchip3/pwm0/duty_cycle
echo normal > /sys/class/pwm/pwmchip3/pwm0/polarity
echo 1 > /sys/class/pwm/pwmchip3/pwm0/enable
```
右电机
```bash
echo 0 > /sys/class/leds/moto1-ena/brightness
echo 1 > /sys/class/leds/moto1-enb/brightness
echo 0 > /sys/class/pwm/pwmchip2/export
echo 40000 > /sys/class/pwm/pwmchip2/pwm0/period
echo 30000 > /sys/class/pwm/pwmchip2/pwm0/duty_cycle
echo normal > /sys/class/pwm/pwmchip2/pwm0/polarity
echo 1 > /sys/class/pwm/pwmchip2/pwm0/enable
```

左电机
```bash
echo 0 > /sys/class/leds/moto2-ena/brightness
echo 1 > /sys/class/leds/moto2-enb/brightness
echo 0 > /sys/class/pwm/pwmchip1/export
echo 40000 > /sys/class/pwm/pwmchip1/pwm0/period
echo 30000 > /sys/class/pwm/pwmchip1/pwm0/duty_cycle
echo normal > /sys/class/pwm/pwmchip1/pwm0/polarity
echo 1 > /sys/class/pwm/pwmchip1/pwm0/enable
```